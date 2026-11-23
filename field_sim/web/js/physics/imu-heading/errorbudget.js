// errorbudget.js - analytic heading/position error growth (spec section 4, all [D] derived) plus a seeded
// Monte Carlo that drives ImuHeadingSystem along deterministic motion profiles.
// Units: seconds, radians, rad/s, inches, in/s unless a name says otherwise (*Deg = degrees).

import { createRng } from './rng.js';
import { se2Exp } from './angles.js';
import { MC_DEFAULTS } from './params.js';
import { ImuHeadingSystem } from './system.js';

// ---------------------------------------------------------------- analytic formulas

/**
 * Heading error stats after tS seconds, having rotated a total angle rotatedRad (rad). All results in rad.
 * terms are 1-sigma contributions: bias (initial bias * t), arw (white noise), rw (bias random walk),
 * gm (Gauss-Markov bias integrated), scale (scale-factor error * rotated angle).
 * mean = deterministic temperature-drift term.
 */
export function headingErrorStats(imuCfg, tS, rotatedRad = 0) {
  const t = Math.max(0, tS);
  const tau = imuCfg.gmTauS;
  const bias = imuCfg.biasInitSigmaRad * t;
  const arw = imuCfg.noiseDensityRad * Math.sqrt(t);
  const rw = imuCfg.biasRwRad * Math.sqrt((t * t * t) / 3);
  const gmVar =
    tau > 0 ? 2 * imuCfg.gmSigmaRad * imuCfg.gmSigmaRad * tau * (t - tau * (1 - Math.exp(-t / tau))) : 0;
  const gm = Math.sqrt(Math.max(0, gmVar));
  const scale = imuCfg.scaleErrSigma * Math.abs(rotatedRad);
  const sigma = Math.sqrt(bias * bias + arw * arw + rw * rw + gm * gm + scale * scale);
  const tt = imuCfg.tempTauS;
  const mean =
    tt > 0 ? imuCfg.tempCoeffRad * imuCfg.tempRiseK * (t - tt * (1 - Math.exp(-t / tt))) : 0;
  return { mean, sigma, terms: { bias, arw, rw, gm, scale } };
}

/**
 * 1-sigma lateral position error (inches) for straight travel at speedInS (in/s) for tS seconds,
 * small-angle: y = v * integral(theta dt). Gauss-Markov and scale terms are not included.
 */
export function lateralErrorSigma(imuCfg, speedInS, tS) {
  const t = Math.max(0, tS);
  const v = Math.abs(speedInS);
  const bias = v * imuCfg.biasInitSigmaRad * (t * t) / 2;
  const arw = v * imuCfg.noiseDensityRad * Math.sqrt((t * t * t) / 3);
  const rw = v * imuCfg.biasRwRad * Math.sqrt(Math.pow(t, 5) / 20);
  return { sigma: Math.sqrt(bias * bias + arw * arw + rw * rw), terms: { bias, arw, rw } };
}

// ---------------------------------------------------------------- deterministic profiles

// Segment tables: [durationS, vf in/s, vl in/s, omega rad/s]
const QUARTER_TURN_S = Math.PI / 2 / 2.5; // 0.6283 s at 2.5 rad/s

function buildAuto30() {
  const segs = [[0.5, 0, 0, 0]];
  for (let i = 0; i < 4; i++) {
    segs.push([5.5, 40, 0, 0]); // drive 40 in/s [A]
    segs.push([0.4, 0, 0, 0]); // stop
    segs.push([QUARTER_TURN_S, 0, 0, 2.5]); // 90 deg turn at 2.5 rad/s
    segs.push([0.4, 0, 0, 0]);
  }
  return segs; // total about 29.8 s; remainder is stopped
}

function buildTeleop120() {
  const block = [
    [2.5, 55, 0, 0], // drive forward
    [0.8, 0, 0, 3.0], // spin 2.4 rad
    [0.2, 0, 0, 0],
    [2.0, 0, 40, 0], // strafe
    [0.5, 0, 0, 0],
    [2.0, -40, 0, 0.5], // reverse arc
    [1.0, 0, 0, -1.0], // spin back
    [0.5, 0, 0, 0],
    [2.5, 30, 0, 0.4], // forward arc
  ]; // 12 s per block
  const segs = [];
  for (let i = 0; i < 10; i++) segs.push(...block);
  return segs;
}

function makeProfile(segs, { flywheelFrom, collisionTimes = [] }) {
  const starts = [];
  let acc = 0;
  for (const s of segs) {
    starts.push(acc);
    acc += s[0];
  }
  const total = acc;
  return (t) => {
    let vf = 0, vl = 0, omega = 0;
    if (t >= 0 && t < total) {
      let lo = 0, hi = segs.length - 1;
      while (lo < hi) {
        const mid = (lo + hi + 1) >> 1;
        if (starts[mid] <= t) lo = mid;
        else hi = mid - 1;
      }
      [, vf, vl, omega] = segs[lo];
    }
    const out = { vf, vl, omega, flywheelOn: t >= flywheelFrom };
    // Convenience marker: collision reported for t in [tc, tc + 0.01). runMonteCarlo uses COLLISION_TIMES
    // with interval logic instead, so it is independent of dt.
    for (const tc of collisionTimes) {
      if (t >= tc - 1e-9 && t < tc + 0.01 - 1e-9) out.collision = { impactSpeedInS: COLLISION_IMPACT_IN_S };
    }
    return out;
  };
}

const COLLISION_IMPACT_IN_S = 40; // in/s [A]
export const COLLISION_TIMES = { auto30: [], teleop120: [35, 70, 100] }; // s

export const PROFILES = {
  auto30: makeProfile(buildAuto30(), { flywheelFrom: 10, collisionTimes: COLLISION_TIMES.auto30 }),
  teleop120: makeProfile(buildTeleop120(), { flywheelFrom: 5, collisionTimes: COLLISION_TIMES.teleop120 }),
};
export const PROFILE_DURATIONS = { auto30: 30, teleop120: 120 };

// ---------------------------------------------------------------- Monte Carlo

/** Percentile of arr with linear interpolation. p is in PERCENT, 0..100 (percentile(a, 95) is the p95). */
export function percentile(arr, p) {
  if (!arr.length) return NaN;
  const a = Float64Array.from(arr).sort();
  const pos = (Math.min(100, Math.max(0, p)) / 100) * (a.length - 1);
  const lo = Math.floor(pos);
  const hi = Math.min(a.length - 1, lo + 1);
  return a[lo] + (a[hi] - a[lo]) * (pos - lo);
}

function summarize(v) {
  let sum = 0, mx = 0;
  for (const x of v) {
    sum += x;
    if (x > mx) mx = x;
  }
  return { mean: v.length ? sum / v.length : NaN, p50: percentile(v, 50), p95: percentile(v, 95), max: mx };
}

/**
 * Run `runs` seeded simulations of ImuHeadingSystem along a profile.
 * overrides: either IMU overrides in human units (IMU_PRESETS field names), or a structured object with any of
 * {imu, odometry, vision, estimator} sub-objects. Returns |heading err| in degrees and |pos err| in inches.
 */
export function runMonteCarlo({
  preset = 'pinpoint',
  seed = MC_DEFAULTS.seed,
  runs = MC_DEFAULTS.runs,
  profile = 'auto30',
  durationS,
  dt = MC_DEFAULTS.dt,
  overrides = {},
  sampleEveryS = 1,
} = {}) {
  const prof = PROFILES[profile];
  if (!prof) throw new Error('unknown profile: ' + profile);
  const dur = durationS ?? PROFILE_DURATIONS[profile];
  const collTimes = COLLISION_TIMES[profile] || [];
  const structured = ['imu', 'odometry', 'vision', 'estimator'].some((k) => k in overrides);
  const ov = structured ? overrides : { imu: overrides };
  const nSteps = Math.round(dur / dt);
  const sampleEvery = Math.max(1, Math.round(sampleEveryS / dt));
  const root = createRng(seed);

  const sampleSteps = [];
  for (let k = sampleEvery; k <= nSteps; k += sampleEvery) sampleSteps.push(k);
  if (!sampleSteps.length || sampleSteps[sampleSteps.length - 1] !== nSteps) sampleSteps.push(nSteps);
  const hSeries = sampleSteps.map(() => []);
  const pSeries = sampleSteps.map(() => []);
  const final = [];

  for (let r = 0; r < runs; r++) {
    const sys = new ImuHeadingSystem({
      preset,
      seed: root.fork('run' + r).seed,
      imuOverrides: ov.imu || {},
      odometryOverrides: ov.odometry || {},
      visionOverrides: ov.vision || {},
      estimatorOverrides: ov.estimator || {},
    });
    let pose = { x: 0, y: 0, h: 0 };
    sys.reset(pose, 0);
    let si = 0;
    for (let k = 0; k < nSteps; k++) {
      const t0 = k * dt;
      const m = prof(t0 + 0.5 * dt);
      pose = se2Exp(pose, m.vf * dt, m.vl * dt, m.omega * dt);
      const collisions = [];
      for (const tc of collTimes) {
        if (tc > t0 + 1e-9 && tc <= t0 + dt + 1e-9) collisions.push({ impactSpeedInS: COLLISION_IMPACT_IN_S });
      }
      sys.step(dt, pose, { collisions, flywheelOn: m.flywheelOn, visionVisible: true });
      if (si < sampleSteps.length && k + 1 === sampleSteps[si]) {
        hSeries[si].push(Math.abs(sys.getHeadingError()) * 180 / Math.PI);
        pSeries[si].push(sys.getPositionError());
        si++;
      }
    }
    final.push({
      headingErrDeg: hSeries[hSeries.length - 1][r],
      posErrIn: pSeries[pSeries.length - 1][r],
    });
  }

  const series = sampleSteps.map((k, i) => ({
    t: k * dt,
    headingP95Deg: percentile(hSeries[i], 95),
    posP95In: percentile(pSeries[i], 95),
  }));
  return {
    headingErrDeg: summarize(final.map((f) => f.headingErrDeg)),
    posErrIn: summarize(final.map((f) => f.posErrIn)),
    final,
    series,
  };
}
