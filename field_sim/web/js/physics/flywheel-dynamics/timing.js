// timing.js - offline timing studies; each function runs a fresh FlywheelPlant.
import { FlywheelPlant } from './flywheel_plant.js';
import { makeFlywheelParams, RADS_TO_RPM } from './params.js';
import { makeDcMotor, driveTorqueAtWheelNm } from './motor.js';
import { lossTorquesNm } from './losses.js';

const num = (x, d) => (typeof x === 'number' && Number.isFinite(x) ? x : d);

/**
 * Run until |rpm - target| stays within tol for holdS; returns the time the stay began, or Infinity.
 * Plant must already be reset and targeted.
 */
function runToSettle(plant, target, tol, holdS, dt, maxS) {
  let inSince = -1;
  let t = 0;
  while (t < maxS) {
    plant.step(dt); t += dt;
    if (Math.abs(plant.state.rpm - target) <= tol) {
      if (inSince < 0) inSince = t;
      if (t - inSince >= holdS) return inSince;
    } else inSince = -1;
  }
  return Infinity;
}

/**
 * Time [s] to go from fromRpm to within settleFrac of toRpm and stay for holdS. Infinity if not reached by maxS.
 * The command is clamped by limits.maxRpm, so an unreachable target gives Infinity.
 */
export function spinUpTimeS(params, fromRpm, toRpm, opts = {}) {
  const dt = num(opts.dt, 0.001), settle = num(opts.settleFrac, 0.02);
  const holdS = num(opts.holdS, 0.05), maxS = num(opts.maxS, 10);
  const plant = new FlywheelPlant(params);
  plant.reset(fromRpm);
  plant.setTargetRpm(toRpm);
  const tol = Math.max(settle * Math.abs(toRpm), 1e-6);
  return runToSettle(plant, toRpm, tol, holdS, dt, maxS);
}

/** Spin up to rpm, hold 1.5 s, fire one ball, return dip and recovery time [s]. */
export function recoveryTimeS(params, rpm, ball, opts = {}) {
  const dt = num(opts.dt, 0.001), settle = num(opts.settleFrac, 0.02);
  const holdS = num(opts.holdS, 0.05), maxS = num(opts.maxS, 10);
  const plant = new FlywheelPlant(params);
  plant.setTargetRpm(rpm);
  const up = runToSettle(plant, rpm, Math.max(settle * rpm, 1e-6), holdS, dt, maxS);
  if (!Number.isFinite(up)) return { recoveryS: Infinity, dipFrac: 0, minRpm: plant.state.rpm, shot: null };
  for (let t = 0; t < 1.5; t += dt) plant.step(dt);
  const pre = plant.state.rpm;
  const shot = plant.fire(ball);
  let minRpm = pre, t = 0, inSince = -1;
  while (t < maxS) {
    plant.step(dt); t += dt;
    const r = plant.state.rpm;
    if (r < minRpm) minRpm = r;
    if (Math.abs(r - rpm) <= settle * rpm) {
      if (inSince < 0) inSince = t;
      if (t - inSince >= holdS) {
        return { recoveryS: inSince, dipFrac: pre > 0 ? 1 - minRpm / pre : 0, minRpm, shot };
      }
    } else inSince = -1;
  }
  return { recoveryS: Infinity, dipFrac: pre > 0 ? 1 - minRpm / pre : 0, minRpm, shot };
}

/** Time [s] to coast from fromRpm down to toRpm (default 0) with target 0. Infinity if not reached by maxS. */
export function coastDownTimeS(params, fromRpm, toRpm = 0, opts = {}) {
  const dt = num(opts.dt, 0.001), maxS = num(opts.maxS, 60);
  const plant = new FlywheelPlant(params);
  plant.reset(fromRpm);
  plant.setTargetRpm(0);
  if (plant.state.rpm <= toRpm) return 0;
  let t = 0;
  while (t < maxS) {
    plant.step(dt); t += dt;
    if (plant.state.rpm <= toRpm) return t;
  }
  return Infinity;
}

/** Fire balls[i] at i*intervalS after settling at rpm. Shot times tS are relative to the first fire. */
export function burstSim(params, rpm, balls, intervalS, opts = {}) {
  const dt = num(opts.dt, 0.001), settleS = num(opts.settleS, 2.0);
  const every = num(opts.traceEveryS, 0.01);
  const plant = new FlywheelPlant(params);
  plant.setTargetRpm(rpm);
  const shots = [], trace = [];
  const list = Array.isArray(balls) ? balls : [];
  for (let t = 0; t < settleS; t += dt) plant.step(dt);
  const total = (list.length ? (list.length - 1) * intervalS : 0) + 0.5;
  let t = 0, next = 0, nextTrace = 0;
  while (t < total - 1e-12) {
    while (next < list.length && t >= next * intervalS - 1e-9) {
      const rpmAtFire = plant.state.rpm;
      const s = plant.fire(list[next]);
      shots.push({ tS: t, rpmAtFire, exitSpeedMps: s.exitSpeedMps, dipFrac: s.dipFrac });
      next++;
    }
    if (t >= nextTrace - 1e-9) { trace.push({ tS: t, rpm: plant.state.rpm }); nextTrace += every; }
    plant.step(dt); t += dt;
  }
  trace.push({ tS: t, rpm: plant.state.rpm });
  return { shots, trace };
}

/** Steady-state rpm with full battery voltage applied, by bisection on motor torque = losses. */
export function reachableRpm(params) {
  const p = makeFlywheelParams(params);
  const motor = makeDcMotor(p.motor);
  const V = p.battery.voltageV;
  const f = (w) =>
    driveTorqueAtWheelNm(motor, p.drive, V, w).torqueNm -
    lossTorquesNm(w, { losses: p.losses, motor, drive: p.drive, radiusM: p.wheel.radiusM }).totalNm;
  let lo = 0, hi = 2 * motor.freeSpeedRadS / p.drive.gearRatio;
  if (f(lo) <= 0) return 0;
  for (let i = 0; i < 100; i++) {
    const mid = 0.5 * (lo + hi);
    if (f(mid) > 0) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi) * RADS_TO_RPM;
}
