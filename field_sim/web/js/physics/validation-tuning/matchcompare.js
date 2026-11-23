// Compare simulated vs real match statistics (cycle times, scoring rate). Real FTC reference numbers were NOT found,
// so only reference_template.json (empty) ships; never fabricate numbers. Units: s, inches.
import { BallWorld, BALL } from '../../balls.js';
import { SHOT } from './params.js';
import { mulberry32, gaussian } from './rng.js';
import { mean, std, median, welchT, ksTwoSample, twoProportionZ, wilson } from './stats.js';

const finiteArr = a => Array.isArray(a) && a.every(Number.isFinite);

/** Parse and validate a user-supplied reference {source, cycles?, shots?, notes?}. */
export function loadReference(json) {
  const o = typeof json === 'string' ? JSON.parse(json) : json;
  if (!o || typeof o !== 'object') throw new TypeError('reference must be an object');
  if (typeof o.source !== 'string') throw new TypeError('reference.source must be a string');
  const cycles = o.cycles === undefined ? [] : o.cycles;
  if (!finiteArr(cycles) || cycles.some(c => c <= 0)) throw new TypeError('reference.cycles must be positive finite seconds');
  let shots = null;
  if (o.shots !== undefined && o.shots !== null) {
    const { made, attempted } = o.shots;
    if (!Number.isInteger(made) || !Number.isInteger(attempted) || made < 0 || attempted < made) throw new TypeError('reference.shots needs integer 0 <= made <= attempted');
    shots = { made, attempted };
  }
  return { source: o.source, cycles: cycles.slice(), shots };
}

const summary = a => ({ n: a.length, mean: mean(a), sd: a.length > 1 ? std(a) : NaN, median: median(a) });

/** Cycle times in seconds. welch = Welch t-test on the means, ks = two-sample Kolmogorov-Smirnov on the distributions. */
export function compareCycles(sim, real) {
  return { sim: summary(sim), real: summary(real), welch: welchT(sim, real), ks: ksTwoSample(sim, real) };
}

/** sim/real = {k, n} (successes of n). z, p from the pooled two-proportion z test; ci = Wilson 95% for each side. */
export function compareScoring(sim, real) {
  const t = twoProportionZ(sim.k, sim.n, real.k, real.n);
  return { z: t.z, p: t.p, sim: wilson(sim.k, sim.n), real: wilson(real.k, real.n) };
}

/** Seeded Monte-Carlo shot model (SHOT, ASSUMED): launch from the standoff point in front of the cell opening with a
 *  ballistic velocity to the aim point A in flightTimeS, speed ~ N(1, speedSigmaFrac), yaw/pitch ~ N(0, angleSigmaRad).
 *  One ball per fresh world, settleS seconds; hit = ball scored in the target hive. env must contain hives. */
export function shotHitRate(env, opts = {}) {
  const n = opts.nTrials ?? SHOT.nTrials, hive = opts.hive || 'red', cell = opts.cell || 'audience';
  const c = env.hives && env.hives[hive] && env.hives[hive].cells && env.hives[hive].cells[cell];
  if (!c) throw new Error(`env has no hive '${hive}' cell '${cell}'`);
  const rng = mulberry32(opts.seed ?? 0x5eed), gauss = gaussian(rng);
  const A = c.aim_point, nn = c.opening_normal, T = SHOT.flightTimeS, dt = 1 / 60;
  const hl = Math.hypot(nn[0], nn[1]) || 1;
  const L = [A[0] + nn[0] / hl * SHOT.standoffIn, A[1] + nn[1] / hl * SHOT.standoffIn, SHOT.launchZIn];
  const v0 = [(A[0] - L[0]) / T, (A[1] - L[1]) / T, (A[2] - L[2]) / T + 0.5 * BALL.g * T];
  const sp0 = Math.hypot(v0[0], v0[1], v0[2]), yaw0 = Math.atan2(v0[1], v0[0]), pit0 = Math.atan2(v0[2], Math.hypot(v0[0], v0[1]));
  let k = 0;
  for (let i = 0; i < n; i++) {
    const sp = sp0 * (1 + gauss() * SHOT.speedSigmaFrac), yaw = yaw0 + gauss() * SHOT.angleSigmaRad, pit = pit0 + gauss() * SHOT.angleSigmaRad;
    const v = [sp * Math.cos(pit) * Math.cos(yaw), sp * Math.cos(pit) * Math.sin(yaw), sp * Math.sin(pit)];
    const w = new BallWorld({ bounds: env.bounds, hives: env.hives }, null);
    const b = w.addBall('pollen', L.slice(), v, 'pollen');
    for (let t = 0; t < SHOT.settleS; t += dt) w.step(dt);
    if (b.state === 'scored' && b.hive === hive) k++;
  }
  return { k, n, p: k / n, ci: wilson(k, n) };
}
