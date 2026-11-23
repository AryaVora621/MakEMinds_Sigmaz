// Step-time benchmark. Method follows Box2D benchmark/main.c (SOURCED): one untimed warm-up step, runCount repeats,
// keep the minimum over repeats. Times are milliseconds per world.step(1/60). Constants from params.js BENCH.
import { BallWorld, BALL } from '../../balls.js';
import { BENCH } from './params.js';
import { mulberry32 } from './rng.js';
import { FIELD_BOUNDS } from './node_env.js';

export const BENCH_SCENES = Object.keys(BENCH.scenes);   // rain, pile, push, drop
const SEED = 1;   // ASSUMED: fixed seed so every run times the same scene

function build(name) {
  const n = BENCH.scenes[name].balls;
  const rng = mulberry32(SEED);
  const world = new BallWorld({ bounds: { ...FIELD_BOUNDS } }, rng);
  let pre = null;
  if (name === 'rain') {          // balls dropped over the whole field from 20..120 in
    for (let i = 0; i < n; i++) world.addBall('pollen', [10 + rng() * 124, 10 + rng() * 124, 20 + rng() * 100], [0, 0, 0], 'pollen');
  } else if (name === 'pile') {   // dense column: 100 balls in a 30 x 30 in region, z 6..106 in
    for (let i = 0; i < n; i++) world.addBall('pollen', [57 + rng() * 30, 57 + rng() * 30, 6 + rng() * 100], [0, 0, 0], 'pollen');
  } else if (name === 'push') {   // resting balls on a grid, robot (9 x 9 x 14 in) driving through at 60 in/s along y=72
    for (let i = 0; i < n; i++) world.addBall('pollen', [40 + (i % 10) * 4 + (rng() - 0.5) * 0.8, 62 + ((i / 10) | 0) * 4 + (rng() - 0.5) * 0.8, BALL.r], [0, 0, 0], 'pollen');
    const dims = { halfX: 9, halfY: 9, height: 14 };
    pre = i => world.setRobot({ x: 14 + 60 * i * BENCH.dt, y: 72, h: 0, vx: 60, vy: 0, om: 0 }, dims);
  } else {                        // drop: 12 balls from 6..66 in
    for (let i = 0; i < n; i++) world.addBall('pollen', [20 + rng() * 104, 20 + rng() * 104, 6 + rng() * 60], [0, 0, 0], 'pollen');
  }
  return { world, pre, balls: n };
}

function pctl(sorted, q) { return sorted[Math.min(sorted.length - 1, Math.ceil(q * sorted.length) - 1)]; }

/** Time one scene. now defaults to performance.now (injectable for tests). */
export function runBenchmark(name, opts = {}) {
  if (!BENCH.scenes[name]) throw new Error('unknown bench scene: ' + name);
  const steps = opts.steps ?? BENCH.steps, runCount = opts.runCount ?? BENCH.runCount;
  const now = opts.now || (() => globalThis.performance.now());
  const runs = [];
  let best = null, balls = 0;
  for (let r = 0; r < runCount; r++) {
    const sc = build(name); balls = sc.balls;
    for (let w = 0; w < BENCH.warmupSteps; w++) { if (sc.pre) sc.pre(0); sc.world.step(BENCH.dt); }   // untimed
    const times = [];
    for (let i = 0; i < steps; i++) {
      if (sc.pre) sc.pre(i + BENCH.warmupSteps);
      const t0 = now(); sc.world.step(BENCH.dt); times.push(now() - t0);
    }
    const s = times.slice().sort((a, b) => a - b);
    const med = s.length % 2 ? s[(s.length - 1) / 2] : 0.5 * (s[s.length / 2 - 1] + s[s.length / 2]);
    runs.push(med);
    if (!best || med < best.medianMs) best = { medianMs: med, p95Ms: pctl(s, 0.95), maxMs: s[s.length - 1] };
  }
  return { scene: name, balls, steps, medianMs: best.medianMs, p95Ms: best.p95Ms, maxMs: best.maxMs, minMedianMs: Math.min(...runs), runs };
}

export function runAll(opts = {}) { return BENCH_SCENES.map(n => runBenchmark(n, opts)); }

export function toCsv(results) {
  const rows = ['scene,balls,steps,median_ms,p95_ms,max_ms'];
  for (const r of results) rows.push([r.scene, r.balls, r.steps, r.medianMs.toFixed(4), r.p95Ms.toFixed(4), r.maxMs.toFixed(4)].join(','));
  return rows.join('\n') + '\n';
}

/** baseline = {results:[{scene, medianMs}]} (as in bench_baseline.json). Regression when medianMs > baseline*factor. */
export function compareBaseline(results, baseline, factor = BENCH.regressFactor) {
  const regressions = [];
  for (const r of results) {
    const b = (baseline.results || []).find(x => x.scene === r.scene);
    if (b && r.medianMs > b.medianMs * factor) regressions.push({ scene: r.scene, medianMs: r.medianMs, baselineMs: b.medianMs });
  }
  return { pass: regressions.length === 0, regressions };
}
