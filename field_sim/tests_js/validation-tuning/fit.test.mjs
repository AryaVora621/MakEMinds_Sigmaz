import test from 'node:test';
import assert from 'node:assert/strict';
import { fitFeedforward, fitRamp, fitZeroPowerDecel, feedforwardToDrive, restitutionFromHeights, restitutionFromVelocities,
  fitRestitutionBySim, fitDrag, dragKToCd, cdToDragK } from '../../web/js/physics/validation-tuning/fit.js';
import { solveLeastSquares } from '../../web/js/physics/validation-tuning/linalg.js';
import { BALL } from '../../web/js/balls.js';
import { ffSamples, coastSamples, dragTrack } from './_fixtures.mjs';

const rel = (a, b) => Math.abs(a - b) / Math.abs(b);

test('solveLeastSquares recovers a line', () => {
  const A = [], y = [];
  for (let i = 0; i < 20; i++) { A.push([1, i]); y.push(3 + 2 * i); }
  const r = solveLeastSquares(A, y);
  assert.ok(Math.abs(r.beta[0] - 3) < 1e-9 && Math.abs(r.beta[1] - 2) < 1e-9);
  assert.ok(r.r2 > 0.999999);
});
test('fitFeedforward recovers kS, kV, kA within 5%', () => {
  const f = fitFeedforward(ffSamples());
  assert.ok(rel(f.kS, 0.05) < 0.05, `kS ${f.kS}`);
  assert.ok(rel(f.kV, 0.0167) < 0.05, `kV ${f.kV}`);
  assert.ok(rel(f.kA, 0.004) < 0.05, `kA ${f.kA}`);
  assert.ok(f.r2 > 0.99 && f.n > 100);
});
test('fitFeedforward is deterministic', () => {
  assert.deepEqual(fitFeedforward(ffSamples({ seed: 3 })), fitFeedforward(ffSamples({ seed: 3 })));
});
test('fitRamp recovers kS and kV from a noiseless ramp', () => {
  const t = [], u = [], v = [];
  for (let i = 0; i < 100; i++) { t.push(i * 0.1); const vv = 2 + i * 0.5; v.push(vv); u.push(0.05 + 0.0167 * vv); }
  const f = fitRamp({ t, u, v });
  assert.ok(rel(f.kS, 0.05) < 1e-6 && rel(f.kV, 0.0167) < 1e-6);
});
test('fitZeroPowerDecel recovers decel within 2%', () => {
  const f = fitZeroPowerDecel(coastSamples({ decel: 120 }));
  assert.ok(rel(f.decel, 120) < 0.02, `decel ${f.decel}`);
  assert.ok(f.decel > 0);
});
test('feedforwardToDrive: vmax, tau and t90 accel', () => {
  const d = feedforwardToDrive({ kS: 0.05, kV: 0.0167, kA: 0.004 });
  assert.ok(rel(d.maxSpeed, 0.95 / 0.0167) < 1e-9);
  assert.ok(rel(d.tau, 0.004 / 0.0167) < 1e-9);
  assert.ok(rel(d.maxAccel, (0.9 * d.maxSpeed) / (d.tau * Math.log(10))) < 1e-9);
  const i = feedforwardToDrive({ kS: 0.05, kV: 0.0167, kA: 0.004 }, { match: 'initial' });
  assert.ok(rel(i.maxAccel, 0.95 / 0.004) < 1e-9);
});
test('restitution from heights: e = sqrt(hb/hd) with error propagation', () => {
  const r = restitutionFromHeights([{ hDrop: 40, hBounce: 16, sigmaDrop: 1, sigmaBounce: 0.5 }]);
  const e = Math.sqrt(16 / 40);
  assert.ok(Math.abs(r.e - e) < 1e-12);
  const sig = e * 0.5 * Math.sqrt((0.5 / 16) ** 2 + (1 / 40) ** 2);
  assert.ok(Math.abs(r.perTrial[0].sigmaE - sig) < 1e-12);
});
test('restitution from heights: mean, sd and se over trials', () => {
  const r = restitutionFromHeights([{ hDrop: 40, hBounce: 16 }, { hDrop: 40, hBounce: 25 }, { hDrop: 40, hBounce: 10 }]);
  const es = [16, 25, 10].map((h) => Math.sqrt(h / 40));
  const m = es.reduce((a, b) => a + b) / 3;
  const sd = Math.sqrt(es.reduce((a, b) => a + (b - m) ** 2, 0) / 2);
  assert.ok(Math.abs(r.e - m) < 1e-12 && Math.abs(r.sd - sd) < 1e-12 && Math.abs(r.se - sd / Math.sqrt(3)) < 1e-12);
  assert.equal(r.n, 3);
});
test('restitution from velocities', () => {
  const r = restitutionFromVelocities([{ vIn: -200, vOut: 100 }, { vIn: -100, vOut: 60 }]);
  assert.ok(Math.abs(r.e - 0.55) < 1e-12);
});
test('fitRestitutionBySim leaves BALL deep-equal to before', () => {
  const before = structuredClone(BALL);
  const out = fitRestitutionBySim(40 - BALL.r, 0.6, { field: 'restitutionFloor' });
  assert.deepEqual(BALL, before);
  assert.ok(out.value >= 0.05 && out.value <= 0.95, `value ${out.value}`);
  assert.ok(out.iterations > 0);
  assert.ok(Math.abs(out.simE - 0.6) < 0.15, `simE ${out.simE}`);
});
test('fitDrag recovers k within 15% and is deterministic', () => {
  const tr = dragTrack({ k: 0.0008 });
  const a = fitDrag(tr, { kInit: 0.0004, seed: 1 });
  assert.ok(rel(a.k, 0.0008) < 0.15, `k ${a.k}`);
  assert.ok(a.rmse < 0.2, `rmse ${a.rmse}`);
  const b = fitDrag(tr, { kInit: 0.0004, seed: 1 });
  assert.equal(a.k, b.k);
});
test('dragKToCd and cdToDragK round-trip', () => {
  const cd = dragKToCd(0.0008, 0.07, 2.5);
  assert.ok(cd > 0);
  assert.ok(rel(cdToDragK(cd, 0.07, 2.5), 0.0008) < 1e-9);
});
