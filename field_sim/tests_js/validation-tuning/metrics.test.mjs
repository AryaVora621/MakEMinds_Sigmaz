import test from 'node:test';
import assert from 'node:assert/strict';
import { allclose, rmse, maxAbsErr, discreteFrechet, hausdorff, dtw, wrapAngle, resampleByTime, alignSE2, ate, rpe, compareTracks }
  from '../../web/js/physics/validation-tuning/metrics.js';

const line = (n, dx = 1, y = 0) => Array.from({ length: n }, (_, i) => [i * dx, y]);
const track = (n, f) => {
  const tr = { t: [], x: [], y: [], h: [] };
  for (let i = 0; i < n; i++) { const p = f(i * 0.1); tr.t.push(i * 0.1); tr.x.push(p[0]); tr.y.push(p[1]); tr.h.push(p[2] ?? 0); }
  return tr;
};
const curve = (t) => [20 * Math.cos(t), 20 * Math.sin(t), t];

test('allclose scalar and array, numpy form with atol', () => {
  assert.equal(allclose(1, 1).pass, true);
  assert.equal(allclose([1, 2, 3], [1, 2, 3.5], { rtol: 1e-6, atol: 1e-3 }).pass, false);
  const r = allclose([1, 2, 3], [1, 2, 3.5], { rtol: 1e-6, atol: 1e-3 });
  assert.equal(r.worstIndex, 2);
  assert.ok(Math.abs(r.maxErr - 0.5) < 1e-12);
});
test('allclose near zero relies on atol', () => {
  assert.equal(allclose(1e-12, 0, { rtol: 1e-6, atol: 1e-9 }).pass, true);
  assert.equal(allclose(1e-12, 0, { rtol: 1e-6, atol: 0 }).pass, false);
  assert.equal(allclose(1e-6, 0, { rtol: 1e-6, atol: 1e-9 }).pass, false);
});
test('rmse and maxAbsErr', () => {
  assert.equal(rmse([1, 2, 3], [1, 2, 3]), 0);
  assert.ok(Math.abs(rmse([0, 0], [3, 4]) - Math.sqrt(12.5)) < 1e-12);
  assert.equal(maxAbsErr([0, 0], [3, -4]), 4);
});
test('identical polylines give 0 for frechet, hausdorff, dtw', () => {
  const P = line(10);
  assert.equal(discreteFrechet(P, P), 0);
  assert.equal(hausdorff(P, P), 0);
  assert.equal(dtw(P, P), 0);
});
test('a d-shifted copy gives frechet = d', () => {
  const P = line(12, 1, 0), Q = line(12, 1, 3);
  assert.ok(Math.abs(discreteFrechet(P, Q) - 3) < 1e-12);
});
test('hausdorff <= frechet', () => {
  const P = track(40, curve), Q = track(40, (t) => [20 * Math.cos(t * 1.1), 20 * Math.sin(t * 1.1)]);
  const A = P.x.map((x, i) => [x, P.y[i]]), B = Q.x.map((x, i) => [x, Q.y[i]]);
  assert.ok(hausdorff(A, B) <= discreteFrechet(A, B) + 1e-12);
});
test('reversed path: hausdorff 0 but frechet large', () => {
  const P = line(11), Q = line(11).reverse();
  assert.equal(hausdorff(P, Q), 0);
  assert.ok(discreteFrechet(P, Q) >= 10 - 1e-12);
});
test('3D points are accepted', () => {
  assert.ok(Math.abs(discreteFrechet([[0, 0, 0], [1, 0, 0]], [[0, 0, 2], [1, 0, 2]]) - 2) < 1e-12);
});
test('wrapAngle', () => {
  assert.ok(Math.abs(wrapAngle(3 * Math.PI) - Math.PI) < 1e-9 || Math.abs(wrapAngle(3 * Math.PI) + Math.PI) < 1e-9);
  assert.ok(Math.abs(wrapAngle(0.3) - 0.3) < 1e-12);
});
test('resampleByTime interpolates linearly and wraps heading the short way', () => {
  const tr = { t: [0, 1], x: [0, 10], y: [0, 20], h: [3.0, -3.0] };
  const r = resampleByTime(tr, [0.5]);
  assert.ok(Math.abs(r.x[0] - 5) < 1e-12 && Math.abs(r.y[0] - 10) < 1e-12);
  // shortest arc from 3.0 to -3.0 passes through pi, not through 0
  assert.ok(Math.abs(Math.abs(wrapAngle(r.h[0])) - Math.PI) < 0.2);
});
test('alignSE2 recovers a known rigid transform', () => {
  const theta = 0.4, tx = 5, ty = -3;
  const P = line(8, 2).map(([x, y], i) => [x, y + Math.sin(i)]);
  const Q = P.map(([x, y]) => [Math.cos(theta) * x - Math.sin(theta) * y + tx, Math.sin(theta) * x + Math.cos(theta) * y + ty]);
  const a = alignSE2(P, Q);
  assert.ok(Math.abs(a.theta - theta) < 1e-9);
  assert.ok(Math.abs(a.tx - tx) < 1e-9 && Math.abs(a.ty - ty) < 1e-9);
});
test('ATE of identical tracks is 0', () => {
  const a = track(50, curve);
  assert.ok(ate(a, a).rmse < 1e-9);
});
test('ATE after a known SE(2) transform: about 0 with align, nonzero without', () => {
  const est = track(60, curve);
  const th = 0.3, tx = 4, ty = -2;
  const ref = { t: est.t.slice(), x: est.x.map((x, i) => Math.cos(th) * x - Math.sin(th) * est.y[i] + tx), y: est.x.map((x, i) => Math.sin(th) * x + Math.cos(th) * est.y[i] + ty), h: est.h.map((h) => h + th) };
  const un = ate(est, ref);
  const al = ate(est, ref, { align: true });
  assert.ok(al.rmse < 1e-6, `aligned rmse ${al.rmse}`);
  assert.ok(un.rmse > 1, `unaligned rmse ${un.rmse}`);
  assert.ok(al.n > 0 && un.n > 0);
});
test('ATE drops est samples outside the ref time span', () => {
  const ref = track(10, curve);
  const est = { t: [-1, 0, 0.5, 5], x: [0, 20, 0, 0], y: [0, 0, 0, 0], h: [0, 0, 0, 0] };
  assert.equal(ate(est, ref).n, 2);
});
test('RPE of identical tracks is 0', () => {
  const a = track(60, curve);
  const r = rpe(a, a, { delta: 1 });
  assert.ok(r.transRmse < 1e-9);
  assert.ok(r.rotRmse < 1e-9);
  assert.ok(r.n > 0);
});
test('RPE is invariant to a global rigid transform', () => {
  const a = track(60, curve);
  const th = 0.5;
  const b = { t: a.t, x: a.x.map((x, i) => Math.cos(th) * x - Math.sin(th) * a.y[i] + 7), y: a.x.map((x, i) => Math.sin(th) * x + Math.cos(th) * a.y[i] - 1), h: a.h.map((h) => h + th) };
  assert.ok(rpe(a, b, { delta: 1 }).transRmse < 1e-9);
});
test('compareTracks reports every metric', () => {
  const a = track(60, curve);
  const c = compareTracks(a, a);
  for (const k of ['ate', 'ateAligned', 'rpe', 'frechet', 'hausdorff', 'dtw', 'durationS']) assert.ok(k in c, k);
  assert.ok(c.frechet < 1e-9);
  assert.ok(Math.abs(c.durationS - 5.9) < 1e-9);
});
