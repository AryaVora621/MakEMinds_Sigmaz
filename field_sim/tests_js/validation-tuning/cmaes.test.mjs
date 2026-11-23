import test from 'node:test';
import assert from 'node:assert/strict';
import { cmaes } from '../../web/js/physics/validation-tuning/cmaes.js';
import { symEig } from '../../web/js/physics/validation-tuning/linalg.js';

const sphere = (x) => x.reduce((s, v) => s + v * v, 0);
const rosen = (x) => 100 * (x[1] - x[0] * x[0]) ** 2 + (1 - x[0]) ** 2;

test('symEig diagonalises a symmetric matrix', () => {
  const C = [[2, 1], [1, 2]];
  const { values, vectors } = symEig(C);
  const v = [...values].sort((a, b) => a - b);
  assert.ok(Math.abs(v[0] - 1) < 1e-9 && Math.abs(v[1] - 3) < 1e-9);
  assert.equal(vectors.length, 2);
});
test('cmaes converges on the sphere function', () => {
  const r = cmaes(sphere, [3, -2, 1], { sigma0: 1, seed: 1, maxEvals: 20000 });
  assert.ok(r.f < 1e-10, `f ${r.f}`);
  assert.ok(['tolFun', 'tolX', 'maxEvals'].includes(r.stop));
});
test('cmaes solves 2-D Rosenbrock from [-1, 1]', () => {
  const r = cmaes(rosen, [-1, 1], { sigma0: 0.5, seed: 2, maxEvals: 4000 }); // maxEvals = 1000*n^2
  assert.ok(r.f < 1e-8, `f ${r.f} after ${r.evals} evals`);
  assert.ok(Math.abs(r.x[0] - 1) < 1e-3 && Math.abs(r.x[1] - 1) < 1e-3);
  assert.ok(r.evals <= 4000 + 50);
});
test('cmaes is deterministic per seed and differs across seeds', () => {
  const a = cmaes(rosen, [-1, 1], { seed: 5, maxEvals: 800 });
  const b = cmaes(rosen, [-1, 1], { seed: 5, maxEvals: 800 });
  const c = cmaes(rosen, [-1, 1], { seed: 6, maxEvals: 800 });
  assert.deepEqual(a, b);
  assert.notDeepEqual(a.x, c.x);
});
test('cmaes respects bounds on every evaluated point', () => {
  const seen = [];
  const f = (x) => { seen.push(x.slice()); return (x[0] - 5) ** 2 + (x[1] + 5) ** 2; };
  const r = cmaes(f, [1, 1], { lower: [0, -2], upper: [2, 3], seed: 3, maxEvals: 1500 });
  for (const x of seen) {
    assert.ok(x[0] >= 0 && x[0] <= 2 && x[1] >= -2 && x[1] <= 3, `out of bounds ${x}`);
  }
  assert.ok(Math.abs(r.x[0] - 2) < 1e-3 && Math.abs(r.x[1] + 2) < 1e-3, `x ${r.x}`);
});
test('cmaes honours maxEvals', () => {
  const r = cmaes(rosen, [-1, 1], { seed: 1, maxEvals: 60, tolFun: 0, tolX: 0 });
  assert.equal(r.stop, 'maxEvals');
  assert.ok(r.evals <= 60 + 20);
});
