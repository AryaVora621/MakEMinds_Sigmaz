import test from 'node:test';
import assert from 'node:assert/strict';
import { buildDrivetrainConfig } from '../../web/js/physics/wheel-friction/geometry.js';
import { computeNormalLoads, solveStaticLoads } from '../../web/js/physics/wheel-friction/loads.js';
import { stribeckMu, selectMu, coulombClamp, clampCircle, clampEllipse, slipRatio } from '../../web/js/physics/wheel-friction/frictionlaw.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b} (tol ${tol})`);
const sum = (a) => Array.from(a).reduce((s, x) => s + x, 0);

test('symmetric robot: W/4 each, sums to W', () => {
  const cfg = buildDrivetrainConfig();
  const { loads, tipping } = computeNormalLoads(cfg, [0, 0]);
  assert.equal(tipping, false);
  for (const n of loads) near(n, cfg.weightN / 4, 1e-6);
  near(sum(loads), cfg.weightN, 1e-6);
});

test('forward CG offset loads the front; forward accel loads the rear', () => {
  const cfg = buildDrivetrainConfig();
  const shifted = { ...cfg, cgM: [0.02, 0] };
  const a = computeNormalLoads(shifted, [0, 0]).loads;
  assert.ok(a[0] + a[1] > a[2] + a[3]);
  const b = computeNormalLoads(cfg, [3, 0]).loads;
  assert.ok(b[2] + b[3] > b[0] + b[1]);
  near(sum(b), cfg.weightN, 1e-6);
});

test('huge accel sets tipping with no negative loads', () => {
  const cfg = buildDrivetrainConfig();
  const r = computeNormalLoads(cfg, [200, 0]);
  assert.equal(r.tipping, true);
  for (const n of r.loads) assert.ok(n >= 0);
  near(sum(r.loads), cfg.weightN, 1e-6);
});

test('out buffer is reused', () => {
  const cfg = buildDrivetrainConfig();
  const out = new Float64Array(4);
  const r = computeNormalLoads(cfg, [0, 0], out);
  assert.equal(r.loads, out);
});

test('solveStaticLoads satisfies the moment equations', () => {
  const pos = [[0.1, 0.12], [0.1, -0.12], [-0.1, 0.12], [-0.1, -0.12]];
  const N = solveStaticLoads(pos, 100, 0.02, -0.01);
  near(sum(N), 100, 1e-9);
  near(Array.from(N).reduce((s, n, i) => s + n * pos[i][0], 0), 2, 1e-9);
  near(Array.from(N).reduce((s, n, i) => s + n * pos[i][1], 0), -1, 1e-9);
});

test('stribeck and selectMu', () => {
  near(stribeckMu(0, 0.6, 0.5, 0.05, 2), 0.6, 1e-12);
  near(stribeckMu(1, 0.6, 0.5, 0.05, 2), 0.5, 1e-6);
  const fr = { stribeckVelocityMS: 0.05, stribeckExponent: 2, stickVelocityMS: 0.01 };
  const w = { muStatic: 0.6, muKinetic: 0.5 };
  near(selectMu(0.005, w, fr, 1), 0.6, 1e-12);
  near(selectMu(1, w, fr, 0.5), 0.25, 1e-6);
});

test('clamps keep boundary and direction', () => {
  const [a, b] = clampCircle(3, 4, 2.5);
  near(Math.hypot(a, b), 2.5, 1e-12); near(a / b, 0.75, 1e-12);
  assert.deepEqual(clampCircle(0.3, 0.4, 2), [0.3, 0.4]);
  const [c, d] = clampEllipse(4, 4, 2, 1);
  near((c / 2) ** 2 + (d / 1) ** 2, 1, 1e-12); near(c / d, 1, 1e-12);
  assert.equal(coulombClamp(5, 2), 2);
  assert.equal(coulombClamp(-5, 2), -2);
  assert.equal(coulombClamp(1, 2), 1);
  const z = clampCircle(1, 1, 0);
  assert.equal(z[0], 0); assert.equal(z[1], 0);
});

test('slipRatio finite at 0/0', () => {
  assert.ok(Number.isFinite(slipRatio(0, 0, 0.05)));
  near(slipRatio(0, 0, 0.05), 0, 1e-12);
  near(slipRatio(2, 1, 0.05), 0.5, 1e-12);
});
