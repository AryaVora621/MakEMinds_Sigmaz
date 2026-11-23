// Foundation tests: params.js, rng.js, angles.js (imu-heading). Units: rad, rad/s, s, in.
import test from 'node:test';
import assert from 'node:assert/strict';
import { createRng, fnv1a } from '../../web/js/physics/imu-heading/rng.js';
import { wrapPi, angDiff, unwrapNear, se2Exp, worldToBody, bodyToWorld, quantize } from '../../web/js/physics/imu-heading/angles.js';
import {
  DEG, IMU_PRESETS, PARAM_SOURCES, ODOMETRY_DEFAULTS, VISION_DEFAULTS, ESTIMATOR_DEFAULTS, SYSTEM_DEFAULTS, MC_DEFAULTS,
  resolveImuConfig, resolveOdometryConfig, resolveVisionConfig, resolveEstimatorConfig,
} from '../../web/js/physics/imu-heading/params.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (tol ${e})`);

test('wrapPi stays in (-pi, pi] and -pi maps to pi', () => {
  const r = createRng(7);
  for (let i = 0; i < 5000; i++) {
    const a = (r.next() - 0.5) * 200;
    const w = wrapPi(a);
    assert.ok(w > -Math.PI && w <= Math.PI, `wrapPi(${a}) = ${w}`);
    near(Math.cos(w), Math.cos(a), 1e-9);
    near(Math.sin(w), Math.sin(a), 1e-9);
  }
  assert.equal(wrapPi(-Math.PI), Math.PI);
  assert.equal(wrapPi(Math.PI), Math.PI);
  assert.equal(wrapPi(0), 0);
  near(wrapPi(3 * Math.PI), Math.PI);
  near(wrapPi(2 * Math.PI), 0);
  near(wrapPi(-2 * Math.PI), 0);
  assert.equal(wrapPi(0.123), 0.123); // in-range values untouched
});

test('angDiff and unwrapNear', () => {
  near(angDiff(0.1, -0.1), 0.2);
  near(angDiff(Math.PI - 0.1, -Math.PI + 0.1), -0.2); // shortest way across the seam
  near(unwrapNear(10, 0.5), 0.5 + 2 * Math.PI * 2, 1e-12);
  near(unwrapNear(-7, 0.1), 0.1 - 2 * Math.PI, 1e-12);
  near(unwrapNear(0.2, 0.1), 0.1, 1e-12);
});

test('se2Exp straight line, both frames', () => {
  let p = se2Exp({ x: 1, y: 2, h: 0 }, 10, 0, 0);
  near(p.x, 11); near(p.y, 2); near(p.h, 0);
  p = se2Exp({ x: 0, y: 0, h: Math.PI / 2 }, 10, 0, 0);
  near(p.x, 0); near(p.y, 10);
  p = se2Exp({ x: 0, y: 0, h: 0 }, 0, 3, 0); // pure left strafe at heading 0 -> +y
  near(p.x, 0); near(p.y, 3);
});

test('se2Exp quarter circle of radius r ends at (r, r) facing +y', () => {
  const r = 5; // in
  const dh = Math.PI / 2;
  const p = se2Exp({ x: 0, y: 0, h: 0 }, r * dh, 0, dh);
  near(p.x, r); near(p.y, r); near(p.h, Math.PI / 2);
  const small = se2Exp({ x: 0, y: 0, h: 0 }, 1, 0, 1e-10); // continuity across the small-angle branch
  near(small.x, 1, 1e-9); near(small.y, 0, 1e-9);
});

test('worldToBody / bodyToWorld round trip and quantize', () => {
  const b = worldToBody(3, 4, 0.7);
  const w = bodyToWorld(b.f, b.l, 0.7);
  near(w.dx, 3); near(w.dy, 4);
  near(Math.hypot(b.f, b.l), 5);
  const f = worldToBody(1, 0, Math.PI / 2); // facing +y: world +x is to the robot's right
  near(f.f, 0); near(f.l, -1);
  near(quantize(0.26, 0.1), 0.3);
  assert.equal(quantize(0.123456, 0), 0.123456);
});

test('fnv1a known vectors', () => {
  assert.equal(fnv1a(''), 0x811c9dc5);
  assert.equal(fnv1a('a'), 0xe40c292c);
});

test('same seed gives an identical sequence; different seed differs', () => {
  const a = createRng(42), b = createRng(42), c = createRng(43);
  const sa = [], sb = [], sc = [];
  for (let i = 0; i < 200; i++) { sa.push(a.next(), a.gauss()); sb.push(b.next(), b.gauss()); sc.push(c.next(), c.gauss()); }
  assert.deepEqual(sa, sb);
  assert.notDeepEqual(sa, sc);
  for (let i = 0; i < sa.length; i += 2) assert.ok(sa[i] >= 0 && sa[i] < 1);
  const s1 = createRng('abc'), s2 = createRng('abc');
  assert.equal(s1.next(), s2.next());
  assert.equal(s1.seed, s2.seed);
});

test('fork is deterministic and independent of parent and siblings', () => {
  const p1 = createRng(9), p2 = createRng(9);
  for (let i = 0; i < 500; i++) p2.gauss(); // consuming the parent must not move forks
  const g1 = p1.fork('gyro'), g2 = p2.fork('gyro');
  for (let i = 0; i < 50; i++) assert.equal(g1.gauss(), g2.gauss());
  // different labels give different streams
  const o = p1.fork('odo'), g = p1.fork('gyro');
  const seqO = [], seqG = [];
  for (let i = 0; i < 50; i++) { seqO.push(o.gauss()); seqG.push(g.gauss()); }
  assert.notDeepEqual(seqO, seqG);
  // draining one fork leaves a sibling untouched
  const gA = p1.fork('gyro'), vA = p1.fork('vision'), vB = p1.fork('vision');
  for (let i = 0; i < 1000; i++) gA.gauss();
  for (let i = 0; i < 50; i++) assert.equal(vA.gauss(), vB.gauss());
});

test('gauss mean and std within 5% over 20k draws; normal() scales', () => {
  const r = createRng(2024);
  const n = 20000;
  let s = 0, s2 = 0;
  for (let i = 0; i < n; i++) { const x = r.gauss(); s += x; s2 += x * x; }
  const mean = s / n, std = Math.sqrt(s2 / n - mean * mean);
  assert.ok(Math.abs(mean) < 0.05, `mean ${mean}`);
  assert.ok(Math.abs(std - 1) < 0.05, `std ${std}`);
  const q = createRng(5);
  let t = 0, t2 = 0;
  for (let i = 0; i < n; i++) { const x = q.normal(10, 3); t += x; t2 += x * x; }
  const m2 = t / n, sd2 = Math.sqrt(t2 / n - m2 * m2);
  assert.ok(Math.abs(m2 - 10) < 0.15 && Math.abs(sd2 - 3) < 0.15, `normal ${m2} ${sd2}`);
});

test('resolveImuConfig converts human units to internal units once', () => {
  const c = resolveImuConfig('pinpoint');
  near(c.biasInitSigmaRad, 0.004 * Math.PI / 180, 1e-15);
  near(c.noiseDensityRad, 0.005 * DEG, 1e-15);
  near(c.gmSigmaRad, (3 / 3600) * DEG, 1e-15); // 3 deg/h
  near(c.fullScaleRad, 2000 * DEG, 1e-9);
  assert.equal(c.latencyS, 0.002);
  assert.equal(c.internalHz, 1000);
  assert.ok(Object.isFrozen(c));
  const d = resolveImuConfig(); // default preset is pinpoint
  assert.equal(d.biasInitSigmaRad, c.biasInitSigmaRad);
  const bno = resolveImuConfig('revBno055');
  near(bno.headingLsbRad, 0.0625 * DEG, 1e-15);
  const o = resolveImuConfig('pinpoint', { biasInitSigmaDps: 0.1 });
  near(o.biasInitSigmaRad, 0.1 * DEG, 1e-15);
  assert.equal(resolveImuConfig('pinpoint').biasInitSigmaRad, c.biasInitSigmaRad); // overrides do not leak into the preset
});

test('ideal preset has every error term zero; all presets resolve', () => {
  const c = resolveImuConfig('ideal');
  for (const k of ['biasInitSigmaRad', 'gmSigmaRad', 'biasRwRad', 'noiseDensityRad', 'scaleErrSigma', 'scaleAsymSigma',
    'tempCoeffRad', 'gSensRad', 'shockSigmaRad', 'latencyS', 'rateLsbRad', 'headingLsbRad']) {
    assert.equal(c[k], 0, k);
  }
  assert.equal(c.fullScaleRad, Infinity);
  for (const name of Object.keys(IMU_PRESETS)) {
    const r = resolveImuConfig(name);
    assert.ok(Number.isFinite(r.internalHz) && r.internalHz > 0, name);
    assert.ok(r.noiseDensityRad >= 0, name);
  }
  for (const n of ['ideal', 'pinpoint', 'revBhi260', 'revBno055', 'navx2', 'otos']) assert.ok(n in IMU_PRESETS, n);
});

test('unknown preset or override key throws', () => {
  assert.throws(() => resolveImuConfig('nope'), Error);
  assert.throws(() => resolveImuConfig('pinpoint', { notAField: 1 }), Error);
  assert.throws(() => resolveOdometryConfig({ notAField: 1 }), Error);
  assert.throws(() => resolveVisionConfig({ notAField: 1 }), Error);
  assert.throws(() => resolveEstimatorConfig({ notAField: 1 }), Error);
});

test('other configs and defaults', () => {
  const v = resolveVisionConfig({ enabled: true });
  assert.equal(v.enabled, true);
  near(v.headingSigmaRad, VISION_DEFAULTS.headingSigmaDeg * DEG, 1e-15);
  const od = resolveOdometryConfig({ podScaleSigma: 0 });
  assert.equal(od.podScaleSigma, 0);
  assert.equal(od.ticksPerIn, ODOMETRY_DEFAULTS.ticksPerIn);
  const e = resolveEstimatorConfig();
  assert.equal(e.mode, ESTIMATOR_DEFAULTS.mode);
  assert.equal(SYSTEM_DEFAULTS.preset, 'pinpoint');
  assert.equal(SYSTEM_DEFAULTS.hostHz, 100);
  assert.equal(MC_DEFAULTS.dt, 0.01);
  assert.equal(typeof PARAM_SOURCES, 'object');
  assert.ok(Object.keys(PARAM_SOURCES).length > 10);
});
