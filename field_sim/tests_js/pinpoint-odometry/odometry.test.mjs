// Tests for the OdometrySim facade (P5): spec section 7 behaviours.
// Truth is integrated with a midpoint (exact-arc) integrator so that truth is a true constant-twist path;
// stepPhysics is called with the pre-integration pose and the world velocity actually used.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { OdometrySim } from '../../web/js/physics/pinpoint-odometry/odometry.js';
import { poseError, wrapAngle } from '../../web/js/physics/pinpoint-odometry/kinematics.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} got ${a} want ${b}`);

// Drives body twist (vf, vl, om) for T seconds. Returns {truth, sim}. hook(step) may return events.
function run(sim, { vf = 0, vl = 0, om = 0, T = 1, dt = 1 / 60, start = { x: 0, y: 0, h: 0 }, hook } = {}) {
  const t = { ...start };
  const n = Math.round(T / dt);
  for (let i = 0; i < n; i++) {
    const hm = t.h + om * dt / 2;
    const c = Math.cos(hm), s = Math.sin(hm);
    const vx = vf * c - vl * s, vy = vf * s + vl * c;
    const ev = hook ? hook(i) : {};
    sim.stepPhysics(dt, { x: t.x, y: t.y, h: t.h, vx, vy, om }, ev || {});
    t.x += vx * dt; t.y += vy * dt; t.h = wrapAngle(t.h + om * dt);
  }
  return t;
}

test('ideal preset: 10 s of 60 in/s + 2 rad/s tracks truth', () => {
  const sim = new OdometrySim({ preset: 'ideal' });
  sim.reset({ x: 0, y: 0, h: 0 });
  const truth = run(sim, { vf: 60, om: 2, T: 10 });
  const e = sim.getTruthError(truth);
  assert.ok(e.dist < 0.1, `dist ${e.dist}`);
  assert.ok(Math.abs(e.dh) < 1e-6, `dh ${e.dh}`);
});

test('nominal preset: error > 0 but bounded', () => {
  const sim = new OdometrySim({ preset: 'nominal' });
  sim.reset({ x: 0, y: 0, h: 0 });
  const truth = run(sim, { vf: 60, om: 2, T: 10 });
  const e = sim.getTruthError(truth);
  assert.ok(e.dist > 0, `dist ${e.dist}`);
  assert.ok(e.dist < 2, `dist ${e.dist}`);
});

test('asConfigured: spin by pi gives ~2*hypot(0.63, 1.2) = 2.71 in error', () => {
  const sim = new OdometrySim({ preset: 'asConfigured' });
  sim.reset({ x: 0, y: 0, h: 0 });
  const N = 1000, dt = (Math.PI / 2) / N; // om = 2 rad/s for pi/2 s -> pi rad
  const truth = run(sim, { om: 2, T: Math.PI / 2, dt });
  near(Math.abs(truth.h), Math.PI, 1e-6);
  const e = sim.getTruthError({ x: 0, y: 0, h: truth.h });
  const want = 2 * Math.hypot(0.63, 1.2);
  assert.ok(Math.abs(e.dist - want) < 0.1 * want, `dist ${e.dist} want ${want}`);
});

test('nominal: velocity at rest is exactly 0 with IMU noise off', () => {
  const sim = new OdometrySim({ preset: 'nominal', overrides: { imu: { noiseRadPerSqrtS: 0, biasWalkRadSPerSqrtS: 0 } } });
  sim.reset({ x: 0, y: 0, h: 0 });
  run(sim, { T: 1 });
  const v = sim.getVelocity();
  assert.equal(v.x, 0); assert.equal(v.y, 0); assert.equal(v.h, 0);
});

test('nominal: velocity tracks ~40 in/s while cruising', () => {
  const sim = new OdometrySim({ preset: 'nominal' });
  sim.reset({ x: 0, y: 0, h: 0 });
  run(sim, { vf: 40, T: 1 });
  near(sim.getVelocity().x, 40, 2);
});

test('data age 0.05 s at 40 in/s: belief lags ~2 in', () => {
  const sim = new OdometrySim({ preset: 'nominal', overrides: { host: { dataAgeS: 0.05, dataAgeJitterS: 0 } } });
  sim.reset({ x: 0, y: 0, h: 0 });
  const truth = run(sim, { vf: 40, T: 2, dt: 1 / 120 });
  const lag = truth.x - sim.getPose().x;
  assert.ok(lag > 1.7 && lag < 2.3, `lag ${lag}`);
});

test('collision severity 1 during motion adds under-read', () => {
  const mk = () => { const s = new OdometrySim({ preset: 'nominal' }); s.reset({ x: 0, y: 0, h: 0 }); return s; };
  const a = mk(), b = mk();
  const ta = run(a, { vf: 40, T: 1 });
  const tb = run(b, { vf: 40, T: 1, hook: (i) => (i === 30 ? { collision: 1 } : {}) });
  assert.equal(ta.x, tb.x); // truth identical
  const ea = a.getTruthError(ta).dist, eb = b.getTruthError(tb).dist;
  assert.ok(eb > ea + 0.05, `with=${eb} without=${ea}`);
  assert.ok(b.getPose().x < a.getPose().x);
});

test('getPose equals the reset pose immediately after reset', () => {
  const sim = new OdometrySim({ preset: 'nominal' });
  sim.reset({ x: 12, y: -7, h: 1.0 });
  const p = sim.getPose();
  near(p.x, 12, 1e-9); near(p.y, -7, 1e-9); near(p.h, 1.0, 1e-9);
  // and stays there with a stationary truth
  run(sim, { T: 0.2, start: { x: 12, y: -7, h: 1.0 } });
  const e = sim.getTruthError({ x: 12, y: -7, h: 1.0 });
  assert.ok(e.dist < 0.01);
});

test('same seed gives identical results; setPose relocalizes belief only', () => {
  const go = () => { const s = new OdometrySim({ preset: 'harsh' }); s.reset({ x: 0, y: 0, h: 0 }); run(s, { vf: 30, om: 1, T: 1 }); return s.getPose(); };
  assert.deepEqual(go(), go());
  const s = new OdometrySim({ preset: 'nominal' });
  s.reset({ x: 0, y: 0, h: 0 });
  s.setPose({ x: 5, y: 6, h: 0.5 });
  near(s.getPose().x, 5, 1e-6); near(s.getPose().y, 6, 1e-6);
});

test('diagnostics has expected fields', () => {
  const sim = new OdometrySim();
  sim.reset({ x: 0, y: 0, h: 0 });
  run(sim, { vf: 10, T: 0.1 });
  const d = sim.diagnostics();
  for (const k of ['simTimeS', 'fwdCount', 'strCount', 'fwdSlip', 'strSlip', 'imuHeadingRad', 'status']) assert.ok(k in d, k);
  assert.ok(d.simTimeS > 0);
});
