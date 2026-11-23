import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { makeParams, wheelGeometry, G } from '../../web/js/physics/weight-transfer/params.js';
import { computeCG } from '../../web/js/physics/weight-transfer/massmodel.js';
import { ChassisCompliance } from '../../web/js/physics/weight-transfer/compliance.js';

const spec = JSON.parse(fs.readFileSync(new URL('../../web/data/robot_spec.json', import.meta.url), 'utf8'));
const P = makeParams({}, spec);
const CG = computeCG(P);
const GEO = wheelGeometry(P);
const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) < e, `${a} !~ ${b} (tol ${e})`);
const run = (c, dt, T, acc) => { for (let t = 0; t < T; t += dt) c.step(dt, acc); };

test('starts at static equilibrium: zero tilt, heave = -zSink (~0.157 in)', () => {
  const c = new ChassisCompliance(P, CG);
  const s = c.state;
  assert.equal(s.pitch, 0); assert.equal(s.roll, 0);
  const zSink = (CG.massLb * G) / (4 * P.compliance.kWheelLbfPerIn * G);
  near(s.heave, -zSink, 1e-9); near(zSink, 0.157, 0.005);
  run(c, 1 / 60, 1, { ax: 0, ay: 0 });
  near(c.state.heave, -zSink, 1e-4); near(c.state.pitch, 0, 1e-9);
});

test('steady braking ax=-140 settles to nose-down pitch M*140*h/Kp (+-5%)', () => {
  const c = new ChassisCompliance(P, CG);
  run(c, 1 / 60, 3, { ax: -140, ay: 0 });
  const Kp = 4 * P.compliance.kWheelLbfPerIn * G * GEO.a * GEO.a;
  const expect = (CG.massLb * 140 * CG.z) / Kp;
  const th = c.state.pitch;
  assert.ok(th > 0, 'positive = nose down');
  assert.ok(Math.abs(th - expect) / expect < 0.05, `${th} vs ${expect}`);
  near((th * 180) / Math.PI, 0.7, 0.2);
});

test('forward accel pitches nose up (negative)', () => {
  const c = new ChassisCompliance(P, CG);
  run(c, 1 / 60, 2, { ax: 140, ay: 0 });
  assert.ok(c.state.pitch < 0);
});

test('ay > 0 gives roll > 0 (left side up)', () => {
  const c = new ChassisCompliance(P, CG);
  run(c, 1 / 60, 2, { ax: 0, ay: 150 });
  assert.ok(c.state.roll > 0);
});

test('pitch natural frequency within 4-12 Hz', () => {
  const f = new ChassisCompliance(P, CG).naturalFreqHz();
  assert.ok(f.pitch > 4 && f.pitch < 12, `pitch ${f.pitch}`);
  assert.ok(f.roll > 0 && f.heave > 0);
});

test('stable at dt = 1/30 under a square pulse train', () => {
  const c = new ChassisCompliance(P, CG);
  for (let i = 0; i < 300; i++) {
    c.step(1 / 30, { ax: i % 40 < 20 ? -200 : 200, ay: 0 });
    assert.ok(Number.isFinite(c.state.pitch) && Math.abs(c.state.pitch) < 0.2, `i=${i} ${c.state.pitch}`);
  }
  run(c, 1 / 30, 5, { ax: 0, ay: 0 });
  near(c.state.pitch, 0, 1e-3);
});

test('kick adds angular velocity and reset clears it', () => {
  const c = new ChassisCompliance(P, CG);
  c.kick(1.5, -0.5);
  near(c.state.pitchRate, 1.5, 1e-9); near(c.state.rollRate, -0.5, 1e-9);
  c.reset();
  assert.equal(c.state.pitchRate, 0); assert.equal(c.state.pitch, 0);
});

test('setCG keeps state', () => {
  const c = new ChassisCompliance(P, CG);
  run(c, 1 / 60, 0.2, { ax: -140, ay: 0 });
  const before = c.state.pitch;
  c.setCG(computeCG(P, { ballsHeld: 2 }));
  near(c.state.pitch, before, 1e-12);
});
