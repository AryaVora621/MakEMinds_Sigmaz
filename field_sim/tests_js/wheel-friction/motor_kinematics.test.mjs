import test from 'node:test';
import assert from 'node:assert/strict';
import { buildDrivetrainConfig } from '../../web/js/physics/wheel-friction/geometry.js';
import { stepWheelMotor, applyCoulombTorque, motorTorqueNm, electricalLine } from '../../web/js/physics/wheel-friction/motor.js';
import { mixWheelPowers, idealWheelSpeeds, twistFromWheelSpeeds } from '../../web/js/physics/wheel-friction/kinematics.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b} (tol ${tol})`);
const cfg = buildDrivetrainConfig();
const J = 6e-4; // kg*m^2, spec wheel inertia

// Original spec restored (round 2): a free-spinning wheel converges to the DATASHEET free speed within 1%.
// The datasheet free speed and stall torque are measured on the unloaded gearmotor and already include
// its own Coulomb (Tc) and viscous (B) friction. Round 1 subtracted them again, giving
// (Ts - Tc)/(Ts/wF + B) = (2.264 - 0.08)/(0.0693 + 0.002) = 30.63 rad/s (6.3% low), and weakened this test
// to that value. motor.js now derives the internal electrical line (Ts_e = Ts + Tc, w0_e = Ts_e/(Ts/wF - B))
// so the net output equals the datasheet line; Tc = 0.08 N*m and B = 0.002 N*m*s/rad are kept unchanged.
test('free spin at power 1 converges in 0.2 s to the datasheet free speed within 1%', () => {
  const m = cfg.motor;
  let w = 0;
  for (let i = 0; i < 100; i++) w = stepWheelMotor(w, 1, 1 / 500, m, J).omega;
  near(w, m.freeSpeedRadS, 0.01 * m.freeSpeedRadS);
  // and the same with the friction terms switched off (the result must not depend on the loss split)
  const m0 = { ...m, coulombTorqueNm: 0, viscousNmPerRadS: 0 };
  let w0 = 0;
  for (let i = 0; i < 100; i++) w0 = stepWheelMotor(w0, 1, 1 / 500, m0, J).omega;
  near(w0, m.freeSpeedRadS, 0.01 * m.freeSpeedRadS);
});

test('net shaft torque at full power reproduces the datasheet line Ts*(1 - w/wF)', () => {
  const m = cfg.motor;
  for (const frac of [0.1, 0.25, 0.5, 0.75, 0.95]) {
    const w = frac * m.freeSpeedRadS;
    const net = motorTorqueNm(1, w, m) - m.coulombTorqueNm; // motorTorqueNm excludes Coulomb
    near(net, m.stallTorqueNm * (1 - frac), 1e-9);
  }
  const e = electricalLine(m);
  near(e.stallNm, m.stallTorqueNm + m.coulombTorqueNm, 1e-12);
  assert.throws(() => electricalLine({ ...m, viscousNmPerRadS: 1 }));
});

test('brake decays faster than float', () => {
  const brake = { ...cfg.motor, zeroPowerBehavior: 'brake' };
  const flt = { ...cfg.motor, zeroPowerBehavior: 'float' };
  let a = 20, b = 20;
  for (let i = 0; i < 50; i++) {
    a = stepWheelMotor(a, 0, 1 / 500, brake, J).omega;
    b = stepWheelMotor(b, 0, 1 / 500, flt, J).omega;
  }
  assert.ok(a < b);
  assert.ok(a >= 0);
});

test('Coulomb clamp never reverses sign', () => {
  assert.equal(applyCoulombTorque(0.001, 0.08, 1 / 500, J), 0);
  assert.equal(applyCoulombTorque(-0.001, 0.08, 1 / 500, J), 0);
  const r = applyCoulombTorque(5, 0.08, 1 / 500, J);
  assert.ok(r > 0 && r < 5);
  assert.ok(applyCoulombTorque(-5, 0.08, 1 / 500, J) < 0);
});

test('mixWheelPowers vectors', () => {
  const eq = (v, e) => e.forEach((x, i) => near(v[i], x, 1e-9));
  eq(mixWheelPowers(1, 0, 0, cfg), [1, 1, 1, 1]);
  eq(mixWheelPowers(0, 1, 0, cfg), [-1, 1, 1, -1]);
  eq(mixWheelPowers(0, 0, 1, cfg), [-1, 1, -1, 1]);
  eq(mixWheelPowers(1, 1, 0, cfg), [0, 1, 1, 0]);
});

test('twist round trip', () => {
  const t = { vx: 0.4, vy: -0.2, om: 0.7 };
  const r = twistFromWheelSpeeds(idealWheelSpeeds(t, cfg), cfg);
  near(r.vx, t.vx, 1e-9); near(r.vy, t.vy, 1e-9); near(r.om, t.om, 1e-6);
});
