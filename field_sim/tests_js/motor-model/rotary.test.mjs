import test from 'node:test';
import assert from 'node:assert/strict';
import { makeGearmotor, radSToRpm } from '../../web/js/physics/motor-model/derive.js';
import { MotorUnit } from '../../web/js/physics/motor-model/dcmotor.js';
import { RotaryMechanism } from '../../web/js/physics/motor-model/rotary.js';

const mk = (extra = {}) => {
  const gm = makeGearmotor('gobilda5203', 'none', { numMotors: 2 });
  return new RotaryMechanism({ motor: new MotorUnit(gm), J: 1.152e-4, ...extra });
};

test('flywheel free speed is 6000 rpm at duty 1 / 12 V', () => {
  const r = mk();
  for (let i = 0; i < 3000; i++) r.step(0.001, 1, 12, 'FLOAT');
  assert.ok(Math.abs(radSToRpm(r.omega) - 6000) / 6000 < 0.01, `rpm ${radSToRpm(r.omega)}`);
});

test('flywheel mechanical time constant is about 0.2728 s', () => {
  const r = mk();
  const target = 0.632 * (6000 * 2 * Math.PI / 60);
  let t = 0;
  while (r.omega < target && t < 3) { r.step(0.001, 1, 12, 'FLOAT'); t += 0.001; }
  assert.ok(Math.abs(t - 0.2728) / 0.2728 < 0.05, `t ${t}`);
});

test('duty 0.02 from rest is held by stiction', () => {
  const r = mk();
  for (let i = 0; i < 500; i++) r.step(0.001, 0.02, 12, 'FLOAT');
  assert.equal(r.omega, 0);
});

test('hard stop clamps theta and zeroes omega', () => {
  const r = mk({ maxTheta: 1 });
  for (let i = 0; i < 1000; i++) r.step(0.001, 1, 12, 'BRAKE');
  assert.equal(r.theta, 1);
  assert.equal(r.omega, 0);
});

test('removeEnergy follows the closed form', () => {
  const r = mk();
  r.omega = 400;
  const E = 2;
  r.removeEnergy(E);
  const exp = Math.sqrt(400 * 400 - 2 * E / r.Jtot);
  assert.ok(Math.abs(r.omega - exp) < 1e-9);
  r.removeEnergy(1e9);
  assert.equal(r.omega, 0);
});
