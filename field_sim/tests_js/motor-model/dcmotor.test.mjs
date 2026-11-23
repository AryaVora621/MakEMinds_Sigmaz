import test from 'node:test';
import assert from 'node:assert/strict';
import { makeGearmotor } from '../../web/js/physics/motor-model/derive.js';
import { MotorUnit } from '../../web/js/physics/motor-model/dcmotor.js';

const near = (a, b, e) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (+-${e})`);
const gb = () => makeGearmotor('gobilda5203', 'gb13_7');

test('torque is linear in omega while motoring', () => {
  const mu = new MotorUnit(gb());
  const t = [0, 5, 10, 15].map((w) => mu.compute(w, 12, 1, 'BRAKE').torqueOut);
  near(t[1] - t[0], t[2] - t[1], 1e-9);
  near(t[2] - t[1], t[3] - t[2], 1e-9);
  assert.ok(t[0] > t[1]);
});

test('BRAKE current opposes omega', () => {
  const mu = new MotorUnit(makeGearmotor('gobilda5203', 'none'));
  assert.ok(mu.compute(100, 12, 0, 'BRAKE').statorA < 0);
  assert.ok(mu.compute(-100, 12, 0, 'BRAKE').statorA > 0);
  assert.ok(mu.compute(100, 12, 0, 'BRAKE').torqueOut < 0);
});

test('FLOAT current is 0 below the diode clamp and nonzero above', () => {
  const mu = new MotorUnit(makeGearmotor('gobilda5203', 'none'));
  assert.equal(mu.compute(300, 12, 0, 'FLOAT').statorA, 0);   // e = 5.6 V < 13.4 V
  assert.ok(mu.compute(800, 12, 0, 'FLOAT').statorA < 0);     // e = 14.9 V > 13.4 V
});

test('efficiency branch flips when back-driven', () => {
  const gm = gb(), mu = new MotorUnit(gm);
  const motoring = mu.compute(0, 12, 1, 'BRAKE');
  near(motoring.torqueOut / (gm.n * gm.G * gm.motor.Kt * motoring.statorA), gm.eta, 1e-9);
  const wOut = 200 / gm.G;                                   // wm = 200 rad/s, e = 3.7 V
  const back = mu.compute(wOut, 12, 0.1, 'BRAKE');           // V = 1.2 V < e -> regenerating
  assert.ok(back.statorA < 0);
  near(back.torqueOut / (gm.n * gm.G * gm.motor.Kt * back.statorA), 1 / gm.eta, 1e-9);
});

test('a hot winding lowers stall current', () => {
  const mu = new MotorUnit(gb());
  const cold = mu.compute(0, 12, 1, 'BRAKE').statorA;
  for (let i = 0; i < 20000; i++) mu.thermal.step(0.01, 20);
  assert.ok(mu.windingC > 100, `windingC ${mu.windingC}`);
  assert.ok(mu.compute(0, 12, 1, 'BRAKE').statorA < cold * 0.9);
});

test('compute is pure', () => {
  const mu = new MotorUnit(gb());
  const T0 = mu.windingC;
  mu.compute(10, 12, 1, 'BRAKE');
  assert.equal(mu.windingC, T0);
});
