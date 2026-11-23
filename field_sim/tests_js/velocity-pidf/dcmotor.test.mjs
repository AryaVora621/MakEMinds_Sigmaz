import test from 'node:test';
import assert from 'node:assert/strict';
import { mergeParams } from '../../web/js/physics/velocity-pidf/params.js';
import { deriveMotorConstants, motorCurrent, frictionTorque, FlywheelPlant, Battery } from '../../web/js/physics/velocity-pidf/dcmotor.js';

const rel = (a, b, e) => assert.ok(Math.abs(a - b) <= e * Math.abs(b), `${a} vs ${b}`);
const COAST = [{ volts: 0, coast: true }, { volts: 0, coast: true }];

test('deriveMotorConstants values', () => {
  const k = deriveMotorConstants(mergeParams().motor);
  rel(k.R, 1.30435, 1e-4);
  rel(k.Kt, 0.0156522, 1e-4);
  rel(k.Ke, 0.0185797, 1e-4);
  rel(k.omegaFreeRadS, 628.319, 1e-4);
  rel(k.tauFricFreeNm, 0.0156522 * 0.25, 1e-4);
  rel(k.tauCoulombNm, 0.5 * k.tauFricFreeNm, 1e-9);
  rel(k.bViscousNmS, 0.5 * k.tauFricFreeNm / k.omegaFreeRadS, 1e-9);
});

test('stall current at 12 V is 9.2 A', () => {
  const k = deriveMotorConstants(mergeParams().motor);
  rel(motorCurrent(k, 12, 0), 9.2, 1e-4);
});

test('coast gives zero current, brake gives negative current while spinning, NaN volts do not poison', () => {
  const k = deriveMotorConstants(mergeParams().motor);
  assert.equal(motorCurrent(k, 12, 300, true), 0);
  assert.ok(motorCurrent(k, 0, 300, false) < 0);
  assert.ok(Number.isFinite(motorCurrent(k, NaN, 100)));
  assert.ok(Number.isFinite(motorCurrent(k, 12, NaN)));
});

test('friction torque has the sign of omega and is zero at rest', () => {
  const k = deriveMotorConstants(mergeParams().motor);
  assert.equal(frictionTorque(k, 0, 1), 0);
  assert.ok(frictionTorque(k, 100, 1) > 0);
  assert.ok(frictionTorque(k, -100, 1) < 0);
  rel(frictionTorque(k, k.omegaFreeRadS, 1), k.tauFricFreeNm, 1e-6);
});

test('free speed at 12 V full power, no battery sag, both motors: 6000 rpm +/-0.5%', () => {
  const plant = new FlywheelPlant(mergeParams({ battery: { internalOhm: 0 } }));
  const d = [{ volts: 12, coast: false }, { volts: 12, coast: false }];
  for (let i = 0; i < 20000; i++) plant.step(0.0005, d);
  rel(plant.rpm, 6000, 0.005);
});

test('stall current per motor is 9.2 A at 12 V; total is the sum', () => {
  const plant = new FlywheelPlant(mergeParams());
  const tot = plant.step(1e-6, [{ volts: 12, coast: false }, { volts: 12, coast: false }]);
  rel(tot, 18.4, 0.005);
  rel(plant.state.currentA[0], 9.2, 0.005);
  assert.equal(plant.state.currentA.length, 2);
});

test('coast decays speed monotonically and never goes negative', () => {
  const plant = new FlywheelPlant(mergeParams());
  plant.reset(3000);
  assert.ok(Math.abs(plant.rpm - 3000) < 1e-6);
  let prev = plant.rpm;
  for (let i = 0; i < 4000; i++) {
    plant.step(0.0005, COAST);
    assert.ok(plant.rpm <= prev + 1e-9 && plant.rpm >= 0);
    prev = plant.rpm;
  }
  assert.ok(prev < 3000);
});

test('theta advances and is unwrapped', () => {
  const plant = new FlywheelPlant(mergeParams());
  plant.reset(3000);
  const t0 = plant.state.thetaMotorRad;
  for (let i = 0; i < 2000; i++) plant.step(0.0005, [{ volts: 4, coast: false }, { volts: 4, coast: false }]);
  assert.ok(plant.state.thetaMotorRad - t0 > 2 * Math.PI * 10);
});

test('ball load removes about the requested energy and never drives omega negative', () => {
  const plant = new FlywheelPlant(mergeParams());
  plant.reset(3000);
  const J = mergeParams().flywheel.inertiaKgM2;
  const w0 = plant.state.omegaWheelRadS;
  const E = 1.0;
  plant.addBallLoad(E, 0.03);
  for (let i = 0; i < 80; i++) plant.step(0.0005, COAST);   // 40 ms > contact time
  const w1 = plant.state.omegaWheelRadS;
  const dE = 0.5 * J * (w0 * w0 - w1 * w1);
  assert.ok(dE > 0.9 * E && dE < 1.3 * E, `dE=${dE}`);   // friction adds a little
  const p2 = new FlywheelPlant(mergeParams());
  p2.reset(100);
  p2.addBallLoad(1e6, 0.03);
  for (let i = 0; i < 200; i++) p2.step(0.0005, COAST);
  assert.ok(p2.state.omegaWheelRadS >= 0);
});

test('plant never returns NaN with NaN volts', () => {
  const plant = new FlywheelPlant(mergeParams());
  const tot = plant.step(0.0005, [{ volts: NaN, coast: false }, { volts: 12, coast: false }]);
  assert.ok(Number.isFinite(tot));
  assert.ok(Number.isFinite(plant.rpm));
});

test('battery terminal voltage sags with current and floors at minV', () => {
  const b = new Battery(mergeParams().battery);
  assert.equal(b.voltage, 12.8);
  assert.ok(Math.abs(b.update(10) - 12.3) < 1e-9);
  assert.ok(Math.abs(b.update(-5) - 12.8) < 1e-9);   // negative (regen) current does not raise the voltage
  assert.equal(b.update(1000), 6);
  b.reset();
  assert.equal(b.voltage, 12.8);
});
