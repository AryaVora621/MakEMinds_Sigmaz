import test from 'node:test';
import assert from 'node:assert/strict';
import { powerToDuty, Battery, PowerSystem } from '../../web/js/physics/motor-model/hub.js';
import { QuadEncoder } from '../../web/js/physics/motor-model/encoder.js';
import { VelocityPIDF } from '../../web/js/physics/motor-model/pidf.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b}`);

test('powerToDuty truncates toward zero', () => {
  assert.equal(powerToDuty(0.5), 16383 / 32767);
  assert.equal(powerToDuty(-0.5), -16383 / 32767);
  assert.equal(powerToDuty(1e-5), 0);
  assert.equal(powerToDuty(1), 1);
  assert.equal(powerToDuty(5), 1);
  assert.equal(powerToDuty(-5), -1);
});

test('NaN maps to 0', () => { assert.equal(powerToDuty(NaN), 0); });

test('battery sag with Rint 0.1', () => {
  const b = new Battery({ openCircuitV: 12, internalResistanceOhm: 0.1 });
  near(b.voltage(10), 11);
  near(b.voltage(0), 12);
  assert.equal(b.voltage(1000), 0);
  near(new Battery().voltage(50), 12); // default: no sag
});

test('PowerSystem bus voltage lags one frame and flags fuse/continuous', () => {
  const ps = new PowerSystem({ battery: { openCircuitV: 12, internalResistanceOhm: 0.1 } });
  ps.register('m');
  near(ps.busVoltage(), 12);
  ps.record('m', 11, 12, 2, 0.02);
  near(ps.busVoltage(), 12);            // still the previous frame's value
  const tel = ps.endFrame(0.02);
  near(tel.totalSupplyA, 24, 1e-6);
  near(ps.busVoltage(), 12 - 2.4, 1e-6);
  assert.ok(ps.overFuse);
  assert.ok(ps.overContinuous('m'));
  near(ps.getCurrent('m'), 11, 1e-6);
});

test('current alert query', () => {
  const ps = new PowerSystem();
  ps.register('m', { alertA: 5 });
  ps.record('m', 6, 6, 1, 0.02); ps.endFrame(0.02);
  assert.ok(ps.isOverCurrent('m'));
});

test('power channel duty and zero power behaviour', () => {
  const ps = new PowerSystem();
  ps.register('m', { zeroPower: 'FLOAT' });
  ps.setPower('m', 0.5);
  assert.equal(ps.duty('m'), 16383 / 32767);
  assert.equal(ps.zeroPower('m'), 'FLOAT');
  ps.setZeroPowerBehavior('m', 'BRAKE');
  assert.equal(ps.zeroPower('m'), 'BRAKE');
});

test('QuadEncoder floors ticks and holds sampled velocity', () => {
  const e = new QuadEncoder(28);
  e.sample(0, 0);
  e.sample(0.02, 2 * Math.PI * 55.5 / 28);   // 55.5 ticks -> floor 55
  assert.equal(e.position, 55);
  near(e.velocity, 55 / 0.02, 1e-6);
});

test('VelocityPIDF: feed-forward, hold between periods, clamp, voltage compensation', () => {
  const p = new VelocityPIDF({ kP: 0, kI: 0, kF: 0.001, periodS: 0.02 });
  near(p.update(0.02, 500, 500), 0.5);
  near(p.update(0.005, 800, 500), 0.5);  // held within the period
  const q = new VelocityPIDF({ kP: 1, kF: 0, periodS: 0.02 });
  assert.equal(q.update(0.02, 100, 0), 1);
  const v = new VelocityPIDF({ kF: 0.001, kP: 0, periodS: 0.02, voltageCompensation: true, nominalV: 12 });
  near(v.update(0.02, 500, 500, 10), 0.5 * 12 / 10);
});
