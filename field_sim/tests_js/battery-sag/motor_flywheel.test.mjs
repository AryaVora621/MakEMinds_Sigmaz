import test from 'node:test';
import assert from 'node:assert/strict';
import { deriveMotor, motorCurrent, motorBusCurrent, motorTorque, motorBusLoad, DcMotorSim } from '../../web/js/physics/battery-sag/motor.js';
import { FlywheelSim } from '../../web/js/physics/battery-sag/flywheel.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b} (tol ${e})`);

test('deriveMotor constants for the goBILDA 5203', () => {
  const m = deriveMotor();
  near(m.rOhm, 1.3043, 1e-4);
  near(m.kE, 0.018580, 1e-5);
  near(m.kT, m.kE, 1e-12);
  near(m.omegaFree, 628.32, 0.01);
  near(m.frictionNm, m.kT * 0.25, 1e-9);
});

test('stall current at 12 V is 9.2 A; free speed draws 0.25 A', () => {
  const m = deriveMotor();
  near(motorCurrent(1, 12, 0, m), 9.2, 1e-6);
  near(motorCurrent(1, 12, m.omegaFree, m), 0.25, 1e-3);
});

test('bus current is duty*motor current, zero at duty 0, negative for regen', () => {
  const m = deriveMotor();
  assert.equal(motorBusCurrent(0, 12, 100, m), 0);
  near(motorBusCurrent(0.5, 12, 0, m), 0.5 * 0.5 * 12 / m.rOhm, 1e-9);
  assert.ok(motorBusCurrent(0.1, 12, m.omegaFree, m) < 0);
});

test('motorTorque = kT*I - sign(w)*friction', () => {
  const m = deriveMotor();
  near(motorTorque(5, 100, m), m.kT * 5 - m.frictionNm, 1e-12);
  near(motorTorque(5, -100, m), m.kT * 5 + m.frictionNm, 1e-12);
});

test('motorBusLoad g and a match the formulas and reproduce Ibus', () => {
  const m = deriveMotor();
  const d = 0.6, w = 200, n = 3, V = 11;
  const l = motorBusLoad(d, w, m, n);
  assert.equal(l.kind, 'linear');
  near(l.g, n * d * d / m.rOhm, 1e-9);
  near(l.a, n * d * m.kE * w / m.rOhm, 1e-9);
  near(l.g * V - l.a, n * motorBusCurrent(d, V, w, m), 1e-9);
});

test('DcMotorSim spins up under load and stays under free speed', () => {
  const s = new DcMotorSim();
  let r;
  for (let i = 0; i < 500; i++) r = s.step(1, 12, 0.004);
  assert.ok(r.omega > 0 && r.omega < s.m.omegaFree);
  assert.ok(r.iBus > 0 && r.iBus <= 9.2 + 1e-6);
  assert.equal(s.busLoad().kind, 'linear');
});

function spinUp(volts, targetRpm, seconds, dt = 0.002) {
  const f = new FlywheelSim();
  f.setTargetRpm(targetRpm);
  const trace = [];
  for (let t = 0; t < seconds; t += dt) { f.step(dt, volts); trace.push(f.state.rpm); }
  return { f, trace, dt };
}

test('flywheel reaches 63% of 4800 rpm between 0.15 and 0.5 s at 12 V', () => {
  const { trace, dt } = spinUp(12, 4800, 2);
  const i = trace.findIndex(r => r >= 0.63 * 4800);
  assert.ok(i >= 0);
  const t = (i + 1) * dt;
  assert.ok(t > 0.15 && t < 0.5, `t63=${t}`);
});

test('flywheel at 10 V cannot reach a 5500 rpm target (saturates below 5300)', () => {
  const { f } = spinUp(10, 5500, 5);
  assert.ok(f.state.rpm < 5300, `rpm=${f.state.rpm}`);
  assert.ok(f.state.rpm > 4000);
  assert.equal(f.state.duty, 1);
});

test('disabled flywheel draws no bus current and coasts', () => {
  const f = new FlywheelSim();
  f.setTargetRpm(4000);
  for (let i = 0; i < 500; i++) f.step(0.002, 12);
  const r = f.step(0.01, 12, { enabled: false });
  assert.equal(r.iBus, 0);
  assert.ok(f.state.rpm < 4100);
});

function recoverTime(volts) {
  const dt = 0.002;
  const f = new FlywheelSim();
  f.setTargetRpm(4000);
  for (let t = 0; t < 4; t += dt) f.step(dt, volts);
  f.kick();
  for (let t = 0; t < 3; t += dt) {
    f.step(dt, volts);
    if (Math.abs(f.state.rpm - 4000) <= 50) return t + dt;
  }
  return Infinity;
}

test('kick dips rpm by 6% and 12 V recovers faster than 10 V', () => {
  const f = new FlywheelSim();
  f.setTargetRpm(3000);
  for (let i = 0; i < 1000; i++) f.step(0.002, 12);
  const before = f.state.rpm;
  f.kick();
  near(f.state.rpm, before * 0.94, 1e-6);
  const t12 = recoverTime(12), t10 = recoverTime(10);
  assert.ok(Number.isFinite(t12), 'recovers at 12 V');
  assert.ok(t12 < t10, `t12=${t12} t10=${t10}`);
});

test('flywheel busLoad is linear; reset clears state', () => {
  const f = new FlywheelSim();
  f.setTargetRpm(4000);
  f.step(0.002, 12);
  const l = f.busLoad();
  assert.equal(l.kind, 'linear');
  assert.ok(l.g > 0);
  f.reset();
  assert.equal(f.state.omega, 0);
  assert.equal(f.state.targetRpm, 0);
});
