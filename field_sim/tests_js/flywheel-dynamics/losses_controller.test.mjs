// Tests for losses.js and controller.js (flywheel-dynamics).
import test from 'node:test';
import assert from 'node:assert/strict';
import { FLYWHEEL_PARAMS, RPM_TO_RADS } from '../../web/js/physics/flywheel-dynamics/params.js';
import { makeDcMotor } from '../../web/js/physics/flywheel-dynamics/motor.js';
import {
  coulombTorqueNm, windageTorqueNm, lossTorquesNm, brakeTorqueAtWheelNm, coastDownOmega, coastDownStopTimeS,
} from '../../web/js/physics/flywheel-dynamics/losses.js';
import { deriveFeedforward, VelocityController, VelocityMeasurement } from '../../web/js/physics/flywheel-dynamics/controller.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg || ''} got ${a} want ${b} (rel ${rel})`);
const P = FLYWHEEL_PARAMS;
const motor = makeDcMotor(P.motor);
const ctx = { losses: P.losses, motor, drive: P.drive, radiusM: P.wheel.radiusM };
const w4000 = 4000 * RPM_TO_RADS;

test('Coulomb torque default 8.4286e-3 N*m', () => {
  near(coulombTorqueNm(P.losses, motor, P.drive), 8.4286e-3, 1e-3);
  const noMotor = { ...P.losses, motorFrictionFromFreeCurrent: false };
  near(coulombTorqueNm(noMotor, motor, P.drive), 0.002, 1e-9);
});

test('loss torques are non-negative magnitudes', () => {
  for (const w of [0, 1, 50, w4000, -w4000, NaN]) {
    const t = lossTorquesNm(w, ctx);
    for (const k of ['coulombNm', 'viscousNm', 'windageNm', 'totalNm']) {
      assert.ok(t[k] >= 0 && Number.isFinite(t[k]), `${k} at ${w}`);
    }
    near(t.totalNm, t.coulombNm + t.viscousNm + t.windageNm, 1e-9);
  }
});

test('windage about 4e-4 N*m at 4000 rpm and scales as w^1.5', () => {
  const w = windageTorqueNm(w4000, P.losses.windage, P.wheel.radiusM);
  assert.ok(w > 3e-4 && w < 5e-4, `windage ${w}`);
  const w2 = windageTorqueNm(2 * w4000, P.losses.windage, P.wheel.radiusM);
  near(w2 / w, Math.pow(2, 1.5), 0.01);
  assert.equal(windageTorqueNm(w4000, { ...P.losses.windage, enabled: false }, P.wheel.radiusM), 0);
  assert.equal(windageTorqueNm(0, P.losses.windage, P.wheel.radiusM), 0);
});

test('brake torque opposes rotation and matches -count*Kt*Ke*w/R', () => {
  const t = brakeTorqueAtWheelNm(w4000, motor, P.drive);
  assert.ok(t < 0);
  near(t, -2 * motor.Kt * motor.Ke * w4000 / motor.R, 1e-6);
  assert.ok(brakeTorqueAtWheelNm(-w4000, motor, P.drive) > 0);
  assert.ok(brakeTorqueAtWheelNm(0, motor, P.drive) === 0); // -0 is acceptable
});

test('coastDownOmega matches numeric integration', () => {
  const J = 2.515e-4, Tc = 8.4286e-3, b = 2e-4, w0 = w4000;
  let w = w0;
  const h = 1e-5;
  for (let i = 0; i < 100000; i++) w += h * (-(Tc + b * w) / J); // 1 s
  near(coastDownOmega(J, Tc, b, w0, 1.0), w, 1e-3);
  assert.equal(coastDownOmega(J, Tc, b, w0, 0), w0);
});

test('coastDownOmega with b=0 is linear and clamps at 0', () => {
  const J = 2.515e-4, Tc = 8.4286e-3, w0 = w4000;
  near(coastDownOmega(J, Tc, 0, w0, 2), w0 - Tc * 2 / J, 1e-9);
  assert.equal(coastDownOmega(J, Tc, 0, w0, 1000), 0);
  assert.equal(coastDownOmega(J, Tc, 2e-4, w0, 1000), 0);
});

test('coastDownStopTimeS closed forms', () => {
  const J = 2.515e-4, Tc = 8.4286e-3, w0 = w4000;
  near(coastDownStopTimeS(J, Tc, 0, w0), J * w0 / Tc, 1e-9);
  const b = 2e-4;
  const ts = coastDownStopTimeS(J, Tc, b, w0);
  near(ts, (J / b) * Math.log(1 + b * w0 / Tc), 1e-9);
  assert.ok(coastDownOmega(J, Tc, b, w0, ts * 0.999) > 0);
  assert.equal(coastDownOmega(J, Tc, b, w0, ts * 1.001), 0);
});

test('derived feedforward kF=0.0186722 and kS=0.35119', () => {
  const tc = coulombTorqueNm(P.losses, motor, P.drive);
  const ff = deriveFeedforward(motor, P.drive, tc);
  near(ff.kFVoltsPerRadS, 0.0186722, 1e-3);
  near(ff.kSVolts, 0.35119, 2e-3);
});

const ff0 = { kSVolts: 0.35119, kFVoltsPerRadS: 0.0186722 };

test('controller floats with target 0 in coast mode and resets the integrator', () => {
  const c = new VelocityController({ ...P.controller, kIVoltsPerRad: 1 }, ff0);
  c.compute(100, 50, 0.02, 12);
  const r = c.compute(0, 300, 0.02, 12);
  assert.equal(r.float, true);
  assert.equal(r.voltageV, 0);
  const r2 = c.compute(100, 100, 0.02, 12); // integrator was reset: pure feedforward at e=0
  near(r2.voltageV, 0.35119 + 0.0186722 * 100, 1e-9);
});

test('controller output is clamped to [0, Vmax]', () => {
  const c = new VelocityController(P.controller, ff0);
  const hi = c.compute(600, 0, 0.02, 12);
  assert.equal(hi.voltageV, 12);
  assert.equal(hi.saturated, true);
  assert.equal(hi.float, false);
  const lo = c.compute(10, 500, 0.02, 12); // very negative raw output
  assert.equal(lo.voltageV, 0);
  assert.equal(lo.float, true);
  assert.equal(c.compute(600, 0, 0.02, 9).voltageV, 9);
  const rev = new VelocityController({ ...P.controller, allowReverse: true }, ff0).compute(10, 500, 0.02, 12);
  assert.ok(rev.voltageV < 0 && rev.voltageV >= -12);
  assert.equal(rev.float, false);
});

test('controller anti-windup: integrator only accumulates when not saturated', () => {
  const c = new VelocityController({ ...P.controller, kIVoltsPerRad: 1 }, ff0);
  for (let i = 0; i < 200; i++) assert.equal(c.compute(300, 0, 0.02, 12).saturated, true);
  const r = c.compute(300, 300, 0.02, 12);
  near(r.voltageV, 0.35119 + 0.0186722 * 300, 1e-9, 'no wound-up integral');
  const c2 = new VelocityController({ ...P.controller, kIVoltsPerRad: 1 }, ff0);
  const a = c2.compute(100, 90, 0.02, 12).voltageV;
  const b = c2.compute(100, 90, 0.02, 12).voltageV;
  near(b - a, 10 * 0.02, 1e-6); // unsaturated error integrates
});

test('controller honours kF/kS overrides and survives NaN', () => {
  const c = new VelocityController({ ...P.controller, kFVoltsPerRadS: 0.02, kSVolts: 0 }, ff0);
  assert.equal(c.kF, 0.02);
  assert.equal(c.kS, 0);
  assert.doesNotThrow(() => c.compute(NaN, NaN, NaN, NaN));
  assert.ok(Number.isFinite(c.compute(NaN, NaN, NaN, NaN).voltageV));
});

test('VelocityMeasurement first-order step response', () => {
  const m = new VelocityMeasurement({ ...P.controller, measFilterTauS: 0.02 });
  m.reset(0);
  m.update(0.02, 100);
  near(m.value, 100 * (1 - Math.exp(-1)), 1e-9);
  m.update(0.02, 100);
  near(m.value, 100 * (1 - Math.exp(-2)), 1e-9);
  const raw = new VelocityMeasurement({ ...P.controller, measFilterTauS: 0 });
  assert.equal(raw.update(0.001, 42), 42);
  m.reset(7);
  assert.equal(m.value, 7);
  assert.doesNotThrow(() => m.update(NaN, NaN));
});
