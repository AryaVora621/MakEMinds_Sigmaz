// Tests for params.js, inertia.js, motor.js (flywheel-dynamics). Oracles are the spec numbers, not the implementation.
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  FLYWHEEL_PARAMS, PARAM_SOURCES, RPM_TO_RADS, RADS_TO_RPM, IN_TO_M,
  makeFlywheelParams, validateFlywheelParams, paramsFromRobotSpec,
} from '../../web/js/physics/flywheel-dynamics/params.js';
import {
  componentInertia, wheelSideInertia, reflectedMotorInertia, totalInertia, storedEnergyJ,
} from '../../web/js/physics/flywheel-dynamics/inertia.js';
import { makeDcMotor, driveTorqueAtWheelNm, batteryBusVoltage } from '../../web/js/physics/flywheel-dynamics/motor.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg || ''} got ${a} want ${b} (rel ${rel})`);

test('unit constants', () => {
  near(RPM_TO_RADS, 2 * Math.PI / 60, 1e-12);
  near(RADS_TO_RPM, 60 / (2 * Math.PI), 1e-12);
  assert.equal(IN_TO_M, 0.0254);
});

test('FLYWHEEL_PARAMS is deeply frozen', () => {
  assert.ok(Object.isFrozen(FLYWHEEL_PARAMS));
  assert.ok(Object.isFrozen(FLYWHEEL_PARAMS.motor));
  assert.ok(Object.isFrozen(FLYWHEEL_PARAMS.inertia.components));
  assert.ok(Object.isFrozen(FLYWHEEL_PARAMS.inertia.components[0]));
  assert.ok(Object.isFrozen(FLYWHEEL_PARAMS.shot.balls.pollen));
  assert.ok(Object.isFrozen(FLYWHEEL_PARAMS.losses.windage));
});

test('spec default values', () => {
  const p = FLYWHEEL_PARAMS;
  assert.equal(p.motor.count, 2);
  assert.equal(p.motor.stallTorqueNm, 0.144);
  assert.equal(p.motor.stallCurrentA, 11.2);
  assert.equal(p.limits.maxRpm, 6000);
  assert.equal(p.shot.exitEta, 0.45);
  near(p.wheel.radiusM, 1.89 * 0.0254, 1e-9);
  assert.equal(p.controller.periodS, 0.02);
  assert.equal(p.integrator.maxSubstepS, 0.0005);
});

test('PARAM_SOURCES tags the key constants', () => {
  const tags = /SOURCED|DATASHEET-FROM-MEMORY|DERIVED|ASSUMPTION/;
  for (const k of ['motor.stallTorqueNm', 'motor.rotorInertiaKgM2', 'shot.exitEta', 'wheel.radiusM', 'battery.internalResOhm']) {
    assert.ok(typeof PARAM_SOURCES[k] === 'string' && tags.test(PARAM_SOURCES[k]), `source for ${k}`);
  }
  assert.match(PARAM_SOURCES['motor.rotorInertiaKgM2'], /ASSUMPTION/);
  assert.match(PARAM_SOURCES['motor.stallTorqueNm'], /DATASHEET-FROM-MEMORY/);
});

test('makeFlywheelParams deep-merges, replaces arrays, is idempotent and unfrozen', () => {
  const p = makeFlywheelParams({ motor: { count: 1 }, losses: { windage: { enabled: false } } });
  assert.equal(p.motor.count, 1);
  assert.equal(p.motor.stallTorqueNm, 0.144); // untouched sibling survives
  assert.equal(p.losses.windage.enabled, false);
  assert.equal(p.losses.windage.airDensityKgM3, 1.2);
  assert.ok(!Object.isFrozen(p));
  assert.equal(FLYWHEEL_PARAMS.motor.count, 2); // defaults untouched

  const q = makeFlywheelParams({ inertia: { components: [{ name: 'x', kind: 'disk', massKg: 0.1, rOuterM: 0.05 }] } });
  assert.equal(q.inertia.components.length, 1);
  assert.equal(q.inertia.components[0].name, 'x');

  assert.deepEqual(makeFlywheelParams(p), p);
  assert.deepEqual(makeFlywheelParams(), makeFlywheelParams({}));
  assert.deepEqual(makeFlywheelParams(), JSON.parse(JSON.stringify(FLYWHEEL_PARAMS)));
});

test('validateFlywheelParams accepts defaults and catches bad values', () => {
  assert.deepEqual(validateFlywheelParams(makeFlywheelParams()), []);
  const bad = (ov) => validateFlywheelParams(makeFlywheelParams(ov));
  assert.ok(bad({ motor: { count: 0 } }).length > 0);
  assert.ok(bad({ motor: { count: 1.5 } }).length > 0);
  assert.ok(bad({ motor: { stallTorqueNm: -1 } }).length > 0);
  assert.ok(bad({ drive: { gearRatio: 0 } }).length > 0);
  assert.ok(bad({ wheel: { radiusM: 0 } }).length > 0);
  assert.ok(bad({ controller: { periodS: 0 } }).length > 0);
  assert.ok(bad({ integrator: { maxSubstepS: -1 } }).length > 0);
  assert.ok(bad({ losses: { zeroPowerBehavior: 'nope' } }).length > 0);
  assert.ok(bad({ controller: { stopMode: 'nope' } }).length > 0);
  assert.ok(bad({ inertia: { components: [{ name: 'z', kind: 'disk', massKg: NaN, rOuterM: 0.05 }] } }).length > 0);
  // freeCurrentA*R >= nominalV is impossible
  assert.ok(bad({ motor: { freeCurrentA: 12 } }).length > 0);
  for (const m of bad({ motor: { count: 0 } })) assert.equal(typeof m, 'string');
});

test('paramsFromRobotSpec handles both shapes and junk', () => {
  const a = paramsFromRobotSpec({ flywheelRadius: 2, efficiency: 0.5, maxRpm: 5000 });
  near(a.wheel.radiusM, 2 * 0.0254, 1e-9);
  assert.equal(a.shot.exitEta, 0.5);
  assert.equal(a.limits.maxRpm, 5000);
  const b = paramsFromRobotSpec({ wheelR: 1.89, eff: 0.45, maxRpm: 6000 });
  near(b.wheel.radiusM, 0.048006, 1e-6);
  assert.equal(b.shot.exitEta, 0.45);
  assert.equal(b.limits.maxRpm, 6000);
  assert.deepEqual(paramsFromRobotSpec(null), {});
  const c = paramsFromRobotSpec({ flywheelRadius: NaN, efficiency: 'x' });
  assert.ok(c.wheel === undefined || c.wheel.radiusM === undefined);
  assert.deepEqual(validateFlywheelParams(makeFlywheelParams(a)), []);
});

test('inertia formulas', () => {
  near(componentInertia({ kind: 'disk', massKg: 2, rOuterM: 0.1 }), 0.5 * 2 * 0.01, 1e-12);
  near(componentInertia({ kind: 'ring', massKg: 2, rOuterM: 0.1 }), 2 * 0.01, 1e-12);
  near(componentInertia({ kind: 'annulus', massKg: 2, rOuterM: 0.1, rInnerM: 0.05 }), 0.5 * 2 * (0.01 + 0.0025), 1e-12);
  assert.equal(componentInertia({ kind: 'given', jKgM2: 1.651e-4 }), 1.651e-4);
  assert.ok(Number.isNaN(componentInertia({ kind: 'disk', massKg: -1, rOuterM: 0.1 })));
  assert.ok(Number.isNaN(componentInertia({ kind: 'bogus' })));
});

test('default total inertia is 2.5150e-4 kg*m^2 within 0.2%', () => {
  const p = makeFlywheelParams();
  const t = totalInertia(p);
  near(t.jTotalKgM2, 2.5150e-4, 2e-3, 'total');
  near(t.jWheelKgM2, 8.112e-5 + 1.651e-4 + 1.28e-6, 2e-3, 'wheel side');
  near(t.jMotorReflectedKgM2, 4.0e-6, 1e-9, 'reflected');
  near(wheelSideInertia(p.inertia), t.jWheelKgM2, 1e-12);
  near(reflectedMotorInertia(p.motor, 1), 2 * 2.0e-6, 1e-12);
  near(reflectedMotorInertia(p.motor, 3), 2 * 2.0e-6 * 9, 1e-12);
  assert.ok(Array.isArray(t.breakdown) && t.breakdown.length >= 3);
  near(t.jTotalKgM2, t.jWheelKgM2 + t.jMotorReflectedKgM2, 1e-12);
});

test('stored energy 0.5*J*w^2', () => {
  near(storedEnergyJ(2.515e-4, 4000 * RPM_TO_RADS), 0.5 * 2.515e-4 * (4000 * RPM_TO_RADS) ** 2, 1e-12);
  assert.equal(storedEnergyJ(1, 0), 0);
});

test('DC motor constants from datasheet numbers', () => {
  const m = makeDcMotor(FLYWHEEL_PARAMS.motor);
  near(m.R, 1.07143, 1e-4);
  near(m.Kt, 0.0128571, 1e-4);
  near(m.Ke, 0.0186722, 1e-4);
  near(m.freeSpeedRadS, 628.319, 1e-4);
  near(m.frictionTorqueNm, 3.2143e-3, 1e-3);
  assert.equal(m.count, 2);
});

test('stall torque at w=0, 12 V, 2 motors is 0.288 N*m; torque equals friction at free speed', () => {
  const m = makeDcMotor(FLYWHEEL_PARAMS.motor);
  const d = FLYWHEEL_PARAMS.drive;
  const s = driveTorqueAtWheelNm(m, d, 12, 0);
  near(s.torqueNm, 0.288, 1e-3);
  near(s.currentA, 22.4, 1e-3);
  const f = driveTorqueAtWheelNm(m, d, 12, m.freeSpeedRadS / d.gearRatio);
  near(f.torqueNm, 2 * 3.2143e-3, 2e-3);
  assert.ok(driveTorqueAtWheelNm(m, d, 12, m.freeSpeedRadS * 1.2).torqueNm < 0); // regeneration
  assert.equal(driveTorqueAtWheelNm(m, d, 0, 0).torqueNm, 0);
});

test('battery bus voltage', () => {
  assert.equal(batteryBusVoltage({ voltageV: 12, internalResOhm: 0 }, 20), 12);
  near(batteryBusVoltage({ voltageV: 12, internalResOhm: 0.02 }, 20), 11.6, 1e-9);
  assert.equal(batteryBusVoltage({ voltageV: 12, internalResOhm: 1 }, 1000), 0);
  assert.equal(batteryBusVoltage({ voltageV: 12, internalResOhm: 0.02 }, -5), 12); // negative current ignored
});
