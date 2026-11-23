import test from 'node:test';
import assert from 'node:assert/strict';
import { WHEEL_TYPES, FRICTION, MOTOR, SOLVER, CHASSIS_DEFAULTS, PROVENANCE, makeParams } from '../../web/js/physics/wheel-friction/params.js';
import { buildDrivetrainConfig } from '../../web/js/physics/wheel-friction/geometry.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b} (tol ${tol})`);

function numericLeaves(prefix, obj, out) {
  for (const [k, v] of Object.entries(obj)) {
    const key = `${prefix}.${k}`;
    if (typeof v === 'number') out.push(key);
    else if (Array.isArray(v)) {
      if (v.length && v.every((e) => typeof e === 'number')) out.push(key);
    } else if (v && typeof v === 'object') numericLeaves(key, v, out);
  }
}

test('every numeric constant has a PROVENANCE tag', () => {
  const keys = [];
  numericLeaves('WHEEL_TYPES', WHEEL_TYPES, keys);
  numericLeaves('FRICTION', FRICTION, keys);
  numericLeaves('MOTOR', MOTOR, keys);
  numericLeaves('SOLVER', SOLVER, keys);
  const { wheels, ...chassisRest } = CHASSIS_DEFAULTS;
  numericLeaves('CHASSIS_DEFAULTS', chassisRest, keys);
  assert.ok(keys.length > 30);
  const missing = keys.filter((k) => !/^(SOURCED|WEAK|ASSUMED|DERIVED)\b/.test(PROVENANCE[k] || ''));
  assert.deepEqual(missing, []);
});

test('makeParams is frozen and merges overrides without touching defaults', () => {
  const p = makeParams({ MOTOR: { freeSpeedRpm: 435 }, WHEEL_TYPES: { mecanum: { muStatic: 0.7 } } });
  assert.ok(Object.isFrozen(p));
  assert.ok(Object.isFrozen(p.MOTOR));
  assert.equal(p.MOTOR.freeSpeedRpm, 435);
  assert.equal(p.MOTOR.stallTorqueNm, MOTOR.stallTorqueNm);
  assert.equal(p.WHEEL_TYPES.mecanum.muStatic, 0.7);
  assert.equal(p.WHEEL_TYPES.mecanum.muKinetic, WHEEL_TYPES.mecanum.muKinetic);
  assert.equal(MOTOR.freeSpeedRpm, 312);
  assert.equal(WHEEL_TYPES.mecanum.muStatic, 0.60);
});

test('default config: mass, motor, axes, rho', () => {
  const cfg = buildDrivetrainConfig();
  near(cfg.massKg, 14.243, 0.001);
  near(cfg.motor.freeSpeedRadS, 32.673, 0.001);
  assert.equal(cfg.wheels.length, 4);
  assert.deepEqual(cfg.wheels.map((w) => w.name), ['FL', 'FR', 'BL', 'BR']);
  const fl = cfg.wheels[0].axes[0], fr = cfg.wheels[1].axes[0];
  near(fl.dir[0], 0.7071, 1e-3); near(fl.dir[1], -0.7071, 1e-3); near(fl.coupling, 0.70711, 1e-4);
  near(fr.dir[0], 0.7071, 1e-3); near(fr.dir[1], 0.7071, 1e-3);
  near(cfg.driver.rhoM, 0.28931, 1e-4);
  near(cfg.weightN, cfg.massKg * 9.80665, 1e-9);
});

test('traction spec gives 2 axes; wrong wheel count throws', () => {
  const spec = {
    chassis: { mass_lb: 31.4, footprint: [[-7.5, -7.4], [7.5, -7.4], [7.5, 7.4], [-7.5, 7.4]] },
    wheels: CHASSIS_DEFAULTS.wheels.map((w) => ({ ...w, type: 'traction' })),
  };
  const cfg = buildDrivetrainConfig(spec);
  for (const w of cfg.wheels) assert.equal(w.axes.length, 2);
  assert.throws(() => buildDrivetrainConfig({ chassis: spec.chassis, wheels: spec.wheels.slice(0, 3) }));
});
