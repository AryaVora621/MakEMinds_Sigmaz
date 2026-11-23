import test from 'node:test';
import assert from 'node:assert/strict';
import { defaultParams, mergeParams, PARAM_SOURCES, rpmToTicksPerS, ticksPerSToRpm } from '../../web/js/physics/velocity-pidf/params.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b}`);

test('default params have the spec shape and key values', () => {
  const p = defaultParams();
  assert.deepEqual(Object.keys(p).sort(), ['battery', 'encoder', 'flywheel', 'hubPidf', 'mode', 'motor', 'ready', 'sdkPidf', 'settings', 'timing']);
  assert.equal(p.mode, 'sdk');
  assert.equal(p.motor.freeRpm, 6000);
  assert.equal(p.motor.stallCurrentA, 9.2);
  assert.equal(p.motor.cpr, 28);
  assert.equal(p.flywheel.motorCount, 2);
  assert.equal(p.settings.minRpm, 2500);
  assert.equal(p.settings.maxRpm, 5000);
  assert.equal(p.settings.atTargetTolTicks, 50);
  assert.equal(p.sdkPidf.kV, 0.0043);
  assert.equal(p.sdkPidf.kP, 0.005);
  assert.equal(p.hubPidf.periodS, 0.05);
  assert.equal(p.encoder.velWindowS, 0.05);
  assert.equal(p.timing.seed, 12345);
  assert.equal(p.timing.powerLevels, 32767);
  assert.equal(p.ready.settleLoops, 1);
});

test('defaultParams returns a fresh deep copy', () => {
  const a = defaultParams();
  a.motor.freeRpm = 1;
  a.sdkPidf.kP = 99;
  const b = defaultParams();
  assert.equal(b.motor.freeRpm, 6000);
  assert.equal(b.sdkPidf.kP, 0.005);
  assert.notEqual(a.motor, b.motor);
});

test('mergeParams merges groups shallowly and never mutates defaults', () => {
  const p = mergeParams({ battery: { internalOhm: 0 }, mode: 'hub', timing: { seed: 7 } });
  assert.equal(p.battery.internalOhm, 0);
  assert.equal(p.battery.openCircuitV, 12.8);
  assert.equal(p.timing.seed, 7);
  assert.equal(p.timing.bulkReadS, 0.003);
  assert.equal(p.mode, 'hub');
  const d = defaultParams();
  assert.equal(d.battery.internalOhm, 0.05);
  assert.equal(d.mode, 'sdk');
  assert.deepEqual(mergeParams(), defaultParams());
});

test('every parameter has a tagged source', () => {
  const p = defaultParams();
  const tag = /\[(REPO|SRC|DERIVED|ASSUMED|UNVERIFIED)\]/;
  for (const g of Object.keys(p)) {
    if (typeof p[g] !== 'object') { assert.match(PARAM_SOURCES[g], tag, g); continue; }
    for (const k of Object.keys(p[g])) assert.match(PARAM_SOURCES[`${g}.${k}`] ?? '', tag, `${g}.${k}`);
  }
  assert.ok(Object.isFrozen(PARAM_SOURCES));
});

test('unit helpers', () => {
  near(rpmToTicksPerS(6000, 28), 2800);
  near(rpmToTicksPerS(3000, 28), 1400);
  near(ticksPerSToRpm(1400, 28), 3000);
  near(ticksPerSToRpm(rpmToTicksPerS(1234.5, 28), 28), 1234.5, 1e-9);
});
