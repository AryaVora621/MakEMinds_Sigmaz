// Tests for params.js (P5). Units: ticksPerMm in ticks/mm; all other values per params.js tags.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { POD_TYPES, DEFAULTS, PRESETS, makeOdoParams, deepMerge } from '../../web/js/physics/pinpoint-odometry/params.js';

test('pod ticksPerMm values match 2000/(pi*D)', () => {
  assert.ok(Math.abs(POD_TYPES.goBILDA_4_BAR_POD.ticksPerMm - 19.894) < 1e-3);
  assert.ok(Math.abs(POD_TYPES.goBILDA_SWINGARM_POD.ticksPerMm - 13.263) < 1e-3);
});

test('every preset resolves with numeric ticksPerMm', () => {
  for (const name of ['ideal', 'nominal', 'asConfigured', 'harsh']) {
    assert.ok(name in PRESETS, name);
    const p = makeOdoParams(name);
    assert.equal(typeof p.config.ticksPerMm, 'number');
    assert.ok(Math.abs(p.ticksPerMmTruth - 19.894) < 1e-3);
  }
});

test('preset values', () => {
  assert.equal(makeOdoParams('ideal').firmware.rateHz, 0);
  assert.equal(makeOdoParams('ideal').slip.enabled, false);
  assert.equal(makeOdoParams('asConfigured').config.forwardPodYIn, 0);
  assert.equal(makeOdoParams('harsh').imu.scaleError, 0.005);
});

test('overrides deep-merge and do not mutate DEFAULTS', () => {
  const before = JSON.stringify(DEFAULTS);
  const p = makeOdoParams('nominal', { imu: { noiseRadPerSqrtS: 5 }, slip: { enabled: false } });
  assert.equal(p.imu.noiseRadPerSqrtS, 5);
  assert.equal(p.imu.biasWalkRadSPerSqrtS, DEFAULTS.imu.biasWalkRadSPerSqrtS); // sibling kept
  assert.equal(p.slip.enabled, false);
  assert.equal(p.slip.accelGain, DEFAULTS.slip.accelGain);
  assert.equal(JSON.stringify(DEFAULTS), before);
  assert.notEqual(DEFAULTS.imu.noiseRadPerSqrtS, 5);
});

test('podType override changes ticksPerMm', () => {
  const p = makeOdoParams('nominal', { podType: 'goBILDA_SWINGARM_POD' });
  assert.ok(Math.abs(p.config.ticksPerMm - 13.263) < 1e-3);
  assert.ok(Math.abs(p.ticksPerMmTruth - 13.263) < 1e-3);
});

test('unknown preset or podType throws', () => {
  assert.throws(() => makeOdoParams('nope'), Error);
  assert.throws(() => makeOdoParams('nominal', { podType: 'bogus' }), Error);
});

test('result is frozen (deeply)', () => {
  const p = makeOdoParams('nominal');
  assert.ok(Object.isFrozen(p));
  assert.ok(Object.isFrozen(p.slip));
  assert.ok(Object.isFrozen(p.truth.forwardPod));
});

test('deepMerge returns new object, arrays/primitives replace', () => {
  const a = { x: { y: 1, z: [1, 2] }, k: 1 };
  const m = deepMerge(a, { x: { z: [3] }, k: 2 });
  assert.deepEqual(m, { x: { y: 1, z: [3] }, k: 2 });
  assert.deepEqual(a, { x: { y: 1, z: [1, 2] }, k: 1 });
});
