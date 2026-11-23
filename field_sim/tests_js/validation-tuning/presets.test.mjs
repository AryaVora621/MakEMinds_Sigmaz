import test from 'node:test';
import assert from 'node:assert/strict';
import { PRESET_VERSION, DEFAULT_PRESET, capturePreset, applyPreset, validatePreset, serializePreset, parsePreset, listPresets,
  savePreset, loadPreset, deletePreset, fitToPreset } from '../../web/js/physics/validation-tuning/presets.js';
import { PARAM_SCHEMA, PRESET } from '../../web/js/physics/validation-tuning/params.js';
import { BALL } from '../../web/js/balls.js';
import { MecanumDrive } from '../../web/js/driver.js';
import { memStorage, throwingStorage } from './_fixtures.mjs';

test('version matches params PRESET.version', () => assert.equal(PRESET_VERSION, PRESET.version));
test('capturePreset has every schema key and validates', () => {
  const p = capturePreset('mine', { drive: new MecanumDrive() });
  for (const e of PARAM_SCHEMA) assert.ok(Number.isFinite(p[e.group][e.field]), e.key);
  assert.equal(p.version, PRESET.version);
  assert.equal(p.name, 'mine');
  assert.ok(!Number.isNaN(Date.parse(p.created)));
  assert.deepEqual(validatePreset(p), { ok: true, errors: [] });
});
test('DEFAULT_PRESET is valid and equals the sim defaults at load', () => {
  assert.equal(validatePreset(DEFAULT_PRESET).ok, true);
  assert.equal(DEFAULT_PRESET.ball.restitutionFloor, 0.45);
  assert.equal(DEFAULT_PRESET.drive.maxSpeed, 60);
});
test('applyPreset writes BALL and drive, restore() puts everything back', () => {
  const before = structuredClone(BALL);
  const drive = new MecanumDrive();
  const driveBefore = { ...drive };
  const p = capturePreset('x', { drive });
  p.ball.restitutionFloor = 0.3; p.ball.dragK = 0.001; p.drive.maxSpeed = 77; p.drive.strafeFactor = 0.7;
  const restore = applyPreset(p, { drive });
  try {
    assert.equal(BALL.restitutionFloor, 0.3); assert.equal(BALL.dragK, 0.001);
    assert.equal(drive.maxSpeed, 77); assert.equal(drive.strafeFactor, 0.7);
  } finally { restore(); }
  assert.deepEqual(BALL, before);
  assert.deepEqual({ ...drive }, driveBefore);
});
test('applyPreset without a drive only touches BALL', () => {
  const before = structuredClone(BALL);
  const p = capturePreset('x'); p.ball.restSpeed = 7;
  const restore = applyPreset(p);
  try { assert.equal(BALL.restSpeed, 7); } finally { restore(); }
  assert.deepEqual(BALL, before);
});
test('validatePreset rejects unknown key, NaN, out of range and wrong version', () => {
  const good = () => structuredClone(DEFAULT_PRESET);
  let p = good(); p.ball.bogus = 1;
  assert.equal(validatePreset(p).ok, false);
  p = good(); p.ball.dragK = NaN;
  assert.equal(validatePreset(p).ok, false);
  p = good(); p.ball.restitutionFloor = 5;
  assert.equal(validatePreset(p).ok, false);
  p = good(); p.drive.maxSpeed = -1;
  assert.equal(validatePreset(p).ok, false);
  p = good(); p.version = 99;
  const r = validatePreset(p);
  assert.equal(r.ok, false); assert.ok(r.errors.length >= 1);
  assert.equal(validatePreset(null).ok, false);
  assert.equal(validatePreset('nope').ok, false);
});
test('serialize / parse round trip and parse rejects garbage', () => {
  const p = capturePreset('rt');
  assert.deepEqual(parsePreset(serializePreset(p)), p);
  assert.throws(() => parsePreset('{not json'));
  const bad = structuredClone(p); bad.ball.restitutionFloor = 9;
  assert.throws(() => parsePreset(JSON.stringify(bad)));
});
test('storage CRUD with an in-memory store', () => {
  const st = memStorage();
  assert.deepEqual(listPresets(st), []);
  const a = capturePreset('alpha'), b = capturePreset('beta'); b.ball.restSpeed = 9;
  assert.equal(savePreset(a, st), true);
  assert.equal(savePreset(b, st), true);
  assert.deepEqual(listPresets(st).sort(), ['alpha', 'beta']);
  assert.deepEqual(loadPreset('beta', st), b);
  assert.equal(loadPreset('missing', st), null);
  assert.ok(st.getItem(PRESET.storageKey));
  assert.equal(deletePreset('alpha', st), true);
  assert.deepEqual(listPresets(st), ['beta']);
});
test('a throwing storage returns false / null / []', () => {
  const st = throwingStorage();
  assert.equal(savePreset(capturePreset('z'), st), false);
  assert.equal(loadPreset('z', st), null);
  assert.deepEqual(listPresets(st), []);
  assert.equal(deletePreset('z', st), false);
});
test('corrupt stored JSON does not throw', () => {
  const st = memStorage(); st.setItem(PRESET.storageKey, '{garbage');
  assert.deepEqual(listPresets(st), []);
  assert.equal(loadPreset('x', st), null);
});
test('fitToPreset merges fit results and stays valid', () => {
  const p = fitToPreset(DEFAULT_PRESET, { ff: { kS: 0.05, kV: 0.0167, kA: 0.004 }, decel: { decel: 120 }, restitution: { field: 'restitutionFloor', value: 0.5 }, drag: { k: 0.0008 } }, 'fitted');
  assert.equal(p.name, 'fitted');
  assert.equal(p.ball.restitutionFloor, 0.5);
  assert.equal(p.ball.dragK, 0.0008);
  assert.equal(p.drive.maxDecel, 120);
  assert.ok(Math.abs(p.drive.maxSpeed - 0.95 / 0.0167) < 1e-6);
  assert.equal(validatePreset(p).ok, true);
  assert.equal(DEFAULT_PRESET.ball.restitutionFloor, 0.45); // base untouched
});
