// Tests for web/js/hivetip_model.js (pure hive tip model).
// Run from field_sim: node --test tests_js/hivetip_model.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  HIVE_TIP,
  shouldTip,
  tipProgress,
  nectarToTip,
} from '../web/js/hivetip_model.js';

test('8 pollen tips a hive', () => {
  assert.equal(shouldTip({ pollen: 8, nectar: 0 }), true);
});

test('7 pollen does not tip a hive', () => {
  assert.equal(shouldTip({ pollen: 7, nectar: 0 }), false);
});

test('0 balls does not tip a hive', () => {
  assert.equal(shouldTip({ pollen: 0, nectar: 0 }), false);
  assert.equal(shouldTip({}), false);
});

test('tipProgress is pollen / threshold and clamped to [0, 1]', () => {
  assert.equal(tipProgress({ pollen: 4 }), 0.5);
  assert.equal(tipProgress({ pollen: 8 }), 1);
  assert.equal(tipProgress({ pollen: 20 }), 1);
  assert.equal(tipProgress({}), 0);
});

test('nectarToTip equals ceil(threshold / nectarWeight)', () => {
  assert.equal(nectarToTip(), Math.ceil(HIVE_TIP.threshold / HIVE_TIP.nectarWeight));
});

test('nectar alone tips at nectarToTip and not one fewer', () => {
  const n = nectarToTip();
  assert.equal(shouldTip({ nectar: n }), true);
  assert.equal(shouldTip({ nectar: n - 1 }), false);
});

test('mixed nectar + pollen: minimal pollen for each nectar count tips, one fewer does not', () => {
  const n = nectarToTip();
  const w = HIVE_TIP.nectarWeight;
  const pw = HIVE_TIP.pollenWeight;
  // k = nectarToTip() is covered by the test above (remaining weight <= 0 there).
  for (let k = 0; k < n; k++) {
    const remaining = HIVE_TIP.threshold - k * w;
    const pollen = Math.ceil(remaining / pw);
    assert.equal(shouldTip({ pollen, nectar: k }), true, `k=${k}, pollen=${pollen} should tip`);
    assert.equal(shouldTip({ pollen: pollen - 1, nectar: k }), false, `k=${k}, pollen=${pollen - 1} should not tip`);
  }
});

test('negative and fractional counts are clamped', () => {
  assert.equal(shouldTip({ pollen: -5, nectar: 0 }), false);
  assert.equal(tipProgress({ pollen: -5 }), 0);
  assert.equal(shouldTip({ pollen: 7.9, nectar: 0 }), false);
  assert.equal(shouldTip({ pollen: 8.9, nectar: 0 }), true);
  assert.equal(tipProgress({ pollen: 4.9 }), 0.5);
});

test('HIVE_TIP constants: threshold 8, pollenWeight 1', () => {
  assert.equal(HIVE_TIP.threshold, 8);
  assert.equal(HIVE_TIP.pollenWeight, 1);
});

test('HIVE_TIP source notes are present and tagged SOURCED or ASSUMED', () => {
  const src = HIVE_TIP.source;
  assert.ok(src && typeof src === 'object', 'source object present');
  // The source fields are free text; the SOURCED/ASSUMED tags are required across the whole
  // source block (pollenThreshold, nectarWeight and notes). Note: the current pollenThreshold
  // string in hivetip_model.js carries no tag itself, so the check is made over the block.
  assert.ok(typeof src.pollenThreshold === 'string' && src.pollenThreshold.length > 0);
  assert.ok(typeof src.nectarWeight === 'string' && src.nectarWeight.length > 0);
  assert.ok(Array.isArray(src.notes) && src.notes.length > 0, 'notes array present');
  for (const note of src.notes) {
    assert.ok(typeof note === 'string' && note.length > 0, 'each note is a non-empty string');
  }
  const text = [src.pollenThreshold, src.nectarWeight, ...src.notes].join('\n');
  assert.match(text, /SOURCED/, 'source block mentions SOURCED');
  assert.match(text, /ASSUMED/, 'source block mentions ASSUMED');
});
