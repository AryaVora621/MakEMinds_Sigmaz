// h_hive-visual_a12: edge cases for hiveTipLabel (web/js/hivemesh.js ~L255). Pure function, no DOM.
// NOTE: the doc example in hivemesh.js ("pollen 5 + nectar 1 = 6.6/8") is stale: 5 + 1.75 = 6.75 -> "6.8".
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { hiveTipLabel } from '../web/js/hivemesh.js';
import { shouldTip } from '../web/js/hivetip_model.js';

const EMPTY = 'pollen 0 + nectar 0 = 0.0/8';

test('empty object and undefined give the zero label', () => {
  assert.equal(hiveTipLabel({}), EMPTY);
  assert.equal(hiveTipLabel(), EMPTY);
});

test('NaN, Infinity, negative and non-numeric counts clamp to 0', () => {
  assert.equal(hiveTipLabel({ upPollen: NaN, upNectar: NaN }), EMPTY);
  assert.equal(hiveTipLabel({ upPollen: Infinity, upNectar: -Infinity }), EMPTY);
  assert.equal(hiveTipLabel({ upPollen: -3, upNectar: -1 }), EMPTY);
  assert.equal(hiveTipLabel({ upPollen: 'abc', upNectar: '' }), EMPTY);
});

test('fractional counts are floored before weighting', () => {
  assert.equal(hiveTipLabel({ upPollen: 5.9, upNectar: 2.99 }), 'pollen 5 + nectar 2 = 8.5/8');
});

test('mixed 2 nectar + 5 pollen = 8.5 and the tip model agrees it tips', () => {
  const label = hiveTipLabel({ upPollen: 5, upNectar: 2 });
  assert.equal(label, 'pollen 5 + nectar 2 = 8.5/8');
  assert.equal(shouldTip({ pollen: 5, nectar: 2 }), true);
});

test('single nectar uses weight 1.75 and rounds to one decimal', () => {
  assert.equal(hiveTipLabel({ upNectar: 1 }), 'pollen 0 + nectar 1 = 1.8/8');
});

test('8 pollen is exactly at the threshold', () => {
  assert.equal(hiveTipLabel({ upPollen: 8 }), 'pollen 8 + nectar 0 = 8.0/8');
});

test('doc example is stale: 5 pollen + 1 nectar renders 6.8, not 6.6', () => {
  assert.equal(hiveTipLabel({ upPollen: 5, upNectar: 1 }), 'pollen 5 + nectar 1 = 6.8/8');
});

test('tipTarget override replaces the /8 denominator', () => {
  assert.equal(hiveTipLabel({ upPollen: 5, upNectar: 2, tipTarget: 10 }), 'pollen 5 + nectar 2 = 8.5/10');
});

test('nectarWeight override changes the nectar total', () => {
  assert.equal(hiveTipLabel({ upNectar: 2, nectarWeight: 2 }), 'pollen 0 + nectar 2 = 4.0/8');
});

test('NaN tipTarget falls to 0 in the denominator (no default applied)', () => {
  // Only undefined selects the default; NaN goes through num() and becomes 0.
  assert.equal(hiveTipLabel({ upPollen: 1, tipTarget: NaN }), 'pollen 1 + nectar 0 = 1.0/0');
});

test('null stats is treated as empty', () => {
  assert.equal(hiveTipLabel(null), EMPTY);
});
