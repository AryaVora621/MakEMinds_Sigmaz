// Tests for mixed pollen + nectar counting passed to a hive (web/js/balls.js hiveBallCounts / tipReady).
// Run from field_sim: node --test tests_js/ballkinds_mixed.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  BallWorld,
  hiveBallCounts,
  recordHiveBall,
  clearHiveBallCounts,
  tipReady,
} from '../web/js/balls.js';
import { HIVE_TIP, shouldTip } from '../web/js/hivetip_model.js';

const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 };

test('tipReady: 8 pollen tips, 7 pollen does not (user-authoritative threshold)', () => {
  assert.equal(tipReady({ pollen: 8, nectar: 0 }), true);
  assert.equal(tipReady({ pollen: 7, nectar: 0 }), false);
  assert.equal(tipReady({ pollen: 8, nectar: 0 }), shouldTip({ pollen: 8, nectar: 0 }));
  assert.equal(tipReady(undefined), false);
});

test('tipReady: mixed 4 pollen + 4 nectar is judged by the hivetip_model weight sum', () => {
  const expected = shouldTip({ pollen: 4, nectar: 4 });
  assert.equal(tipReady({ pollen: 4, nectar: 4 }), expected);
  // 4 nectar alone is 4 * 1.6545 = 6.6 pollen-equivalent, below 8, plus 4 pollen = 10.6 >= 8
  assert.equal(expected, 4 * HIVE_TIP.pollenWeight + 4 * HIVE_TIP.nectarWeight >= HIVE_TIP.threshold);
  assert.equal(expected, true);
});

test('hiveBallCounts: unknown hive reads zeros and returns a fresh object', () => {
  clearHiveBallCounts();
  const a = hiveBallCounts('red');
  assert.deepEqual(a, { pollen: 0, nectar: 0 });
  a.pollen = 99;
  assert.deepEqual(hiveBallCounts('red'), { pollen: 0, nectar: 0 });
});

test('mixed counts propagate: 4 pollen + 4 nectar reported to blue read back as {pollen:4,nectar:4}', () => {
  clearHiveBallCounts();
  for (let i = 0; i < 4; i++) recordHiveBall('blue', 'pollen');
  for (let i = 0; i < 4; i++) recordHiveBall('blue', 'nectar');
  assert.deepEqual(hiveBallCounts('blue'), { pollen: 4, nectar: 4 });
  assert.deepEqual(hiveBallCounts('red'), { pollen: 0, nectar: 0 }, 'other hive unaffected');
  assert.equal(tipReady(hiveBallCounts('blue')), shouldTip({ pollen: 4, nectar: 4 }));
});

test('recordHiveBall: 8 pollen reported -> tipReady true, 7 -> false', () => {
  clearHiveBallCounts();
  for (let i = 0; i < 7; i++) recordHiveBall('red', 'pollen');
  assert.equal(tipReady(hiveBallCounts('red')), false);
  recordHiveBall('red', 'pollen');
  assert.deepEqual(hiveBallCounts('red'), { pollen: 8, nectar: 0 });
  assert.equal(tipReady(hiveBallCounts('red')), true);
});

test('recordHiveBall: an unrecognised kind counts as pollen, never as nectar', () => {
  clearHiveBallCounts();
  recordHiveBall('red', 'something-else');
  assert.deepEqual(hiveBallCounts('red'), { pollen: 1, nectar: 0 });
});

test('BallWorld.reset clears the reported hive counts', () => {
  recordHiveBall('blue', 'nectar');
  assert.equal(hiveBallCounts('blue').nectar, 1);
  const w = new BallWorld({ bounds });
  w.reset([]);
  assert.deepEqual(hiveBallCounts('blue'), { pollen: 0, nectar: 0 });
});
