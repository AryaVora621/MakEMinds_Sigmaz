// t18: tip threshold boundaries via HiveDynamics.addScoredBall. Run: node --test tests_js/h_hive-physics_t18.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { MAX_HELD } from '../web/js/balls.js';

const scene = JSON.parse(fs.readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const make = () => new HiveDynamics(scene.groups.hive_red, scene.constants.hives.red.states, HIVE_PARAMS);
const settle = (h) => { for (let t = 0; t < 2; t += 1 / 60) h.step(1 / 60); };
const launch = (h, n, type, cell = h.upCell) => { for (let i = 0; i < n; i++) h.addScoredBall(cell, null, type); };
test('7 pollen does not tip; 8 pollen tips', () => {
  const h = make();
  launch(h, 7, 'pollen'); assert.equal(h.moving, false);
  launch(h, 1, 'pollen'); assert.equal(h.moving, true);
  settle(h); assert.equal(h.state, 'far_up');
});
test('4 nectar (7.0) does not tip; 5 nectar (8.75) tips', () => {
  const h = make();
  launch(h, 4, 'nectar'); assert.equal(h.moving, false);
  launch(h, 1, 'nectar'); assert.equal(h.moving, true);
});
test('6 pollen + 1 nectar (7.75) does not tip; + 1 more nectar (9.5) tips', () => {
  const h = make();
  launch(h, 6, 'pollen'); launch(h, 1, 'nectar');
  assert.equal(h.moving, false);
  assert.ok(Math.abs(h.cycleStats().upCellWeight - 7.75) < 1e-9);
  launch(h, 1, 'nectar'); assert.equal(h.moving, true);
});
test('balls added to the DOWN cell never count', () => {
  const h = make();
  launch(h, 10, 'pollen', 'far'); launch(h, 10, 'nectar', 'far');
  settle(h);
  assert.equal(h.moving, false);
  assert.equal(h.state, 'audience_up');
  assert.equal(h.cycleStats().upCellWeight, 0);
});
test('3 starting nectar are uncounted (7 pollen on top still does not tip)', () => {
  const h = make();
  assert.equal(h.cycleStats().upCellWeight, 0);
  launch(h, 7, 'pollen');
  assert.equal(h.moving, false);
  assert.equal(h.cycleStats().scoredInCell.audience, 3 + 7);
});
test('after a tip the new up cell needs a fresh 8', () => {
  const h = make();
  launch(h, 8, 'pollen'); settle(h);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellWeight, 0);
  launch(h, 7, 'pollen'); assert.equal(h.moving, false);
  launch(h, 1, 'pollen'); assert.equal(h.moving, true);
});
test('hold limit MAX_HELD in balls.js equals 4', () => assert.equal(MAX_HELD, 4));
// Last run: tests 7, pass 7, fail 0, todo 0. No FAILING-BY-DESIGN cases needed.
