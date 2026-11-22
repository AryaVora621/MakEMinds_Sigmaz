// Regression tests for HiveDynamics tip dynamics (web/js/hivecycle.js).
// Tip threshold = HIVE_TIP.threshold (8 pollen, user fact, NOT in the manual). Not changed here.
// Run from field_sim: node --test tests_js/hive_tip_dynamics.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { HIVE_TIP } from '../web/js/hivetip_model.js';
import fs from 'node:fs';

const scene = JSON.parse(fs.readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const POLLEN_TO_TIP = HIVE_TIP.threshold / HIVE_TIP.pollenWeight; // 8
const DT = 1 / 60;

function make() {
  return new HiveDynamics(scene.groups.hive_red, scene.constants.hives.red.states, { ...HIVE_PARAMS });
}

/** Step the hive for `seconds` at 60 Hz, recording the shown angle each frame. */
function runRecord(h, seconds) {
  const angles = [];
  for (let t = 0; t < seconds - 1e-12; t += DT) {
    h.step(DT);
    angles.push(h.angleDeg);
  }
  return angles;
}

test('7 pollen in the up cell do not tip, even after settling', () => {
  const h = make();
  const tips = [];
  h.onTip = (s) => tips.push(s);
  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) {
    h.addScoredBall(h.upCell, null, 'pollen');
    runRecord(h, 2);
  }
  assert.equal(h.state, 'audience_up');
  assert.equal(h.moving, false);
  assert.equal(h.angleDeg, -30);
  assert.equal(h.cycleStats().upCellCount, POLLEN_TO_TIP - 1);
  assert.equal(h.cycleStats().tips, 0);
  assert.deepEqual(tips, []);
});

test('the 8th pollen tips exactly once', () => {
  const h = make();
  const tips = [];
  h.onTip = (s) => tips.push(s);
  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) {
    h.addScoredBall(h.upCell, null, 'pollen');
    runRecord(h, 2);
  }
  h.addScoredBall(h.upCell, null, 'pollen'); // 8th: starts the tip
  assert.equal(h.moving, true);
  runRecord(h, 3);
  assert.equal(h.state, 'far_up');
  assert.equal(h.angleDeg, 30);
  assert.equal(h.moving, false);
  assert.deepEqual(tips, ['far_up']);
  assert.equal(h.cycleStats().tips, 1);
});

test('hive angle approaches the tipped target monotonically, overshoot at most 5% of the 60 deg swing', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  const angles = runRecord(h, 3);
  const span = 60;
  const maxAllowed = 30 + 0.05 * span;
  let prev = -30;
  for (const a of angles) {
    assert.ok(Number.isFinite(a));
    assert.ok(a >= prev - 1e-9, `angle went backwards: ${a} < ${prev}`);
    assert.ok(a <= maxAllowed + 1e-9, `overshoot ${a} exceeds ${maxAllowed}`);
    prev = a;
  }
  assert.equal(angles[angles.length - 1], 30);
});

test('reverse tip (far to audience) is also monotone with no overshoot', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  runRecord(h, 3);
  assert.equal(h.state, 'far_up');
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  const angles = runRecord(h, 3);
  let prev = 30;
  for (const a of angles) {
    assert.ok(a <= prev + 1e-9, `angle went up on the way down: ${a} > ${prev}`);
    assert.ok(a >= -30 - 0.05 * 60 - 1e-9, `overshoot ${a}`);
    prev = a;
  }
  assert.equal(h.state, 'audience_up');
  assert.equal(h.angleDeg, -30);
});

test('counts reset after a tip: the 9th ball into the old up cell does not retip', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  runRecord(h, 3);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellCount, 0, 'new up cell starts with zero counted balls');
  h.addScoredBall('audience', null, 'pollen'); // 9th ball, now the down cell
  runRecord(h, 3);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().tips, 1);
});

test('counts reset after a tip: balls in the new up cell start from zero', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  runRecord(h, 3);
  h.addScoredBall('far', null, 'pollen');
  runRecord(h, 2);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellCount, 1);
  assert.equal(h.cycleStats().upCellWeight, 1);
  for (let i = 0; i < POLLEN_TO_TIP - 2; i++) h.addScoredBall('far', null, 'pollen');
  runRecord(h, 2);
  assert.equal(h.state, 'far_up', '7 counted pollen in the new up cell must not tip');
  h.addScoredBall('far', null, 'pollen');
  runRecord(h, 3);
  assert.equal(h.state, 'audience_up', 'the 8th fresh pollen tips it back');
  assert.equal(h.cycleStats().tips, 2);
});

test('a ball added during a tip animation is not counted toward the next tip', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  assert.equal(h.moving, true);
  h.addScoredBall('far', null, 'pollen'); // lands in the new up cell mid-motion
  runRecord(h, 3);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellCount, 0);
  assert.equal(h.cycleStats().tips, 1);
});
