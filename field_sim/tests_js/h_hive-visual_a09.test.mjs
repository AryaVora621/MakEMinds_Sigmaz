// Task a09: tip threshold through the real HiveDynamics (not only the pure model).
// Run from field_sim: node --test tests_js/h_hive-visual_a09.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { HiveDynamics } from '../web/js/hivecycle.js';

const SQUARE = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]];
const cellsByState = {
  audience_up: { cells: { audience: { floor_corners: SQUARE }, far: { floor_corners: SQUARE } } },
  far_up: { cells: { audience: { floor_corners: SQUARE }, far: { floor_corners: SQUARE } } },
};

function makeHive() {
  const group = {
    pivot: [0, 0, 0],
    stable_angles_deg: { audience_up: -30, far_up: 30 },
    cad_pose_state: 'audience_up',
  };
  return new HiveDynamics(group, cellsByState);
}

// Launch n balls into the audience cell (the up cell at reset). Balls are stored at the floor centre.
function launch(dyn, n, type) {
  for (let i = 0; i < n; i++) dyn.addScoredBall('audience', null, type);
}

// Advance the sim by `seconds` in 60 Hz steps.
function runFor(dyn, seconds) {
  const dt = 1 / 60;
  const steps = Math.round(seconds / dt);
  for (let i = 0; i < steps; i++) dyn.step(dt);
}

test('7 pollen in the up cell do not tip within 5 s', () => {
  const dyn = makeHive();
  launch(dyn, 7, 'pollen');
  assert.equal(dyn.moving, false, 'no tip starts on the 7th ball');
  runFor(dyn, 5);
  assert.equal(dyn.cycleStats().tips, 0);
  assert.equal(dyn.state, 'audience_up');
  assert.equal(dyn.moving, false);
});

test('8 pollen in the up cell tip the hive', () => {
  const dyn = makeHive();
  launch(dyn, 7, 'pollen');
  assert.equal(dyn.moving, false);
  launch(dyn, 1, 'pollen'); // 8th ball
  assert.equal(dyn.moving, true, 'tip starts on the 8th ball');
  runFor(dyn, 5);
  assert.equal(dyn.cycleStats().tips, 1);
  assert.equal(dyn.state, 'far_up');
  assert.equal(dyn.moving, false);
});

test('5 nectar alone tip the hive (1.75 each, INFERRED)', () => {
  const dyn = makeHive();
  launch(dyn, 4, 'nectar');
  assert.equal(dyn.moving, false, '4 nectar (7.0) do not start a tip');
  launch(dyn, 1, 'nectar');
  assert.equal(dyn.moving, true, '5 nectar (8.75) start a tip');
  runFor(dyn, 5);
  assert.equal(dyn.cycleStats().tips, 1);
  assert.equal(dyn.state, 'far_up');
});

test('4 nectar alone do not tip within 5 s', () => {
  const dyn = makeHive();
  launch(dyn, 4, 'nectar');
  runFor(dyn, 5);
  assert.equal(dyn.cycleStats().tips, 0);
  assert.equal(dyn.state, 'audience_up');
  assert.equal(dyn.moving, false);
});
