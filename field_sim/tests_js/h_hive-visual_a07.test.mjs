// Contract test for HiveDynamics tip-angle curve (web/js/hivecycle.js).
// Tip trigger: 8 pollen stored in the up cell (audience_up) via addScoredBall.
// Checks: angleDeg monotone toward +30, overshoot <= 5 deg, within 1 deg by 3 s, no NaN.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { HiveDynamics } from '../web/js/hivecycle.js';

const DT = 1 / 60;

function makeHive() {
  const pivot = [0, 0, 0];
  const hiveGroup = { pivot, stable_angles_deg: { audience_up: -30, far_up: 30 }, cad_pose_state: 'audience_up' };
  const corners = [[0, 1, -1], [0, 2, -1], [0, 2, 1], [0, 1, 1]];
  const cells = {
    audience: { floor_corners: corners },
    far: { floor_corners: corners.map((p) => [p[0], -p[1], p[2]]) },
  };
  const cellsByState = { audience_up: { cells }, far_up: { cells } };
  return new HiveDynamics(hiveGroup, cellsByState);
}

test('no tip: angleDeg stays at the audience_up stable angle', () => {
  const h = makeHive();
  for (let i = 0; i < 7; i++) h.addScoredBall('audience', null, 'pollen'); // 7 < 8 pollen
  assert.equal(h.moving, false);
  for (let i = 0; i < 120; i++) {
    h.step(DT);
    assert.equal(h.angleDeg, -30);
  }
});

test('tip from audience_up to far_up: monotone, bounded overshoot, settles within 3 s', () => {
  const h = makeHive();
  for (let i = 0; i < 8; i++) h.addScoredBall('audience', null, 'pollen'); // 8 pollen trigger the tip
  assert.equal(h.moving, true, 'hive should be moving after 8 pollen');

  let prev = h.angleDeg;
  let max = prev;
  let reachedAt = -1;
  for (let i = 1; i <= 180; i++) { // 180 steps at 1/60 s = 3 s
    h.step(DT);
    const a = h.angleDeg;
    assert.ok(Number.isFinite(a), `angleDeg is not finite at step ${i}`);
    assert.ok(a >= prev - 1e-9, `angleDeg decreased at step ${i}: ${prev} -> ${a}`);
    prev = a;
    max = Math.max(max, a);
    if (reachedAt < 0 && Math.abs(a - 30) <= 1) reachedAt = i;
  }
  assert.ok(max <= 35, `overshoot beyond 5 deg: max ${max}`);
  assert.ok(reachedAt > 0 && reachedAt <= 180, 'did not get within 1 deg of +30 within 3 s');
  assert.equal(h.moving, false);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().tips, 1);
});
