// t17: tip releases balls, conservation. Tests HiveDynamics with releaseOnTip=true (web/js/hivecycle.js).
// Run from field_sim: node --test tests_js/h_hive-physics_t17.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { HIVE_TIP } from '../web/js/hivetip_model.js';
import fs from 'node:fs';

const scene = JSON.parse(fs.readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const POLLEN_TO_TIP = HIVE_TIP.threshold / HIVE_TIP.pollenWeight; // 8
const DT = 1 / 60;

function make() {
  return new HiveDynamics(scene.groups.hive_red, scene.constants.hives.red.states, { ...HIVE_PARAMS, releaseOnTip: true });
}

/** Fill the up cell with POLLEN_TO_TIP pollen, then let the tip animation finish. */
function tipOnce(h) {
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  for (let t = 0; t < 3 - 1e-12; t += DT) h.step(DT);
}

test('after the 8th pollen tip, the lowered cell is empty and the new up cell is empty', () => {
  const h = make();
  tipOnce(h);
  assert.equal(h.state, 'far_up');
  assert.equal(h.moving, false);
  assert.equal(h.cycleStats().scoredInCell.audience, 0, 'lowered (audience) cell must be emptied');
  assert.equal(h.cycleStats().scoredInCell.far, 0, 'new up (far) cell must be empty');
  assert.equal(h.scored.filter((b) => b.cell === 'audience' || b.cell === 'far').length, 0);
});

test('conservation: released pollen + stored pollen equals 8', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall(h.upCell, null, 'pollen');
  const before = h.scored.slice();
  for (let t = 0; t < 3 - 1e-12; t += DT) h.step(DT);
  const stored = h.scored.filter((b) => b.type === 'pollen');
  const released = before.filter((b) => b.type === 'pollen' && !h.scored.includes(b));
  assert.equal(released.length + stored.length, POLLEN_TO_TIP);
  assert.equal(released.length, POLLEN_TO_TIP, 'all 8 pollen were in the lowered cell and must be released');
});

test('the tip state flips exactly once', () => {
  const h = make();
  const tips = [];
  h.onTip = (s) => tips.push(s);
  tipOnce(h);
  assert.equal(h.state, 'far_up');
  assert.deepEqual(tips, ['far_up']);
  assert.equal(h.cycleStats().tips, 1);
});

/* Run result (node --test tests_js/h_hive-physics_t17.test.mjs, executed by this agent):
 * ✔ after the 8th pollen tip, the lowered cell is empty and the new up cell is empty
 * ✔ conservation: released pollen + stored pollen equals 8
 * ✔ the tip state flips exactly once
 * ℹ tests 3 | pass 3 | fail 0 | todo 0 | duration_ms 144.6
 * No FAILING-BY-DESIGN todo tests: releaseOnTip is implemented in web/js/hivecycle.js (_settleAt).
 */
