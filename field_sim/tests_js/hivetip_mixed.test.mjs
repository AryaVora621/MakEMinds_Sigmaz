// Tests for direction/balance and shooting BOTH ball types into a HiveDynamics hive.
// Run from field_sim: node --test tests_js/hivetip_mixed.test.mjs
//
// Tip rule under test (see web/js/hivetip_model.js ledger):
//  - 8 POLLEN tip a hive (SOURCED: project owner's observation; NOT in the manual).
//  - nectar weight = 0.091 / 0.055 (SOURCED mass ratio) used as the tip weight (ASSUMED).
//  - mixed loads are a linear weight sum (ASSUMED).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { HIVE_TIP, nectarToTip } from '../web/js/hivetip_model.js';

const scene = JSON.parse(fs.readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));

const EPS = 1e-9;
const POLLEN_TO_TIP = Math.ceil(HIVE_TIP.threshold / HIVE_TIP.pollenWeight - EPS); // 8
const NECTAR_TO_TIP = nectarToTip(); // ceil(8 / 1.6545) = 5 (derived, ASSUMED weight link)

function make(params) {
  return new HiveDynamics(
    scene.groups.hive_red,
    scene.constants.hives.red.states,
    params ? { ...HIVE_PARAMS, ...params } : HIVE_PARAMS,
  );
}

function run(h, seconds, dt = 1 / 60) {
  for (let t = 0; t < seconds; t += dt) h.step(dt);
}

/** Shoot one ball into the cell and let the hive settle. */
function shoot(h, cell, type = 'pollen') {
  h.addScoredBall(cell, null, type);
  run(h, 2);
}

/** Minimal pollen count needed on top of `nectar` nectar to reach the threshold. */
function pollenNeededWith(nectar) {
  const remaining = HIVE_TIP.threshold - nectar * HIVE_TIP.nectarWeight;
  return Math.ceil(remaining / HIVE_TIP.pollenWeight - EPS);
}

test('(1) 8 pollen into a fresh up cell tip exactly once, on the 8th, flipping audience_up -> far_up', () => {
  const h = make();
  const calls = [];
  h.onTip = (s, n) => calls.push([s, n]);

  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) shoot(h, 'audience', 'pollen');
  assert.equal(h.state, 'audience_up', 'seven pollen must not tip');
  assert.equal(h.cycleStats().tips, 0);
  assert.equal(calls.length, 0);

  shoot(h, 'audience', 'pollen');
  assert.equal(h.state, 'far_up', 'the 8th pollen tips the hive');
  assert.equal(h.angleDeg, 30);
  assert.equal(h.moving, false);
  assert.equal(h.cycleStats().tips, 1);
  assert.equal(calls.length, 1, 'onTip fires exactly once');
  assert.equal(calls[0][0], 'far_up');

  // Further settled time does not produce a second tip.
  run(h, 5);
  assert.equal(h.cycleStats().tips, 1);
  assert.equal(calls.length, 1);
});

test('(1b) the tip flips back the other way on the next full threshold (far_up -> audience_up)', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) shoot(h, 'audience', 'pollen');
  assert.equal(h.state, 'far_up');
  for (let i = 0; i < POLLEN_TO_TIP; i++) shoot(h, 'far', 'pollen');
  assert.equal(h.state, 'audience_up');
  assert.equal(h.angleDeg, -30);
  assert.equal(h.cycleStats().tips, 2);
});

test('(2) nectarToTip() nectar into a fresh up cell tips; one fewer does not', () => {
  assert.equal(NECTAR_TO_TIP, Math.ceil(HIVE_TIP.threshold / HIVE_TIP.nectarWeight - EPS));
  assert.ok(NECTAR_TO_TIP >= 1);

  const under = make();
  for (let i = 0; i < NECTAR_TO_TIP - 1; i++) shoot(under, 'audience', 'nectar');
  assert.equal(under.state, 'audience_up', 'one fewer nectar than nectarToTip() must not tip');
  assert.equal(under.cycleStats().tips, 0);

  const at = make();
  for (let i = 0; i < NECTAR_TO_TIP; i++) shoot(at, 'audience', 'nectar');
  assert.equal(at.state, 'far_up', 'nectarToTip() nectar tips the hive');
  assert.equal(at.cycleStats().tips, 1);
});

test('(3) mixed: 3 nectar + the minimal pollen tips; one pollen fewer does not', () => {
  const need = pollenNeededWith(3);
  assert.equal(need, 3, 'derived from HIVE_TIP: 8 - 3 * 1.75 = 2.75 -> 3 pollen (matches team 14343 table 3n3p)');

  const under = make();
  for (let i = 0; i < 3; i++) shoot(under, 'audience', 'nectar');
  for (let i = 0; i < need - 1; i++) shoot(under, 'audience', 'pollen');
  assert.equal(under.state, 'audience_up', '3 nectar + (need-1) pollen must not tip');
  assert.equal(under.cycleStats().tips, 0);

  const at = make();
  for (let i = 0; i < 3; i++) shoot(at, 'audience', 'nectar');
  for (let i = 0; i < need; i++) shoot(at, 'audience', 'pollen');
  assert.equal(at.state, 'far_up', '3 nectar + need pollen tips');
  assert.equal(at.cycleStats().tips, 1);
});

test('(4) a ball added to the DOWN cell never counts toward the tip', () => {
  const h = make();
  for (let i = 0; i < 2 * POLLEN_TO_TIP; i++) h.addScoredBall('far', null, 'pollen');
  run(h, 5);
  assert.equal(h.state, 'audience_up', 'down-cell balls must not tip the hive');
  const s = h.cycleStats();
  assert.equal(s.upCellCount, 0);
  assert.equal(s.upPollen, 0);
  assert.equal(s.upNectar, 0);
  assert.equal(s.tipProgress, 0);
  assert.equal(s.scoredInCell.far, 2 * POLLEN_TO_TIP);
  assert.equal(s.tips, 0);
});

test('(4b) nectar into the down cell also never counts', () => {
  const h = make();
  for (let i = 0; i < 2 * NECTAR_TO_TIP; i++) h.addScoredBall('far', null, 'nectar');
  run(h, 5);
  assert.equal(h.state, 'audience_up');
  assert.equal(h.cycleStats().upNectar, 0);
  assert.equal(h.cycleStats().tipProgress, 0);
});

test('(5) a ball added while hive.moving is true never counts', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) shoot(h, 'audience', 'pollen');
  // Start a tip back (far_up -> audience_up) by filling the far up cell, but do not let it settle.
  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) shoot(h, 'far', 'pollen');
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellCount, POLLEN_TO_TIP - 1);

  h.addScoredBall('far', null, 'pollen'); // the 8th: this is the tipping ball, counted at launch
  assert.equal(h.moving, true);
  const before = h.cycleStats();
  assert.equal(before.upCellCount, POLLEN_TO_TIP, 'the tipping ball counted; the animation is in flight');

  // Balls added mid-motion into either cell must not count or start another tip.
  h.addScoredBall('far', null, 'pollen');
  h.addScoredBall('audience', null, 'pollen');
  h.addScoredBall('audience', null, 'nectar');
  assert.equal(h.moving, true);
  assert.equal(h.cycleStats().upCellCount, POLLEN_TO_TIP, 'mid-motion balls do not add to the count');
  assert.equal(h.cycleStats().upPollen, before.upPollen);
  assert.equal(h.cycleStats().upNectar, before.upNectar);

  run(h, 2);
  assert.equal(h.moving, false);
  assert.equal(h.state, 'audience_up');
  assert.equal(h.cycleStats().tips, 2, 'two tips in total (first tip, then the tip back); mid-motion balls added none');
});

test('(6) after a tip the new up cell starts at 0 and needs a full threshold again', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) shoot(h, 'audience', 'pollen');
  assert.equal(h.state, 'far_up');
  let s = h.cycleStats();
  assert.equal(s.upCellCount, 0);
  assert.equal(s.upPollen, 0);
  assert.equal(s.upNectar, 0);
  assert.equal(s.upCellWeight, 0);
  assert.equal(s.tipProgress, 0);

  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) shoot(h, 'far', 'pollen');
  assert.equal(h.state, 'far_up', 'seven fresh pollen in the new up cell do not tip');
  assert.equal(h.cycleStats().upPollen, POLLEN_TO_TIP - 1);

  shoot(h, 'far', 'pollen');
  assert.equal(h.state, 'audience_up', 'the 8th fresh pollen tips it back');
  assert.equal(h.cycleStats().tips, 2);
});

test('(6b) the old up cell balls do not carry over into the new up cell after a tip', () => {
  const h = make();
  // 3 nectar (5.25) + 2 pollen (7.25): still below 8, so no tip yet.
  for (let i = 0; i < 3; i++) shoot(h, 'audience', 'nectar');
  for (let i = 0; i < 2; i++) shoot(h, 'audience', 'pollen');
  assert.equal(h.state, 'audience_up');
  // One more pollen (8.25) tips; the 5 counted balls must not carry over.
  shoot(h, 'audience', 'pollen');
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellCount, 0);
  assert.equal(h.cycleStats().upNectar, 0);
  assert.equal(h.cycleStats().upPollen, 0);
  // Default releaseOnTip: the 6 launched balls and the 3 starting nectar dropped out of the now-down cell.
  assert.equal(h.cycleStats().scoredInCell.audience, 0);
});

test('(7) stats expose upPollen, upNectar and tipProgress consistent with the balls added', () => {
  const h = make();
  let s = h.cycleStats();
  assert.equal(s.upPollen, 0);
  assert.equal(s.upNectar, 0);
  assert.equal(s.tipProgress, 0);

  shoot(h, 'audience', 'pollen');
  shoot(h, 'audience', 'pollen');
  shoot(h, 'audience', 'pollen');
  shoot(h, 'audience', 'nectar');
  shoot(h, 'audience', 'nectar');
  s = h.cycleStats();
  assert.equal(s.upPollen, 3);
  assert.equal(s.upNectar, 2);
  assert.equal(s.upCellCount, 5);
  const weight = 3 * HIVE_TIP.pollenWeight + 2 * HIVE_TIP.nectarWeight;
  assert.ok(Math.abs(s.upCellWeight - weight) < EPS);
  assert.ok(Math.abs(s.tipProgress - weight / HIVE_TIP.threshold) < EPS);
  assert.equal(h.state, 'audience_up');

  // A ball into the down cell changes none of the up-cell fields.
  h.addScoredBall('far', null, 'nectar');
  h.addScoredBall('far', null, 'pollen');
  s = h.cycleStats();
  assert.equal(s.upPollen, 3);
  assert.equal(s.upNectar, 2);
  assert.equal(s.scoredInCell.far, 2);
  assert.equal(s.scoredTotal, 7);
});

test('(7b) tipProgress is monotone and clamped to 1 at or past the threshold', () => {
  const h = make();
  let prev = -1;
  for (let i = 0; i < POLLEN_TO_TIP; i++) {
    const p = h.cycleStats().tipProgress;
    assert.ok(p >= prev, 'monotone');
    prev = p;
    shoot(h, 'audience', 'pollen');
  }
  // After the tip the up cell is empty, so progress is back to 0 (not stuck at 1).
  assert.equal(h.cycleStats().tipProgress, 0);
});
