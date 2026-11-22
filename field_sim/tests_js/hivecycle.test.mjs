import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { HIVE_TIP } from '../web/js/hivetip_model.js';

const scene = JSON.parse(fs.readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));

// Tip rule under test (user fact, NOT in the manual): 8 pollen tip a hive. Starting nectar never counts.
const POLLEN_TO_TIP = HIVE_TIP.threshold / HIVE_TIP.pollenWeight; // 8
// Pure nectar tip count is derived from the nectar weight (ASSUMED: weight is proportional to mass).
const NECTAR_TO_TIP = Math.ceil(HIVE_TIP.threshold / HIVE_TIP.nectarWeight - 1e-9); // 5

function make(params) {
  return new HiveDynamics(scene.groups.hive_red, scene.constants.hives.red.states, params ? { ...HIVE_PARAMS, ...params } : HIVE_PARAMS);
}
function run(h, seconds, dt = 1 / 60) { for (let t = 0; t < seconds; t += dt) h.step(dt); }
/** Launch balls into the up cell until it tips; returns balls launched, or -1. */
function launchesToTip(h, type = 'pollen', max = 30) {
  const before = h.state;
  for (let n = 1; n <= max; n++) {
    h.addScoredBall(h.upCell, null, type);
    run(h, 2);
    if (h.state !== before) return n;
  }
  return -1;
}

test('starts in audience_up at -30 with 3 starting nectar stored but NOT counted', () => {
  const h = make();
  assert.equal(h.state, 'audience_up');
  assert.equal(h.angleDeg, -30);
  assert.equal(h.moving, false);
  const s = h.cycleStats();
  assert.equal(s.upCellCount, 0);
  assert.equal(s.upCellWeight, 0);
  assert.equal(s.scoredInCell.audience, 3);
  assert.ok(h.scored.every((b) => b.type === 'nectar' && b.counts === false));
  assert.equal(s.scoredTotal, 0);
});

test('defaults: tip at 8 pollen (HIVE_TIP), weight mode, nectar heavier', () => {
  assert.equal(HIVE_TIP.threshold, 8); // user fact: 8 pollen tip a hive
  assert.equal(HIVE_PARAMS.ballsToTip, 8);
  assert.equal(HIVE_PARAMS.tipWeightThreshold, 8);
  assert.equal(HIVE_PARAMS.mode, 'weight');
  assert.equal(HIVE_PARAMS.startingNectar, 3);
  assert.equal(HIVE_PARAMS.nectarWeight, HIVE_TIP.nectarWeight);
  assert.ok(HIVE_PARAMS.nectarWeight > HIVE_PARAMS.pollenWeight);
});

test('no tip without launching, even over time', () => {
  const h = make();
  run(h, 20);
  assert.equal(h.state, 'audience_up');
  assert.equal(h.cycleStats().tips, 0);
});

test('pure pollen: the 8th launched ball tips (launchesToTip stops at the first tip), exactly to +30', () => {
  const h = make();
  // launchesToTip returns the launch count of the FIRST tip, so 8 means balls 1..7 did not tip.
  assert.equal(launchesToTip(h), POLLEN_TO_TIP);
  assert.equal(h.state, 'far_up');
  assert.equal(h.angleDeg, 30);
  assert.equal(h.moving, false);
});

test('pure pollen: 7 pollen leave the hive latched (boundary below threshold)', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) { h.addScoredBall('audience', null, 'pollen'); run(h, 2); }
  assert.equal(h.state, 'audience_up');
  assert.equal(h.moving, false);
  assert.equal(h.cycleStats().upCellCount, 7);
  assert.equal(h.cycleStats().tips, 0);
});

test('pure nectar: 4 nectar do not tip, the 5th does (ceil(8 / nectarWeight))', () => {
  assert.equal(NECTAR_TO_TIP, 5);
  const h = make();
  assert.equal(launchesToTip(h, 'nectar'), NECTAR_TO_TIP);
  assert.equal(h.state, 'far_up');
});

test('mixed: 7 pollen + 1 nectar (8.65) just tips; 7 pollen alone (7.0) does not', () => {
  const h = make();
  for (let i = 0; i < 7; i++) { h.addScoredBall('audience', null, 'pollen'); run(h, 2); }
  assert.equal(h.state, 'audience_up');
  assert.ok(Math.abs(h.cycleStats().upCellWeight - 7) < 1e-9);
  h.addScoredBall('audience', null, 'nectar');
  assert.ok(Math.abs(h.cycleStats().upCellWeight - (7 + HIVE_TIP.nectarWeight)) < 1e-9);
  run(h, 2);
  assert.equal(h.state, 'far_up');
});

test('mixed: 4 pollen + 2 nectar (7.31) does not tip; a 3rd nectar (8.96) does', () => {
  const h = make();
  for (let i = 0; i < 4; i++) { h.addScoredBall('audience', null, 'pollen'); run(h, 2); }
  h.addScoredBall('audience', null, 'nectar'); run(h, 2);
  h.addScoredBall('audience', null, 'nectar'); run(h, 2);
  assert.equal(h.state, 'audience_up');
  assert.ok(Math.abs(h.cycleStats().upCellWeight - (4 + 2 * HIVE_TIP.nectarWeight)) < 1e-9);
  h.addScoredBall('audience', null, 'nectar'); run(h, 2);
  assert.equal(h.state, 'far_up');
});

test('a ball into the down cell never counts, before or after a tip', () => {
  const h = make();
  for (let i = 0; i < 10; i++) h.addScoredBall('far');
  run(h, 5);
  assert.equal(h.state, 'audience_up');
  const s = h.cycleStats();
  assert.equal(s.upCellCount, 0);
  assert.equal(s.scoredInCell.far, 10);
  assert.equal(s.scoredTotal, 10);

  // After a real tip, audience is the down cell: 20 more balls into it must not tip back.
  assert.equal(launchesToTip(h), POLLEN_TO_TIP);
  assert.equal(h.state, 'far_up');
  for (let i = 0; i < 20; i++) h.addScoredBall('audience');
  run(h, 5);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellCount, 0);
  assert.equal(h.cycleStats().tips, 1);
});

test('after a tip the new up cell starts empty and needs 8 fresh pollen', () => {
  const h = make();
  assert.equal(launchesToTip(h), POLLEN_TO_TIP);
  assert.equal(h.upCell, 'far');
  assert.equal(h.cycleStats().upCellCount, 0);
  // The 8 pollen that tipped the hive sit in the (now down) audience cell and do not carry over.
  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) { h.addScoredBall('far'); run(h, 2); }
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().upCellCount, 7);
  h.addScoredBall('far'); run(h, 2);
  assert.equal(h.state, 'audience_up');
  assert.equal(h.angleDeg, -30);
  assert.equal(h.cycleStats().tips, 2);
});

test('balls stay in the cell that became down only when releaseOnTip is false; default releases them', () => {
  const h = make({ releaseOnTip: false });
  launchesToTip(h);
  assert.equal(h.cycleStats().scoredInCell.audience, 3 + POLLEN_TO_TIP);
  assert.equal(make().p.releaseOnTip, true, 'default: down-cell balls drop out');
  const r = make();
  launchesToTip(r);
  assert.equal(r.cycleStats().scoredInCell.audience, 0);
});

test('setRule changes take effect on the next ball', () => {
  const h = make({ mode: 'count', ballsToTip: 10 });
  h.addScoredBall('audience'); run(h, 2);
  assert.equal(h.state, 'audience_up');
  h.setRule({ ballsToTip: 2 });
  h.addScoredBall('audience'); run(h, 2);
  assert.equal(h.state, 'far_up');
  h.setRule({ mode: 'weight', nectarWeight: 3 });
  assert.equal(h.p.mode, 'weight');
  assert.equal(h.p.nectarWeight, 3);
  assert.throws(() => h.setRule({ mode: 'bogus' }));
});

test('angle stays within +-30 and finite during a tip, ends exactly on the stop', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall('audience');
  assert.equal(h.moving, true);
  let prev = -30;
  for (let i = 0; i < 120; i++) {
    h.step(1 / 60);
    assert.ok(Number.isFinite(h.angleDeg));
    assert.ok(Math.abs(h.angleDeg) <= 30 + 1e-9);
    assert.ok(h.angleDeg >= prev - 1e-9);
    prev = h.angleDeg;
  }
  assert.equal(h.angleDeg, 30);
});

test('onTip fires once per tip with state and stored count', () => {
  const h = make();
  const calls = [];
  h.onTip = (s, n) => calls.push([s, n]);
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall('audience');
  h.addScoredBall('audience'); // mid-motion: stored, does not count, no second tip
  run(h, 5);
  // default releaseOnTip: the whole old up cell (3 nectar + 9 pollen) dropped out, so 0 stored
  assert.deepEqual(calls, [['far_up', 0]]);
  assert.equal(h.cycleStats().tips, 1);
});

test('stats: tips, up count/weight, ballsToTip, totals, tipsByAuto', () => {
  const h = make();
  let s = h.cycleStats();
  assert.equal(s.ballsToTip, 8);
  assert.equal(s.upCellCount, 0);
  assert.equal(s.upCellWeight, 0);
  h.addScoredBall('audience', null, 'pollen');
  s = h.cycleStats();
  assert.equal(s.upCellCount, 1);
  assert.ok(Math.abs(s.upCellWeight - 1) < 1e-9);
  assert.equal(s.scoredTotal, 1);
  assert.equal(s.tipsByAuto, null);
  h.addScoredBall('audience', null, 'nectar');
  assert.ok(Math.abs(h.cycleStats().upCellWeight - (1 + HIVE_TIP.nectarWeight)) < 1e-9);
  // 1 pollen + 1 nectar + 6 pollen = 8.65 >= 8: the tip happens during the last of these
  for (let i = 0; i < 6; i++) h.addScoredBall('audience', null, 'pollen');
  run(h, 2);
  h.markAutoEnd();
  assert.equal(h.cycleStats().tipsByAuto, 1);
});

test('reset restores initial state', () => {
  const h = make();
  launchesToTip(h);
  h.reset();
  assert.equal(h.state, 'audience_up'); assert.equal(h.angleDeg, -30);
  assert.equal(h.scored.length, 3);
  const s = h.cycleStats();
  assert.equal(s.upCellWeight, 0);
  assert.equal(s.upPollen, 0);
  assert.equal(s.upNectar, 0);
  assert.equal(s.tipProgress, 0);
  for (const k of ['upCellWeight', 'upPollen', 'upNectar', 'tipProgress']) delete s[k];
  assert.deepEqual(s, {
    tips: 0, upCellCount: 0, ballsToTip: 8, scoredTotal: 0,
    scoredInCell: { audience: 3, far: 0 }, tipsByAuto: null,
  });
});

test('huge dt is stable: no NaN, no overshoot', () => {
  const h = make();
  for (let i = 0; i < POLLEN_TO_TIP; i++) h.addScoredBall('audience');
  h.step(1e9);
  assert.equal(h.angleDeg, 30);
  h.step(Infinity); h.step(NaN); h.step(-1);
  assert.equal(h.angleDeg, 30);
  assert.equal(h.state, 'far_up');
});

test('deterministic: same inputs give identical traces', () => {
  const trace = () => {
    const h = make();
    const out = [];
    for (let i = 0; i < 400; i++) {
      if (i % 25 === 0) h.addScoredBall(h.upCell, null, i % 50 ? 'nectar' : 'pollen');
      h.step(1 / 60);
      out.push([h.angleDeg, h.state, h.cycleStats().tips]);
    }
    return JSON.stringify(out);
  };
  assert.equal(trace(), trace());
});

test('setState instant and animated, no onTip, no tip count', () => {
  const h = make();
  let fired = 0; h.onTip = () => fired++;
  h.setState('far_up');
  assert.equal(h.state, 'far_up'); assert.equal(h.angleDeg, 30);
  h.setState('audience_up', true);
  assert.equal(h.moving, true);
  run(h, 3);
  assert.equal(h.state, 'audience_up'); assert.equal(h.angleDeg, -30); assert.equal(h.moving, false);
  assert.equal(fired, 0);
  assert.equal(h.cycleStats().tips, 0);
  assert.throws(() => h.setState('bogus'));
});

test('scored balls are stored in the hive frame and rotate with it', () => {
  const h = make();
  const c = scene.constants.hives.red.states.audience_up.cells.audience.floor_corners;
  const wy = c.reduce((s, p) => s + p[1], 0) / 4, wz = c.reduce((s, p) => s + p[2], 0) / 4;
  for (let i = 0; i < POLLEN_TO_TIP - 1; i++) h.addScoredBall('audience', [59.25, wy, wz]);
  h.addScoredBall('audience', [59.25, wy, wz]); // the 8th pollen: starts the tip
  const ball = h.scored[h.scored.length - 1];
  const before = ball.local.slice();
  run(h, 3);
  assert.equal(h.state, 'far_up');
  assert.deepEqual(ball.local, before);
  const a = 30 * Math.PI / 180;
  const wzNow = before[0] * Math.sin(a) + before[1] * Math.cos(a);
  const fc = scene.constants.hives.red.states.far_up.cells.audience.floor_corners;
  const fcz = fc.reduce((s, p) => s + p[2], 0) / 4 - h.pivot[2];
  assert.ok(Math.abs(wzNow - fcz) < 0.3, `${wzNow} vs ${fcz}`);
});
