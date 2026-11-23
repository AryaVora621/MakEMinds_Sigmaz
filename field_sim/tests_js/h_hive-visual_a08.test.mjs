// a08: mirrored red/blue hive tip symmetry (HiveDynamics, web/js/hivecycle.js).
// Red pivot x=59.267, blue x=84.75 (scene.json). Mirror about x=72 gives 84.733, a 0.017 in gap (CAD, measured).
// Run from field_sim: node --test tests_js/h_hive-visual_a08.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { HIVE_TIP } from '../web/js/hivetip_model.js';

const scene = JSON.parse(fs.readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const DT = 1 / 60;
const N = HIVE_TIP.threshold / HIVE_TIP.pollenWeight; // 8 pollen

function make(color) {
  return new HiveDynamics(scene.groups[`hive_${color}`], scene.constants.hives[color].states, { ...HIVE_PARAMS });
}

/** Load N pollen into the up cell, then step at 60 Hz until at rest, recording the shown angle per frame. */
function tipTrace(h) {
  const tips = [];
  h.onTip = (s) => tips.push(s);
  for (let i = 0; i < N; i++) h.addScoredBall(h.upCell, null, 'pollen');
  const angles = [h.angleDeg];
  let frames = 0;
  while (h.moving && frames < 600) { h.step(DT); frames++; angles.push(h.angleDeg); }
  return { angles, frames, end: h.angleDeg, state: h.state, tips };
}

test('red and blue CAD poses are mirrored: opposite stable angles', () => {
  const r = make('red'), b = make('blue');
  assert.equal(r.state, 'audience_up');
  assert.equal(b.state, 'far_up');
  assert.equal(b.angleDeg, -r.angleDeg);
  assert.equal(Math.abs(r.angleDeg), 30);
});

test('mirror pivot: blue pivot is the red pivot mirrored about x=72 (CAD tolerance 0.05 in)', () => {
  const pr = scene.groups.hive_red.pivot, pb = scene.groups.hive_blue.pivot;
  assert.ok(Math.abs((pr[0] + pb[0]) / 2 - 72) < 0.05, `x mid ${(pr[0] + pb[0]) / 2}`);
  assert.equal(pr[1], pb[1]);
  assert.equal(pr[2], pb[2]);
});

test('equal loads: red and blue tip with equal-magnitude opposite angles in the same frame count', () => {
  const r = tipTrace(make('red')), b = tipTrace(make('blue'));
  assert.equal(r.frames, b.frames);
  assert.equal(r.end, -b.end);
  assert.equal(Math.abs(r.end), 30);
  assert.equal(r.angles.length, b.angles.length);
  r.angles.forEach((a, i) => assert.ok(Math.abs(a + b.angles[i]) < 1e-9, `frame ${i}: ${a} vs ${b.angles[i]}`));
  assert.deepEqual(r.tips, ['far_up']);
  assert.deepEqual(b.tips, ['audience_up']);
});

// With the dynamic (rigid-body) tip the angle trace depends on the stored load. The 3 starting nectar
// are stored in each hive's CAD-pose up cell: red 'audience', blue 'far'. After setState('audience_up')
// red therefore holds them in its up cell and blue in its down cell, so the loads were NOT equal (the old
// 0.6 s ease ignored load, which hid this). Measured with the original setup at 60 Hz: the traces diverge
// from frame 1 (red -29.995420 deg vs blue -29.995553 deg) and red settles in 60 frames, blue in 105
// (the nectar in blue's down cell resists the tip); both still end at +30. To test what this test claims
// (same logical state + equal loads => identical traces) both hives drop their starting nectar first.
test('same logical state on both hives gives identical angle traces (mirrored states, equal loads)', () => {
  const r = make('red'), b = make('blue');
  r.setState('audience_up');
  b.setState('audience_up');
  assert.equal(r.forgetBall('audience', 3), 3);
  assert.equal(b.forgetBall('far', 3), 3);
  assert.equal(r.scored.length, 0);
  assert.equal(b.scored.length, 0);
  const tr = tipTrace(r), tb = tipTrace(b);
  assert.deepEqual(tr.angles, tb.angles);
  assert.equal(tr.frames, tb.frames);
  assert.equal(tr.end, 30);
  assert.equal(tb.end, 30);
});
