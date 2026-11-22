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

test('same logical state on both hives gives identical angle traces (mirrored states, equal loads)', () => {
  const r = make('red'), b = make('blue');
  r.setState('audience_up');
  b.setState('audience_up');
  const tr = tipTrace(r), tb = tipTrace(b);
  assert.deepEqual(tr.angles, tb.angles);
  assert.equal(tr.frames, tb.frames);
  assert.equal(tr.end, 30);
  assert.equal(tb.end, 30);
});
