// Drop-out: balls stored in a cell that becomes the DOWN cell leave the hive under gravity; up-cell balls stay.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL } from '../web/js/balls.js';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { parseHiveCollider } from '../web/js/hivemesh.js';

const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const DT = 1 / 60, G = BALL.g;
function build(opts = {}) {
  const env = { bounds: { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 }, hives: {}, ...opts };
  const g = scene.groups.hive_red;
  env.hives.red = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives.red.states[g.cad_pose_state].cells, mesh: meshes.red };
  const dyn = new HiveDynamics(g, scene.constants.hives.red.states, { ...HIVE_PARAMS });
  const w = new BallWorld(env);
  w.onCapture = (b, a, c) => dyn.addScoredBall(c, [b.x, b.y, b.z], 'pollen');
  w.onRelease = (b, a, c) => { (w.log = w.log || []).push(c); };
  return { w, dyn };
}
const tick = (s) => { s.dyn.step(DT); s.w.setHiveAngles({ red: s.dyn.angleDeg }); s.w.step(DT); };
const run = (s, secs) => { for (let t = 0; t < secs; t += DT) tick(s); };
function feed(s) {
  const up = Object.values(scene.constants.hives.red.states[s.dyn.state].cells).find((c) => c.up);
  const n = up.opening_normal, ap = up.aim_point, from = [ap[0], ap[1] + n[1] * 45, ap[2] + n[2] * 45], sp = 300;
  const d = [ap[0] - from[0], ap[1] - from[1], ap[2] - from[2]], T = Math.hypot(...d) / sp;
  return s.w.addBall('pollen', from, [d[0] / T, d[1] / T, d[2] / T + 0.5 * G * T]);
}

test('up-cell balls stay stored while the hive rests', () => {
  const s = build();
  for (let i = 0; i < 3; i++) { feed(s); run(s, 0.8); }
  const stored = s.w.balls.filter((b) => b.state === 'scored').length;
  assert.ok(stored >= 2, 'captured ' + stored);
  run(s, 3);
  assert.equal(s.w.balls.filter((b) => b.state === 'scored').length, stored);
  assert.equal(s.w.released, 0);
});

test('after a tip the balls of the old up cell fall out and end on the floor outside the hive', () => {
  const s = build();
  for (let i = 0; i < 8; i++) { feed(s); run(s, 0.6); }
  run(s, 4);
  assert.equal(s.dyn.state, 'far_up', 'hive tipped');
  const g = scene.groups.hive_red, piv = g.pivot;
  assert.ok(s.w.released >= 6, 'released ' + s.w.released);
  assert.equal(s.w.balls.filter((b) => b.state === 'scored' && b.cell === 'audience').length, 0, 'no ghost stuck in down cell');
  // every released ball is a real free ball: no teleport (finite), on or above the floor, and rolled off the hive in y
  let out = 0;
  for (const b of s.w.balls) {
    assert.ok(Number.isFinite(b.x + b.y + b.z));
    assert.ok(b.z >= b.r - 0.5, 'above floor');
    if (b.state === 'free' && Math.abs(b.y - piv[1]) > 8) out++;
  }
  assert.ok(out >= 4, 'balls outside the hive footprint: ' + out);
});

test('dropOnTip=false keeps the old behaviour (balls stay stored)', () => {
  const s = build({ dropOnTip: false });
  for (let i = 0; i < 8; i++) { feed(s); run(s, 0.6); }
  run(s, 4);
  assert.equal(s.w.released, 0);
});

test('hivecycle forgets down-cell records after a tip (releaseOnTip default true, no ghost in the model)', () => {
  const s = build();
  for (let i = 0; i < 8; i++) { feed(s); run(s, 0.6); }
  run(s, 4);
  assert.equal(s.dyn.cycleStats().scoredInCell.audience, 0);
});

test('forgetBall removes the oldest record of a cell and reports the count', () => {
  const s = build();
  const before = s.dyn.cycleStats().scoredInCell.audience;
  assert.equal(s.dyn.forgetBall('audience', 2), 2);
  assert.equal(s.dyn.cycleStats().scoredInCell.audience, before - 2);
  assert.equal(s.dyn.forgetBall('far', 1), 0);
});
