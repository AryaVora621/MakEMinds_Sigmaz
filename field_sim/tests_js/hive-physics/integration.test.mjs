// Integration: unmodified BallWorld (balls.js) + default HiveDynamics (rule tip + dynamic motion). Inches, seconds.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL } from '../../web/js/balls.js';
import { HiveDynamics, HIVE_PARAMS } from '../../web/js/hivecycle.js';
import { parseHiveCollider } from '../../web/js/hivemesh.js';

const scene = JSON.parse(readFileSync(new URL('../../web/data/scene.json', import.meta.url), 'utf8'));
const bin = readFileSync(new URL('../../web/data/hive_collider.bin', import.meta.url));
const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const DT = 1 / 60, G = BALL.g;

function build(params = {}) {
  const env = { bounds: { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 }, hives: {} };
  const g = scene.groups.hive_red;
  env.hives.red = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives.red.states[g.cad_pose_state].cells, mesh: meshes.red };
  const dyn = new HiveDynamics(g, scene.constants.hives.red.states, { ...HIVE_PARAMS, ...params });
  const w = new BallWorld(env);
  w.onCapture = (b, a, c) => dyn.addScoredBall(c, [b.x, b.y, b.z], 'pollen');
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

test('8 pollen fed through BallWorld tip the default hive to far_up with no NaN, no ball under the floor, >= 6 released', () => {
  const s = build();
  const angles = [];
  for (let i = 0; i < 8; i++) { feed(s); for (let t = 0; t < 0.6; t += DT) { tick(s); angles.push(s.dyn.angleDeg); } }
  for (let t = 0; t < 4; t += DT) { tick(s); angles.push(s.dyn.angleDeg); }
  assert.equal(s.dyn.state, 'far_up', 'hive tipped');
  assert.equal(s.dyn.angleDeg, 30);
  assert.equal(s.dyn.cycleStats().tips, 1);
  assert.ok(angles.every(Number.isFinite), 'angle finite');
  // the swing is monotone: once the angle leaves -30 it never goes back
  const first = angles.findIndex((a) => a > -30 + 1e-9);
  for (let i = first + 1; i < angles.length; i++) assert.ok(angles[i] >= angles[i - 1] - 1e-9, 'monotone at ' + i);
  assert.ok(s.w.released >= 6, 'released ' + s.w.released);
  for (const b of s.w.balls) {
    assert.ok(Number.isFinite(b.x + b.y + b.z + b.vx + b.vy + b.vz), 'ball finite');
    assert.ok(b.z >= b.r - 0.5, 'above floor: z=' + b.z);
  }
});

test('hive model forgets the down-cell records after the tip (no ghosts)', () => {
  const s = build();
  for (let i = 0; i < 8; i++) { feed(s); run(s, 0.6); }
  run(s, 4);
  assert.equal(s.dyn.state, 'far_up');
  assert.equal(s.dyn.cycleStats().scoredInCell.audience, 0);
});

test('legacy ease motion still tips with BallWorld', () => {
  const s = build({ motionModel: 'ease' });
  for (let i = 0; i < 8; i++) { feed(s); run(s, 0.6); }
  run(s, 3);
  assert.equal(s.dyn.state, 'far_up');
  assert.ok(s.w.released >= 6, 'released ' + s.w.released);
});
