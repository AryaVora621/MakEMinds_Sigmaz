// t19: BallWorld ball conservation, floor and teleport checks with the real red hive (headless).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL } from '../web/js/balls.js';
import { parseHiveCollider } from '../web/js/hivemesh.js';
const D = new URL('../web/data/', import.meta.url);
const scene = JSON.parse(readFileSync(new URL('scene.json', D), 'utf8')), bin = readFileSync(new URL('hive_collider.bin', D));
const mesh = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength)).red;
const g = scene.groups.hive_red, hc = scene.constants.hives.red;
const hives = { red: { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells, mesh } };
const up = Object.values(hives.red.cells).reduce((b, c) => (c.opening_normal[2] > b.opening_normal[2] ? c : b));
const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 };
const DT = 1 / 240, STEPS = 10 * 240, FLOOR_TOL = 0.01 + 0.1 * BALL.r, JUMP = 12; // FLOOR_TOL: ASSUMED reading
// Three pollen balls fed into the up cell along -normal at 60 in/s (a vertical drop misses the opening).
function setup() {
  const w = new BallWorld({ bounds, hives }), n = up.opening_normal, ap = up.aim_point;
  for (let i = 0; i < 3; i++) {
    const p = [ap[0] + n[0] * 4 + (i - 1) * 3, ap[1] + n[1] * 4, ap[2] + n[2] * 4];
    w.addBall('pollen', p, [-n[0] * 60, -n[1] * 60, -n[2] * 60]);
  }
  return w;
}
test('ball count is conserved for 10 s of stepFixed in the up cell', () => {
  const w = setup(), n0 = w.balls.length;
  for (let f = 0; f < 600; f++) {
    w.stepFixed(1 / 60, 240);
    assert.equal(w.balls.length, n0, `count changed at frame ${f}`);
  }
});
test('no NaN/Infinity position and no ball below the floor', () => {
  const w = setup();
  for (let s = 0; s < STEPS; s++) {
    w.step(DT);
    for (const b of w.balls) {
      for (const k of ['x', 'y', 'z']) assert.ok(Number.isFinite(b[k]), `step ${s} ${b.id} ${k}=${b[k]}`);
      assert.ok(b.z >= -FLOOR_TOL, `step ${s} ${b.id} z=${b.z.toFixed(3)} below floor`);
    }
  }
});
test('no ball jumps more than 12 in between consecutive substeps', () => {
  const w = setup(), pos = () => new Map(w.balls.map(b => [b, [b.x, b.y, b.z]])), prevHolder = { prev: pos() };
  for (let s = 0; s < STEPS; s++) {
    w.step(DT);
    for (const b of w.balls) {
      const p = prevHolder.prev.get(b), d = p ? Math.hypot(b.x - p[0], b.y - p[1], b.z - p[2]) : 0;
      assert.ok(d <= JUMP, `step ${s} ${b.id} jumped ${d.toFixed(2)} in`);
    }
    prevHolder.prev = pos();
  }
});
test('stats() scored count equals the number of balls flagged scored', () => {
  const w = setup();
  for (let s = 0; s < STEPS; s++) w.step(DT);
  const st = w.stats(), flagged = w.balls.filter(b => b.state === 'scored').length;
  assert.ok(flagged > 0, 'precondition: at least one ball must score');
  assert.equal(st.scoredRed + st.scoredBlue, flagged);
});
// Result: node --test tests_js/h_hive-physics_t19.test.mjs -> tests 4, pass 4, fail 0, todo 0, ~0.3 s (one run)
