// Hive tipping through the real BallWorld (real collider or fallback mesh) + HiveDynamics, wired as app.js does.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL } from '../web/js/balls.js';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { parseHiveCollider } from '../web/js/hivemesh.js';
import { HIVE_TIP } from '../web/js/hivetip_model.js';

const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const DT = 1 / 60, G = BALL.g;
// User fact (NOT in the manual): 8 pollen tip a hive. Physics tests launch pollen only.
const POLLEN_TO_TIP = HIVE_TIP.threshold / HIVE_TIP.pollenWeight;

function build(mesh, params) {
  const env = { bounds: { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 }, hives: {} }, dyn = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a];
    env.hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives[a].states[g.cad_pose_state].cells };
    if (mesh) env.hives[a].mesh = meshes[a];
    dyn[a] = new HiveDynamics(g, scene.constants.hives[a].states, { ...HIVE_PARAMS, ...params });
  }
  const w = new BallWorld(env), s = { w, dyn, log: [], tips: [] };
  w.onCapture = (b, a, c) => { s.log.push(a + '/' + c); dyn[a].addScoredBall(c, [b.x, b.y, b.z], 'pollen'); };
  for (const a of ['red', 'blue']) dyn[a].onTip = (st) => s.tips.push(a + ':' + st);
  return s;
}
const tick = (s) => { s.dyn.red.step(DT); s.dyn.blue.step(DT); s.w.setHiveAngles({ red: s.dyn.red.angleDeg, blue: s.dyn.blue.angleDeg }); s.w.step(DT); };
const run = (s, secs) => { for (let t = 0; t < secs; t += DT) tick(s); };
/** a ball on an arc through the up cell's aim point, entering against its normal */
function feed(s, a) {
  const up = Object.values(scene.constants.hives[a].states[s.dyn[a].state].cells).find((c) => c.up);
  const n = up.opening_normal, ap = up.aim_point, from = [ap[0], ap[1] + n[1] * 45, ap[2] + n[2] * 45], sp = 300;
  const d = [ap[0] - from[0], ap[1] - from[1], ap[2] - from[2]], L = Math.hypot(...d);
  const T = L / sp;
  return s.w.addBall('pollen', from, [d[0] / T, d[1] / T, d[2] / T + 0.5 * G * T]);
}

for (const mesh of [false, true]) {
  const tag = mesh ? 'real collider' : 'fallback mesh';
  test(`tip (${tag}): 7 pollen do not tip; the 8th tips and the angle animates by 60 deg`, () => {
    for (const a of ['red', 'blue']) {
      const s = build(mesh), a0 = s.dyn[a].angleDeg;
      for (let i = 1; i < POLLEN_TO_TIP; i++) { feed(s, a); run(s, 1.2); }
      assert.equal(s.tips.length, 0, a + ' tipped before ' + POLLEN_TO_TIP + ' pollen');
      assert.equal(s.dyn[a].angleDeg, a0, a + ' moved before the 8th pollen');
      feed(s, a);
      let moved = false;
      for (let i = 0; i < 120; i++) { tick(s); if (s.dyn[a].moving && Math.abs(s.dyn[a].angleDeg - a0) > 1) moved = true; }
      assert.equal(s.log.filter((x) => x.startsWith(a)).length, POLLEN_TO_TIP, a + ' capture count');
      assert.deepEqual(s.tips, [a + ':' + (a0 < 0 ? 'far_up' : 'audience_up')]);
      assert.ok(moved, 'angle animated');
      assert.equal(Math.abs(s.dyn[a].angleDeg - a0), 60);
      assert.ok(Math.abs(s.w._angles[a] - s.dyn[a].angleDeg) < 1e-9, 'physics hive follows the shown angle');
    }
  });

  test(`tip (${tag}): 8 pollen tip, then the NEW up cell needs 8 more and tips back`, () => {
    const s = build(mesh), a = 'red';
    for (let i = 1; i < POLLEN_TO_TIP; i++) { assert.equal(s.tips.length, 0, 'tipped early'); feed(s, a); run(s, 1.2); }
    assert.equal(s.tips.length, 0, 'tipped early');
    feed(s, a); run(s, 1.2);
    assert.equal(s.tips.length, 1);
    const first = s.dyn[a].state, cell = s.dyn[a].upCell;
    for (let i = 1; i < POLLEN_TO_TIP; i++) { feed(s, a); run(s, 1.2); }
    assert.equal(s.tips.length, 1, 'tipped back before 8 fresh pollen');
    // Wait window 1.2 s -> 2.5 s for the tip-back only. Proof (dynamic hive, default params, measured):
    // the first tip (3 starting nectar + 8 pollen, the calibration load) takes 1.00 s after capture
    // (= nominalTipTimeS by construction). The starting nectar dropped out with that tip, so the tip-back
    // carries 8 pollen only, below the calibrated hold torque; the rule assist (assistMinNm) drives it and
    // it takes 1.53 s after capture. Capture is 0.17 s after feed, so the hive settles 1.70 s after feed,
    // which the old 1.2 s window (sized for the 0.6 s ease) cannot see. The claim tested (8 fresh pollen
    // tip back, 7 do not) is unchanged.
    feed(s, a); run(s, 2.5);
    assert.equal(s.tips.length, 2);
    assert.notEqual(s.dyn[a].state, first);
    assert.ok(s.log.some((x) => x === a + '/' + cell) && s.log.some((x) => x !== a + '/' + cell), 'both cells received balls');
  });
}

test('tip: scored balls ride the hive when it tips and each ball is captured exactly once', () => {
  const s = build(true), a = 'blue';
  const b = feed(s, a); run(s, 2);
  assert.equal(b.state, 'scored');
  const y0 = b.y, z0 = b.z;
  assert.equal(s.tips.length, 0);
  for (let i = 2; i < POLLEN_TO_TIP; i++) { feed(s, a); run(s, 1); }
  assert.equal(s.tips.length, 0, 'tipped before the 8th pollen');
  feed(s, a); run(s, 2);
  assert.equal(s.tips.length, 1);
  assert.ok(Math.hypot(b.y - y0, b.z - z0) > 5, 'ball rotated with the hive');
  assert.equal(s.log.length, POLLEN_TO_TIP);
});

test('tip: a full cell recycles its oldest stored ball instead of stacking balls out of the hive', () => {
  // Raise the weight threshold far above 70 pollen so the hive stays latched and the cell fills.
  const s = build(true, { tipWeightThreshold: 1000 }), a = 'red';
  for (let i = 0; i < 70; i++) { feed(s, a); run(s, 0.25); }
  const stored = s.w.balls.filter((b) => b.state === 'scored');
  assert.ok(stored.length <= 135, 'stored ' + stored.length);
  assert.ok(s.log.length >= 60, 'captures ' + s.log.length);
  assert.equal(s.dyn[a].cycleStats().scoredTotal, s.log.length);
});
