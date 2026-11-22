// End to end 4-ball hopper, mixed kinds: the robot intakes 4 balls (pollen/nectar), fires all 4 through BallWorld.fireHeld at
// 0.5 s spacing at the up cell of a hive, and the hive reports its counted balls by kind (balls.js hiveBallCounts).
// Same headless setup as shootend2end.test.mjs. Read-only use of hivecycle.js (tipProgress, scored, upCell, state).
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL, hiveBallCounts } from '../web/js/balls.js';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { HIVE_TIP } from '../web/js/hivetip_model.js';
import * as SR from '../web/js/shotrules.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const scene = JSON.parse(rd('../web/data/scene.json')), field = JSON.parse(rd('../web/data/colliders.json'));
const root = {}; vm.runInNewContext(rd('../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const DIMS = { halfX: 9, halfY: 9, height: 18 }, DT = 1 / 60;
const SPACING_TICKS = 30;           // 0.5 s between shots (flywheel and reload recover time)
// Robot-local intake positions for 4 balls in front of the front roller (ball surface about 1 in from the face), 2.2 in apart.
const LOAD_LOCAL = [[11.4, -3.3], [11.4, -1.1], [11.4, 1.1], [11.4, 3.3]];

function build(a) {
  const env = { bounds: field.bounds, boxes: field.boxes, circles: field.circles, hives: {} }, dyn = {};
  for (const h of ['red', 'blue']) {
    const g = scene.groups['hive_' + h];
    env.hives[h] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives[h].states[g.cad_pose_state].cells };
    dyn[h] = new HiveDynamics(g, scene.constants.hives[h].states, { ...HIVE_PARAMS });
  }
  const w = new BallWorld(env); w.reset([]);
  const s = { w, dyn, caps: 0, tips: { red: 0, blue: 0 }, a };
  // A captured ball is reported to its hive by kind: nectar stays nectar, anything else is pollen.
  w.onCapture = (b, h, c) => { s.caps++; dyn[h].addScoredBall(c, [b.x, b.y, b.z], b.kind === 'nectar' ? 'nectar' : 'pollen'); };
  for (const h of ['red', 'blue']) dyn[h].onTip = () => s.tips[h]++;
  w.setIntake({ enabled: true, space: Infinity });
  return s;
}
function tick(s, pose) { s.dyn.red.step(DT); s.dyn.blue.step(DT); s.w.setRobot(pose, DIMS); s.w.setHiveAngles({ red: s.dyn.red.angleDeg, blue: s.dyn.blue.angleDeg }); s.w.step(DT); }
const place = (pose, lx, ly) => { const c = Math.cos(pose.h), sn = Math.sin(pose.h); return [pose.x + c * lx - sn * ly, pose.y + sn * lx + c * ly, BALL.r]; };
// Up-cell aim target of hive a (same aim point the auto solver uses).
function upTarget(s) {
  const up = Object.values(scene.constants.hives[s.a].states[s.dyn[s.a].state].cells).find((c) => c.up);
  return { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal };
}
// Robot pose 40 in on the opening side of the up cell (the opening normal points +y), inside the 144 in field, facing the hive.
function poseFor(s) {
  const up = Object.values(scene.constants.hives[s.a].states[s.dyn[s.a].state].cells).find((c) => c.up);
  return { x: up.aim_point[0] + 8, y: up.aim_point[1] + 40, h: -Math.PI / 2, vx: 0, vy: 0, om: 0 };
}
// Load the 4 balls of the given types through the real intake (roller pull + FIFO hold).
function loadFour(s, pose, types) {
  for (let i = 0; i < 4; i++) s.w.balls.push(s.w._mk(types[i], place(pose, ...LOAD_LOCAL[i]), [0, 0, 0]));
  for (let i = 0; i < 900 && s.w.held.length < 4; i++) tick(s, pose);
}
// Fire every held ball, FIFO, each from the real turret/hood/rpm solved for its own kind, with SPACING_TICKS between shots.
function fireAll(s, pose, tg) {
  const n = s.w.held.length;
  for (let i = 0; i < n; i++) {
    const kind = s.w.held[0].kind;
    const sol = SR.solveAutoShot(pose, SH, tg, kind, Core);
    assert.equal(sol.ok, true, 'no auto solution for a ' + kind + ' shot ' + i);
    const f = SR.forwardShot({ pose, shooter: SH, target: tg, turretDeg: sol.turretDeg, hoodDeg: sol.elev * 180 / Math.PI, rpm: sol.rpm, kind });
    s.w.setRobot(pose, DIMS);
    const b = s.w.fireHeld(f.exit, f.vel);
    assert.ok(b, 'nothing fired');
    assert.equal(b.kind, kind, 'fired ball lost its kind');
    for (let k = 0; k < SPACING_TICKS; k++) tick(s, pose);
  }
}
const settle = (s, pose, n = 180) => { for (let k = 0; k < n; k++) tick(s, pose); };

test('e2e mixed hopper: [pollen, nectar, pollen, nectar] all enter the blue hive and hiveBallCounts is {pollen:2, nectar:2}', () => {
  const s = build('blue'), pose = poseFor(s), tg = upTarget(s);
  loadFour(s, pose, ['pollen', 'nectar_blue', 'pollen', 'nectar_blue']);
  assert.deepEqual(s.w.heldKindCounts(), { pollen: 2, nectar: 2 }, 'intake did not hold the mixed load');
  fireAll(s, pose, tg);
  settle(s, pose);
  assert.equal(s.caps, 4, 'only ' + s.caps + '/4 mixed balls scored');
  assert.deepEqual(hiveBallCounts('blue'), { pollen: 2, nectar: 2 });
  assert.deepEqual(hiveBallCounts('red'), { pollen: 0, nectar: 0 }, 'red hive counted balls it never received');
  // Weight model: 2 pollen + 2 nectar = 2 + 2 * nectarWeight < 8, so no tip yet.
  const want = (2 + 2 * HIVE_TIP.nectarWeight) / HIVE_TIP.threshold;
  assert.ok(Math.abs(s.dyn.blue.tipProgress - want) < 1e-9, 'tip progress ' + s.dyn.blue.tipProgress + ' != ' + want);
  assert.equal(s.tips.blue, 0, 'mixed load of 4 tipped the hive early');
});
