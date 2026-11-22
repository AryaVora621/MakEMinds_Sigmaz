// Moving-robot shooting: shots fired while the robot drives sideways must still score through the hive opening.
// The ball leaves with the pivot velocity (v + omega x r), and the solver is given the same robot velocity, so the
// closed-form solve (uh = D/T - vs) aims the ball's ground velocity at the opening. Each shot is a snapshot of the robot
// passing the firing spot at lateral speed v (the robot velocity is what enters the physics; the pose is held fixed
// between shots so the hive geometry stays comparable with the 0-speed baseline).
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld } from '../web/js/balls.js';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import * as SR from '../web/js/shotrules.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const scene = JSON.parse(rd('../web/data/scene.json')), field = JSON.parse(rd('../web/data/colliders.json'));
const root = {}; vm.runInNewContext(rd('../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const DIMS = { halfX: 9, halfY: 9, height: 18 }, DT = 1 / 60, NEVER_TIP = { mode: 'count', ballsToTip: 1000 };

function build() {
  const env = { bounds: field.bounds, boxes: field.boxes, circles: field.circles, hives: {} }, dyn = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a];
    env.hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives[a].states[g.cad_pose_state].cells };
    dyn[a] = new HiveDynamics(g, scene.constants.hives[a].states, { ...HIVE_PARAMS, ...NEVER_TIP });
  }
  const w = new BallWorld(env); w.reset(scene.balls.balls);
  const s = { w, dyn, caps: 0 };
  w.onCapture = (b, a, c) => { s.caps++; dyn[a].addScoredBall(c, [b.x, b.y, b.z], 'pollen'); };
  return s;
}
function tick(s, pose) { s.dyn.red.step(DT); s.dyn.blue.step(DT); s.w.setRobot(pose, DIMS); s.w.setHiveAngles({ red: s.dyn.red.angleDeg, blue: s.dyn.blue.angleDeg }); s.w.step(DT); }
const upCell = (a, s) => Object.values(scene.constants.hives[a].states[s.dyn[a].state].cells).find((c) => c.up);
function aimOf(a, s) {
  const up = upCell(a, s);
  return { tg: { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal }, n: up.opening_normal };
}
// Robot start pose (same placement as shootend2end: 8 in off the aim point, 60 in out in front of the opening)
function poseOf(a, s) {
  const { tg, n } = aimOf(a, s);
  return { x: tg.x + 8, y: tg.y + Math.sign(n[1]) * 60, h: 1, vx: 0, vy: 0, om: 0 };
}
// Auto solution computed with the robot velocity (compensated) or with zero velocity (uncompensated control).
function solveWith(tg, pose, rv) {
  const p = { x: pose.x, y: pose.y, psi: pose.h, vx: rv.vx, vy: rv.vy, om: rv.om, rx: 0, ry: 0, z0: SH.exitHeight, X: tg.x, Y: tg.y, Z: tg.z };
  let r = Core.solve(p, 'gamma', SH.entryDeg * Math.PI / 180, 'low');
  const e = r.elev * 180 / Math.PI, c = Math.min(SH.hoodMax, Math.max(SH.hoodMin, e));
  if (c !== e) r = Core.solve(p, 'theta', c * Math.PI / 180, 'low');
  return { ok: r.ok, turretDeg: r.turret * 180 / Math.PI, elevDeg: r.elev * 180 / Math.PI, rpm: Core.rpmEstimate(r.speed, SH.wheelR, SH.eff) };
}
// Fire N shots at lateral speed v (in/s, along the opening plane). compensate=false fires with the 0-speed solution.
function fire(a, v, compensate, N = 20) {
  const s = build(), pose0 = poseOf(a, s), { tg, n } = aimOf(a, s);
  const lat = [-n[1], n[0]], hn = Math.hypot(lat[0], lat[1]) || 1;
  const rv = { vx: v * lat[0] / hn, vy: v * lat[1] / hn, om: 0 };
  const movingPose = { ...pose0, vx: rv.vx, vy: rv.vy, om: 0 };
  const solRv = compensate ? rv : { vx: 0, vy: 0, om: 0 };
  const sol = solveWith(tg, pose0, solRv);
  assert.ok(sol.ok, 'solver failed at v=' + v);
  for (let n = 0; n < N; n++) {
    const f = SR.forwardShot({ pose: movingPose, shooter: SH, target: tg, turretDeg: sol.turretDeg, hoodDeg: sol.elevDeg, rpm: sol.rpm, robotVel: rv });
    s.w.setRobot(movingPose, DIMS);
    const b = s.w.addBall('pollen', f.exit, f.vel); b.shot = true;
    for (let i = 0; i < 60; i++) tick(s, movingPose);
  }
  for (let i = 0; i < 120; i++) tick(s, movingPose);
  return s.caps;
}

test('moving shots: 20 shots at robot lateral speed 0, 30, 60 in/s score >= 90% of the 0-speed baseline (both hives)', () => {
  for (const a of ['red', 'blue']) {
    const base = fire(a, 0, true);
    assert.ok(base >= 18, a + ' baseline too low: ' + base + '/20');
    for (const v of [30, 60]) {
      const caps = fire(a, v, true);
      assert.ok(caps >= 0.9 * base, `${a} v=${v} in/s scored ${caps}/20, baseline ${base}/20`);
    }
  }
});

test('moving shots: solver sees robot velocity (compensated solution differs from the 0-speed one)', () => {
  const s = build(), pose0 = poseOf('blue', s), { tg, n } = aimOf('blue', s);
  const lat = [-n[1], n[0]], hn = Math.hypot(lat[0], lat[1]) || 1;
  const rv = { vx: 60 * lat[0] / hn, vy: 60 * lat[1] / hn, om: 0 };
  const a0 = solveWith(tg, pose0, { vx: 0, vy: 0, om: 0 }), a1 = solveWith(tg, pose0, rv);
  assert.ok(Math.abs(a0.turretDeg - a1.turretDeg) > 0.01, 'turret did not change with robot velocity');
});

test('moving shots: control, the same shots fired WITHOUT the robot-velocity compensation miss at 60 in/s', () => {
  for (const a of ['red', 'blue']) {
    const caps = fire(a, 60, false);
    assert.ok(caps < 0.5 * fire(a, 0, true), `${a} uncompensated 60 in/s scored ${caps}/20, expected a clear miss`);
  }
});
