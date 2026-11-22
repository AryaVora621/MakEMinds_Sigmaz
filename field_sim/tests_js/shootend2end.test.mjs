// End to end shooting: app.js shoot() path = forwardShot(real turret/hood/rpm) -> world.addBall -> world.onCapture -> HiveDynamics.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL } from '../web/js/balls.js';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';
import { ShooterPlant, modeCommand } from '../web/js/aimcontrol.js';
import * as SR from '../web/js/shotrules.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const scene = JSON.parse(rd('../web/data/scene.json')), field = JSON.parse(rd('../web/data/colliders.json'));
const root = {}; vm.runInNewContext(rd('../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const SPEC = { turretRateDegS: 540, turretKp: 14, turretMinDeg: -180, turretMaxDeg: 180, hoodMinDeg: 25, hoodMaxDeg: 65, flywheelTauS: 0.35, maxRpm: 6000, rpmDipFrac: 0.06 };
const DIMS = { halfX: 9, halfY: 9, height: 18 }, DT = 1 / 60;

function build(params) {
  const env = { bounds: field.bounds, boxes: field.boxes, circles: field.circles, hives: {} }, dyn = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a];
    env.hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives[a].states[g.cad_pose_state].cells };
    dyn[a] = new HiveDynamics(g, scene.constants.hives[a].states, { ...HIVE_PARAMS, ...params });
  }
  const w = new BallWorld(env); w.reset(scene.balls.balls);
  const s = { w, dyn, caps: 0, tips: { red: 0, blue: 0 } };
  w.onCapture = (b, a, c) => { s.caps++; dyn[a].addScoredBall(c, [b.x, b.y, b.z], 'pollen'); };
  for (const a of ['red', 'blue']) dyn[a].onTip = () => s.tips[a]++;
  return s;
}
function tick(s, pose) { s.dyn.red.step(DT); s.dyn.blue.step(DT); s.w.setRobot(pose, DIMS); s.w.setHiveAngles({ red: s.dyn.red.angleDeg, blue: s.dyn.blue.angleDeg }); s.w.step(DT); }
function solveFor(s, a, pose) {
  const up = Object.values(scene.constants.hives[a].states[s.dyn[a].state].cells).find((c) => c.up);
  const tg = { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal };
  const p = { x: pose.x, y: pose.y, psi: pose.h, vx: 0, vy: 0, om: 0, rx: 0, ry: 0, z0: SH.exitHeight, X: tg.x, Y: tg.y, Z: tg.z };
  let r = Core.solve(p, 'gamma', SH.entryDeg * Math.PI / 180, 'low');
  const e = r.elev * 180 / Math.PI, c = Math.min(65, Math.max(25, e));
  if (c !== e) r = Core.solve(p, 'theta', c * Math.PI / 180, 'low');
  return { tg, sol: { ok: r.ok, turretDeg: r.turret * 180 / Math.PI, elevDeg: r.elev * 180 / Math.PI, rpm: Core.rpmEstimate(r.speed, SH.wheelR, SH.eff) } };
}
const poseFor = (a, s) => { const up = Object.values(scene.constants.hives[a].states[s.dyn[a].state].cells).find((c) => c.up); return { x: up.aim_point[0] + 8, y: up.aim_point[1] + Math.sign(up.opening_normal[1]) * 60, h: 1, vx: 0, vy: 0, om: 0 }; };

// Tip rule (user fact, manual silent): a hive tips at 8 POLLEN. HIVE_PARAMS.ballsToTip is only read in 'count' mode, so
// tests that want a hive that never tips must use mode:'count' (weight mode ignores ballsToTip and tips at 8 pollen).
const NEVER_TIP = { mode: 'count', ballsToTip: 1000 };
test('e2e: 8 solver shots from the real turret/hood/rpm score and tip the hive once (default rule, either alliance)', () => {
  for (const a of ['red', 'blue']) {
    const s = build(), pose = poseFor(a, s), { tg, sol } = solveFor(s, a, pose);
    for (let n = 0; n < 8; n++) {
      const f = SR.forwardShot({ pose, shooter: SH, target: tg, turretDeg: sol.turretDeg, hoodDeg: sol.elevDeg, rpm: sol.rpm });
      s.w.setRobot(pose, DIMS);
      const b = s.w.addBall('pollen', f.exit, f.vel); b.shot = true;
      for (let i = 0; i < 60; i++) tick(s, pose);
      if (n === 6) assert.equal(s.tips[a], 0, a + ' tipped after 7 pollen (needs 8)');
    }
    for (let i = 0; i < 120; i++) tick(s, pose);   // the tip animation runs 0.6 s after the 8th ball
    assert.equal(s.caps, 8, a + ' scored ' + s.caps + '/8');
    assert.equal(s.tips[a], 1, a + ' did not tip at 8 pollen');
    const g = scene.groups['hive_' + a];
    assert.notEqual(s.dyn[a].angleDeg, g.stable_angles_deg[g.cad_pose_state]);
  }
});

test('e2e: continuous auto fire every 0.15 s (plant rpm dips on every shot) keeps hitting the opening', () => {
  const s = build(NEVER_TIP), a = 'blue', pose = poseFor(a, s), plant = new ShooterPlant(SPEC);
  let prev = null, t = 0, next = 3, shots = 0;
  const { tg, sol } = solveFor(s, a, pose);
  for (let i = 0; i < 60 * 12; i++) {
    const cmd = modeCommand('auto', sol, null, SPEC, prev); prev = cmd; plant.step(DT, cmd); t += DT;
    if (t >= next && shots < 30) {
      shots++; next = t + 0.15;
      const f = SR.forwardShot({ pose, shooter: SH, target: tg, turretDeg: plant.state.turretDeg, hoodDeg: plant.state.hoodDeg, rpm: plant.state.rpm });
      s.w.setRobot(pose, DIMS); const b = s.w.addBall('pollen', f.exit, f.vel); b.shot = true; plant.kick();
    }
    tick(s, pose);
  }
  assert.equal(shots, 30);
  assert.ok(s.caps >= 28, 'only ' + s.caps + '/30 scored (flywheel rpm sag)');
});

test('e2e: flywheel recovers a shot dip well inside the 0.15 s shot spacing', () => {
  const plant = new ShooterPlant(SPEC); plant.state.rpm = 2500;
  const cmd = { turretDeg: 0, hoodDeg: 45, rpm: 2500 };
  plant.kick(); for (let i = 0; i < 9; i++) plant.step(DT, cmd);
  assert.ok(plant.state.rpm > 2500 * 0.995, 'rpm ' + plant.state.rpm);
  const cold = new ShooterPlant(SPEC); for (let i = 0; i < 6; i++) cold.step(DT, cmd);
  assert.ok(cold.state.rpm < 2500 * 0.5, 'spin-up from rest keeps the slow time constant');
});

test('e2e: a ball fired from inside the robot box keeps its exit height and speed (no robot collision at spawn)', () => {
  const w = new BallWorld({ bounds: field.bounds }), pose = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  w.setRobot(pose, DIMS);
  const b = w.addBall('pollen', [72, 72, SH.exitHeight], [200, 0, 0]);
  w.setRobot(pose, DIMS); w.step(DT);
  assert.ok(b.z < SH.exitHeight, 'nudged up to ' + b.z);
  assert.ok(b.vx > 190, 'slowed to ' + b.vx);
  for (let i = 0; i < 20; i++) { w.setRobot(pose, DIMS); w.step(DT); }
  assert.ok(b.x > 72 + 20);
});

test('e2e: unlimited shots are recycled (world stays bounded) and stay fast', () => {
  const s = build(NEVER_TIP), a = 'blue', pose = poseFor(a, s), { tg, sol } = solveFor(s, a, pose), start = s.w.balls.length;
  const t0 = performance.now();
  for (let n = 0; n < 400; n++) {
    const f = SR.forwardShot({ pose, shooter: SH, target: tg, turretDeg: sol.turretDeg, hoodDeg: sol.elevDeg, rpm: sol.rpm });
    s.w.setRobot(pose, DIMS);
    s.w.addBall('pollen', f.exit, f.vel).shot = true;
    for (let k = 0; k < 9; k++) tick(s, pose);
    for (const b of s.w.balls) assert.ok(Number.isFinite(b.x + b.y + b.z));
  }
  assert.ok(s.w.balls.length <= start + BALL.maxFired + 2 * 135, 'balls ' + s.w.balls.length + ' ' + JSON.stringify(s.w.stats()) + ' fired ' + s.w.balls.filter((b) => b.fired).length);
  assert.ok(performance.now() - t0 < 20000);
  assert.ok(s.caps > 350, 'caps ' + s.caps);
});

// Held-ball shooting: a robot holding 4 balls (intake loads them, FIFO hold) fires one shot through BallWorld.fireHeld.
const heldRobot = () => {
  const w = new BallWorld({ bounds: field.bounds, boxes: field.boxes, circles: field.circles });
  w.reset([]); w.setIntake({ enabled: true, space: Infinity });
  const pose = { x: 60, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  for (let i = 0; i < 4; i++) w.balls.push(w._mk('pollen', [71.5, 69 + i * 2.2, 1.4], [0, 0, 0]));
  for (let i = 0; i < 600 && w.held.length < 4; i++) { w.setRobot(pose, DIMS); w.step(DT); }
  return { w, pose };
};
test('e2e: robot holding 4 balls fires one aimed shot: the ball flies >1 m in 1 s and the held count drops by 1', () => {
  const { w, pose } = heldRobot();
  assert.equal(w.held.length, 4, 'intake did not load 4 balls');
  const f = SR.forwardShot({ pose, shooter: SH, target: { x: 130, y: 72, z: 50 }, turretDeg: 0, hoodDeg: 35, rpm: 2400 });
  w.setRobot(pose, DIMS);
  const b = w.fireHeld(f.exit, f.vel);
  assert.ok(b, 'nothing fired');
  assert.equal(w.held.length, 3, 'held count did not drop by 1');
  const p0 = [b.x, b.y, b.z];
  let far = 0;
  for (let i = 0; i < 60; i++) { w.setRobot(pose, DIMS); w.step(DT); far = Math.max(far, Math.hypot(b.x - p0[0], b.y - p0[1], b.z - p0[2])); }
  assert.ok(far > 39.37, 'ball travelled only ' + far.toFixed(1) + ' in in 1 s');
});
test('e2e: a ball fired from a held slot is not taken straight back in by the roller (low shot dropping in front)', () => {
  const { w, pose } = heldRobot();
  const f = SR.forwardShot({ pose, shooter: SH, target: { x: 130, y: 72, z: 50 }, turretDeg: 0, hoodDeg: 25, rpm: 300 });
  w.setRobot(pose, DIMS);
  const b = w.fireHeld(f.exit, f.vel);
  for (let i = 0; i < 60; i++) { w.setRobot(pose, DIMS); w.step(DT); }
  assert.notEqual(b.state, 'intaken', 'fired ball was re-intaken');
  assert.equal(w.held.length, 3, 'held count ' + w.held.length + ' after firing one ball');
});

// Full 4-ball hopper, repeated shots (pollen): intake 4 balls, fire all 4 through BallWorld.fireHeld at 0.5 s spacing, reload, repeat.
// Robot sits 40 in on the opening side of the blue up cell (opening normal +y), inside the field, facing the hive.
// Helpers are local copies of the ballkinds_e2e.test.mjs ones (test files do not import each other).
const hopperPlace = (pose, lx, ly) => { const c = Math.cos(pose.h), sn = Math.sin(pose.h); return [pose.x + c * lx - sn * ly, pose.y + sn * lx + c * ly, BALL.r]; };
const HOPPER_LOCAL = [[11.4, -3.3], [11.4, -1.1], [11.4, 1.1], [11.4, 3.3]];
function hopperLoad(s, pose) {
  for (let i = 0; i < 4; i++) s.w.balls.push(s.w._mk('pollen', hopperPlace(pose, ...HOPPER_LOCAL[i]), [0, 0, 0]));
  for (let i = 0; i < 900 && s.w.held.length < 4; i++) tick(s, pose);
}
function hopperFire(s, pose, tg) {
  const n = s.w.held.length;
  for (let i = 0; i < n; i++) {
    const sol = SR.solveAutoShot(pose, SH, tg, 'pollen', Core);
    assert.equal(sol.ok, true, 'no auto solution for shot ' + i);
    const f = SR.forwardShot({ pose, shooter: SH, target: tg, turretDeg: sol.turretDeg, hoodDeg: sol.elev * 180 / Math.PI, rpm: sol.rpm });
    s.w.setRobot(pose, DIMS);
    assert.ok(s.w.fireHeld(f.exit, f.vel), 'nothing fired');
    for (let k = 0; k < 30; k++) tick(s, pose);   // 0.5 s between shots
  }
}
test('e2e full hopper (pollen): 4 intaken balls all enter the blue hive, a reload to 8 tips it', () => {
  const s = build(), a = 'blue', pose = { x: 92.767, y: 128.483, h: -Math.PI / 2, vx: 0, vy: 0, om: 0 };
  const up = Object.values(scene.constants.hives[a].states[s.dyn[a].state].cells).find((c) => c.up);
  const tg = { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal };
  const st0 = s.dyn[a].state;
  s.w.reset([]); s.w.setIntake({ enabled: true, space: Infinity });
  hopperLoad(s, pose);
  assert.equal(s.w.held.length, 4, 'intake did not hold 4 pollen');
  hopperFire(s, pose, tg);
  for (let k = 0; k < 180; k++) tick(s, pose);
  assert.equal(s.caps, 4, 'only ' + s.caps + '/4 pollen scored');
  assert.equal(s.dyn[a].scored.filter((b) => b.counts).length, 4, 'hive up cell count');
  assert.ok(Math.abs(s.dyn[a].tipProgress - 4 / 8) < 1e-9, 'tip progress ' + s.dyn[a].tipProgress);
  assert.equal(s.tips[a], 0, 'hive tipped after 4 pollen (needs 8)');
  hopperLoad(s, pose);
  assert.equal(s.w.held.length, 4, 'reload did not hold 4 pollen');
  hopperFire(s, pose, tg);
  for (let k = 0; k < 240; k++) tick(s, pose);
  assert.equal(s.caps, 8, 'only ' + s.caps + '/8 pollen scored');
  assert.equal(s.tips[a], 1, 'hive did not tip at 8 pollen');
  assert.notEqual(s.dyn[a].state, st0, 'hive state did not change after the tip');
});
