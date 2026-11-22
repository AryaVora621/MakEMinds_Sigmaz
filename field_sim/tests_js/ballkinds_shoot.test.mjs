// Kind-specific shooting: a pollen and a nectar fired from the same pose at the same hive both enter the opening, the
// projectile keeps the kind of the held ball, and the two kinds differ in radius (ball state, flight clearance, rim) and in
// launch speed (mass factor, ASSUMPTION in balls.js launchScaleFor).
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL_KINDS, launchScaleFor } from '../web/js/balls.js';
import * as SR from '../web/js/shotrules.js';
import { rimClearanceIn } from '../web/js/aimcontrol.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const scene = JSON.parse(rd('../web/data/scene.json')), field = JSON.parse(rd('../web/data/colliders.json'));
const root = {}; vm.runInNewContext(rd('../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const DIMS = { halfX: 9, halfY: 9, height: 18 }, DT = 1 / 60;
const TYPE = { pollen: 'pollen', nectar: 'nectar_red' };

function makeWorld() {
  const env = { bounds: field.bounds, boxes: field.boxes, circles: field.circles, hives: {} };
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a];
    env.hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives[a].states[g.cad_pose_state].cells };
  }
  const w = new BallWorld(env); w.reset([]);
  const caps = [];
  w.onCapture = (b, alliance, cell) => caps.push({ kind: b.kind, r: b.r, hive: alliance, cell });
  return { w, caps };
}

// Shooter pose and aim target for the CAD up cell of hive a (same construction as shootend2end.test.mjs).
function aimFor(a) {
  const g = scene.groups['hive_' + a];
  const up = Object.values(scene.constants.hives[a].states[g.cad_pose_state].cells).find((c) => c.up);
  const tg = { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal };
  const pose = { x: up.aim_point[0] + 8, y: up.aim_point[1] + Math.sign(up.opening_normal[1]) * 60, h: 1, vx: 0, vy: 0, om: 0 };
  return { tg, pose };
}

// The auto solution for one kind (ball speed from the aim, rpm from the kind's mass factor), then the launch from it.
function shootKind(a, kind) {
  const { tg, pose } = aimFor(a);
  const sol = SR.solveAutoShot(pose, SH, tg, kind, Core);
  assert.ok(sol.ok, kind + ' has no auto solution from the test pose');
  const f = SR.forwardShot({ pose, shooter: SH, target: tg, kind, turretDeg: sol.turretDeg, hoodDeg: sol.elevDeg ?? sol.elev * 180 / Math.PI, rpm: sol.rpm });
  return { pose, tg, sol, f };
}

test('launch scale: pollen is the reference (1.0), nectar is lighter-scaled from the assumed mass and launches slower at equal rpm', () => {
  assert.equal(launchScaleFor('pollen'), 1);
  const s = launchScaleFor('nectar');
  assert.ok(Math.abs(s - Math.sqrt(BALL_KINDS.pollen.massKg / BALL_KINDS.nectar.massKg)) < 1e-12, 'nectar scale ' + s);
  assert.ok(s > 0 && s < 1, 'nectar scale must be below 1 (heavier ball)');
  assert.equal(launchScaleFor('unknown_kind'), 1);   // unknown kinds fall back to the reference
  const p = SR.forwardShot({ pose: { x: 60, y: 72, h: 0 }, shooter: SH, kind: 'pollen', turretDeg: 0, hoodDeg: 45, rpm: 2500 });
  const n = SR.forwardShot({ pose: { x: 60, y: 72, h: 0 }, shooter: SH, kind: 'nectar', turretDeg: 0, hoodDeg: 45, rpm: 2500 });
  assert.ok(Math.abs(n.speed / p.speed - s) < 1e-12, 'same rpm: nectar speed must be pollen speed times the scale');
  assert.equal(p.kind, 'pollen');
  assert.equal(n.kind, 'nectar');
});

test('auto solution: nectar needs proportionally more rpm for the same aim (same ball speed from the same pose)', () => {
  const { sol: sp } = shootKind('blue', 'pollen');
  const { sol: sn } = shootKind('blue', 'nectar');
  assert.ok(Math.abs(sp.speed - sn.speed) < 1e-9, 'the aim needs the same ball exit speed for both kinds');
  assert.ok(Math.abs(sn.rpm / sp.rpm - 1 / launchScaleFor('nectar')) < 1e-9, 'rpm ratio ' + (sn.rpm / sp.rpm));
});

test('rim clearance and flight radius follow the kind radius', () => {
  assert.equal(rimClearanceIn('pollen'), BALL_KINDS.pollen.radiusIn);
  assert.equal(rimClearanceIn('nectar'), BALL_KINDS.nectar.radiusIn);
  assert.ok(rimClearanceIn('nectar') > rimClearanceIn('pollen'));
  const { f: fp } = shootKind('red', 'pollen'), { f: fn } = shootKind('red', 'nectar');
  assert.equal(fp.ballR, BALL_KINDS.pollen.radiusIn);
  assert.equal(fn.ballR, BALL_KINDS.nectar.radiusIn);
});

for (const a of ['red', 'blue']) {
  test(`pollen and nectar from the same pose at the ${a} hive both enter the opening; each keeps its kind and radius`, () => {
    const out = {};
    for (const kind of ['pollen', 'nectar']) {
      const { w, caps } = makeWorld();
      const { pose, f } = shootKind(a, kind);
      w.setRobot(pose, DIMS);
      const b = w.addBall(TYPE[kind], f.exit, f.vel, kind);
      b.shot = true;
      assert.equal(b.kind, kind, 'projectile kind must match the shot kind');
      assert.equal(b.r, BALL_KINDS[kind].radiusIn, kind + ' ball state radius');
      for (let i = 0; i < 240 && b.state === 'free'; i++) { w.setRobot(pose, DIMS); w.step(DT); }
      assert.equal(b.state, 'scored', kind + ' ball did not enter the ' + a + ' opening (state ' + b.state + ')');
      assert.equal(b.hive, a, kind + ' scored in the wrong hive');
      assert.equal(caps.length, 1, kind + ' capture count');
      assert.equal(caps[0].kind, kind, 'capture reports the projectile kind');
      assert.equal(caps[0].r, BALL_KINDS[kind].radiusIn, 'capture reports the kind radius');
      out[kind] = b.r;
    }
    assert.ok(out.nectar > out.pollen, 'nectar ball must be larger than pollen in the ball state');
  });
}

test('a held nectar fired through fireHeld stays nectar; FIFO order keeps mixed holds intact', () => {
  const { w } = makeWorld();
  w.held.push({ id: 'n1', type: 'nectar_red', kind: 'nectar' }, { id: 'p1', type: 'pollen', kind: 'pollen' });
  const { f } = shootKind('blue', 'nectar');
  const first = w.fireHeld(f.exit, f.vel);
  assert.equal(first.kind, 'nectar');
  assert.equal(first.r, BALL_KINDS.nectar.radiusIn);
  const second = w.fireHeld(f.exit, f.vel);
  assert.equal(second.kind, 'pollen');
  assert.equal(second.r, BALL_KINDS.pollen.radiusIn);
  assert.equal(w.fireHeld(f.exit, f.vel), null);
});
