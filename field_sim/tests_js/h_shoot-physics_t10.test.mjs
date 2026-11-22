// t10: pollen and nectar both shootable: 40 fixed poses (5x8 grid, heading at the blue up-cell aim). Measured 2026-11-07
// by node probe: auto allowed 9/40 (same set for both kinds), sim captures pollen 17/40, nectar 15/40. Bands sit under these. Not failing by design.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld, launchScaleFor } from '../web/js/balls.js';
import * as SR from '../web/js/shotrules.js';
const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const scene = JSON.parse(rd('../web/data/scene.json')), field = JSON.parse(rd('../web/data/colliders.json'));
const root = {}; vm.runInNewContext(rd('../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const DIMS = { halfX: 9, halfY: 9, height: 18 }, TYPE = { pollen: 'pollen', nectar: 'nectar_red' }, hiveTall = field.boxes.filter(b => b.kind === 'hive_frame').map(b => ({ ...b, zMax: 80 }));
const up = Object.values(scene.constants.hives.blue.states[scene.groups.hive_blue.cad_pose_state].cells).find(c => c.up); const tg = { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal };
const env = { bounds: field.bounds, boxes: field.boxes, circles: field.circles, hives: {} };
for (const a of ['red', 'blue']) { const g = scene.groups['hive_' + a]; env.hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: scene.constants.hives[a].states[g.cad_pose_state].cells }; }
const POSES = [];
for (const x of [20, 45, 70, 95, 120]) for (const y of [10, 28, 46, 64, 82, 100, 118, 136]) POSES.push({ x, y, h: Math.atan2(tg.y - y, tg.x - x), vx: 0, vy: 0, om: 0 });
const solveKind = (pose, kind) => SR.solveAutoShot(pose, SH, tg, kind, Core);  // auto ctx: turret/hood/rpm on the setpoint
const ctxOf = (pose, kind, s) => ({ mode: 'auto', pose, shooter: SH, target: tg, kind, turretDeg: s.turretDeg, hoodDeg: s.elev * 180 / Math.PI, rpm: s.rpm, obstacles: field, hiveBoxes: hiveTall, alliance: 'blue', strict: true, ammo: { limited: false, count: 5 } });
function simulate(pose, kind, ctx) {  // launch from ctx, then 60 Hz BallWorld for up to 4 s
  const f = SR.launchBall(ctx), w = new BallWorld(env);
  w.reset([]); w.setRobot(pose, DIMS);
  const b = w.addBall(TYPE[kind], f.pos, f.vel, kind); b.shot = true;
  for (let i = 0; i < 240 && b.state === 'free'; i++) { w.setRobot(pose, DIMS); w.step(1 / 60); }
  return b;
}
test('auto: finite launch, equal-rpm speed ratio, identical allowed set for both kinds (measured 9 of 40)', () => {
  let allowed = 0;
  for (const pose of POSES) {
    const sp = solveKind(pose, 'pollen'), sn = solveKind(pose, 'nectar');
    assert.equal(sp.ok, sn.ok, 'same exit speed, so same solvability');
    const args = { pose, shooter: SH, target: tg, turretDeg: 0, hoodDeg: 45, rpm: 2500 };
    const p = SR.launchBall({ ...args, kind: 'pollen' }), n = SR.launchBall({ ...args, kind: 'nectar' });
    assert.ok(Math.abs(n.speed / p.speed - launchScaleFor('nectar')) < 1e-12, 'equal rpm ratio ' + n.speed / p.speed);
    if (!sp.ok) continue;
    for (const [kind, s] of [['pollen', sp], ['nectar', sn]]) {
      const l = SR.launchBall(ctxOf(pose, kind, s));
      assert.ok(l.vel.every(Number.isFinite) && Number.isFinite(l.speed) && l.speed > 0, `${kind} launch at ${pose.x},${pose.y}`);
    }
    assert.ok(Math.abs(sn.rpm / sp.rpm - 1 / launchScaleFor('nectar')) < 1e-9, 'solved rpm ratio');
    const ap = SR.evaluateShot(ctxOf(pose, 'pollen', sp)).allowed, an = SR.evaluateShot(ctxOf(pose, 'nectar', sn)).allowed;
    assert.equal(an, ap, `allowed differs at ${pose.x},${pose.y}`);
    if (ap) allowed++;
  }
  assert.ok(allowed >= 8 && allowed <= 10, 'allowed count ' + allowed + ' (measured 9)');
});
test('forward sim: every allowed shot scores in the blue hive; capture counts within measured bands', () => {
  const cap = { pollen: 0, nectar: 0 };
  for (const pose of POSES) for (const kind of ['pollen', 'nectar']) { const s = solveKind(pose, kind); if (!s.ok) continue;
    const ctx = ctxOf(pose, kind, s), b = simulate(pose, kind, ctx);
    if (b.state === 'scored') { assert.equal(b.hive, 'blue', kind + ' hive'); cap[kind]++; }
    if (SR.evaluateShot(ctx).allowed) assert.equal(b.state, 'scored', `${kind} allowed at ${pose.x},${pose.y} but ${b.state}`);
  }
  assert.ok(cap.pollen >= 15, 'pollen captures ' + cap.pollen + ' (measured 17)');
  assert.ok(cap.nectar >= 13, 'nectar captures ' + cap.nectar + ' (measured 15)');
});
