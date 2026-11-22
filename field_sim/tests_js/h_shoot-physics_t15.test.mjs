// t15 evaluateShot mode matrix. Expectations from the web/js/shotrules.js header: strict=true follows MODES;
// strict falsy blocks only NO_AMMO, ILLEGAL_ZONE, WRONG_MODE_PARAM. FAILING-BY-DESIGN cases: none (56 cells agree).
// Blocked-path: far wall for auto/assist; wall 5 in from exit for manual (manual blocks only within EXIT_BLOCK_IN).
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import * as SR from '../web/js/shotrules.js';

const root = {}; vm.runInNewContext(readFileSync(new URL('../web/js/shot_core.js', import.meta.url), 'utf8'), { window: root });
const Core = root.Core; SR.setCore(Core);
const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8')), obstacles = JSON.parse(readFileSync(new URL('../web/data/colliders.json', import.meta.url), 'utf8'));
const cell = Object.values(scene.constants.hives.blue.states.audience_up.cells).find(c => c.up);
const target = { x: cell.aim_point[0], y: cell.aim_point[1], z: cell.aim_point[2], normal: cell.opening_normal };
const SH = { exitHeight: 12, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65,
  pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const rad = d => d * Math.PI / 180, deg = r => r * 180 / Math.PI;

/** Ready ctx: mechanisms on the solver setpoint (construction as in shotrules.test.mjs). */
function readyCtx(x, y) {
  const h = Math.atan2(target.y - y, target.x - x) + 0.4;
  const p = { x, y, psi: h, vx: 0, vy: 0, om: 0, rx: 0, ry: 0, z0: SH.exitHeight, X: target.x, Y: target.y, Z: target.z };
  let r = Core.solve(p, 'gamma', rad(SH.entryDeg), 'low');
  const c = Math.min(SH.hoodMax, Math.max(SH.hoodMin, deg(r.elev)));
  if (r.ok && c !== deg(r.elev)) { const r2 = Core.solve(p, 'theta', rad(c), 'low'); if (r2.ok) r = r2; }
  return { sol: r, pose: { x, y, h, vx: 0, vy: 0, om: 0 }, turretDeg: deg(r.turret), hoodDeg: deg(r.elev),
    rpm: Core.rpmEstimate(r.speed, SH.wheelR, SH.eff), target, shooter: SH, ammo: { limited: false, count: 5 }, obstacles, alliance: 'blue' };
}
const nearWall = [{ minX: 105, minY: 70, maxX: 120, maxY: 130, zMin: 0, zMax: 100, kind: 'wall_test' }], farWall = [{ minX: 100, minY: 20, maxX: 130, maxY: 40, zMin: 0, zMax: 100, kind: 'test_wall' }];
const zone = { contains: () => false, describe: () => 'nowhere' };
const CONDS = {
  ready: c => c,
  noAmmo: c => ({ ...c, ammo: { limited: true, count: 0 } }),
  flywheelLow: c => ({ ...c, rpm: 100 }),
  rpmNotReady: c => ({ ...c, rpm: c.rpm * 0.8 }),
  blockedPath: c => c.mode === 'manual'
    ? { ...c, pose: { x: 100, y: 100, h: 0, vx: 0, vy: 0, om: 0 }, turretDeg: 0, hoodDeg: 45, rpm: 3000, hiveBoxes: nearWall }
    : { ...c, hiveBoxes: farWall },
  illegalZone: c => ({ ...c, legalZone: zone }),
  outOfField: c => ({ ...c, pose: { ...c.pose, x: -5 } }),
};
// expected allowed (1/0) per mode, order [auto, assist, manual, free]
const STRICT = {
  ready: [1, 1, 1, 1], noAmmo: [0, 0, 0, 0], flywheelLow: [0, 0, 0, 1], rpmNotReady: [0, 1, 1, 1],
  blockedPath: [0, 0, 0, 1], illegalZone: [0, 0, 0, 1], outOfField: [0, 0, 0, 1] };
const LOOSE = {
  ready: [1, 1, 1, 1], noAmmo: [0, 0, 0, 0], flywheelLow: [1, 1, 1, 1], rpmNotReady: [1, 1, 1, 1],
  blockedPath: [1, 1, 1, 1], illegalZone: [0, 0, 0, 1], outOfField: [1, 1, 1, 1] };
const MODES = ['auto', 'assist', 'manual', 'free'], GOOD = [130, 10];

for (const [label, table, strict] of [['strict', STRICT, true], ['physics-only', LOOSE, undefined]]) {
  test(`mode matrix (${label}): allowed/blocked per header`, () => {
    for (const [cond, want] of Object.entries(table)) MODES.forEach((mode, i) => {
      const ev = SR.evaluateShot({ ...CONDS[cond]({ ...readyCtx(...GOOD), mode }), strict });
      assert.equal(ev.allowed, want[i] === 1, `${cond}/${mode}/${label}: ${JSON.stringify(ev.reasons)}`);
    });
  });
}
