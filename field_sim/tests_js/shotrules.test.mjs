import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import * as SR from '../web/js/shotrules.js';

const { evaluateShot, trajectoryBlocked, forwardShot, feasibleRegion, maxRangeEstimate, nearestRangeEstimate, REASON, SHOOT_MODES } = SR;

const root = {};
vm.runInNewContext(readFileSync(new URL('../web/js/shot_core.js', import.meta.url), 'utf8'), { window: root });
const Core = root.Core;
SR.setCore(Core);

const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const obstacles = JSON.parse(readFileSync(new URL('../web/data/colliders.json', import.meta.url), 'utf8'));
const cell = Object.values(scene.constants.hives.blue.states.audience_up.cells).find(c => c.up);
const target = { x: cell.aim_point[0], y: cell.aim_point[1], z: cell.aim_point[2], normal: cell.opening_normal };
const SH = { exitHeight: 12, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65,
  pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const rad = d => d * Math.PI / 180;

function aimedPose(x, y) { return { x, y, h: Math.atan2(target.y - y, target.x - x) + 0.4, vx: 0, vy: 0, om: 0 }; }
/** ctx with mechanisms sitting exactly on the solver setpoint. */
function readyCtx(x, y, extra = {}) {
  const pose = extra.pose || aimedPose(x, y);
  const sh = extra.shooter || SH;
  const p = { x: pose.x, y: pose.y, psi: pose.h, vx: pose.vx, vy: pose.vy, om: pose.om, rx: 0, ry: 0, z0: sh.exitHeight, X: target.x, Y: target.y, Z: target.z };
  let r = Core.solve(p, 'gamma', rad(sh.entryDeg), 'low');
  if (r.ok) { const e = r.elev * 180 / Math.PI, c = Math.min(sh.hoodMax, Math.max(sh.hoodMin, e)); if (c !== e) { const r2 = Core.solve(p, 'theta', rad(c), 'low'); if (r2.ok) r = r2; } }
  const rpm = r.ok ? Core.rpmEstimate(r.speed, sh.wheelR, sh.eff) * sh.rpmScale : 0;
  return { sol: r, mode: 'auto', pose, turretDeg: r.ok ? r.turret * 180 / Math.PI : 0, hoodDeg: r.ok ? r.elev * 180 / Math.PI : 45, rpm,
    target, shooter: sh, ammo: { limited: false, count: 5 }, obstacles, alliance: 'blue', strict: true, ...extra, pose };
}
const codes = ev => ev.reasons.filter(r => r.severity === 'block').map(r => r.code);
const GOOD = [130, 10];

test('exports and constants', () => {
  assert.deepEqual(SHOOT_MODES, ['auto', 'assist', 'manual', 'free']);
  for (const k of ['NO_SOLUTION', 'OUT_OF_RANGE_FAR', 'OUT_OF_RANGE_NEAR', 'RPM_TOO_HIGH', 'HOOD_LIMIT', 'TURRET_LIMIT', 'BEHIND_OPENING', 'TRAJECTORY_BLOCKED',
    'FLYWHEEL_NOT_READY', 'TURRET_NOT_READY', 'HOOD_NOT_READY', 'NO_AMMO', 'ILLEGAL_ZONE', 'OUT_OF_FIELD', 'WRONG_MODE_PARAM']) assert.equal(REASON[k], k);
});

test('auto: allowed from a good spot with mechanisms on setpoint', () => {
  const ev = evaluateShot(readyCtx(...GOOD));
  assert.equal(ev.allowed, true, JSON.stringify(ev));
  assert.equal(ev.summary, 'READY');
  assert.equal(ev.hint, null);
});

test('auto: accepts a precomputed Core solve result as ctx.sol', () => {
  const c = readyCtx(...GOOD);
  assert.ok(c.sol.ok);
  assert.equal(evaluateShot(c).allowed, true);
  assert.equal(evaluateShot({ ...c, sol: undefined }).allowed, true);
});

test('auto: far away gives OUT_OF_RANGE_FAR with a hint', () => {
  const sh = { ...SH, maxRpm: 2400 };
  const far = maxRangeEstimate(sh, target);
  assert.ok(far < 60 && far > 43, 'far=' + far);
  const ev = evaluateShot(readyCtx(...GOOD, { shooter: sh }));
  assert.ok(codes(ev).includes(REASON.OUT_OF_RANGE_FAR), JSON.stringify(ev.reasons));
  assert.equal(ev.allowed, false);
  assert.match(ev.hint, /closer than \d+ in/);
});

test('auto: too close gives OUT_OF_RANGE_NEAR', () => {
  const ev = evaluateShot(readyCtx(84.8, 40));
  assert.ok(codes(ev).includes(REASON.OUT_OF_RANGE_NEAR), JSON.stringify(ev.reasons));
  assert.match(ev.hint, /at least \d+ in/);
});

test('auto: blocked from behind the opening', () => {
  const ev = evaluateShot(readyCtx(110, 130));
  // the blue hive is audience-up here: standing on the far side puts the ball behind the opening
  assert.ok(codes(ev).includes(REASON.BEHIND_OPENING), JSON.stringify(ev.reasons));
  assert.equal(ev.allowed, false);
});

test('auto: rpm needed above maxRpm -> RPM_TOO_HIGH', () => {
  const sh = { ...SH, maxRpm: 1800 };
  const ev = evaluateShot(readyCtx(...GOOD, { shooter: sh }));
  assert.ok(codes(ev).includes(REASON.RPM_TOO_HIGH), JSON.stringify(ev.reasons));
});

test('auto: hood limit when the required elevation is out of range', () => {
  const sh = { ...SH, hoodMin: 25, hoodMax: 40 };
  const ev = evaluateShot(readyCtx(...GOOD, { shooter: sh }));
  assert.ok(codes(ev).includes(REASON.HOOD_LIMIT) || codes(ev).includes(REASON.NO_SOLUTION), JSON.stringify(ev.reasons));
  assert.equal(ev.allowed, false);
});

test('auto: turret travel limit', () => {
  const pose = { x: GOOD[0], y: GOOD[1], h: Math.atan2(target.y - GOOD[1], target.x - GOOD[0]) + Math.PI, vx: 0, vy: 0, om: 0 };  // facing away
  const sh = { ...SH, turretMinDeg: -90, turretMaxDeg: 90 };
  const ev = evaluateShot(readyCtx(0, 0, { pose, shooter: sh }));
  assert.ok(codes(ev).includes(REASON.TURRET_LIMIT), JSON.stringify(ev.reasons));
  assert.equal(evaluateShot(readyCtx(0, 0, { pose })).allowed, true);   // +-180 reaches everything
});

test('trajectoryBlocked: hive leg box stops a flat ball, open path passes', () => {
  const leg = obstacles.boxes.find(b => b.kind === 'hive_frame' && b.zMax === 2.15);
  const y = (leg.minY + leg.maxY) / 2;
  const hit = trajectoryBlocked([leg.minX - 10, y, 3], [300, 0, 0], obstacles, null, { ignoreStart: 4 });
  assert.equal(hit.blocked, true);
  assert.equal(hit.by, 'hive_frame');
  assert.ok(hit.at[0] < leg.minX + 1);
  const hiPass = trajectoryBlocked([leg.minX - 30, y, 12], [100, 0, 150], obstacles, null, {});
  assert.equal(hiPass.blocked, false);
  const wall = trajectoryBlocked([20, 20, 12], [-300, 0, 100], obstacles, null, {});
  assert.deepEqual([wall.blocked, wall.by], [true, 'wall']);
});

test('auto: ball path through a collider box -> TRAJECTORY_BLOCKED', () => {
  const extra = [{ minX: 100, minY: 20, maxX: 130, maxY: 40, zMin: 0, zMax: 100, kind: 'test_wall' }];
  const ev = evaluateShot(readyCtx(...GOOD, { hiveBoxes: extra }));
  assert.ok(codes(ev).includes(REASON.TRAJECTORY_BLOCKED), JSON.stringify(ev.reasons));
  assert.equal(evaluateShot(readyCtx(...GOOD)).allowed, true);
});

test('auto: flywheel / turret / hood not ready', () => {
  const c = readyCtx(...GOOD);
  let ev = evaluateShot({ ...c, rpm: c.rpm * 0.8 });
  assert.deepEqual(codes(ev), [REASON.FLYWHEEL_NOT_READY]);
  assert.equal(evaluateShot({ ...c, rpm: c.rpm * 1.03 }).allowed, true);       // inside the 4% tolerance
  ev = evaluateShot({ ...c, turretDeg: c.turretDeg + 6 });
  assert.deepEqual(codes(ev), [REASON.TURRET_NOT_READY]);
  ev = evaluateShot({ ...c, hoodDeg: c.hoodDeg - 8 });
  assert.deepEqual(codes(ev), [REASON.HOOD_NOT_READY]);
  assert.ok(codes(evaluateShot({ ...c, rpm: 0 })).includes(REASON.FLYWHEEL_NOT_READY));
});

test('no ammo blocks only when ammo is limited, in every non-invalid mode', () => {
  const c = readyCtx(...GOOD);
  assert.equal(evaluateShot({ ...c, ammo: { limited: false, count: 0 } }).allowed, true);
  for (const mode of SHOOT_MODES) {
    const ev = evaluateShot({ ...c, mode, ammo: { limited: true, count: 0 } });
    assert.ok(codes(ev).includes(REASON.NO_AMMO), mode);
    assert.equal(ev.allowed, false);
  }
  assert.equal(evaluateShot({ ...c, ammo: { limited: true, count: 3 } }).allowed, true);
});

test('wrong mode is rejected', () => {
  const ev = evaluateShot({ ...readyCtx(...GOOD), mode: 'turbo' });
  assert.deepEqual(codes(ev), [REASON.WRONG_MODE_PARAM]);
});

test('free mode allows almost everything (out of field, nonsense setpoints, behind, zone)', () => {
  const c = readyCtx(110, 130);
  assert.equal(evaluateShot({ ...c, mode: 'free', rpm: 0, hoodDeg: 90 }).allowed, true);
  assert.equal(evaluateShot({ ...c, mode: 'free', pose: { ...c.pose, x: -20, y: 400 } }).allowed, true);
  const zone = { contains: () => false, describe: () => 'nowhere' };
  assert.equal(evaluateShot({ ...c, mode: 'free', legalZone: zone }).allowed, true);
});

test('forwardShot matches rpmEstimate and includes robot velocity', () => {
  const c = readyCtx(...GOOD);
  const sol = Core.solve({ x: c.pose.x, y: c.pose.y, psi: c.pose.h, vx: 0, vy: 0, om: 0, rx: 0, ry: 0, z0: 12, X: target.x, Y: target.y, Z: target.z }, 'gamma', rad(50), 'low');
  const f = forwardShot({ ...c, hoodDeg: sol.elev * 180 / Math.PI, rpm: Core.rpmEstimate(sol.speed, SH.wheelR, SH.eff) });
  assert.ok(Math.abs(f.speed - sol.speed) < 1e-9);
  assert.ok(Math.abs(f.elevRad - sol.elev) < 1e-9);
  const f2 = forwardShot({ ...c, pose: { ...c.pose, vx: 20 }, rpm: 3000, shooter: { ...SH, rpmScale: 1.1 } });
  const f3 = forwardShot({ ...c, pose: { ...c.pose, vx: 0 }, rpm: 3000, shooter: { ...SH, rpmScale: 1.1 } });
  assert.ok(Math.abs(f2.vel[0] - f3.vel[0] - 20) < 1e-9);
  assert.deepEqual(f3.exit, [c.pose.x, c.pose.y, 12]);
  assert.ok(Math.abs(Core.rpmEstimate(f3.speed, SH.wheelR, SH.eff) * 1.1 - 3000) < 1e-6);
});

test('manual: forward sim lands in the opening with solver setpoints, misses with a bad hood', () => {
  const c = { ...readyCtx(...GOOD), mode: 'manual' };
  const ev = evaluateShot(c);
  assert.equal(ev.allowed, true);
  assert.equal(ev.forward.lands, true, JSON.stringify(ev.forward));
  const miss = evaluateShot({ ...c, hoodDeg: c.hoodDeg - 12 });
  assert.equal(miss.allowed, true);                   // manual never judges the aim
  assert.equal(miss.forward.lands, false);
});

test('manual: only hard impossibilities block', () => {
  const c = { ...readyCtx(110, 130), mode: 'manual' };            // behind the opening, still allowed
  assert.equal(evaluateShot({ ...c, rpm: 3000 }).allowed, true);
  assert.deepEqual(codes(evaluateShot({ ...c, rpm: 100 })), [REASON.FLYWHEEL_NOT_READY]);
  assert.ok(codes(evaluateShot({ ...c, rpm: 3000, pose: { ...c.pose, x: 200 } })).includes(REASON.OUT_OF_FIELD));
  // a wall 5 in in front of the exit, turret pointed straight at it
  const p = { x: 100, y: 100, h: 0, vx: 0, vy: 0, om: 0 };
  const wall = [{ minX: 105, minY: 70, maxX: 120, maxY: 130, zMin: 0, zMax: 100, kind: 'wall_test' }];
  const ev = evaluateShot({ ...c, pose: p, turretDeg: 0, hoodDeg: 45, rpm: 3000, hiveBoxes: wall });
  assert.ok(codes(ev).includes(REASON.TRAJECTORY_BLOCKED), JSON.stringify(ev.reasons));
  assert.equal(evaluateShot({ ...c, pose: p, turretDeg: 180, hoodDeg: 45, rpm: 3000, hiveBoxes: wall }).allowed, true);
});

test('assist: physics enforced on actual setpoints, forward result is informational', () => {
  const c = { ...readyCtx(...GOOD), mode: 'assist' };
  let ev = evaluateShot(c);
  assert.equal(ev.allowed, true, JSON.stringify(ev.reasons));
  assert.equal(ev.forward.lands, true);
  ev = evaluateShot({ ...c, hoodDeg: c.hoodDeg - 12 });                // misses but allowed
  assert.equal(ev.allowed, true);
  assert.equal(ev.forward.lands, false);
  assert.ok(codes(evaluateShot({ ...c, rpm: 6500 })).includes(REASON.RPM_TOO_HIGH));
  assert.ok(codes(evaluateShot({ ...c, hoodDeg: 80 })).includes(REASON.HOOD_LIMIT));
  assert.ok(codes(evaluateShot({ ...c, turretDeg: 200 })).includes(REASON.TURRET_LIMIT));
  assert.ok(codes(evaluateShot({ ...c, rpm: 50 })).includes(REASON.FLYWHEEL_NOT_READY));
  const near = evaluateShot({ ...readyCtx(84.8, 40), mode: 'assist' });  // solver infeasible here -> blocked even with manual setpoints
  assert.equal(near.allowed, false);
});

test('legal zone is enforced for auto/assist/manual and toggleable; free ignores it', () => {
  const zone = { contains: (x) => x < 100, describe: () => 'x below 100' };
  for (const mode of ['auto', 'assist', 'manual']) {
    const c = { ...readyCtx(...GOOD), mode, legalZone: zone };
    const ev = evaluateShot(c);
    assert.ok(codes(ev).includes(REASON.ILLEGAL_ZONE), mode);
    assert.match(ev.reasons.find(r => r.code === REASON.ILLEGAL_ZONE).text, /x below 100/);
    assert.equal(evaluateShot({ ...c, enforceZone: false }).allowed, true, mode + ' toggled off');
  }
  const inside = readyCtx(90, 5, { legalZone: zone });
  assert.ok(!codes(evaluateShot(inside)).includes(REASON.ILLEGAL_ZONE));
});

test('out of field blocks auto', () => {
  const c = readyCtx(...GOOD);
  assert.ok(codes(evaluateShot({ ...c, pose: { ...c.pose, x: -5 } })).includes(REASON.OUT_OF_FIELD));
});

test('range estimates for the default shooter', () => {
  const far = maxRangeEstimate(SH, target), near = nearestRangeEstimate(SH, target);
  assert.ok(near > 20 && near < 70, 'near=' + near);
  assert.ok(far > 200, 'far=' + far);
  assert.ok(maxRangeEstimate({ ...SH, maxRpm: 3000 }, target) < far);
});

test('feasibleRegion: has ok and not-ok cells, is deterministic, respects zone', () => {
  const o = { shooter: SH, target, obstacles, step: 4 };
  const a = feasibleRegion(o), b = feasibleRegion(o);
  assert.equal(a.step, 4);
  assert.ok(a.cells.some(c => c.ok) && a.cells.some(c => !c.ok));
  assert.deepEqual(a.cells, b.cells);
  const reasons = new Set(a.cells.filter(c => !c.ok).map(c => c.reasonCode));
  assert.ok(reasons.has(REASON.BEHIND_OPENING) && reasons.has(REASON.OUT_OF_RANGE_NEAR));
  for (const c of a.cells) assert.ok(c.ok === (c.reasonCode === null));
  const z = feasibleRegion({ ...o, legalZone: { contains: (x, y) => y < 40 }, alliance: 'blue' });
  assert.ok(z.cells.filter(c => c.ok).every(c => c.y < 40));
  assert.ok(z.cells.some(c => c.reasonCode === REASON.ILLEGAL_ZONE));
  const nz = feasibleRegion({ ...o, legalZone: { contains: () => false }, enforceZone: false });
  assert.deepEqual(nz.cells, a.cells);
});

test('feasibleRegion agrees with evaluateShot on sampled cells', () => {
  const reg = feasibleRegion({ shooter: SH, target, obstacles, step: 6 });
  for (let i = 0; i < reg.cells.length; i += 17) {
    const c = reg.cells[i];
    const ev = evaluateShot(readyCtx(c.x, c.y));
    assert.equal(ev.allowed, c.ok, `${c.x},${c.y} ${c.reasonCode} vs ${JSON.stringify(ev.reasons)}`);
  }
});

test('performance: 48x48 grid under 150 ms', () => {
  const step = 141.348 / 48;
  feasibleRegion({ shooter: SH, target, obstacles, step });           // warm up
  const t0 = performance.now();
  const r = feasibleRegion({ shooter: SH, target, obstacles, step });
  const ms = performance.now() - t0;
  console.log(`48x48 region: ${ms.toFixed(1)} ms, ${r.cells.length} cells`);
  assert.ok(ms < 150, ms + ' ms');
});
