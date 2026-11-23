// Legacy regression: with shooter.exitModel absent and ctx.noise / ctx.dispersion / ctx.leadS absent, shotrules must return
// bit-identical numbers to the ORIGINAL formulas. The reference below is an inline copy of the old speedFromRpm / forwardShot /
// solveAuto (copied from shotrules.js before the shooter-exit integration).
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { launchScaleFor, BALL_KINDS } from '../../web/js/balls.js';
import * as SR from '../../web/js/shotrules.js';
import { rimClearanceIn } from '../../web/js/aimcontrol.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const root = {}; vm.runInNewContext(rd('../../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 2, pivotY: -1, turretMinDeg: -180, turretMaxDeg: 180 };
const TG = { x: 100, y: 72, z: 34, normal: [-1, 0, 0] };
const DEG = 180 / Math.PI, RAD = Math.PI / 180, G = 386.0886;

// ---- reference (OLD code, verbatim formulas)
const refSpeedFromRpm = (rpm, sh, massScale) => Math.max(0, rpm) / (sh.rpmScale || 1) * 2 * Math.PI * sh.wheelR * sh.eff / 60 * (massScale == null ? 1 : massScale);
const refRpmFromSpeed = (speed, sh, core, massScale) => core.rpmEstimate(speed / (massScale == null ? 1 : massScale), sh.wheelR, sh.eff) * (sh.rpmScale || 1);
function refForward(ctx) {
  const { pose, shooter: sh } = ctx, kind = ctx.kind || 'pollen', ms = launchScaleFor(kind);
  const az = pose.h + (ctx.turretDeg || 0) * RAD, el = (ctx.hoodDeg || 0) * RAD;
  const speed = refSpeedFromRpm(ctx.rpm || 0, sh, ms);
  const c = Math.cos(pose.h), s = Math.sin(pose.h), px0 = sh.pivotX || 0, py0 = sh.pivotY || 0;
  const px = pose.x + c * px0 - s * py0, py = pose.y + s * px0 + c * py0;
  const rv = ctx.robotVel || pose;
  const vx0 = (rv.vx || 0) - (rv.om || 0) * (py - pose.y), vy0 = (rv.vy || 0) + (rv.om || 0) * (px - pose.x);
  const vel = [vx0 + Math.cos(az) * Math.cos(el) * speed, vy0 + Math.sin(az) * Math.cos(el) * speed, Math.sin(el) * speed];
  return { speed, elevRad: el, azRad: az, vel, exit: [px, py, sh.exitHeight], kind, ballR: rimClearanceIn(kind), massScale: ms };
}
function refSolve(pose, sh, tg, core, kind, robotVel) {
  const rv = robotVel || pose;
  const p = { x: pose.x, y: pose.y, psi: pose.h, vx: rv.vx || 0, vy: rv.vy || 0, om: rv.om || 0, rx: sh.pivotX || 0, ry: sh.pivotY || 0, z0: sh.exitHeight, X: tg.x, Y: tg.y, Z: tg.z };
  let r = core.solve(p, 'gamma', (sh.entryDeg == null ? 50 : sh.entryDeg) * RAD, 'low');
  if (r.ok) { const e = r.elev * DEG, c = Math.min(sh.hoodMax, Math.max(sh.hoodMin, e)); if (c !== e) { const r2 = core.solve(p, 'theta', c * RAD, 'low'); if (r2.ok) r = r2; } }
  r.rpm = r.ok ? refRpmFromSpeed(r.speed, sh, core, launchScaleFor(kind)) : 0;
  r.turretDeg = r.ok ? r.turret * DEG : 0;
  return r;
}

// 20 poses at assorted distances/angles in front of the opening (-x side normal is [-1,0,0], front is -x).
const POSES = Array.from({ length: 20 }, (_, i) => {
  const d = 36 + i * 3.2, a = (i % 7 - 3) * 0.12;
  const x = TG.x - d * Math.cos(a), y = TG.y + d * Math.sin(a);
  return { x, y, h: Math.atan2(TG.y - y, TG.x - x) + (i % 3 - 1) * 0.1, vx: (i % 4) * 6 - 9, vy: (i % 5) * 4 - 8, om: (i % 3 - 1) * 0.6 };
});
const KINDS = ['pollen', 'nectar'];
const sameBall = (a, b) => { for (const k of ['speed', 'elevRad', 'azRad', 'kind', 'ballR']) assert.equal(a[k], b[k], k); assert.deepEqual(a.vel, b.vel); assert.deepEqual(a.exit ?? a.pos, b.exit); };

test('exitModelOf is legacy unless exitModel === physical', () => {
  assert.equal(SR.exitModelOf(SH), 'legacy');
  assert.equal(SR.exitModelOf({ ...SH, exitModel: 'bogus' }), 'legacy');
  assert.equal(SR.exitModelOf({ ...SH, exitModel: 'physical' }), 'physical');
  assert.equal(SR.REASON.LOW_HIT_PROB, 'LOW_HIT_PROB');
});

test('legacy forwardShot / launchBall are bit-identical to the old formulas (20 poses x 2 kinds x 3 modes)', () => {
  let n = 0;
  for (const pose of POSES) for (const kind of KINDS) for (const mode of ['auto', 'assist', 'manual']) {
    const sol = refSolve(pose, SH, TG, Core, kind);
    const ctx = { mode, pose, shooter: SH, target: TG, kind, rpm: sol.ok ? sol.rpm : 3000, turretDeg: sol.ok ? sol.turretDeg : 0, hoodDeg: sol.ok ? sol.elev * DEG : 45 };
    const ref = refForward(ctx);
    sameBall(SR.forwardShot(ctx), ref);
    const lb = SR.launchBall(ctx);
    sameBall(lb, ref);
    assert.equal(lb.spinRadS, undefined);
    assert.equal(lb.noiseApplied, false);
    // the explicit robotVel override path
    const ctx2 = { ...ctx, robotVel: { vx: 5, vy: -3, om: 0.4 } };
    sameBall(SR.launchBall(ctx2), refForward(ctx2));
    n++;
  }
  assert.equal(n, 120);
});

test('legacy solveAutoShot equals the old solver and hoodDeg === elev*DEG', () => {
  for (const pose of POSES) for (const kind of KINDS) {
    const a = SR.solveAutoShot(pose, SH, TG, kind, Core), b = refSolve(pose, SH, TG, Core, kind);
    assert.equal(a.ok, b.ok);
    if (!a.ok) continue;
    for (const k of ['speed', 'elev', 'az', 'turret', 'rpm', 'turretDeg', 'T']) assert.equal(a[k], b[k], k);
    assert.equal(a.hoodDeg, a.elev * DEG);
    assert.equal(a.exitOffset, undefined);
    const rv = { vx: 7, vy: 2, om: 0.3 };
    const a2 = SR.solveAutoShot(pose, SH, TG, kind, Core, rv), b2 = refSolve(pose, SH, TG, Core, kind, rv);
    assert.equal(a2.rpm, b2.rpm); assert.equal(a2.turretDeg, b2.turretDeg);
  }
});

test('legacy evaluateShot: no dispersion fields, forward crossing matches the old launch, auto accepts the old setpoint', () => {
  for (const pose of POSES) for (const kind of KINDS) {
    const sol = refSolve(pose, SH, TG, Core, kind);
    if (!sol.ok) continue;
    for (const mode of ['auto', 'assist', 'manual']) {
      const ctx = { mode, pose, shooter: SH, target: TG, kind, rpm: sol.rpm, turretDeg: sol.turretDeg, hoodDeg: sol.elev * DEG };
      const ev = SR.evaluateShot(ctx);
      assert.equal(ev.hitProb, undefined);
      assert.ok(!ev.reasons.some((r) => r.code === 'LOW_HIT_PROB'));
      if (mode === 'auto') {
        assert.ok(!ev.reasons.some((r) => ['HOOD_NOT_READY', 'TURRET_NOT_READY', 'FLYWHEEL_NOT_READY'].includes(r.code)), 'auto accepts the solver setpoint');
        assert.equal(ev.forward, undefined);
      } else {
        assert.equal(ev.forward.hitProb, undefined); assert.equal(ev.forward.method, undefined);
        const ref = refForward(ctx), cr = Core.planeCrossing(ref.exit, ref.vel, TG);
        if (cr && !ev.forward.blocked && ev.forward.cross) for (let i = 0; i < 3; i++) assert.ok(Math.abs(ev.forward.cross[i] - cr.p[i]) < 0.3, 'crossing ' + i);
      }
    }
  }
});

test('ballkinds expectations: nectar/pollen speed ratio = launchScaleFor(nectar), rpm ratio = 1/scale', () => {
  const base = { mode: 'manual', pose: POSES[3], shooter: SH, target: TG, rpm: 3000, turretDeg: 5, hoodDeg: 45 };
  const sp = SR.launchBall({ ...base, kind: 'pollen' }).speed, sn = SR.launchBall({ ...base, kind: 'nectar' }).speed;
  assert.equal(sn / sp, launchScaleFor('nectar'));
  assert.ok(BALL_KINDS.nectar);
  const ap = SR.solveAutoShot(POSES[3], SH, TG, 'pollen', Core), an = SR.solveAutoShot(POSES[3], SH, TG, 'nectar', Core);
  assert.ok(Math.abs(an.rpm / ap.rpm - 1 / launchScaleFor('nectar')) < 1e-9);
});

test('legacy range estimates unchanged by the rangeKey extension', () => {
  const a = SR.maxRangeEstimate(SH, TG, Core, 'pollen'), b = SR.maxRangeEstimate({ ...SH }, TG, Core, 'pollen');
  assert.equal(a, b);
  assert.ok(a > 50);
  const c = SR.maxRangeEstimate({ ...SH, exitModel: 'physical' }, TG, Core, 'pollen');
  assert.ok(Number.isFinite(c));
});
