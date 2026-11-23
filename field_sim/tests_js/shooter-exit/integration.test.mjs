// Integration of the shooter-exit physics into shotrules.js (physical exit model, dispersion, noise, latency lead).
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { launchScaleFor } from '../../web/js/balls.js';
import * as SR from '../../web/js/shotrules.js';
import { predictPose } from '../../web/js/physics/shooter-exit/lead.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const root = {}; vm.runInNewContext(rd('../../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180, exitModel: 'physical' };
const TG = { x: 100, y: 72, z: 34, normal: [-1, 0, 0], halfW: 10, halfH: 7 };
const DEG = 180 / Math.PI;
const poseAt = (d, dy = 0) => ({ x: TG.x - d, y: TG.y + dy, h: 0, vx: 0, vy: 0, om: 0 });
const POSES5 = [poseAt(50), poseAt(60, 6), poseAt(70, -8), poseAt(80, 10), poseAt(90, -4)];
const setpoint = (pose, kind, sh = SH) => { const s = SR.solveAutoShot(pose, sh, TG, kind, Core); assert.ok(s.ok); return s; };
const assistCtx = (pose, kind, s, extra = {}) => ({ mode: 'assist', pose, shooter: SH, target: TG, kind, rpm: s.rpm, turretDeg: s.turretDeg, hoodDeg: s.hoodDeg, ...extra });

test('physical pollen auto solution lands in the opening for 5 stationary poses', () => {
  for (const pose of POSES5) {
    const s = setpoint(pose, 'pollen');
    assert.ok(s.exitOffset, 'physical solve reports the exit offset');
    assert.ok(s.hoodDeg >= SH.hoodMin - 1e-6 && s.hoodDeg <= SH.hoodMax + 1e-6);
    const ev = SR.evaluateShot(assistCtx(pose, 'pollen', s));
    assert.equal(ev.forward.lands, true, JSON.stringify(ev.forward.miss));
    assert.ok(ev.forward.spinRadS === undefined);   // spin is reported by forwardShot, not the landing record
    const f = SR.forwardShot(assistCtx(pose, 'pollen', s));
    assert.ok(f.spinRadS > 0 && f.compression > 0 && f.contactTimeS > 0 && f.sigma.speedFrac > 0);
    const auto = SR.evaluateShot({ ...assistCtx(pose, 'pollen', s), mode: 'auto' });
    assert.ok(!auto.reasons.some((r) => ['HOOD_NOT_READY', 'TURRET_NOT_READY', 'FLYWHEEL_NOT_READY'].includes(r.code)), 'auto accepts its own setpoint');
  }
});

test('physical nectar rpm / pollen rpm differs from the legacy 1/launchScaleFor', () => {
  const p = poseAt(70);
  const sp = setpoint(p, 'pollen'), sn = setpoint(p, 'nectar');
  const ratio = sn.rpm / sp.rpm, legacy = 1 / launchScaleFor('nectar');
  assert.ok(Math.abs(ratio - legacy) > 1e-4, `ratio ${ratio} vs legacy ${legacy}`);
  // and the nectar setpoint still lands
  assert.equal(SR.evaluateShot(assistCtx(p, 'nectar', sn)).forward.lands, true);
  // pollen equals the legacy rpm (calibrated to the legacy efficiency)
  const lp = SR.solveAutoShot(p, { ...SH, exitModel: undefined }, TG, 'pollen', Core);
  assert.ok(Math.abs(sp.rpm / lp.rpm - 1) < 0.02);
});

// Turret offset (deg) at which the nominal lateral miss is a fraction of the half-width: a marginal shot.
function marginalCtx(frac = 1.0) {
  const pose = poseAt(70), s = setpoint(pose, 'pollen');
  const lat = (dt) => SR.evaluateShot(assistCtx(pose, 'pollen', s, { turretDeg: s.turretDeg + dt })).forward.miss[0];
  const hw = TG.halfW - 1.391 * 0 - SR.forwardShot(assistCtx(pose, 'pollen', s)).ballR;
  let lo = 0, hi = 20; const target = frac * hw;
  for (let i = 0; i < 40; i++) { const m = 0.5 * (lo + hi); if (Math.abs(lat(m)) < target) lo = m; else hi = m; }
  return assistCtx(pose, 'pollen', s, { turretDeg: s.turretDeg + 0.5 * (lo + hi) });
}

test('hitProb: in (0,1) for a marginal shot, ~1 for a centered short shot; MC agrees with analytic', () => {
  const ctx = marginalCtx(1.0);
  const ev = SR.evaluateShot({ ...ctx, dispersion: true });
  assert.equal(ev.forward.method, 'analytic');
  assert.ok(ev.forward.hitProb > 0.2 && ev.forward.hitProb < 0.8, 'marginal p ' + ev.forward.hitProb);
  assert.equal(ev.forward.spread.length, 2);
  const hp = SR.hitProbability(ctx, { method: 'both', n: 4000, seed: 7 });
  assert.ok(hp.analytic && hp.mc);
  assert.ok(Math.abs(hp.analytic.p - hp.mc.p) < 0.05, `analytic ${hp.analytic.p} vs mc ${hp.mc.p}`);
  assert.equal(hp.p, hp.analytic.p);
  assert.equal(SR.hitProbability(ctx, { method: 'mc', n: 500, seed: 3 }).method, 'mc');

  const pose = poseAt(34), s = setpoint(pose, 'pollen');
  const centered = SR.evaluateShot({ ...assistCtx(pose, 'pollen', s), dispersion: { method: 'both', n: 2000, seed: 5 } });
  assert.ok(centered.forward.hitProb > 0.97, 'centered p ' + centered.forward.hitProb);
  assert.equal(centered.forward.method, 'both');
  // auto mode reports hitProb on the solver launch
  const auto = SR.evaluateShot({ ...assistCtx(pose, 'pollen', s), mode: 'auto', dispersion: true });
  assert.ok(auto.hitProb > 0.97);
});

test('ctx.noise: seeded, reproducible, persistent rng, hit rate strictly between 0 and 1 at a marginal pose', () => {
  const ctx = marginalCtx(1.0);
  const hits = (seed) => {
    const c = { ...ctx, noise: { seed } };
    let h = 0, last = null;
    for (let i = 0; i < 200; i++) {
      const b = SR.launchBall(c);
      assert.equal(b.noiseApplied, true);
      const cr = Core.planeCrossing(b.pos, b.vel, TG);
      if (cr) {
        const a = cr.p[1] - TG.y, v = cr.p[2] - TG.z;   // opening normal is -x: lateral = y, up = z
        if (Math.abs(a) <= TG.halfW - b.ballR && Math.abs(v) <= TG.halfH - b.ballR) h++;
      }
      last = b;
    }
    return { h, last, rngKept: typeof c.noise.rng === 'function' };
  };
  const a = hits(42), b = hits(42), c = hits(43);
  assert.deepEqual(a.last.vel, b.last.vel);
  assert.equal(a.h, b.h);
  assert.ok(a.rngKept);
  assert.ok(a.h > 0 && a.h < 200, 'hits ' + a.h);
  assert.notDeepEqual(a.last.vel, c.last.vel);
  assert.equal(SR.launchBall(ctx).noiseApplied, false);
});

test('ctx.leadS changes the turret solution for a moving robot and lands when fired from the predicted pose', () => {
  const pose = { x: 30, y: 60, h: 0.1, vx: 40, vy: 20, om: 0.8 };
  const s0 = SR.solveAutoShot(pose, SH, TG, 'pollen', Core), sL = SR.solveAutoShot(pose, SH, TG, 'pollen', Core, undefined, 0.15);
  assert.ok(s0.ok && sL.ok);
  assert.ok(Math.abs(sL.turretDeg - s0.turretDeg) > 0.5, `${sL.turretDeg} vs ${s0.turretDeg}`);
  // evaluateShot picks up ctx.leadS in auto mode
  const evL = SR.evaluateShot({ mode: 'auto', pose, shooter: SH, target: TG, kind: 'pollen', rpm: sL.rpm, turretDeg: sL.turretDeg, hoodDeg: sL.hoodDeg, leadS: 0.15 });
  assert.ok(!evL.reasons.some((r) => r.code === 'TURRET_NOT_READY'), 'lead-solved turret accepted');
  const ev0 = SR.evaluateShot({ mode: 'auto', pose, shooter: SH, target: TG, kind: 'pollen', rpm: sL.rpm, turretDeg: sL.turretDeg, hoodDeg: sL.hoodDeg });
  assert.ok(ev0.reasons.some((r) => r.code === 'TURRET_NOT_READY'), 'without lead the same turret is off target');
  // ball released at the predicted pose with the solved setpoint lands
  const pp = predictPose(pose, pose, 0.15);
  const rel = { x: pp.x, y: pp.y, h: pp.h, vx: pp.vx, vy: pp.vy, om: pp.om };
  const ev = SR.evaluateShot({ mode: 'assist', pose: rel, shooter: SH, target: TG, kind: 'pollen', rpm: sL.rpm, turretDeg: sL.turretDeg, hoodDeg: sL.hoodDeg });
  assert.equal(ev.forward.lands, true, JSON.stringify(ev.forward.miss));
});

test('LOW_HIT_PROB: raised below ctx.minHitProb, a warning when not strict, a block when strict', () => {
  const ctx = marginalCtx(1.0);
  const soft = SR.evaluateShot({ ...ctx, dispersion: true, minHitProb: 0.99 });
  const r = soft.reasons.find((x) => x.code === SR.REASON.LOW_HIT_PROB);
  assert.ok(r); assert.equal(r.severity, 'warn'); assert.equal(r.wouldBlock, true);
  const strict = SR.evaluateShot({ ...ctx, dispersion: true, minHitProb: 0.99, strict: true });
  assert.equal(strict.reasons.find((x) => x.code === SR.REASON.LOW_HIT_PROB).severity, 'block');
  assert.equal(strict.allowed, false);
  const ok = SR.evaluateShot({ ...ctx, dispersion: true, minHitProb: 0.01 });
  assert.ok(!ok.reasons.some((x) => x.code === SR.REASON.LOW_HIT_PROB));
  // minHitProb alone (no ctx.dispersion) still evaluates but does not add forward.hitProb
  const only = SR.evaluateShot({ ...ctx, minHitProb: 0.99 });
  assert.ok(only.reasons.some((x) => x.code === SR.REASON.LOW_HIT_PROB));
  assert.equal(only.forward.hitProb, undefined);
});
