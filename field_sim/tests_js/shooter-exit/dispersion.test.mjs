import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { mulberry32, makeGaussian, normCdf, erf } from '../../web/js/physics/shooter-exit/rng.js';
import { shotSigmas, openingCoords, launchVel, perturbLaunch, analyticHitProbability, monteCarloHit } from '../../web/js/physics/shooter-exit/dispersion.js';

const root = {};
vm.runInNewContext(readFileSync(new URL('../../web/js/shot_core.js', import.meta.url), 'utf8'), { window: root });
const Core = root.ShotCore || root.Core || Object.values(root).find(v => v && v.solve);
const G = 386.0886;
// prefer Core.planeCrossing; local fallback only if absent
const core = Core && Core.planeCrossing ? Core : {
  planeCrossing(P0, v, pl) {
    const n = pl.normal, A = -0.5 * G * n[2], B = n[0] * v[0] + n[1] * v[1] + n[2] * v[2];
    const C = n[0] * (P0[0] - pl.x) + n[1] * (P0[1] - pl.y) + n[2] * (P0[2] - pl.z);
    const roots = Math.abs(A) < 1e-12 ? [-C / B] : (() => { const d = B * B - 4 * A * C; if (d < 0) return []; const s = Math.sqrt(d); return [(-B - s) / (2 * A), (-B + s) / (2 * A)].sort((a, b) => a - b); })();
    for (const t of roots) if (t > 1e-9 && B + 2 * A * t < 0) return { t, p: [P0[0] + v[0] * t, P0[1] + v[1] * t, P0[2] + v[2] * t - 0.5 * G * t * t] };
    return null;
  },
};

const target = { x: 100, y: 0, z: 40, normal: [-1, 0, 0], halfW: 10, halfH: 7 };
const ballR = 2.5, speed = 400;
function launchAt(vOff) {
  const base = { pos: [0, 0, 12], speed, elevRad: 0, azRad: 0, vBase: [0, 0, 0] };
  let lo = 0.05, hi = 1.2;   // bisection on elevation so crossing height offset = vOff
  for (let i = 0; i < 80; i++) {
    const mid = (lo + hi) / 2, c = core.planeCrossing(base.pos, launchVel({ ...base, elevRad: mid }), target);
    const v = c ? c.p[2] - target.z : -1e9;
    if (v < vOff) lo = mid; else hi = mid;
  }
  return { ...base, elevRad: (lo + hi) / 2 };
}
const rectSim = (l) => {
  const c = core.planeCrossing(l.pos, launchVel(l), target);
  if (!c) return { lands: false, miss: null };
  const [a, v] = openingCoords(c.p, target);
  return { lands: Math.abs(a) <= 10 - ballR && Math.abs(v) <= 7 - ballR, miss: [a, v] };
};

test('rng deterministic per seed', () => {
  const a = mulberry32(42), b = mulberry32(42), c = mulberry32(43);
  const xs = Array.from({ length: 5 }, a), ys = Array.from({ length: 5 }, b);
  assert.deepEqual(xs, ys);
  assert.notDeepEqual(xs, Array.from({ length: 5 }, c));
  assert.ok(xs.every(x => x >= 0 && x < 1));
});

test('gaussian mean/std', () => {
  const g = makeGaussian(mulberry32(7)); const n = 20000; let s = 0, q = 0;
  for (let i = 0; i < n; i++) { const x = g(); s += x; q += x * x; }
  const m = s / n, sd = Math.sqrt(q / n - m * m);
  assert.ok(Math.abs(m) < 0.02, `mean ${m}`);
  assert.ok(Math.abs(sd - 1) < 0.02, `std ${sd}`);
});

test('normCdf / erf', () => {
  assert.ok(Math.abs(normCdf(0) - 0.5) < 1e-7);
  assert.ok(Math.abs(normCdf(1.96) - 0.975) < 1e-4);
  assert.ok(Math.abs(erf(-0.5) + erf(0.5)) < 1e-12);
});

test('shotSigmas sane and deadband adds in quadrature', () => {
  const s = shotSigmas('pollen'), s2 = shotSigmas('pollen', { hoodDeadbandDeg: 0.6, turretDeadbandDeg: 0.5 });
  assert.ok(s.speedFrac > 0.015 && s.speedFrac < 0.1);
  assert.ok(s2.elevRad > s.elevRad && s2.azRad > s.azRad);
  assert.ok(Math.abs(s2.elevRad - Math.sqrt(s.elevRad ** 2 + (0.6 * Math.PI / 180 / Math.sqrt(12)) ** 2)) < 1e-12);
});

test('zero sigmas give indicator matching nominal landing', () => {
  const zero = { speedFrac: 0, elevRad: 0, azRad: 0, feedS: 0 };
  const hit = launchAt(0), miss = launchAt(6);
  const ph = analyticHitProbability({ launch: hit, sig: zero, target, ballR, core });
  const pm = analyticHitProbability({ launch: miss, sig: zero, target, ballR, core });
  assert.equal(ph.p, 1); assert.equal(pm.p, 0);
  assert.equal(rectSim(hit).lands, true); assert.equal(rectSim(miss).lands, false);
  assert.equal(analyticHitProbability({ launch: { ...hit, speed: 1 }, sig: zero, target, ballR, core }).p, 0);
});

test('analytic agrees with Monte Carlo', () => {
  const sig = { speedFrac: 0.01, elevRad: 0.03, azRad: 0.03, feedS: 0 };
  const l = launchAt(2);
  const an = analyticHitProbability({ launch: l, sig, target, ballR, core });
  const mc = monteCarloHit({ launch: l, sig, n: 4000, seed: 99, simulate: rectSim });
  assert.ok(an.p > 0.2 && an.p < 0.9, `analytic p ${an.p}`);
  assert.ok(Math.abs(an.p - mc.p) < 0.04, `analytic ${an.p} vs mc ${mc.p}`);
  assert.ok(mc.spread[0] > 0 && mc.spread[1] > 0);
  assert.ok(Math.abs(an.sigma[1] - mc.spread[1]) / mc.spread[1] < 0.1);
});

test('p decreases monotonically as elevation sigma grows', () => {
  const l = launchAt(0); let prev = 2;
  for (const e of [0.002, 0.01, 0.02, 0.04, 0.08]) {
    const p = analyticHitProbability({ launch: l, sig: { speedFrac: 0.005, elevRad: e, azRad: 0.005, feedS: 0 }, target, ballR, core }).p;
    assert.ok(p <= prev + 1e-12, `p ${p} > ${prev}`); prev = p;
  }
  assert.ok(prev < 0.9);
});

test('perturbLaunch moves pos by vBase*dt only', () => {
  const l = { pos: [0, 0, 10], speed: 300, elevRad: 0.5, azRad: 0.1, vBase: [20, 0, 0] };
  const p = perturbLaunch(l, { speedFrac: 0, elevRad: 0, azRad: 0, feedS: 0.01 }, makeGaussian(mulberry32(1)));
  assert.equal(p.speed, 300); assert.equal(p.pos[1], 0); assert.notEqual(p.pos[0], 0);
  assert.equal(l.pos[0], 0);
});
