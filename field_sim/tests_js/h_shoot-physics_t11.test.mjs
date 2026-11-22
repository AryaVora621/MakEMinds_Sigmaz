// t11 moving-shot compensation. Observed (seed 20260711, 100 poses, robot 0-60 in/s): solver ok 100/100, comp hits 100/100,
// uncomp 33/100, robotVel=0 identical 100/100. Hit = analytic ballistic (shotrules G, no obstacles) inside halfW 10 / halfH 7 less R.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import * as SR from '../web/js/shotrules.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const root = {}; vm.runInNewContext(rd('../web/js/shot_core.js'), { window: root }); SR.setCore(root.Core);
const scene = JSON.parse(rd('../web/data/scene.json')), Core = root.Core;
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const up = Object.values(scene.constants.hives.blue.states[scene.groups.hive_blue.cad_pose_state].cells).find((c) => c.up);
const tg = { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal }, sgn = Math.sign(tg.normal[1]) || 1;
const rng = (seed) => () => { seed = (seed + 0x6D2B79F5) | 0; let x = Math.imul(seed ^ (seed >>> 15), 1 | seed); x = (x + Math.imul(x ^ (x >>> 7), 61 | x)) ^ x; return ((x ^ (x >>> 14)) >>> 0) / 4294967296; };
const pose = (rnd) => ({ x: tg.x + (rnd() * 2 - 1) * 15, y: tg.y + sgn * (40 + rnd() * 30), h: rnd() * 2 * Math.PI - Math.PI, vx: 0, vy: 0, om: 0 });
// analytic ballistic: first crossing of the opening plane from the front, inside the rectangle
function hits(fs, t) {
  const G = SR.G, n = t.normal, R = fs.ballR, e = fs.exit, v = fs.vel;
  const A = -0.5 * G * n[2], B = n[0] * v[0] + n[1] * v[1] + n[2] * v[2], C = n[0] * (e[0] - t.x) + n[1] * (e[1] - t.y) + n[2] * (e[2] - t.z);
  const disc = B * B - 4 * A * C;
  if (!(C > 0) || disc < 0) return false;
  const roots = Math.abs(A) < 1e-12 ? [-C / B] : [(-B - Math.sqrt(disc)) / (2 * A), (-B + Math.sqrt(disc)) / (2 * A)];
  const tc = roots.filter((x) => x > 0).sort((a, b) => a - b)[0];
  if (!(tc > 0)) return false;
  const p = [e[0] + v[0] * tc, e[1] + v[1] * tc, e[2] + v[2] * tc - 0.5 * G * tc * tc].map((q, k) => q - [t.x, t.y, t.z][k]);
  const hn = Math.hypot(n[0], n[1]) || 1, lat = [-n[1] / hn, n[0] / hn, 0];
  const upv = [n[1] * lat[2] - n[2] * lat[1], n[2] * lat[0] - n[0] * lat[2], n[0] * lat[1] - n[1] * lat[0]];
  const dot = (u) => p[0] * u[0] + p[1] * u[1] + p[2] * u[2];
  return Math.abs(dot(lat)) <= (t.halfW || 10) - R && Math.abs(dot(upv)) <= (t.halfH || 7) - R;
}

test('moving shot: compensated solution hits the opening far more often than uncompensated (100 seeded poses, <=60 in/s)', () => {
  const rnd = rng(20260711);
  let valid = 0, comp = 0, unc = 0;
  for (let i = 0; i < 100; i++) {
    const p0 = pose(rnd), th = rnd() * 2 * Math.PI, sp = 60 * rnd();
    const rv = { vx: sp * Math.cos(th), vy: sp * Math.sin(th), om: 0 };
    const sc = SR.solveAutoShot(p0, SH, tg, 'pollen', Core, rv), su = SR.solveAutoShot(p0, SH, tg, 'pollen', Core, { vx: 0, vy: 0, om: 0 });
    if (!sc.ok || !su.ok) continue;
    valid++;
    const shot = (s) => SR.forwardShot({ pose: { ...p0, ...rv }, shooter: SH, target: tg, turretDeg: s.turretDeg, hoodDeg: s.elev * 180 / Math.PI, rpm: s.rpm, robotVel: rv });
    const fc = shot(sc), fu = shot(su);
    assert.ok([...fc.vel, ...fu.vel, fc.speed, fu.speed].every(Number.isFinite), 'non-finite output at pose ' + i);
    if (hits(fc, tg)) comp++;
    if (hits(fu, tg)) unc++;
  }
  assert.ok(valid >= 90, `solver ok on only ${valid}/100 poses`);
  assert.ok(comp / valid >= 0.95, `compensated hit rate ${comp}/${valid} below band 0.95`);
  assert.ok((comp - unc) / valid >= 0.3, `compensation margin too small: comp ${comp}, uncomp ${unc} of ${valid}`);
});

test('moving shot: robotVel = 0 reproduces the stationary solution exactly', () => {
  const rnd = rng(7);
  for (let i = 0; i < 100; i++) {
    const p0 = pose(rnd), a = SR.solveAutoShot(p0, SH, tg, 'pollen', Core), b = SR.solveAutoShot(p0, SH, tg, 'pollen', Core, { vx: 0, vy: 0, om: 0 });
    assert.deepEqual([b.ok, b.turretDeg, b.elev, b.rpm, b.speed], [a.ok, a.turretDeg, a.elev, a.rpm, a.speed]);
  }
});
