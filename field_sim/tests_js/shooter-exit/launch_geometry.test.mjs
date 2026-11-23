// P2 geometry/core tests: Core.solveExit / planeCrossing / launchVelocityFrom and launch_geometry.js.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import * as SR from '../../web/js/shotrules.js';
import * as LG from '../../web/js/physics/shooter-exit/launch_geometry.js';
import { PARAMS } from '../../web/js/physics/shooter-exit/params.js';

const root = {};
vm.runInNewContext(readFileSync(new URL('../../web/js/shot_core.js', import.meta.url), 'utf8'), { window: root });
const Core = root.Core;
const G = Core.G;
const DEG = 180 / Math.PI;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg || ''} ${a} vs ${b} (tol ${tol})`);
const SH = { exitHeight: 12, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, pivotX: 3, pivotY: -2, hoodMin: 10, hoodMax: 80 };
const PHYS = { model: 'physical' };

test('legacy launchVelocityWorld equals shotrules forwardShot (moving, rotating robot)', () => {
  const pose = { x: 40, y: 55, h: 0.7, vx: 30, vy: -12, om: 1.3 };
  const ctx = { pose, shooter: SH, kind: 'pollen', turretDeg: 25, hoodDeg: 47, rpm: 2500, target: null };
  const f = SR.forwardShot(ctx);
  const exit = LG.exitPointWorld(pose, SH, f.azRad, f.elevRad, 'pollen', {});
  const { vel } = LG.launchVelocityWorld({ pose, sh: SH, azRad: f.azRad, elevRad: f.elevRad, speed: f.speed, exit });
  for (let i = 0; i < 3; i++) { near(exit[i], f.exit[i], 1e-12, 'exit' + i); near(vel[i], f.vel[i], 1e-12, 'vel' + i); }
  // explicit robotVel override
  const rv = { vx: -5, vy: 9, om: -0.4 };
  const f2 = SR.forwardShot({ ...ctx, robotVel: rv });
  const l2 = LG.launchVelocityWorld({ pose, robotVel: rv, sh: SH, azRad: f2.azRad, elevRad: f2.elevRad, speed: f2.speed, exit: f2.exit });
  for (let i = 0; i < 3; i++) near(l2.vel[i], f2.vel[i], 1e-12, 'rv vel' + i);
});

test('turret rate adds omegaTurret x (exit - pivot)', () => {
  const pose = { x: 0, y: 0, h: 0, vx: 0, vy: 0, om: 0 };
  const exit = [5, 0, 12]; // pivot is (3,-2) -> r = (2, 2)
  const a = LG.launchVelocityWorld({ pose, sh: SH, azRad: 0, elevRad: 0, speed: 0, exit, turretRateDegS: 90 });
  const w = Math.PI / 2;
  near(a.vBase[0], -w * 2, 1e-12); near(a.vBase[1], w * 2 + 0 * 0, 1e-12 + 0);
});

test('hood map inverse round-trip and identity when disabled', () => {
  for (const kind of ['pollen', 'nectar']) for (const h of [10, 33.3, 60, 80]) {
    near(LG.elevToHoodDeg(LG.hoodToElevDeg(h, kind, PHYS), kind, PHYS), h, 1e-10);
    assert.equal(LG.hoodToElevDeg(h, kind, {}), h);
    assert.equal(LG.elevToHoodDeg(h, kind, {}), h);
  }
  // non-trivial map still inverts exactly
  const params = { ...PARAMS, hood: { ...PARAMS.hood, ELEV_OFFSET_DEG: 5, ELEV_GAIN: 0.8, THROWDOWN_DEG_PER_UNIT_C: 3 } };
  for (const h of [12, 44, 70]) near(LG.elevToHoodDeg(LG.hoodToElevDeg(h, 'nectar', { model: 'physical', params }), 'nectar', { model: 'physical', params }), h, 1e-10);
});

test('exit up = 0 at 45 deg for pollen with default geometry; disabled gives zero offset', () => {
  const o = LG.exitOffset(Math.PI / 4, 'pollen', PHYS);
  near(o.up, 0, 1e-12);
  assert.ok(o.fwd < 0);
  assert.deepEqual(LG.exitOffset(Math.PI / 4, 'pollen', {}), { fwd: 0, up: 0 });
  const pose = { x: 10, y: 20, h: 0.3 };
  const e = LG.exitPointWorld(pose, SH, 0.9, Math.PI / 4, 'pollen', PHYS);
  near(e[2], SH.exitHeight, 1e-12);
  const [px, py] = LG.pivotWorld(pose, SH);
  near(Math.hypot(e[0] - px, e[1] - py), Math.abs(o.fwd), 1e-12);
});

function mkP(extra = {}) {
  return { x: 30, y: 40, psi: 0.5, vx: 20, vy: -10, om: 0.8, rx: SH.pivotX, ry: SH.pivotY, z0: SH.exitHeight, X: 110, Y: 90, Z: 38, ...extra };
}
const strip = (r) => { const o = { ...r }; delete o.exitOffset; delete o.iters; delete o.p; return JSON.parse(JSON.stringify(o)); };

test('solveExit with zero exitFn equals Core.solve exactly', () => {
  const zero = () => ({ dx: 0, dy: 0, dz: 0 });
  for (const [mode, param, br] of [['speed', 260, 'low'], ['speed', 260, 'high'], ['theta', 55 / DEG, 'low'], ['gamma', 60 / DEG, 'low'], ['minspeed', 0, 'low']]) {
    const a = Core.solve(mkP(), mode, param, br), b = Core.solveExit(mkP(), mode, param, br, zero);
    assert.equal(a.ok, b.ok);
    assert.deepEqual(strip(b), JSON.parse(JSON.stringify(a)));
    assert.equal(b.iters, 1);
  }
});

test('solveExit converges and the offset exit lands on the target', () => {
  const pose = { x: 30, y: 40, h: 0.5 };
  const exitFn = LG.makeExitFn(pose, SH, 'pollen', PHYS);
  const p = mkP();
  const r = Core.solveExit(p, 'speed', 260, 'low', exitFn, 4);
  assert.ok(r.ok);
  assert.ok(r.iters <= 4, 'iters ' + r.iters);
  assert.ok(r.fwd.err < 1e-6, 'fwd err ' + r.fwd.err);
  // exit offset of the result matches the geometry at the solved (elev, az)
  const o = exitFn(r.elev, r.az);
  near(o.dx, r.exitOffset.dx, 2e-3); near(o.dy, r.exitOffset.dy, 2e-3); near(o.dz, r.exitOffset.dz, 2e-3);
  // independent forward integration from the offset exit with the shooter + rotation velocity
  const exit = LG.exitPointWorld({ ...pose }, SH, r.az, r.elev, 'pollen', PHYS);
  const { vel } = LG.launchVelocityWorld({ pose: { ...pose, ...{ vx: p.vx, vy: p.vy, om: p.om } }, sh: SH, azRad: r.az, elevRad: r.elev, speed: r.speed, exit });
  const t = r.T;
  const x = exit[0] + vel[0] * t, y = exit[1] + vel[1] * t, z = exit[2] + vel[2] * t - 0.5 * G * t * t;
  near(x, p.X, 5e-3, 'x'); near(y, p.Y, 5e-3, 'y'); near(z, p.Z, 5e-3, 'z');
  assert.ok(r.p.z0 !== p.z0 || r.exitOffset.dz === 0);
});

test('planeCrossing agrees with a fine sampled crossing and returns null when never reached', () => {
  const cases = [
    { P0: [0, 0, 12], v: [200, 0, 150], plane: { x: 100, y: 0, z: 40, normal: [-1, 0, 0] } },
    { P0: [5, 3, 12], v: [180, 90, 170], plane: { x: 110, y: 60, z: 38, normal: [-0.8, -0.1, 0.6] } },
  ];
  for (const { P0, v, plane } of cases) {
    const n = plane.normal;
    const s = (t) => n[0] * (P0[0] + v[0] * t - plane.x) + n[1] * (P0[1] + v[1] * t - plane.y) + n[2] * (P0[2] + v[2] * t - 0.5 * G * t * t - plane.z);
    // first sampled front-to-back sign change at 0.1 ms
    let tc = null, prev = s(0);
    for (let t = 1e-4; t < 10; t += 1e-4) { const cur = s(t); if (prev > 0 && cur <= 0) { let lo = t - 1e-4, hi = t; for (let i = 0; i < 80; i++) { const m = 0.5 * (lo + hi); if (s(m) > 0) lo = m; else hi = m; } tc = 0.5 * (lo + hi); break; } prev = cur; }
    const c = Core.planeCrossing(P0, v, plane);
    assert.ok(c && tc !== null);
    near(c.t, tc, 1e-6);
    near(c.p[0], P0[0] + v[0] * c.t, 1e-9); near(c.v[2], v[2] - G * c.t, 1e-9);
    near(n[0] * (c.p[0] - plane.x) + n[1] * (c.p[1] - plane.y) + n[2] * (c.p[2] - plane.z), 0, 1e-6);
  }
  assert.equal(Core.planeCrossing([0, 0, 12], [-10, 0, 10], { x: 100, y: 0, z: 0, normal: [-1, 0, 0] }), null);
  assert.equal(Core.planeCrossing([0, 0, 12], [10, 0, 10], { x: 0, y: 0, z: 1000, normal: [0, 0, -1] }), null);
});

test('launchVelocityFrom', () => {
  const v = Core.launchVelocityFrom(100, 0.4, 0.9, [1, 2, 3]);
  near(v[0], 1 + 100 * Math.cos(0.9) * Math.cos(0.4), 1e-12);
  near(v[2], 3 + 100 * Math.sin(0.9), 1e-12);
  const w = Core.launchVelocityFrom(100, 0.4, 0.9, null);
  near(w[1], 100 * Math.cos(0.9) * Math.sin(0.4), 1e-12);
});
