// Tests for kinematics.js and rng.js (P5). Units: inches/rad unless noted.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  wrapAngle, worldToBody, bodyToWorld, podGroundRate, pinpointBodyDelta, poseExp, poseError,
} from '../../web/js/physics/pinpoint-odometry/kinematics.js';
import { makeRng } from '../../web/js/physics/pinpoint-odometry/rng.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} got ${a} want ${b}`);

test('wrapAngle', () => {
  near(wrapAngle(0), 0, 1e-12);
  near(wrapAngle(2 * Math.PI), 0, 1e-12);
  near(wrapAngle(1.5 * Math.PI), -0.5 * Math.PI, 1e-12);
  near(wrapAngle(-1.5 * Math.PI), 0.5 * Math.PI, 1e-12);
  near(wrapAngle(7), 7 - 2 * Math.PI, 1e-12);
  near(Math.abs(wrapAngle(Math.PI)), Math.PI, 1e-9);
  near(Math.abs(wrapAngle(-Math.PI)), Math.PI, 1e-9);
  const w = wrapAngle(-100);
  assert.ok(w > -Math.PI - 1e-12 && w <= Math.PI + 1e-12);
});

test('worldToBody / bodyToWorld', () => {
  const b = worldToBody(0, 1, Math.PI / 2); // facing +y: forward is world +y
  near(b.vf, 1, 1e-12); near(b.vl, 0, 1e-12);
  const b2 = worldToBody(-1, 0, Math.PI / 2); // left of a +y-facing robot is world -x
  near(b2.vl, 1, 1e-12);
  const w = bodyToWorld(3, -2, 0.7);
  const r = worldToBody(w.vx, w.vy, 0.7);
  near(r.vf, 3, 1e-12); near(r.vl, -2, 1e-12);
});

test('poseExp: straight move is exact', () => {
  let p = poseExp({ x: 1, y: 2, h: 0 }, 3, 0, 0);
  near(p.x, 4, 1e-12); near(p.y, 2, 1e-12); near(p.h, 0, 1e-12);
  p = poseExp({ x: 1, y: 2, h: Math.PI / 2 }, 3, 0, 0);
  near(p.x, 1, 1e-12); near(p.y, 5, 1e-12);
});

test('poseExp: pure rotation gives no translation', () => {
  const p = poseExp({ x: 5, y: -3, h: 0.2 }, 0, 0, 0.5);
  near(p.x, 5, 1e-12); near(p.y, -3, 1e-12); near(p.h, 0.7, 1e-12);
});

test('poseExp: quarter arc ends at (R, R)', () => {
  const R = 10;
  const p = poseExp({ x: 0, y: 0, h: 0 }, R * Math.PI / 2, 0, Math.PI / 2);
  near(p.x, R, 1e-9); near(p.y, R, 1e-9); near(p.h, Math.PI / 2, 1e-12);
});

test('poseExp: continuous across |dth| = 1e-6', () => {
  const a = poseExp({ x: 0, y: 0, h: 0.3 }, 1, 0.5, 0.99999e-6);
  const b = poseExp({ x: 0, y: 0, h: 0.3 }, 1, 0.5, 1.00001e-6);
  near(a.x, b.x, 1e-9); near(a.y, b.y, 1e-9);
  const n = poseExp({ x: 0, y: 0, h: 0.3 }, 1, 0.5, -1e-6); // sign flips O(dth) terms only
  near(n.x, a.x, 5e-6); near(n.y, a.y, 5e-6);
});

test('poseExp wraps heading', () => {
  const p = poseExp({ x: 0, y: 0, h: 3 }, 0, 0, 1);
  near(p.h, 4 - 2 * Math.PI, 1e-12);
});

test('pinpointBodyDelta inverts podGroundRate for random pod positions', () => {
  const rng = makeRng(7);
  for (let i = 0; i < 50; i++) {
    const fp = { x: (rng.next() - 0.5) * 10, y: (rng.next() - 0.5) * 10, angleRad: 0 };
    const sp = { x: (rng.next() - 0.5) * 10, y: (rng.next() - 0.5) * 10, angleRad: 0 };
    const vf = (rng.next() - 0.5) * 100, vl = (rng.next() - 0.5) * 100, om = (rng.next() - 0.5) * 6;
    const dt = 0.01;
    const dF = podGroundRate(fp, 'forward', vf, vl, om) * dt;
    const dS = podGroundRate(sp, 'strafe', vf, vl, om) * dt;
    const { dxB, dyB } = pinpointBodyDelta(dF, dS, om * dt, fp.y, sp.x);
    near(dxB, vf * dt, 1e-9, 'dxB'); near(dyB, vl * dt, 1e-9, 'dyB');
  }
});

test('podGroundRate forward/strafe at a=0', () => {
  near(podGroundRate({ x: 1, y: 2, angleRad: 0 }, 'forward', 10, 4, 0.5), 10 - 0.5 * 2, 1e-12);
  near(podGroundRate({ x: 3, y: 2, angleRad: 0 }, 'strafe', 10, 4, 0.5), 4 + 0.5 * 3, 1e-12);
});

test('wrong offset by delta yields delta per radian', () => {
  const py = -0.63, px = -1.2, vf = 20, vl = 5, om = 2, dt = 0.01;
  const dF = (vf - om * py) * dt, dS = (vl + om * px) * dt, dth = om * dt;
  const ok = pinpointBodyDelta(dF, dS, dth, py, px);
  const bad = pinpointBodyDelta(dF, dS, dth, py + 0.25, px - 0.4);
  near(bad.dxB - ok.dxB, 0.25 * dth, 1e-12);
  near(bad.dyB - ok.dyB, 0.4 * dth, 1e-12); // dyB = dS - Xs*dth, Xs lowered by 0.4
});

test('misalignment gives cos(a) / sin(a) readings', () => {
  const a = 0.1;
  const f = { x: 0, y: 0, angleRad: a }, s = { x: 0, y: 0, angleRad: a };
  near(podGroundRate(f, 'forward', 1, 0, 0), Math.cos(a), 1e-12);
  near(podGroundRate(f, 'forward', 0, 1, 0), Math.sin(a), 1e-12);
  near(podGroundRate(s, 'strafe', 0, 1, 0), Math.cos(a), 1e-12);
  near(podGroundRate(s, 'strafe', 1, 0, 0), -Math.sin(a), 1e-12);
});

test('poseError', () => {
  const e = poseError({ x: 3, y: 4, h: 3.1 }, { x: 0, y: 0, h: -3.1 });
  near(e.dist, 5, 1e-12);
  assert.ok(Math.abs(e.dh) < 0.1); // wrapped
});

test('makeRng deterministic, distinct seeds differ, gauss ~ N(0,1)', () => {
  const a = makeRng(42), b = makeRng(42), c = makeRng(43);
  const sa = [], sb = [], sc = [], sa2 = [];
  for (let i = 0; i < 20; i++) { const u = a.next(); sa.push(u, a.gauss()); sa2.push(u); sb.push(b.next(), b.gauss()); sc.push(c.next()); }
  assert.deepEqual(sa, sb);
  assert.notDeepEqual(sa2, sc);
  const r = makeRng(1);
  let s = 0, s2 = 0; const n = 20000;
  for (let i = 0; i < n; i++) { const u = r.next(); assert.ok(u >= 0 && u < 1); const g = r.gauss(); s += g; s2 += g * g; }
  near(s / n, 0, 0.05); near(s2 / n - (s / n) ** 2, 1, 0.08);
});
