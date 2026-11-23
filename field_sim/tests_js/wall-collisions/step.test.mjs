import test from 'node:test';
import assert from 'node:assert/strict';
import { createWallContact, stepRobotContact, resetContactCache, setRobotGeometry } from '../../web/js/collision.js';
import { bodyMass } from '../../web/js/physics/wall-collisions/geometry.js';
import { WALL_PARAMS } from '../../web/js/physics/wall-collisions/params.js';

const DT = 1 / 60; // s, frame
const HX = WALL_PARAMS.robot.halfX; // 9 in
const SLOP = WALL_PARAMS.solver.linearSlop; // 0.19685 in
const BOUNDS = { minX: 0, minY: 0, maxX: 144, maxY: 144 };
const obsOf = (boxes = [], circles = []) => ({ bounds: BOUNDS, boxes, circles });
const mat0 = { mu: 0, e: 0.08 };
const MU0 = { materials: { default: { ...mat0 }, bounds: { ...mat0 }, hive_frame: { ...mat0 }, flower: { ...mat0 } } };
const matE0 = { mu: 0.62, e: 0 };
const E0 = { materials: { default: { ...matE0 }, bounds: { ...matE0 }, hive_frame: { ...matE0 }, flower: { ...matE0 } } };
const mkS = (x, y, h, vx, vy, om = 0) => ({ x, y, h, vx, vy, om, bodyV: [0, 0, 0] });
const finite = (S) => [S.x, S.y, S.h, S.vx, S.vy, S.om].every(Number.isFinite);

test('T1a: head-on into +x bounds at 60 in/s, e = 0.08: vx in [-5.5, -4.1]', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(144 - HX - 0.5, 72, 0, 60, 0);
  const r = stepRobotContact(S, DT, ctx);
  assert.ok(r.hit);
  assert.ok(S.vx >= -5.5 && S.vx <= -4.1, `vx=${S.vx}`);
});

test('T1b: e = 0 -> |vx| < 0.5 and x <= maxX - hx + 0.05', () => {
  const ctx = createWallContact(obsOf(), E0);
  const S = mkS(144 - HX - 0.5, 72, 0, 60, 0);
  for (let i = 0; i < 5; i++) stepRobotContact(S, DT, ctx);
  assert.ok(Math.abs(S.vx) < 0.5, `vx=${S.vx}`);
  assert.ok(S.x <= 144 - HX + 0.05, `x=${S.x}`);
});

test('T1c: below restitution threshold (vx = 8 in/s) -> |vx| < 0.2', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(144 - HX - 0.05, 72, 0, 8, 0);
  for (let i = 0; i < 5; i++) stepRobotContact(S, DT, ctx);
  assert.ok(Math.abs(S.vx) < 0.2, `vx=${S.vx}`);
});

// T2a spec change (round 2): the original spec also demanded om < 0. That is physically wrong for this
// case, so it is replaced by |om| < 1e-6. Proof (rigid 2D body, flush face contact at the two corners
// r = (9, +-9) in, wall normal (-1,0), friction impulse Jt on the -y side at lever arm hx = 9 in):
// staying flush needs torque balance jn(+9) - jn(-9) = Jt * hx / hy = Jt, with both jn >= 0, which is
// feasible whenever mu * hx / hy <= 1 (here 0.62 * 9 / 9 = 0.62). The impulsive LCP solution is
// therefore vn = target at BOTH corners, i.e. om = 0 exactly (vx = -e*(-20) = +1.6 outward). The old
// om < 0 only appeared because sequential Gauss-Seidel left a residual (solver round 1: om = -0.138 rad/s,
// vx = -2.84 in/s, an over-pushed trailing corner that also drove vy below the 26 floor).
// Analytic vy: Box2D-style friction is capped by the compression impulse (20 in/s * m) -> vy = 40 -
// 0.62*20 = 27.6; capped by the total incl. restitution (21.6 in/s * m) -> 26.61. Range [26, 29.2] kept.
test('T2a: sliding along +x wall, v=(20,40): one frame vy in [26, 29.2], flush so |om| < 1e-6', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(144 - HX, 72, 0, 20, 40);
  const r = stepRobotContact(S, DT, ctx);
  assert.ok(r.hit);
  assert.ok(S.vy >= 26 && S.vy <= 29.2, `vy=${S.vy}`);
  assert.ok(Math.abs(S.om) < 1e-6, `om=${S.om}`);
  assert.ok(Math.abs(S.vx - -1.6) < 0.05, `vx=${S.vx}`); // restitution e = 0.08 of 20 in/s, exact LCP value
});

// Friction does yaw the robot when the face cannot stay flush: mu * hx / hy > 1 (here mu = 1.5, so
// jn(-9 corner) would have to be negative). The leading corner then pivots and om < 0 is the physics.
test('T2c: slide with mu = 1.5 (> hy/hx) tips about the leading corner: om < 0', () => {
  const m = { mu: 1.5, e: 0.08 };
  const ctx = createWallContact(obsOf(), { materials: { default: m, bounds: m } });
  const S = mkS(144 - HX, 72, 0, 20, 40);
  stepRobotContact(S, DT, ctx);
  assert.ok(S.om < -1e-3, `om=${S.om}`);
  assert.ok(finite(S));
});

test('T2b: same slide with mu = 0 -> vy within 0.01 of 40, |om| < 1e-6', () => {
  const ctx = createWallContact(obsOf(), MU0);
  const S = mkS(144 - HX, 72, 0, 20, 40);
  stepRobotContact(S, DT, ctx);
  assert.ok(Math.abs(S.vy - 40) < 0.01, `vy=${S.vy}`);
  assert.ok(Math.abs(S.om) < 1e-6, `om=${S.om}`);
});

test('T3: corner strike at h=+30 deg gives om < 0, then squares up within 120 frames', () => {
  const ctx = createWallContact(obsOf());
  const h = Math.PI / 6;
  const reach = HX * (Math.cos(h) + Math.sin(h));
  const S = mkS(144 - reach - 0.3, 72, h, 30, 0);
  let firstOm = null;
  for (let i = 0; i < 120; i++) {
    S.vx = 30; S.vy = 0; // drivetrain keeps commanding 30 in/s into the wall
    const r = stepRobotContact(S, DT, ctx);
    if (firstOm === null && r.hit) firstOm = S.om;
    assert.ok(finite(S));
  }
  assert.ok(firstOm !== null && firstOm < 0, `first contact om=${firstOm}`);
  assert.ok(Math.abs(S.h) < 10 * Math.PI / 180, `h=${(S.h * 180 / Math.PI).toFixed(2)} deg`);
});

test('T4: no tunnelling through a 0.25 in thick box at 300 in/s over 60 frames', () => {
  const box = { minX: 70, minY: 0, maxX: 70.25, maxY: 144, kind: 'hive_frame' };
  const ctx = createWallContact(obsOf([box]));
  const S = mkS(40, 72, 0, 300, 0);
  let worst = -Infinity;
  for (let i = 0; i < 60; i++) {
    stepRobotContact(S, DT, ctx);
    assert.ok(finite(S));
    assert.ok(S.x < 70, `centre crossed the box at frame ${i}: x=${S.x}`);
    worst = Math.max(worst, S.x + HX);
  }
  assert.ok(worst <= 70.25 + 0.5, `max leading-face x = ${worst}`);
});

test('T5: resting push vx=20 into wall for 240 frames is steady', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(144 - HX - 0.5, 72, 0, 20, 0);
  const xs = [];
  let maxPen = 0;
  for (let i = 0; i < 240; i++) {
    S.vx = 20; S.vy = 0; S.om = 0;
    stepRobotContact(S, DT, ctx);
    if (i >= 120) { xs.push(S.x); maxPen = Math.max(maxPen, S.x + HX - 144); }
  }
  const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
  const std = Math.sqrt(xs.reduce((a, b) => a + (b - mean) ** 2, 0) / xs.length);
  assert.ok(std < 0.01, `x std=${std}`);
  assert.ok(maxPen < SLOP, `penetration=${maxPen}`);
});

test('T6: seam between two adjacent boxes produces no ghost normal', () => {
  const A = { minX: 60, minY: 30, maxX: 80, maxY: 70, kind: 'hive_frame' };
  const B = { minX: 60, minY: 70, maxX: 80, maxY: 110, kind: 'hive_frame' };
  const ctx = createWallContact(obsOf([A, B]), MU0);
  const S = mkS(60 - HX, 35, 0, 2, 40);
  let hits = 0;
  for (let i = 0; i < 90; i++) {
    S.vx = 2; // gentle push into the faces
    const r = stepRobotContact(S, DT, ctx);
    if (r.hit) hits++;
    assert.ok(Math.abs(S.vy - 40) <= 0.4, `frame ${i}: vy=${S.vy}`);
    for (const c of r.contacts) assert.ok(Math.abs(c.n[1]) <= 0.01, `frame ${i}: normal ${c.n} key ${c.key}`);
  }
  assert.ok(hits > 50, `hits=${hits}`);
});

test('T7: energy never increases over 500 seeded random strikes', () => {
  let seed = 20260101;
  const rnd = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; };
  const { m, I } = bodyMass(createWallContact(obsOf()).P);
  let bad = null;
  let contactsSeen = 0;
  for (let k = 0; k < 500; k++) {
    const ctx = createWallContact(obsOf());
    const h = (rnd() * 2 - 1) * Math.PI;
    const S = mkS(0, 20 + rnd() * 100, h, rnd() * 80, (rnd() * 2 - 1) * 50, (rnd() * 2 - 1) * 3);
    const ext = HX * (Math.abs(Math.cos(h)) + Math.abs(Math.sin(h)));
    S.x = 144 - ext - rnd() * 1.0; // gap 0..1 in from the +x wall, not penetrating
    const ke = 0.5 * m * (S.vx ** 2 + S.vy ** 2) + 0.5 * I * S.om ** 2;
    const r = stepRobotContact(S, DT, ctx);
    if (r.hit) contactsSeen++;
    const ke2 = 0.5 * m * (S.vx ** 2 + S.vy ** 2) + 0.5 * I * S.om ** 2;
    if (!(ke2 <= ke * (1 + 1e-6)) && !bad) bad = { k, ke, ke2 };
  }
  assert.ok(contactsSeen > 200, `only ${contactsSeen} strikes made contact`);
  assert.equal(bad, null, bad && `KE grew ${bad.ke} -> ${bad.ke2} at strike ${bad.k}`);
});

test('T8: wedged in a gap narrower than the robot stays finite and reports hit', () => {
  const L = { minX: 0, minY: 0, maxX: 60, maxY: 144, kind: 'hive_frame' };
  const R = { minX: 70, minY: 0, maxX: 144, maxY: 144, kind: 'hive_frame' };
  const ctx = createWallContact(obsOf([L, R]));
  const S = mkS(65, 72, 0.1, 10, 5, 0.5);
  for (let i = 0; i < 20; i++) {
    const r = stepRobotContact(S, DT, ctx);
    assert.ok(finite(S), `frame ${i} non-finite`);
    assert.ok(r.hit, `frame ${i} no hit`);
    assert.ok(Number.isFinite(r.maxPenetration));
    assert.ok(Number.isFinite(r.forceN.x) && Number.isFinite(r.torqueNm));
  }
});

test('report fields and bodyV are consistent', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(144 - HX, 72, 0.3, 20, 10);
  const r = stepRobotContact(S, DT, ctx);
  for (const k of ['hit', 'chunks', 'fallback', 'maxPenetration', 'impulse', 'forceN', 'torqueNm', 'contacts', 'normals']) assert.ok(k in r, k);
  assert.ok(r.chunks >= 1);
  const c = Math.cos(S.h), s = Math.sin(S.h);
  assert.ok(Math.abs(S.bodyV[0] - (c * S.vx + s * S.vy)) < 1e-9);
  assert.ok(Math.abs(S.bodyV[1] - (-s * S.vx + c * S.vy)) < 1e-9);
  assert.equal(S.bodyV[2], S.om);
  assert.ok(Math.abs(r.forceN.x - r.impulse.x / DT * 0.0115212462) < 1e-9);
});

test('free flight: no hit, state integrates', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(72, 72, 0, 30, -12, 0.4);
  const r = stepRobotContact(S, DT, ctx);
  assert.equal(r.hit, false);
  assert.ok(Math.abs(S.x - (72 + 30 * DT)) < 1e-9 && Math.abs(S.y - (72 - 12 * DT)) < 1e-9);
});

test('resetContactCache and setRobotGeometry', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(144 - HX, 72, 0, 20, 0);
  for (let i = 0; i < 3; i++) stepRobotContact(S, DT, ctx);
  assert.ok(ctx.cache.size > 0);
  resetContactCache(ctx);
  assert.equal(ctx.cache.size, 0);
  setRobotGeometry(ctx, { halfX: 8, halfY: 8 });
  assert.equal(ctx.poly.hx, 8);
});

// Round-2 regression: the legacy fallback used to read box cx/cy/hx/hy that raw boxes lack -> NaN pose.
test('T8b: fallback path with raw boxes returns a finite pose (fallback actually exercised)', () => {
  const L = { minX: 0, minY: 0, maxX: 60, maxY: 144, kind: 'hive_frame' };
  const R = { minX: 70, minY: 0, maxX: 144, maxY: 144, kind: 'hive_frame' };
  const ctx = createWallContact(obsOf([L, R]));
  const S = mkS(65, 72, 0.1, 10, 5, 0.5);
  const r = stepRobotContact(S, DT, ctx);
  assert.ok(r.fallback, 'expected the legacy fallback to run');
  assert.ok(finite(S), JSON.stringify(S));
  assert.ok(Number.isFinite(r.impulse.x) && Number.isFinite(r.impulse.y) && Number.isFinite(r.impulse.ang));
});

test('T9: non-finite inputs never produce NaN (velocities, dt, missing bounds)', () => {
  const ctx = createWallContact(obsOf());
  const S = mkS(144 - HX, 72, 0, NaN, Infinity, NaN);
  const r = stepRobotContact(S, DT, ctx);
  assert.ok(finite(S), JSON.stringify(S));
  assert.ok(Number.isFinite(r.maxPenetration) && Number.isFinite(r.forceN.x) && Number.isFinite(r.torqueNm));
  for (const dt of [0, -1, NaN, Infinity]) {
    const S2 = mkS(144 - HX, 72, 0, 20, 0);
    const r2 = stepRobotContact(S2, dt, ctx);
    assert.ok(finite(S2), `dt=${dt}`);
    assert.ok(Number.isFinite(r2.forceN.x) && Number.isFinite(r2.torqueNm), `dt=${dt}`);
  }
  const ctxNB = createWallContact({ boxes: [{ minX: 60, minY: 0, maxX: 70, maxY: 144 }] });
  const S3 = mkS(60, 72, 0, 50, 0); // starts 9 in deep in the box -> fallback without bounds
  for (let i = 0; i < 5; i++) { stepRobotContact(S3, DT, ctxNB); assert.ok(finite(S3), JSON.stringify(S3)); }
  const S4 = mkS(144 - HX, 72, 0, 1e300, -1e300, 1e300); // overflow-scale speeds
  const r4 = stepRobotContact(S4, DT, ctx);
  assert.ok(finite(S4), JSON.stringify(S4));
  assert.ok(Number.isFinite(r4.forceN.x) && Number.isFinite(r4.forceN.y) && Number.isFinite(r4.torqueNm));
});
