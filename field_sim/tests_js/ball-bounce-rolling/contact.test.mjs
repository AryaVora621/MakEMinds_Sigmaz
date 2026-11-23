// Acceptance criteria 5, 6, 8, 10 (ball-bounce-rolling). Units: in, s, rad/s.
import test from 'node:test';
import assert from 'node:assert/strict';
import { BBR, SURFACES, inertiaFactor } from '../../web/js/physics/ball-bounce-rolling/params.js';
import { resolveContact, resolvePairFriction, contactPointVelocity } from '../../web/js/physics/ball-bounce-rolling/contact.js';
import { BALL_KINDS } from '../../web/js/balls.js';

const g = BBR.g;
const R = BALL_KINDS.pollen.radiusIn;
const K = inertiaFactor(R, BALL_KINDS.pollen.massKg);
const mkBall = (o = {}) => Object.assign({ x: 0, y: 0, z: R, vx: 0, vy: 0, vz: 0, wx: 0, wy: 0, wz: 0, r: R, k: K, _c: false, _jnSum: 0 }, o);
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg || ''} ${a} vs ${b} (tol ${tol})`);

function seededLcg(seed) { let s = seed >>> 0; return () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s / 4294967296; }; }

test('6. backspin lowers exit vx, topspin raises it', () => {
  const exitVx = wy => {
    const b = mkBall({ vx: 100, vz: -150, wy });
    resolveContact(b, 0, 0, 1, 0, SURFACES.floor, 0, 0, 0, 1 / 240);
    return b.vx;
  };
  const none = exitVx(0), back = exitVx(-100 / R), top = exitVx(100 / R);
  assert.ok(back < none, `backspin ${back} should be < ${none}`);
  assert.ok(top > none, `topspin ${top} should be > ${none}`);
});

test('contactPointVelocity: rolling ball has zero contact slip', () => {
  const b = mkBall({ vx: 40, wy: 40 / R });
  const out = [9, 9, 9];
  contactPointVelocity(b, 0, 0, 1, 0, 0, 0, out);
  near(out[0], 0, 1e-9); near(out[1], 0, 1e-9); near(out[2], 0, 1e-9);
});

test('step 2: no impulse when separating', () => {
  const b = mkBall({ vz: 50, vx: 10 });
  const r = resolveContact(b, 0, 0, 1, 0.01, SURFACES.floor, 0, 0, 0, 1 / 240);
  assert.equal(r.impact, false); assert.equal(r.jn, 0);
  assert.equal(b.vx, 10); assert.equal(b.vz, 50); assert.equal(b.wy, 0);
  near(b.z, R + 0.01, 1e-12); assert.equal(b._c, true);
});

test('5. incline 15 deg: a = g sin(th)/(1+k) within 5% (crr = 0)', () => {
  const crr0 = SURFACES.floor.crr;
  SURFACES.floor.crr = 0;
  try {
    const th = 15 * Math.PI / 180, h = 1 / 240;
    // plane z = x tan(th) through the origin, normal toward the ball:
    const nx = -Math.sin(th), nz = Math.cos(th);
    const dx = -Math.cos(th), dz = -Math.sin(th);      // downhill unit vector
    const b = mkBall({ x: nx * R, z: nz * R });
    const vDown = () => b.vx * dx + b.vz * dz;
    let v1 = 0, v2 = 0;
    for (let i = 1; i <= 240; i++) {
      b.vz -= g * h;
      b.x += b.vx * h; b.y += b.vy * h; b.z += b.vz * h;
      const dist = b.x * nx + b.z * nz;
      if (dist < R) resolveContact(b, nx, 0, nz, R - dist, SURFACES.floor, 0, 0, 0, h);
      if (i === 60) v1 = vDown();
      if (i === 120) v2 = vDown();
    }
    const a = (v2 - v1) / (60 * h);
    const expect = g * Math.sin(th) / (1 + K);
    near(a, expect, 0.05 * expect, 'incline accel');
  } finally { SURFACES.floor.crr = crr0; }
});

test('8. energy v^2 + k r^2 w^2 never grows over 200 random contacts, all surfaces', () => {
  const rnd = seededLcg(12345);
  const names = Object.keys(SURFACES);
  for (let i = 0; i < 200; i++) {
    const surf = SURFACES[names[i % names.length]];
    let nx, ny, nz, m;
    do { nx = rnd() * 2 - 1; ny = rnd() * 2 - 1; nz = rnd() * 2 - 1; m = Math.hypot(nx, ny, nz); } while (m < 0.1);
    nx /= m; ny /= m; nz /= m;
    const k = 0.4 + rnd() * (2 / 3 - 0.4);
    const b = mkBall({ k, vx: (rnd() - .5) * 400, vy: (rnd() - .5) * 400, vz: (rnd() - .5) * 400,
      wx: (rnd() - .5) * 200, wy: (rnd() - .5) * 200, wz: (rnd() - .5) * 200 });
    const vn = b.vx * nx + b.vy * ny + b.vz * nz;
    if (vn > 0) { b.vx -= 2 * vn * nx; b.vy -= 2 * vn * ny; b.vz -= 2 * vn * nz; }
    const E = q => q.vx ** 2 + q.vy ** 2 + q.vz ** 2 + q.k * q.r * q.r * (q.wx ** 2 + q.wy ** 2 + q.wz ** 2);
    const before = E(b);
    resolveContact(b, nx, ny, nz, 0, surf, 0, 0, 0, 1 / 240);
    const after = E(b);
    assert.ok(after <= before + 1e-6, `contact ${i} (${names[i % names.length]}): ${before} -> ${after}`);
  }
});

test('10. pair friction conserves linear momentum and spins both balls', () => {
  const a = mkBall({ vy: 50, vx: -3, vz: 1 }), b = mkBall({ vx: 4, vy: -2 });
  b.k = 0.5; b.r = 1.8;
  const px = a.vx + b.vx, py = a.vy + b.vy, pz = a.vz + b.vz;
  resolvePairFriction(a, b, 1, 0, 0, 20, true, SURFACES.ball);
  near(a.vx + b.vx, px, 1e-9); near(a.vy + b.vy, py, 1e-9); near(a.vz + b.vz, pz, 1e-9);
  assert.ok(a.vy < 50, 'a slowed tangentially');
  assert.ok(Math.abs(a.wz) > 0 && Math.abs(b.wz) > 0, 'both spun about z');
  assert.ok(a.wz * b.wz > 0, 'same torque sense (like gears)');
});

test('10. pair friction with jn = 0 does nothing', () => {
  const a = mkBall({ vy: 50 }), b = mkBall();
  resolvePairFriction(a, b, 1, 0, 0, 0, false, SURFACES.ball);
  assert.equal(a.vy, 50); assert.equal(a.wz, 0);
});
