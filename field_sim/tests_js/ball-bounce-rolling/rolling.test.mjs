// Rolling resistance, spin helpers, sleep (ball-bounce-rolling). Units: in, s, rad/s.
import test from 'node:test';
import assert from 'node:assert/strict';
import { BBR, SURFACES } from '../../web/js/physics/ball-bounce-rolling/params.js';
import { applyRollingResistance, applyTorsionalFriction, sleepMetric, updateSleep, isAirborne, rollingSpinFor, launchSpin, clampSpin }
  from '../../web/js/physics/ball-bounce-rolling/rolling.js';

const R = 1.391;
const mkBall = (o = {}) => Object.assign({ x: 0, y: 0, z: R, vx: 0, vy: 0, vz: 0, wx: 0, wy: 0, wz: 0, r: R, k: 0.636, _c: true, _cPrev: true, sleepT: 0, sleeping: false }, o);
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg || ''} ${a} vs ${b} (tol ${tol})`);

test('rollingSpinFor: rolling in +x on the floor gives +wy = v/r', () => {
  const out = [0, 0, 0];
  rollingSpinFor(40, 0, 0, 0, 0, 1, R, out);
  near(out[0], 0, 1e-12); near(out[1], 40 / R, 1e-12); near(out[2], 0, 1e-12);
});

test('launchSpin: ratio > 0 is backspin (wy < 0 for +x travel); zero for vertical shot', () => {
  const out = [0, 0, 0];
  launchSpin(100, 0, 0, R, 1, out);
  near(out[1], -100 / R, 1e-9); near(out[0], 0, 1e-12); near(out[2], 0, 1e-12);
  launchSpin(0, 0, 80, R, 1, out);
  assert.deepEqual([out[0], out[1], out[2]], [0, 0, 0]);
  launchSpin(100, 0, 0, R, -1, out);
  assert.ok(out[1] > 0, 'negative ratio is topspin');
});

test('rolling resistance keeps no-slip and slows by jn*crr', () => {
  const b = mkBall({ vx: 40, wy: 40 / R });
  const h = 1 / 240, jn = 386.0886 * h;
  applyRollingResistance(b, 0, 0, 1, jn, 0, 0, 0, SURFACES.floor);
  near(b.vx, 40 - jn * SURFACES.floor.crr, 1e-9);
  near(b.vx - R * b.wy, 0, 1e-9, 'no-slip preserved');
});

test('rolling resistance never reverses the velocity', () => {
  const b = mkBall({ vx: 0.1, wy: 0.1 / R });
  applyRollingResistance(b, 0, 0, 1, 100, 0, 0, 0, SURFACES.floor);
  assert.equal(b.vx, 0); assert.equal(b.wy, 0);
});

test('rolling resistance is a no-op with jn = 0', () => {
  const b = mkBall({ vx: 40, wy: 40 / R });
  applyRollingResistance(b, 0, 0, 1, 0, 0, 0, 0, SURFACES.floor);
  assert.equal(b.vx, 40);
});

test('torsional friction reduces |wn| toward 0 and never past it', () => {
  const b = mkBall({ wz: 10 });
  applyTorsionalFriction(b, 0, 0, 1, 1.6, SURFACES.floor);
  assert.ok(b.wz < 10 && b.wz >= 0);
  applyTorsionalFriction(b, 0, 0, 1, 1e9, SURFACES.floor);
  assert.equal(b.wz, 0);
  const c = mkBall({ wz: -10 });
  applyTorsionalFriction(c, 0, 0, 1, 1e9, SURFACES.floor);
  assert.equal(c.wz, 0);
});

test('sleepMetric = |v| + |w|*r', () => {
  near(sleepMetric(mkBall({ vx: 3, vy: 4, wz: 2 })), 5 + 2 * R, 1e-12);
});

test('updateSleep: sleeps after sleepTime in slow contact, zeroing v and w', () => {
  const b = mkBall({ vx: 0.5, wy: 0.2 });
  const h = 0.01;
  let n = 0, slept = false;
  while (!slept && n < 1000) { slept = updateSleep(b, h); n++; }
  assert.ok(slept);
  near(n * h, BBR.sleepTime, 2 * h);
  assert.equal(b.sleeping, true);
  assert.equal(b.vx, 0); assert.equal(b.wy, 0);
});

test('updateSleep: fast, contact-less or spinning balls do not sleep and reset the timer', () => {
  const fast = mkBall({ vx: 20, sleepT: 0.2 });
  assert.equal(updateSleep(fast, 0.1), false); assert.equal(fast.sleepT, 0);
  const noContact = mkBall({ _c: false, sleepT: 0.2 });
  assert.equal(updateSleep(noContact, 0.1), false); assert.equal(noContact.sleepT, 0);
  const spinning = mkBall({ wy: 5 });   // |w|*r = 6.9 > 1.5
  assert.equal(updateSleep(spinning, 0.5), false); assert.equal(spinning.sleeping, false);
});

test('isAirborne uses contact memory and height', () => {
  assert.equal(isAirborne(mkBall({ _cPrev: false, z: R + 5 })), true);
  assert.equal(isAirborne(mkBall({ _cPrev: true, z: R + 5 })), false);
  assert.equal(isAirborne(mkBall({ _cPrev: false, z: R + 0.01 })), false);
});

test('clampSpin limits |w|', () => {
  const b = mkBall({ wx: 3000, wy: 4000 });
  clampSpin(b, 1000);
  near(Math.hypot(b.wx, b.wy, b.wz), 1000, 1e-9);
  const c = mkBall({ wx: 10 });
  clampSpin(c, 1000);
  assert.equal(c.wx, 10);
});
