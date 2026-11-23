// spin.test.mjs - backspin convention, Magnus direction, launch spin, decay, contact, CL curve.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AERO } from '../../web/js/physics/ball-aerodynamics/params.js';
import { buildAeroModel } from '../../web/js/physics/ball-aerodynamics/air.js';
import { accel, backspinAxis, clOfSpin } from '../../web/js/physics/ball-aerodynamics/forces.js';
import { launchSpin, decaySpin, applyContactSpin } from '../../web/js/physics/ball-aerodynamics/spin.js';

const withLift = buildAeroModel(AERO, AERO.kinds.pollen, { model: 'physical', lift: { ...AERO.lift, enabled: true } });
const noLift = buildAeroModel(AERO, AERO.kinds.pollen, { model: 'physical', lift: { ...AERO.lift, enabled: false } });
const g = withLift.g; // in/s^2

test('backspinAxis(+x) = [0,-1,0]', () => {
  const a = backspinAxis(1, 0, 0);
  assert.deepEqual([a[0] + 0, a[1] + 0, a[2] + 0], [0, -1, 0]);
  assert.deepEqual(backspinAxis(0, 0, 5), [0, 0, 0]);
});

test('Magnus: backspin lifts, topspin pushes down (+x flight)', () => {
  const out = [0, 0, 0];
  const v = 500, w = 200; // in/s, rad/s
  accel(withLift, v, 0, 0, 0, -w, 0, true, out);
  assert.ok(out[2] > -g, `backspin az ${out[2]}`);
  accel(withLift, v, 0, 0, 0, +w, 0, true, out);
  assert.ok(out[2] < -g, `topspin az ${out[2]}`);
});

test('Magnus magnitude = base*CL*v^2 (in/s^2)', () => {
  const out = [0, 0, 0], v = 500, w = 200;
  accel(withLift, v, 0, 0, 0, -w, 0, true, out);
  const S = withLift.radiusIn * w / v;
  const lift = out[2] + g; // drag has no z component for a +x velocity
  const want = withLift.base * clOfSpin(S, withLift.lift) * v * v;
  assert.ok(Math.abs(lift - want) / want < 1e-9, `${lift} vs ${want}`);
});

test('launchSpin magnitude = slip*v/r (rad/s) along backspin axis', () => {
  const v = 600, r = 1.391, slip = 0.5;
  const w = launchSpin(v, 0, 0, r, slip);
  assert.ok(Math.abs(Math.hypot(...w) - slip * v / r) < 1e-9);
  assert.ok(w[1] < 0 && Math.abs(w[0]) < 1e-12);
  const w2 = launchSpin(300, 300, 400, r, slip);
  assert.ok(Math.abs(Math.hypot(...w2) - slip * Math.hypot(300, 300, 400) / r) < 1e-9);
});

test('decaySpin over 1 s with tau 2 s gives exp(-0.5)', () => {
  const s = { wx: 10, wy: -20, wz: 30 };
  decaySpin(s, 1, 2);
  const f = Math.exp(-0.5);
  assert.ok(Math.abs(s.wx - 10 * f) < 1e-12 && Math.abs(s.wy + 20 * f) < 1e-12 && Math.abs(s.wz - 30 * f) < 1e-12);
  const n = { x: 1 };
  decaySpin(n, 1, 2);
  assert.equal(n.wx, undefined);
});

test('applyContactSpin halves w', () => {
  const s = { wx: 8, wy: -4, wz: 2 };
  applyContactSpin(s, 0.5);
  assert.deepEqual([s.wx, s.wy, s.wz], [4, -2, 1]);
});

test('clOfSpin: 0 at S=0, monotone, capped at clMax', () => {
  const L = withLift.lift;
  assert.equal(clOfSpin(0, L), 0);
  let prev = 0;
  for (let S = 0.01; S < 5; S += 0.01) {
    const c = clOfSpin(S, L);
    assert.ok(c >= prev - 1e-15, `not monotone at S=${S}`);
    assert.ok(c <= L.clMax + 1e-15);
    prev = c;
  }
  assert.equal(clOfSpin(100, L), L.clMax);
});

test('with lift disabled accel ignores spin', () => {
  const a = [0, 0, 0], b = [0, 0, 0];
  accel(noLift, 500, 100, 50, 0, 0, 0, true, a);
  accel(noLift, 500, 100, 50, 300, -200, 100, true, b);
  assert.deepEqual(a, b);
});
