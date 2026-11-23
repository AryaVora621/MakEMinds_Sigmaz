// Round-2 fixes in ball-bounce-rolling. Units: in, s, rad/s.
//  1. rolling.js: every helper treats missing wx/wy/wz as 0 (a ball built outside BallWorld has no spin fields).
//  2. contact.js: rolling resistance acts only once the contact is ROLLING (stuck); while it slides, Coulomb friction is
//     the only tangential force, so L = v_t + k*r*w (angular momentum about the contact point / (m r)) is conserved.
import test from 'node:test';
import assert from 'node:assert/strict';
import { BBR, SURFACES } from '../../web/js/physics/ball-bounce-rolling/params.js';
import { applyRollingResistance, applyTorsionalFriction, sleepMetric, updateSleep, clampSpin }
  from '../../web/js/physics/ball-bounce-rolling/rolling.js';
import { resolveContact } from '../../web/js/physics/ball-bounce-rolling/contact.js';

const R = 1.391;   // in, pollen radius (balls.js BALL_KINDS)
const K = 0.636;   // I/(m r^2), pollen hollow shell (params.js inertiaFactor)
const noSpin = (o = {}) => Object.assign({ x: 0, y: 0, z: R, vx: 0, vy: 0, vz: 0, r: R, k: K, _c: true, sleepT: 0, sleeping: false }, o);
const finite = (b) => ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'].every(k => b[k] === undefined || Number.isFinite(b[k]));

test('rolling helpers accept a ball with no spin fields (|| 0 guards)', () => {
  const b1 = noSpin({ vx: 30 });
  applyRollingResistance(b1, 0, 0, 1, BBR.g * 0.004, 0, 0, 0, SURFACES.floor);
  assert.ok(finite(b1) && b1.vx < 30 && b1.wx === 0 && b1.wy === 0 && b1.wz === 0, JSON.stringify(b1));
  const b2 = noSpin();
  applyTorsionalFriction(b2, 0, 0, 1, 1, SURFACES.floor);
  assert.ok(finite(b2));
  assert.equal(sleepMetric(noSpin({ vx: 1 })), 1);
  const b3 = noSpin(); updateSleep(b3, 1); assert.ok(b3.sleeping && finite(b3));
  const b4 = noSpin(); clampSpin(b4, 10); assert.ok(finite(b4));
});

test('sliding floor contact keeps vx + k*r*wy (no rolling resistance until the contact rolls)', () => {
  // the state of a rolling ball just after a head-on hit (balls.test.mjs): moving back, still spinning forward
  const b = noSpin({ vx: -7.65, wx: 0, wy: 29.10 / R, wz: 0 });
  const L0 = b.vx + K * R * b.wy;
  const h = 0.004;   // s, BALL.maxSubstep
  let rolledAt = -1;
  for (let i = 0; i < 400; i++) {
    b.vz -= BBR.g * h; b.z += b.vz * h;   // gravity, then floor contact
    resolveContact(b, 0, 0, 1, Math.max(0, R - b.z), SURFACES.floor, 0, 0, 0, h);
    const slip = b.vx - R * b.wy;
    if (rolledAt < 0 && Math.abs(slip) < 1e-6) rolledAt = i;
    if (rolledAt < 0) assert.ok(Math.abs(b.vx + K * R * b.wy - L0) < 1e-9 * Math.abs(L0) + 1e-9, `L drifted at ${i}`);
  }
  assert.ok(rolledAt > 0, 'it reaches rolling');
  // analytic sliding time: slip s0 = -36.75 in/s falls at mu*g*(1+1/k) per second -> t = |s0| / (mu g (1 + 1/k))
  const tSlide = 36.75 / (SURFACES.floor.muSlide * BBR.g * (1 + 1 / K));
  assert.ok(Math.abs(rolledAt * h - tSlide) < 3 * h, `rolled at ${rolledAt * h} s, analytic ${tSlide} s`);
  // once rolling it decelerates at crr*g: v(end) = L0/(1+k) - crr*g*(t_end - t_roll)
  const vEnd = L0 / (1 + K) - SURFACES.floor.crr * BBR.g * (400 - rolledAt) * h;
  assert.ok(Math.abs(b.vx - Math.max(0, vEnd)) < 0.05, `vx ${b.vx} vs ${vEnd}`);
});
