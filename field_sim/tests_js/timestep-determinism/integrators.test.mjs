import test from 'node:test';
import assert from 'node:assert/strict';
import { TS } from '../../web/js/physics/timestep-determinism/params.js';
import {
  semiImplicitEuler, explicitEuler, rk4Step, makeRk4Scratch, approach, lagCoeff, lagStep,
  forwardEulerLagStep, integrateBodyTwist,
} from '../../web/js/physics/timestep-determinism/integrators.js';
import {
  tauFromPerFrame, lpfAlpha, LowPass, FiniteDiffVelocity,
} from '../../web/js/physics/timestep-determinism/smoothing.js';

const H = 1 / 240;

test('semi-implicit Euler keeps spring energy within 2% over 90 s; explicit Euler grows', () => {
  const k = 15; // [1/s^2], unit mass
  const energy = (x, v) => 0.5 * v * v + 0.5 * k * x * x;
  const e0 = energy(1, 0);
  let s = { x: 1, v: 0 }, e = { x: 1, v: 0 };
  let maxDev = 0;
  const n = 90 * 240;
  for (let i = 0; i < n; i++) {
    s = semiImplicitEuler(s.x, s.v, -k * s.x, H);
    e = explicitEuler(e.x, e.v, -k * e.x, H);
    maxDev = Math.max(maxDev, Math.abs(energy(s.x, s.v) - e0) / e0);
  }
  assert.ok(maxDev < 0.02, `semi-implicit max deviation ${maxDev}`);
  assert.ok(energy(e.x, e.v) > 10 * e0, 'explicit Euler should gain energy');
});

test('rk4Step on x\' = -x matches exp(-t) within 1e-10 at t = 1', () => {
  const y = new Float64Array([1]);
  const scratch = makeRk4Scratch(1);
  const f = (t, yy, out) => { out[0] = -yy[0]; };
  for (let i = 0; i < 240; i++) rk4Step(f, i * H, y, H, scratch);
  assert.ok(Math.abs(y[0] - Math.exp(-1)) < 1e-10, `err ${Math.abs(y[0] - Math.exp(-1))}`);
});

test('lagStep follows the exact step response independent of h', () => {
  const tau = 0.03; // [s]
  const t = 0.1; // [s]
  const exact = 1 - Math.exp(-t / tau);
  for (const [h, n] of [[1 / 240, 24], [1 / 60, 6]]) {
    let x = 0;
    for (let i = 0; i < n; i++) x = lagStep(x, 1, h, tau);
    assert.ok(Math.abs(x - exact) < 1e-12, `h=${h}: ${x} vs ${exact}`);
  }
  assert.ok(Math.abs(lagCoeff(H, tau) - (1 - Math.exp(-H / tau))) < 1e-15);
  assert.equal(lagCoeff(H, 0), 1);
});

test('forwardEulerLagStep at h=1/240, tau=0.02 has effective tau ~17.8 ms', () => {
  const tau = 0.02;
  const g = forwardEulerLagStep(0, 1, H, tau); // one-step gain
  const tauEff = -H / Math.log(1 - g);
  assert.ok(Math.abs(tauEff * 1000 - 17.8) < 0.2, `tauEff ${tauEff * 1000} ms`);
});

test('approach respects accel*h bound and reaches the target', () => {
  let v = 0;
  const target = 5, accel = 10, decel = 30;
  let prev = v;
  for (let i = 0; i < 400; i++) {
    v = approach(v, target, accel, decel, H);
    assert.ok(Math.abs(v - prev) <= accel * H + 1e-12);
    prev = v;
  }
  assert.ok(Math.abs(v - target) < 1e-12);
  // decelerating uses the decel bound
  const v1 = approach(5, 0, accel, decel, H);
  assert.ok(Math.abs((5 - v1) - decel * H) < 1e-12);
  // sign flip counts as decel (first step bounded by decel)
  const v2 = approach(1, -1, accel, decel, H);
  assert.ok(Math.abs((1 - v2) - decel * H) < 1e-12);
});

test('integrateBodyTwist: full circle returns to start; step splitting is exact', () => {
  const n = 2400, dt = (2 * Math.PI) / n;
  const p = { x: 0, y: 0, h: 0 };
  for (let i = 0; i < n; i++) integrateBodyTwist(p, 10, 0, 1, dt);
  assert.ok(Math.abs(p.x) < 1e-9 && Math.abs(p.y) < 1e-9, `${p.x},${p.y}`);

  const a = { x: 1, y: 2, h: 0.3 };
  const b = { x: 1, y: 2, h: 0.3 };
  integrateBodyTwist(a, 10, 3, 1.5, 0.2);
  integrateBodyTwist(b, 10, 3, 1.5, 0.1);
  integrateBodyTwist(b, 10, 3, 1.5, 0.1);
  assert.ok(Math.abs(a.x - b.x) < 1e-9 && Math.abs(a.y - b.y) < 1e-9 && Math.abs(a.h - b.h) < 1e-9);
});

test('integrateBodyTwist small-angle branch is continuous at EXP_MAP_EPS', () => {
  const dt = 1;
  const eps = TS.EXP_MAP_EPS;
  const lo = { x: 0, y: 0, h: 0 }, hi = { x: 0, y: 0, h: 0 };
  integrateBodyTwist(lo, 3, 2, eps * 0.999, dt);
  integrateBodyTwist(hi, 3, 2, eps * 1.001, dt);
  assert.ok(Math.abs(lo.x - hi.x) < 1e-8);
  assert.ok(Math.abs(lo.y - hi.y) < 1e-8);
  assert.ok(Math.abs(lo.x - 3) < 1e-8 && Math.abs(lo.y - 2) < 1e-8);
});

test('tauFromPerFrame(0.3, 60) ~ 0.046727', () => {
  assert.ok(Math.abs(tauFromPerFrame(0.3, 60) - 0.046727) < 1e-6);
  assert.ok(Math.abs(tauFromPerFrame(0.3, 60) - TS.LINKED_VEL_LPF_TAU) < 1e-6);
});

test('lpfAlpha edge cases', () => {
  assert.equal(lpfAlpha(0, 0.1), 0);
  assert.equal(lpfAlpha(-1, 0.1), 0);
  assert.equal(lpfAlpha(0.1, 0), 1);
  assert.ok(Math.abs(lpfAlpha(0.05, 0.05) - (1 - Math.exp(-1))) < 1e-15);
});

test('LowPass step response is frame-rate independent', () => {
  const tau = 0.0467;
  const vals = [30, 60, 144].map((rate) => {
    const lp = new LowPass(tau, 0);
    for (let i = 0; i < rate; i++) lp.step(1, 1 / rate);
    return lp.value;
  });
  const exact = 1 - Math.exp(-1 / tau);
  for (const v of vals) assert.ok(Math.abs(v - exact) < 1e-12, `${v} vs ${exact}`);
  assert.ok(Math.abs(vals[0] - vals[2]) < 1e-12);
});

test('FiniteDiffVelocity unwraps angle across pi', () => {
  const fd = new FiniteDiffVelocity({ h: H, angular: true });
  assert.equal(fd.push(3.1), 0);
  const v = fd.push(-3.1); // short arc is +0.0832 rad, not -6.2
  const expected = (2 * Math.PI - 6.2) / H;
  assert.ok(Math.abs(v - expected) < 1e-9, `${v} vs ${expected}`);
  const lin = new FiniteDiffVelocity({ h: H });
  lin.push(0);
  assert.ok(Math.abs(lin.push(1) - 240) < 1e-9);
});
