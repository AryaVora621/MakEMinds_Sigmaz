// integrators.js - pure numerical kernels for the fixed-timestep layer (spec section 2).
// No randomness, no time reads. Units: lengths in, angles rad, time s.
// Tags: [sourced] = cited reference, [assumed] = modelling choice.
import { TS } from './params.js';

const TWO_PI = 2 * Math.PI;

// Wrap to (-pi, pi]. Local copy (independent of interp.js).
export function wrapAngle(a) {
  if (!Number.isFinite(a)) return a;
  let r = a % TWO_PI;
  if (r > Math.PI) r -= TWO_PI;
  else if (r <= -Math.PI) r += TWO_PI;
  return r === 0 ? 0 : r;
}

// Same wrap as driver.js `wrap` (while loops, range [-pi, pi]); used only for bit compatibility.
function legacyWrap(a) {
  while (a > Math.PI) a -= TWO_PI;
  while (a < -Math.PI) a += TWO_PI;
  return a;
}

// ---- second-order states -------------------------------------------------------------------

// Semi-implicit (symplectic) Euler [sourced: Fiedler "Integration Basics",
// https://gafferongames.com/post/integration_basics/]. Order 1, stable on oscillators.
// x [any unit], v [unit/s], a [unit/s^2], h [s].
export function semiImplicitEuler(x, v, a, h) {
  const v1 = v + a * h;
  return { x: x + v1 * h, v: v1 };
}

// Explicit Euler. Reference/testing only: gains energy on oscillators.
export function explicitEuler(x, v, a, h) {
  return { x: x + v * h, v: v + a * h };
}

export function makeRk4Scratch(n) {
  return { k1: new Float64Array(n), k2: new Float64Array(n), k3: new Float64Array(n),
           k4: new Float64Array(n), tmp: new Float64Array(n) };
}

// Classic RK4 [sourced: Fiedler "Integration Basics"], weights 1/6, 2/6, 2/6, 1/6.
// f(t, y, out) writes dy/dt into out. y is mutated in place and returned.
export function rk4Step(f, t, y, h, scratch = null) {
  const n = y.length;
  const s = scratch && scratch.k1.length === n ? scratch : makeRk4Scratch(n);
  const { k1, k2, k3, k4, tmp } = s;
  const hh = 0.5 * h;
  f(t, y, k1);
  for (let i = 0; i < n; i++) tmp[i] = y[i] + hh * k1[i];
  f(t + hh, tmp, k2);
  for (let i = 0; i < n; i++) tmp[i] = y[i] + hh * k2[i];
  f(t + hh, tmp, k3);
  for (let i = 0; i < n; i++) tmp[i] = y[i] + h * k3[i];
  f(t + h, tmp, k4);
  for (let i = 0; i < n; i++) {
    y[i] += h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6;
  }
  return y;
}

// ---- slew limiter --------------------------------------------------------------------------

// Move v toward target limited to accel*h (speeding up) or decel*h (slowing down / reversing).
// Decel applies when |target| < |v| or the sign flips. accel/decel [unit/s^2], h [s].
export function approach(v, target, accel, decel, h) {
  const slowing = Math.abs(target) < Math.abs(v) || (v !== 0 && target !== 0 && (v < 0) !== (target < 0));
  const step = Math.abs(slowing ? decel : accel) * h;
  const d = target - v;
  if (d > step) return v + step;
  if (d < -step) return v - step;
  return target;
}

// ---- first-order lag, exact zero-order-hold ------------------------------------------------

// x' = (u - x)/tau  =>  x_{n+1} = x + (1 - exp(-h/tau)) * (u - x)   [exact for u held over h].
// Cache keyed on (tau, h) so Math.exp runs once per pair. tau<=0 means no lag (coefficient 1).
const lagCache = new Map();
export function lagCoeff(h, tau) {
  if (!(tau > 0)) return 1;
  let inner = lagCache.get(tau);
  if (!inner) {
    if (lagCache.size > 256) lagCache.clear();
    inner = new Map();
    lagCache.set(tau, inner);
  }
  let c = inner.get(h);
  if (c === undefined) {
    if (inner.size > 64) inner.clear();
    c = 1 - Math.exp(-h / tau);
    inner.set(h, c);
  }
  return c;
}

export function lagStep(x, u, h, tau) {
  return x + lagCoeff(h, tau) * (u - x);
}

// Forward Euler lag, for comparison tests only (effective tau differs from tau).
export function forwardEulerLagStep(x, u, h, tau) {
  return x + (h / tau) * (u - x);
}

// ---- pose integration ----------------------------------------------------------------------

// Exact SE(2) exponential for a constant body twist over dt (spec 2.2).
// pose {x [in], y [in], h [rad]} mutated; vxBody/vyBody [in/s], om [rad/s], dt [s].
export function integrateBodyTwist(pose, vxBody, vyBody, om, dt) {
  const dth = om * dt;
  let s, c;
  if (Math.abs(dth) > TS.EXP_MAP_EPS) {
    s = Math.sin(dth) / dth;
    c = (1 - Math.cos(dth)) / dth;
  } else {
    s = 1 - (dth * dth) / 6;
    c = dth / 2;
  }
  const bx = vxBody * dt, by = vyBody * dt;
  const dxb = s * bx - c * by;
  const dyb = c * bx + s * by;
  const ch = Math.cos(pose.h), sh = Math.sin(pose.h);
  pose.x += ch * dxb - sh * dyb;
  pose.y += sh * dxb + ch * dyb;
  pose.h = wrapAngle(pose.h + dth);
  return pose;
}

// Euler update for field-frame velocities; bit-compatible with driver.js integrate().
export function integrateFieldVel(pose, vx, vy, om, dt) {
  pose.x += vx * dt;
  pose.y += vy * dt;
  pose.h = legacyWrap(pose.h + om * dt);
  return pose;
}
