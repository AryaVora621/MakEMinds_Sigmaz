// smoothing.js - frame-rate-independent filters and finite differences (spec 2.5, 2.6).
// No randomness, no time reads; dt is always passed in. Units: s, rad, in.
import { TS } from './params.js';

const TWO_PI = 2 * Math.PI;
function wrapAngle(a) {
  if (!Number.isFinite(a)) return a;
  let r = a % TWO_PI;
  if (r > Math.PI) r -= TWO_PI;
  else if (r <= -Math.PI) r += TWO_PI;
  return r === 0 ? 0 : r;
}

// Per-frame smoothing factor k at refHz -> time constant [s]. tau = -1/(refHz*ln(1-k)).
// Valid for 0<k<1. [derived] k=0.3 @ 60 Hz gives 0.046727 s.
export function tauFromPerFrame(k, refHz = TS.LPF_REF_HZ) {
  return -1 / (refHz * Math.log(1 - k));
}

// Exact first-order smoothing gain over dt [s]: 1 - exp(-dt/tau). dt<=0 -> 0, tau<=0 -> 1.
export function lpfAlpha(dt, tau) {
  if (!(dt > 0)) return 0;
  if (!(tau > 0)) return 1;
  return 1 - Math.exp(-dt / tau);
}

export class LowPass {
  constructor(tau, initial = 0) {
    this.tau = tau;
    this.value = initial;
  }
  step(x, dt) {
    this.value += lpfAlpha(dt, this.tau) * (x - this.value);
    return this.value;
  }
  reset(v = 0) { this.value = v; }
}

// Render-side damping, lambda [1/s]: cur + (target-cur)*(1-exp(-lambda*dt)). Never hashed.
export function damp(cur, target, lambda, dt) {
  if (!(dt > 0)) return cur;
  return cur + (target - cur) * (1 - Math.exp(-lambda * dt));
}

// Same along the shortest arc; result wrapped to (-pi, pi].
export function dampAngle(cur, target, lambda, dt) {
  const d = wrapAngle(target - cur);
  return wrapAngle(cur + d * (dt > 0 ? 1 - Math.exp(-lambda * dt) : 0));
}

// Finite-difference velocity at fixed step h [s]. First push returns 0.
// angular: difference wrapped to (-pi, pi]. tau>0: output passes through an exact LPF.
export class FiniteDiffVelocity {
  constructor({ h = TS.PHYSICS_DT, angular = false, tau = 0 } = {}) {
    this.h = h;
    this.angular = angular;
    this.tau = tau;
    this.reset();
  }
  push(value) {
    if (!this._has) {
      this._has = true;
      this._prev = value;
      this.value = 0;
      return 0;
    }
    let d = value - this._prev;
    if (this.angular) d = wrapAngle(d);
    this._prev = value;
    const raw = d / this.h;
    this.value = this.tau > 0 ? this.value + lpfAlpha(this.h, this.tau) * (raw - this.value) : raw;
    return this.value;
  }
  reset() {
    this._has = false;
    this._prev = 0;
    this.value = 0;
  }
}
