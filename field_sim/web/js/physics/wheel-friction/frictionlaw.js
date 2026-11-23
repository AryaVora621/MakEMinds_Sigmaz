// frictionlaw.js - Stribeck friction, grip clamps, slip ratio (spec section 4).
// Pure numeric, no imports. SI units. mu is dimensionless; speeds in m/s.

/**
 * Stribeck curve: muK + (muS - muK) * exp(-(|v|/vS)^delta).
 * @param {number} slipSpeedMS slip speed, m/s
 * @param {number} muStatic dimensionless
 * @param {number} muKinetic dimensionless
 * @param {number} vS Stribeck velocity, m/s
 * @param {number} delta Stribeck exponent, dimensionless
 */
export function stribeckMu(slipSpeedMS, muStatic, muKinetic, vS, delta) {
  if (!(vS > 0)) return muKinetic;
  const r = Math.abs(slipSpeedMS) / vS;
  return muKinetic + (muStatic - muKinetic) * Math.exp(-Math.pow(r, delta));
}

/**
 * Effective mu for a wheel given previous-substep slip speed (Karnopp stick band).
 * @param {number} prevSlipMS previous slip speed, m/s
 * @param {{muStatic:number, muKinetic:number}} wheel
 * @param {{stribeckVelocityMS:number, stribeckExponent:number, stickVelocityMS:number}} friction
 * @param {number} muScale dimensionless grip scale
 */
export function selectMu(prevSlipMS, wheel, friction, muScale) {
  const a = Math.abs(prevSlipMS);
  const mu = a < friction.stickVelocityMS
    ? wheel.muStatic
    : stribeckMu(a, wheel.muStatic, wheel.muKinetic, friction.stribeckVelocityMS, friction.stribeckExponent);
  return mu * muScale;
}

/** Clamp value to [-|limit|, |limit|]. A limit <= 0 returns 0. */
export function coulombClamp(value, limit) {
  const L = Math.abs(limit);
  if (!(L > 0)) return 0;
  return value > L ? L : (value < -L ? -L : value);
}

/** Radially scale (a,b) so hypot <= limit. limit <= 0 returns [0,0]. */
export function clampCircle(a, b, limit) {
  if (!(limit > 0)) return [0, 0];
  const n = Math.hypot(a, b);
  if (n <= limit) return [a, b];
  const s = limit / n;
  return [a * s, b * s];
}

/** Radially scale (a,b) so (a/limA)^2 + (b/limB)^2 <= 1. Any limit <= 0 returns [0,0]. */
export function clampEllipse(a, b, limA, limB) {
  if (!(limA > 0) || !(limB > 0)) return [0, 0];
  const q = (a / limA) * (a / limA) + (b / limB) * (b / limB);
  if (q <= 1) return [a, b];
  const s = 1 / Math.sqrt(q);
  return [a * s, b * s];
}

/**
 * Slip ratio (telemetry only): (surface - ground) / max(|ground|, |surface|, vMin). Finite at 0/0.
 * @param {number} surfaceSpeedMS wheel surface speed, m/s
 * @param {number} groundSpeedMS ground speed along the drive direction, m/s
 * @param {number} vMinMS regularizer, m/s
 */
export function slipRatio(surfaceSpeedMS, groundSpeedMS, vMinMS) {
  const d = Math.max(Math.abs(groundSpeedMS), Math.abs(surfaceSpeedMS), vMinMS);
  if (!(d > 0)) return 0;
  return (surfaceSpeedMS - groundSpeedMS) / d;
}
