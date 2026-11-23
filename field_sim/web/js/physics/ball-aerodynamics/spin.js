// spin.js - ball spin helpers (ESM, pure). Units: rad/s, inches, seconds.
// Launch spin basis: single-wheel shooter, fixed hood, ball rolling without slip gives w = v/r; slip scales it.
// slip (0.5), decay tau (2.0 s) and contact keep (0.5) are ASSUMED (see params.js SOURCES.lift).
import { backspinAxis } from './forces.js';

// Launch spin vector [wx,wy,wz] rad/s: |w| = slip*|v|/radiusIn along the backspin axis.
export function launchSpin(vx, vy, vz, radiusIn, slip) {
  const ax = backspinAxis(vx, vy, vz);
  const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
  const mag = radiusIn > 0 ? (slip * sp) / radiusIn : 0;
  return [ax[0] * mag, ax[1] * mag, ax[2] * mag];
}

// Exponential spin decay over h seconds: w *= exp(-h/tau). No-op if fields absent or tau <= 0.
export function decaySpin(s, h, tau) {
  if (!(tau > 0) || s.wx === undefined) return s;
  const f = Math.exp(-h / tau);
  s.wx *= f; s.wy *= f; s.wz *= f;
  return s;
}

// Spin parameter S = r|w|/|v| (dimensionless); 0 if |v| < 1e-9 or no spin fields.
export function spinParameter(s, radiusIn) {
  const sp = Math.sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);
  if (sp < 1e-9 || s.wx === undefined) return 0;
  const w = Math.sqrt(s.wx * s.wx + s.wy * s.wy + s.wz * s.wz);
  return (radiusIn * w) / sp;
}

// Floor/wall contact: w *= keep (dimensionless fraction). No-op if fields absent.
export function applyContactSpin(s, keep) {
  if (s.wx === undefined) return s;
  s.wx *= keep; s.wy *= keep; s.wz *= keep;
  return s;
}
