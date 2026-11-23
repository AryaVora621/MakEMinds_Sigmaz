// integrator.js - per-substep integrators for free balls. State s mutated in place: {x,y,z,vx,vy,vz,wx?,wy?,wz?}.
import { accel } from './forces.js';

// Exact copy of balls.js _substep lines 526-532 (operation order matters for bit-identity).
export function stepLegacy(s, h, model, airborne) {
  const g = model.g;
  if (airborne) {
    const sp = Math.sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz), f = 1 / (1 + model.kDrag * sp * h);
    s.vx *= f; s.vy *= f; s.vz *= f;
  }
  s.x += s.vx * h; s.y += s.vy * h; s.z += s.vz * h - 0.5 * g * h * h; s.vz -= g * h;
}

const A1 = [0, 0, 0], A2 = [0, 0, 0], A3 = [0, 0, 0], A4 = [0, 0, 0];

// Classic RK4 on (p, v); spin held fixed over the step, then decays by exp(-h/tau).
export function stepRK4(s, h, model, airborne) {
  const wx = s.wx || 0, wy = s.wy || 0, wz = s.wz || 0;
  const vx = s.vx, vy = s.vy, vz = s.vz, hh = 0.5 * h;
  accel(model, vx, vy, vz, wx, wy, wz, airborne, A1);
  const v2x = vx + hh * A1[0], v2y = vy + hh * A1[1], v2z = vz + hh * A1[2];
  accel(model, v2x, v2y, v2z, wx, wy, wz, airborne, A2);
  const v3x = vx + hh * A2[0], v3y = vy + hh * A2[1], v3z = vz + hh * A2[2];
  accel(model, v3x, v3y, v3z, wx, wy, wz, airborne, A3);
  const v4x = vx + h * A3[0], v4y = vy + h * A3[1], v4z = vz + h * A3[2];
  accel(model, v4x, v4y, v4z, wx, wy, wz, airborne, A4);
  const h6 = h / 6;
  s.x += h6 * (vx + 2 * v2x + 2 * v3x + v4x);
  s.y += h6 * (vy + 2 * v2y + 2 * v3y + v4y);
  s.z += h6 * (vz + 2 * v2z + 2 * v3z + v4z);
  s.vx = vx + h6 * (A1[0] + 2 * A2[0] + 2 * A3[0] + A4[0]);
  s.vy = vy + h6 * (A1[1] + 2 * A2[1] + 2 * A3[1] + A4[1]);
  s.vz = vz + h6 * (A1[2] + 2 * A2[2] + 2 * A3[2] + A4[2]);
  const tau = model.spinDecayTau;
  if (tau > 0 && (s.wx !== undefined || s.wy !== undefined || s.wz !== undefined)) {
    const d = Math.exp(-h / tau);
    if (s.wx !== undefined) s.wx *= d;
    if (s.wy !== undefined) s.wy *= d;
    if (s.wz !== undefined) s.wz *= d;
  }
}

export function stepFlight(s, h, model, airborne, method = model.integrator) {
  if (method === 'rk4') stepRK4(s, h, model, airborne);
  else stepLegacy(s, h, model, airborne);
}

export function clampSpeed(s, maxSpeed) {
  const sp2 = s.vx * s.vx + s.vy * s.vy + s.vz * s.vz;
  if (sp2 > maxSpeed * maxSpeed) { const k = maxSpeed / Math.sqrt(sp2); s.vx *= k; s.vy *= k; s.vz *= k; }
}

export function substepCount(speed, dt, model) {
  return Math.max(1, Math.ceil(dt / model.maxSubstep), Math.ceil(speed * dt / model.maxStepTravel));
}
