// rolling.js - rolling resistance, torsional spin friction, sleep and spin helpers.
// Units: inches, seconds, radians, in/s, rad/s. Pedro frame (x right, y far wall, z up).
// Imports only params.js (never contact.js: that would be a cycle). No allocation in hot paths.
import { BBR } from './params.js';

// Rolling resistance: deceleration of the contact-point speed proportional to the normal impulse jn.
// dv = jn*(crr + cv*s/g) [in/s]. vt and the spin component perpendicular to n are scaled by the same
// factor f, so a ball rolling without slip keeps rolling without slip.
export function applyRollingResistance(b, nx, ny, nz, jn, vsx, vsy, vsz, surf) {
  if (!(jn > 0)) return;
  const rx = b.vx - vsx, ry = b.vy - vsy, rz = b.vz - vsz;
  const vn = rx * nx + ry * ny + rz * nz;
  const tx = rx - vn * nx, ty = ry - vn * ny, tz = rz - vn * nz;
  const s = Math.sqrt(tx * tx + ty * ty + tz * tz);
  if (s < 1e-12) return;
  const dv = jn * (surf.crr + surf.cv * s / BBR.g);
  const f = s <= dv ? 0 : (s - dv) / s;
  b.vx = vsx + vn * nx + tx * f;
  b.vy = vsy + vn * ny + ty * f;
  b.vz = vsz + vn * nz + tz * f;
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;   // rad/s; a ball made outside BallWorld may lack spin fields
  const wn = wx * nx + wy * ny + wz * nz;
  b.wx = wn * nx + (wx - wn * nx) * f;
  b.wy = wn * ny + (wy - wn * ny) * f;
  b.wz = wn * nz + (wz - wn * nz) * f;
}

// Torsional (spin about the contact normal) friction: dw = jn*spinLen/(k r^2) [rad/s], never past zero.
export function applyTorsionalFriction(b, nx, ny, nz, jn, surf) {
  if (!(jn > 0)) return;
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;   // rad/s, missing spin fields read as 0
  const wn = wx * nx + wy * ny + wz * nz;
  if (wn === 0) return;
  const k = b.k === undefined ? 2 / 3 : b.k;
  const dw = jn * surf.spinLen / (k * b.r * b.r);
  const aw = Math.abs(wn);
  const nw = aw <= dw ? 0 : (wn > 0 ? aw - dw : -(aw - dw));
  const d = nw - wn;
  b.wx = wx + d * nx; b.wy = wy + d * ny; b.wz = wz + d * nz;
}

// Box2D-style sleep metric [in/s]: |v| + |w|*r.
export function sleepMetric(b) {
  const sp = Math.sqrt(b.vx * b.vx + b.vy * b.vy + b.vz * b.vz);
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;   // rad/s, missing spin fields read as 0
  const sw = Math.sqrt(wx * wx + wy * wy + wz * wz);
  return sp + sw * b.r;
}

// Accumulate slow-contact time; returns true at the moment the ball falls asleep (v and w zeroed).
export function updateSleep(b, h) {
  if (b._c && sleepMetric(b) < BBR.sleepSpeed) {
    b.sleepT = (b.sleepT || 0) + h;
    if (b.sleepT >= BBR.sleepTime) {
      b.sleeping = true;
      b.vx = b.vy = b.vz = 0;
      b.wx = b.wy = b.wz = 0;
      return true;
    }
    return false;
  }
  b.sleepT = 0;
  return false;
}

// Airborne: no contact last substep and centre above floor contact height by airEps [in].
export function isAirborne(b) {
  return !b._cPrev && b.z > b.r + BBR.airEps;
}

// Spin for rolling without slip on a surface with unit normal n: w = (n x v)/r. Rolling +x on floor gives +wy.
export function rollingSpinFor(vx, vy, vz, nx, ny, nz, r, out) {
  out[0] = (ny * vz - nz * vy) / r;
  out[1] = (nz * vx - nx * vz) / r;
  out[2] = (nx * vy - ny * vx) / r;
  return out;
}

// Launch spin: axis = z x v_hat_horizontal, w = -ratio*|v|/r*axis. ratio > 0 is backspin.
export function launchSpin(vx, vy, vz, r, ratio, out) {
  const vh = Math.sqrt(vx * vx + vy * vy);
  if (vh < 1e-9) { out[0] = 0; out[1] = 0; out[2] = 0; return out; }
  const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
  const m = -ratio * sp / r;
  out[0] = (-vy / vh) * m;
  out[1] = (vx / vh) * m;
  out[2] = 0;
  return out;
}

// Limit |w| to maxSpin [rad/s].
export function clampSpin(b, maxSpin) {
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;   // rad/s, missing spin fields read as 0
  const s2 = wx * wx + wy * wy + wz * wz;
  if (s2 > maxSpin * maxSpin) {
    const f = maxSpin / Math.sqrt(s2);
    b.wx = wx * f; b.wy = wy * f; b.wz = wz * f;
  }
}
