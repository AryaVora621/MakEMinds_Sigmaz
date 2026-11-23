// contact.js - spin-aware impulse contact solver (ball-bounce-rolling spec section 3).
// Units: inches, seconds, radians, in/s, rad/s. Pedro frame. Mass is unitless 1.0, so impulses are in in/s.
// Normal n points toward the ball centre; contact point relative to the centre is p = -r*n.
// Tunables live in ./params.js (SOURCED / ASSUMED tags are there). Nothing is allocated per call.
import { restitutionAt, bounceArmed } from './params.js';
import { applyRollingResistance, applyTorsionalFriction } from './rolling.js';

// Shared result object, reused across calls (callers must read it before the next resolveContact call).
const RES = { impact: false, jn: 0, jt: 0 };

// out = (v - vs) - r*(w x n): velocity of the ball surface point at the contact, relative to the surface.
export function contactPointVelocity(b, nx, ny, nz, vsx, vsy, vsz, out) {
  const r = b.r;
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;
  out[0] = (b.vx - vsx) - r * (wy * nz - wz * ny);
  out[1] = (b.vy - vsy) - r * (wz * nx - wx * nz);
  out[2] = (b.vz - vsz) - r * (wx * ny - wy * nx);
  return out;
}

export function resolveContact(b, nx, ny, nz, pen, surf, vsx, vsy, vsz, h) {
  // 1. push out
  b.x += nx * pen; b.y += ny * pen; b.z += nz * pen;
  b._c = true;
  RES.impact = false; RES.jn = 0; RES.jt = 0;

  // 2. normal velocity (no friction without compression)
  const vn = (b.vx - vsx) * nx + (b.vy - vsy) * ny + (b.vz - vsz) * nz;
  if (vn >= 0) return RES;

  // 3. restitution
  const vin = -vn;
  const e = restitutionAt(surf, vin);
  const armed = bounceArmed(surf, vin, e, nz);

  // 4. normal impulse
  const jn = (1 + (armed ? e : 0)) * vin;
  b.vx += jn * nx; b.vy += jn * ny; b.vz += jn * nz;
  b._jnSum = (b._jnSum || 0) + jn;
  RES.impact = armed; RES.jn = jn;

  // 5. contact-point slip after the normal impulse
  const r = b.r;
  const k = b.k === undefined ? 2 / 3 : b.k;
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;
  const ux = (b.vx - vsx) - r * (wy * nz - wz * ny);
  const uy = (b.vy - vsy) - r * (wz * nx - wx * nz);
  const uz = (b.vz - vsz) - r * (wx * ny - wy * nx);
  const un = ux * nx + uy * ny + uz * nz;
  const tx = ux - un * nx, ty = uy - un * ny, tz = uz - un * nz;
  const s = Math.sqrt(tx * tx + ty * ty + tz * tz);

  let sliding = false;
  if (s >= 1e-9) {
    // 6. tangential impulse (Coulomb-capped)
    const jtStick = (1 + (armed ? surf.et : 0)) * s / (1 + 1 / k);
    const cap = surf.muSlide * jn;
    sliding = jtStick > cap;
    const jt = sliding ? cap : jtStick;
    const f = -jt / s;
    const Jx = tx * f, Jy = ty * f, Jz = tz * f;
    // 7. apply: v += Jt; w += -(n x Jt)/(k r)
    b.vx += Jx; b.vy += Jy; b.vz += Jz;
    const c = -1 / (k * r);
    b.wx = wx + c * (ny * Jz - nz * Jy);
    b.wy = wy + c * (nz * Jx - nx * Jz);
    b.wz = wz + c * (nx * Jy - ny * Jx);
    RES.jt = jt;
  }

  // 8. resting contact: rolling resistance and torsional spin friction.
  // Rolling resistance acts only in the ROLLING state (contact point stuck after step 6). While the contact still
  // slides, Coulomb friction is the only tangential force (pooltool sliding vs rolling states,
  // https://raw.githubusercontent.com/ekiefl/pooltool/main/pooltool/physics/evolve/__init__.py). applyRollingResistance
  // scales v_t and w_perp by one factor, which is only valid with no slip: applied while sliding (small v, large w, e.g.
  // a rolling ball just stopped by a head-on hit) it zeroed the spin in one substep (round-2 fix).
  if (!armed) {
    if (!sliding) applyRollingResistance(b, nx, ny, nz, jn, vsx, vsy, vsz, surf);
    applyTorsionalFriction(b, nx, ny, nz, jn, surf);
  }
  return RES;
}

// Friction between two balls. n points from b to a; jn is the per-ball normal impulse already applied (may be 0).
// `impact` is accepted for API symmetry and unused (no et term between balls).
export function resolvePairFriction(a, b, nx, ny, nz, jn, impact, surf) {
  const ra = a.r, rb = b.r;
  const ka = a.k === undefined ? 2 / 3 : a.k;
  const kb = b.k === undefined ? 2 / 3 : b.k;
  const awx = a.wx || 0, awy = a.wy || 0, awz = a.wz || 0;
  const bwx = b.wx || 0, bwy = b.wy || 0, bwz = b.wz || 0;
  const ux = (a.vx - b.vx) - ra * (awy * nz - awz * ny) - rb * (bwy * nz - bwz * ny);
  const uy = (a.vy - b.vy) - ra * (awz * nx - awx * nz) - rb * (bwz * nx - bwx * nz);
  const uz = (a.vz - b.vz) - ra * (awx * ny - awy * nx) - rb * (bwx * ny - bwy * nx);
  const un = ux * nx + uy * ny + uz * nz;
  const tx = ux - un * nx, ty = uy - un * ny, tz = uz - un * nz;
  const s = Math.sqrt(tx * tx + ty * ty + tz * tz);
  if (s < 1e-9) return;
  const invM = 2 + 1 / ka + 1 / kb;
  const cap = surf.muSlide * (jn > 0 ? jn : 0);
  const stick = s / invM;
  const jt = stick < cap ? stick : cap;
  if (jt <= 0) return;
  const f = -jt / s;
  const Jx = tx * f, Jy = ty * f, Jz = tz * f;
  const cx = ny * Jz - nz * Jy, cy = nz * Jx - nx * Jz, cz = nx * Jy - ny * Jx; // n x Jt
  a.vx += Jx; a.vy += Jy; a.vz += Jz;
  b.vx -= Jx; b.vy -= Jy; b.vz -= Jz;
  const ca = -1 / (ka * ra), cb = -1 / (kb * rb);
  a.wx = awx + ca * cx; a.wy = awy + ca * cy; a.wz = awz + ca * cz;
  b.wx = bwx + cb * cx; b.wy = bwy + cb * cy; b.wz = bwz + cb * cz;
}
