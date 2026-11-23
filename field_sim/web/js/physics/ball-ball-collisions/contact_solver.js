// contact_solver.js - ball-ball narrowphase, contact cache and sequential-impulse solver (spec section 4).
// Units: length in, time s, mass kg, velocity in/s, angular velocity rad/s,
// linear impulse kg*in/s, angular impulse kg*in^2/s, inertia kg*in^2.
// Imports only ./params.js and ./bodies.js (must not import balls.js).
//
// Conventions: n points from b toward a. rA = -ra*n, rB = +rb*n (contact point relative to centres).
// A linear impulse P is applied +P to a and -P to b. Static bodies (sleeping or pinned) have
// inverse mass/inertia 0, read as zero velocity, and are never written to.

import { BB_PARAMS } from './params.js';
import { restitutionFor, isStatic } from './bodies.js';

// key = min(uid)*2^21 + max(uid): exact as a JS number for uids < 2^21 (~2M balls).
export function pairKey(a, b) {
  const ua = a._bbUid, ub = b._bbUid;
  return ua < ub ? ua * 2097152 + ub : ub * 2097152 + ua;
}

// Warm-start cache: key -> {ln,ptx,pty,ptz,mrx,mry,mrz,ms,stamp}. Records are reused when overwritten.
export class ContactCache {
  constructor() { this._m = new Map(); }
  get(key) { return this._m.get(key); }
  set(key, c, stamp) {
    let r = this._m.get(key);
    if (r === undefined) { r = { ln: 0, ptx: 0, pty: 0, ptz: 0, mrx: 0, mry: 0, mrz: 0, ms: 0, stamp: 0 }; this._m.set(key, r); }
    r.ln = c.ln; r.ptx = c.ptx; r.pty = c.pty; r.ptz = c.ptz;
    r.mrx = c.mrx; r.mry = c.mry; r.mrz = c.mrz; r.ms = c.ms; r.stamp = stamp;
  }
  sweep(minStamp) {
    for (const [k, r] of this._m) if (r.stamp < minStamp) this._m.delete(k);
  }
  clear() { this._m.clear(); }
  get size() { return this._m.size; }
}

function newContact() {
  return {
    a: null, b: null, key: 0,
    nx: 0, ny: 0, nz: 1, s: 0, rr: 0, ra: 0, rb: 0,
    ima: 0, imb: 0, iia: 0, iib: 0,       // effective inverse mass / inertia (0 for static)
    kn: 0, kt: 0, kr: 0, rEff: 0, e: 0, vn0: 0,
    ln: 0, ptx: 0, pty: 0, ptz: 0, mrx: 0, mry: 0, mrz: 0, ms: 0,
    mu: 0,        // dimensionless Coulomb friction for this contact (params.mu for ball-ball, plane.mu for planes)
    plane: null,  // null for ball-ball; the plane object for a ball-plane contact
    pd: 0,        // in, plane offset d (plane is n.p = d, ball side n.p > d)
  };
}

// Immovable stand-in for the static side of a ball-plane contact. Never written (imb = 0).
// z = -Infinity so plane contacts sort first in the bottom-up order.
export const STATIC_BODY = Object.freeze({
  x: 0, y: 0, z: -Infinity, vx: 0, vy: 0, vz: 0, wx: 0, wy: 0, wz: 0, r: 0,
  state: 'static', sleeping: false, pinned: true, _bbUid: -1,
});

// Static half-space key: negative so it never collides with pairKey (>= 0). Up to 64 planes.
export function planeKey(a, planeIndex) { return -(a._bbUid * 64 + planeIndex + 1); }

// Scratch for relative velocity (module-level, no allocation).
let rvx = 0, rvy = 0, rvz = 0;

// vrel = (va + wa x rA) - (vb + wb x rB), rA = -ra*n, rB = +rb*n. Static bodies read as zero.
function relVel(c) {
  const nx = c.nx, ny = c.ny, nz = c.nz;
  let x = 0, y = 0, z = 0;
  if (c.ima > 0) {
    const a = c.a, k = -c.ra;
    const wx = a.wx, wy = a.wy, wz = a.wz;
    // wa x rA = -ra * (wa x n)
    x += a.vx + k * (wy * nz - wz * ny);
    y += a.vy + k * (wz * nx - wx * nz);
    z += a.vz + k * (wx * ny - wy * nx);
  }
  if (c.imb > 0) {
    const b = c.b, k = c.rb;
    const wx = b.wx, wy = b.wy, wz = b.wz;
    x -= b.vx + k * (wy * nz - wz * ny);
    y -= b.vy + k * (wz * nx - wx * nz);
    z -= b.vz + k * (wx * ny - wy * nx);
  }
  rvx = x; rvy = y; rvz = z;
}

// Apply linear impulse P at the contact (+P to a, -P to b).
function applyP(c, px, py, pz) {
  const nx = c.nx, ny = c.ny, nz = c.nz;
  // n x P
  const cx = ny * pz - nz * py, cy = nz * px - nx * pz, cz = nx * py - ny * px;
  if (c.ima > 0) {
    const a = c.a, m = c.ima, k = -c.ra * c.iia; // rA x P = -ra*(n x P)
    a.vx += m * px; a.vy += m * py; a.vz += m * pz;
    a.wx += k * cx; a.wy += k * cy; a.wz += k * cz;
  }
  if (c.imb > 0) {
    const b = c.b, m = c.imb, k = c.rb * c.iib;   // rB x P = +rb*(n x P)
    b.vx -= m * px; b.vy -= m * py; b.vz -= m * pz;
    b.wx -= k * cx; b.wy -= k * cy; b.wz -= k * cz;
  }
}

// Apply angular-only impulse M (+M to a, -M to b).
function applyM(c, mx, my, mz) {
  if (c.ima > 0) { const a = c.a, k = c.iia; a.wx += k * mx; a.wy += k * my; a.wz += k * mz; }
  if (c.imb > 0) { const b = c.b, k = c.iib; b.wx -= k * mx; b.wy -= k * my; b.wz -= k * mz; }
}

// Narrowphase. Writes contacts into the pool `out`; returns count. Accumulated impulses are zeroed.
export function buildContacts(pairs, pairCount, h, params, out) {
  const spec = params.speculativeDistance;
  let n = 0;
  for (let i = 0; i < pairCount; i++) {
    const a = pairs[2 * i], b = pairs[2 * i + 1];
    if (a.state !== 'free' || b.state !== 'free') continue;
    const ima = isStatic(a) ? 0 : a.invM, imb = isStatic(b) ? 0 : b.invM;
    if (ima + imb === 0) continue;
    const iia = ima > 0 ? a.invI : 0, iib = imb > 0 ? b.invI : 0;
    const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    const rr = a.r + b.r;
    const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
    let nx, ny, nz;
    if (d < 1e-9) { nx = 0; ny = 0; nz = 1; } else { const id = 1 / d; nx = dx * id; ny = dy * id; nz = dz * id; }
    const s = d - rr;

    let c = out[n];
    if (c === undefined) { c = newContact(); out[n] = c; }
    c.a = a; c.b = b;
    c.nx = nx; c.ny = ny; c.nz = nz; c.s = s; c.rr = rr; c.ra = a.r; c.rb = b.r;
    c.ima = ima; c.imb = imb; c.iia = iia; c.iib = iib;
    relVel(c);
    const vn0 = rvx * nx + rvy * ny + rvz * nz;
    if (s >= spec + (vn0 < 0 ? -vn0 : 0) * h) continue; // not kept (c is simply overwritten next time)
    c.vn0 = vn0;
    c.key = pairKey(a, b);
    c.kn = 1 / (ima + imb);
    c.kt = 1 / (ima + imb + a.r * a.r * iia + b.r * b.r * iib);
    const iab = iia + iib;
    c.kr = iab > 0 ? 1 / iab : 0;
    c.rEff = a.r * b.r / (a.r + b.r);
    c.e = vn0 < 0 ? restitutionFor(-vn0, params) : 0;
    c.mu = params.mu; c.plane = null; c.pd = 0;
    c.ln = 0; c.ptx = 0; c.pty = 0; c.ptz = 0; c.mrx = 0; c.mry = 0; c.mrz = 0; c.ms = 0;
    n++;
  }
  return n;
}

// Ball vs static half-space contacts (floor, field walls), appended to `out` from index `start`.
// A plane is {nx, ny, nz, d, mu?, e?}: unit normal n pointing into the free side, offset d (in),
// so a ball is clear when n.p - d >= r. mu (dimensionless, default 0) and e (restitution, default 0)
// belong to the caller's surface model. Rolling/spin resistance is NOT applied here (kr = 0): the
// environment code (balls.js floor model) already does rolling resistance and roll sync.
// Solving planes inside the same sequential-impulse loop lets a pile's weight reach the floor within
// one solve. Clamping the floor/walls after the solve (the old approach) re-injects ball-ball overlap
// every substep (measured on the 150-ball pile: 0.199 in overlap before a solve vs 0.157 in after it).
export function buildPlaneContacts(list, planes, h, params, out, start) {
  let n = start;
  if (!planes || planes.length === 0) return n;
  const spec = params.speculativeDistance;
  for (let i = 0; i < list.length; i++) {
    const a = list[i];
    if (a.state !== 'free' || isStatic(a)) continue;
    const ima = a.invM;
    if (!(ima > 0)) continue;
    for (let pi = 0; pi < planes.length; pi++) {
      const pl = planes[pi];
      const nx = pl.nx, ny = pl.ny, nz = pl.nz;
      const s = a.x * nx + a.y * ny + a.z * nz - pl.d - a.r;
      if (s >= spec + a.r) continue; // cheap reject: 1 radius of travel per substep is far above maxStepTravel
      let c = out[n];
      if (c === undefined) { c = newContact(); out[n] = c; }
      c.a = a; c.b = STATIC_BODY;
      c.nx = nx; c.ny = ny; c.nz = nz; c.s = s; c.rr = a.r; c.ra = a.r; c.rb = 0;
      c.ima = ima; c.imb = 0; c.iia = a.invI; c.iib = 0;
      relVel(c);
      const vn0 = rvx * nx + rvy * ny + rvz * nz;
      if (s >= spec + (vn0 < 0 ? -vn0 : 0) * h) continue;
      c.vn0 = vn0;
      c.key = planeKey(a, pi);
      c.kn = 1 / ima;
      c.kt = 1 / (ima + a.r * a.r * a.invI);
      c.kr = 0;
      c.rEff = a.r;
      const e = pl.e === undefined ? 0 : pl.e;
      c.e = (vn0 < -params.restitutionThreshold) ? e : 0;
      c.mu = pl.mu === undefined ? 0 : pl.mu;
      c.plane = pl; c.pd = pl.d;
      c.ln = 0; c.ptx = 0; c.pty = 0; c.ptz = 0; c.mrx = 0; c.mry = 0; c.mrz = 0; c.ms = 0;
      n++;
    }
  }
  return n;
}

// Scale vector (x,y,z) to length <= max; result in module scratch.
let cvx = 0, cvy = 0, cvz = 0;
function clampVec(x, y, z, max) {
  const l2 = x * x + y * y + z * z;
  if (l2 > max * max) {
    const k = max > 0 ? max / Math.sqrt(l2) : 0;
    x *= k; y *= k; z *= k;
  }
  cvx = x; cvy = y; cvz = z;
}

export class ContactSolver {
  constructor(params = BB_PARAMS) {
    this.params = params;
    this.contacts = null;
    this.n = 0;
    this.h = 0;
    this.order = [];
  }

  prepare(contacts, n, h, cache, stamp) {
    const P = this.params;
    this.contacts = contacts; this.n = n; this.h = h;
    // ordering (insertion sort on reused index array, nearly sorted frame to frame)
    const order = this.order;
    order.length = n;
    for (let i = 0; i < n; i++) order[i] = i;
    if (P.sortByHeight) {
      for (let i = 1; i < n; i++) {
        const ci = contacts[order[i]];
        const zi = ci.a.z < ci.b.z ? ci.a.z : ci.b.z;
        let j = i - 1;
        while (j >= 0) {
          const cj = contacts[order[j]];
          const zj = cj.a.z < cj.b.z ? cj.a.z : cj.b.z;
          if (zj > zi || (zj === zi && cj.key > ci.key)) { order[j + 1] = order[j]; j--; } else break;
        }
        order[j + 1] = i;
      }
    }
    // warm-start impulses from cache
    for (let i = 0; i < n; i++) {
      const c = contacts[i];
      c.ln = 0; c.ptx = 0; c.pty = 0; c.ptz = 0; c.mrx = 0; c.mry = 0; c.mrz = 0; c.ms = 0;
      if (!cache || !P.warmStart) continue;
      const r = cache.get(c.key);
      if (r === undefined) continue;
      const nx = c.nx, ny = c.ny, nz = c.nz;
      const ln = r.ln * P.warmStartFactor;
      if (!(ln > 0)) continue;
      c.ln = ln;
      let dot = r.ptx * nx + r.pty * ny + r.ptz * nz;
      clampVec(r.ptx - dot * nx, r.pty - dot * ny, r.ptz - dot * nz, c.mu * ln);
      c.ptx = cvx; c.pty = cvy; c.ptz = cvz;
      const mrMax = P.muRoll * c.rEff * ln;
      dot = r.mrx * nx + r.mry * ny + r.mrz * nz;
      clampVec(r.mrx - dot * nx, r.mry - dot * ny, r.mrz - dot * nz, mrMax);
      c.mrx = cvx; c.mry = cvy; c.mrz = cvz;
      const msMax = P.muSpin * c.rEff * ln;
      c.ms = r.ms > msMax ? msMax : (r.ms < -msMax ? -msMax : r.ms);
    }
  }

  warmStart() {
    const cs = this.contacts, n = this.n;
    for (let i = 0; i < n; i++) {
      const c = cs[i];
      if (c.ln === 0) continue;
      applyP(c, c.ln * c.nx + c.ptx, c.ln * c.ny + c.pty, c.ln * c.nz + c.ptz);
      applyM(c, c.mrx + c.ms * c.nx, c.mry + c.ms * c.ny, c.mrz + c.ms * c.nz);
    }
  }

  solveVelocity(iterations) {
    const P = this.params, cs = this.contacts, n = this.n, order = this.order, h = this.h;
    const muRoll = P.muRoll, muSpin = P.muSpin;
    for (let it = 0; it < iterations; it++) {
      for (let oi = 0; oi < n; oi++) {
        const c = cs[order[oi]];
        const nx = c.nx, ny = c.ny, nz = c.nz;
        const ln = c.ln;
        const a = c.a, b = c.b;
        // --- rolling resistance
        if (c.kr > 0) {
          let wx = 0, wy = 0, wz = 0;
          if (c.ima > 0) { wx += a.wx; wy += a.wy; wz += a.wz; }
          if (c.imb > 0) { wx -= b.wx; wy -= b.wy; wz -= b.wz; }
          const wn = wx * nx + wy * ny + wz * nz;
          wx -= wn * nx; wy -= wn * ny; wz -= wn * nz;
          clampVec(c.mrx - c.kr * wx, c.mry - c.kr * wy, c.mrz - c.kr * wz, muRoll * c.rEff * ln);
          const dx = cvx - c.mrx, dy = cvy - c.mry, dz = cvz - c.mrz;
          c.mrx = cvx; c.mry = cvy; c.mrz = cvz;
          applyM(c, dx, dy, dz);
          // --- spin about n
          let ws = 0;
          if (c.ima > 0) ws += a.wx * nx + a.wy * ny + a.wz * nz;
          if (c.imb > 0) ws -= b.wx * nx + b.wy * ny + b.wz * nz;
          const msMax = muSpin * c.rEff * ln;
          let nm = c.ms - c.kr * ws;
          nm = nm > msMax ? msMax : (nm < -msMax ? -msMax : nm);
          const dms = nm - c.ms;
          c.ms = nm;
          applyM(c, dms * nx, dms * ny, dms * nz);
        }
        // --- tangent friction
        relVel(c);
        const vn = rvx * nx + rvy * ny + rvz * nz;
        const tx = rvx - vn * nx, ty = rvy - vn * ny, tz = rvz - vn * nz;
        clampVec(c.ptx - c.kt * tx, c.pty - c.kt * ty, c.ptz - c.kt * tz, c.mu * ln);
        {
          const dx = cvx - c.ptx, dy = cvy - c.pty, dz = cvz - c.ptz;
          c.ptx = cvx; c.pty = cvy; c.ptz = cvz;
          applyP(c, dx, dy, dz);
        }
        // --- normal (speculative bias; no positional bias)
        relVel(c);
        const vn2 = rvx * nx + rvy * ny + rvz * nz;
        const bias = c.s > 0 ? c.s / h : 0;
        let nl = c.ln - c.kn * (vn2 + bias);
        if (nl < 0) nl = 0;
        const dl = nl - c.ln;
        c.ln = nl;
        if (dl !== 0) applyP(c, dl * nx, dl * ny, dl * nz);
      }
    }
  }

  applyRestitution() {
    const P = this.params, cs = this.contacts, n = this.n, order = this.order;
    const thr = P.restitutionThreshold, slop = P.linearSlop;
    for (let oi = 0; oi < n; oi++) {
      const c = cs[order[oi]];
      if (c.e <= 0 || c.vn0 >= -thr || c.ln <= 0 || c.s > slop) continue;
      relVel(c);
      const vn = rvx * c.nx + rvy * c.ny + rvz * c.nz;
      let nl = c.ln - c.kn * (vn + c.e * c.vn0);
      if (nl < 0) nl = 0;
      const dl = nl - c.ln;
      c.ln = nl;
      if (dl !== 0) applyP(c, dl * c.nx, dl * c.ny, dl * c.nz);
    }
  }

  // NGS position correction: positions only, velocities untouched.
  solvePositions(iterations) {
    const P = this.params, cs = this.contacts, n = this.n, order = this.order;
    const bg = P.baumgarte, slop = P.linearSlop, maxC = P.maxLinearCorrection;
    for (let it = 0; it < iterations; it++) {
      for (let oi = 0; oi < n; oi++) {
        const c = cs[order[oi]];
        const a = c.a, b = c.b;
        let nx, ny, nz, s;
        if (c.plane !== null) {
          nx = c.nx; ny = c.ny; nz = c.nz;
          s = a.x * nx + a.y * ny + a.z * nz - c.pd - c.rr;
        } else {
          const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
          const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
          if (d < 1e-9) { nx = 0; ny = 0; nz = 1; } else { const id = 1 / d; nx = dx * id; ny = dy * id; nz = dz * id; }
          s = d - c.rr;
        }
        let C = bg * (s + slop);
        if (C < -maxC) C = -maxC;
        if (C >= 0) continue;
        const imp = -C / (c.ima + c.imb);
        if (c.ima > 0) { const k = c.ima * imp; a.x += k * nx; a.y += k * ny; a.z += k * nz; }
        if (c.imb > 0) { const k = c.imb * imp; b.x -= k * nx; b.y -= k * ny; b.z -= k * nz; }
      }
    }
  }

  storeImpulses(cache, stamp) {
    const cs = this.contacts, n = this.n, maxW = this.params.maxAngularSpeed;
    const maxW2 = maxW * maxW;
    for (let i = 0; i < n; i++) {
      const c = cs[i];
      if (cache) cache.set(c.key, c, stamp);
      // clamp |w| (dynamic bodies only)
      if (c.ima > 0) clampW(c.a, maxW2, maxW);
      if (c.imb > 0) clampW(c.b, maxW2, maxW);
    }
  }

  solve(contacts, n, h, cache, stamp) {
    const P = this.params;
    this.prepare(contacts, n, h, cache, stamp);
    if (P.warmStart) this.warmStart();
    this.solveVelocity(P.velocityIterations);
    this.applyRestitution();
    this.solvePositions(P.positionIterations);
    this.storeImpulses(cache, stamp);
  }
}

function clampW(b, maxW2, maxW) {
  const l2 = b.wx * b.wx + b.wy * b.wy + b.wz * b.wz;
  if (l2 > maxW2) { const k = maxW / Math.sqrt(l2); b.wx *= k; b.wy *= k; b.wz *= k; }
}
