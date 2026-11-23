// drive.js - traction-limited velocity motor (spec section 6).
// The target is a body-frame velocity (+x forward, +y left, rad/s). The accumulated impulse is capped by a shared
// friction ellipse over (forward, strafe, turn), so collisions can load the wheels until they slip. SI units.
import { PARAMS } from './params.js';

export function createDrive(opts = {}, params = PARAMS) {
  const d = params.drive;
  return {
    enabled: opts.enabled !== undefined ? opts.enabled : true,
    target: { vx: 0, vy: 0, om: 0 },
    muWheel: opts.muWheel !== undefined ? opts.muWheel : d.muWheel,
    strafeEfficiency: opts.strafeEfficiency !== undefined ? opts.strafeEfficiency : d.strafeEfficiency,
    turnLeverArm: opts.turnLeverArm !== undefined ? opts.turnLeverArm : d.turnLeverArm,
    motorForceMax: opts.motorForceMax !== undefined ? opts.motorForceMax : d.motorForceMax,
    impulse: { f: 0, s: 0, w: 0 }, // N*s, N*s, N*m*s
    h: 0,
    limits: { F: 0, S: 0, T: 0 }, // N, N, N*m
    usage: 0,
    slipping: false,
    force: { f: 0, s: 0, w: 0 }, // N, N, N*m
  };
}

export function attachDrive(body, drive) {
  body.drive = drive;
  return drive;
}

// Body frame: vx forward m/s, vy left m/s, om rad/s.
export function setDriveTarget(drive, vx, vy, om) {
  drive.target.vx = vx;
  drive.target.vy = vy;
  drive.target.om = om;
}

// Active only for dynamic bodies with an enabled drive (kinematic velocities are set by the world).
function active(body) {
  return body.drive !== null && body.drive.enabled && body.type === 'dynamic';
}

function termSq(x, cap) {
  if (cap > 0) { const r = x / cap; return r * r; }
  return x !== 0 ? Infinity : 0;
}

// Friction-ellipse usage L: 1 means the budget is exactly used.
function ellipseUsage(f, s, w, lim, h) {
  return Math.sqrt(termSq(f, lim.F * h) + termSq(s, lim.S * h) + termSq(w, lim.T * h));
}

export function prepareDrive(body, h, params) {
  if (!active(body)) return;
  const dr = body.drive;
  const N = body.mass * params.motion.gravity;
  const F = Math.min(dr.muWheel * N, dr.motorForceMax);
  dr.limits.F = F;
  dr.limits.S = F * dr.strafeEfficiency;
  dr.limits.T = F * dr.turnLeverArm;
  if (dr.h > 0 && h !== dr.h) {
    const k = h / dr.h; // impulse = force*h, so rescale to keep the same force
    dr.impulse.f *= k;
    dr.impulse.s *= k;
    dr.impulse.w *= k;
  }
  dr.h = h;
}

function applyDelta(body, Pf, Ps, Pw) {
  const c = body.q.c, s = body.q.s; // ex = (c,s), ey = (-s,c)
  const v = body.linearVelocity;
  v.x += body.invMass * (Pf * c - Ps * s);
  v.y += body.invMass * (Pf * s + Ps * c);
  body.angularVelocity += body.invInertia * Pw;
}

export function warmStartDrive(body, params) {
  if (!active(body)) return;
  const dr = body.drive;
  if (!params.solver.warmStarting) {
    dr.impulse.f = 0; dr.impulse.s = 0; dr.impulse.w = 0;
    return;
  }
  const k = params.solver.warmStartFactor;
  dr.impulse.f *= k; dr.impulse.s *= k; dr.impulse.w *= k;
  applyDelta(body, dr.impulse.f, dr.impulse.s, dr.impulse.w);
}

export function solveDrive(body, h) {
  if (!active(body)) return;
  const dr = body.drive;
  const c = body.q.c, s = body.q.s;
  const v = body.linearVelocity;
  const uf = v.x * c + v.y * s;
  const us = -v.x * s + v.y * c;
  const old = dr.impulse;
  let f = old.f - body.mass * (uf - dr.target.vx);
  let sd = old.s - body.mass * (us - dr.target.vy);
  let w = old.w - body.inertia * (body.angularVelocity - dr.target.om);
  const L = ellipseUsage(f, sd, w, dr.limits, h);
  if (L > 1) { f /= L; sd /= L; w /= L; }
  applyDelta(body, f - old.f, sd - old.s, w - old.w);
  old.f = f; old.s = sd; old.w = w;
}

export function finishDrive(body, h) {
  if (body.drive === null) return;
  const dr = body.drive;
  if (!active(body)) {
    dr.usage = 0; dr.slipping = false;
    dr.force.f = 0; dr.force.s = 0; dr.force.w = 0;
    return;
  }
  dr.force.f = dr.impulse.f / h;
  dr.force.s = dr.impulse.s / h;
  dr.force.w = dr.impulse.w / h;
  dr.usage = ellipseUsage(dr.impulse.f, dr.impulse.s, dr.impulse.w, dr.limits, h);
  dr.slipping = dr.usage >= 0.999;
}
