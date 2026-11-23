// bodies.js - per-ball rigid-body data for ball-ball collisions v2.
// Units: length in, time s, mass kg, velocity in/s, angular velocity rad/s, inertia kg*in^2.
// Imports only ./params.js. Hot functions do not allocate.

import { BB_PARAMS } from './params.js';

let uidCounter = 0;

export function resetUidCounter() { uidCounter = 0; }

// kg
export function massOf(kind, params = BB_PARAMS) {
  const m = params.massKg[kind];
  return m === undefined ? params.defaultMassKg : m;
}

// kg*in^2 = k*m*r^2 (solid sphere k = 0.4 by default)
export function inertiaOf(kind, r, params = BB_PARAMS) {
  const k = params.inertiaK[kind];
  return (k === undefined ? 0.4 : k) * massOf(kind, params) * r * r;
}

export function isStatic(b) { return b.pinned === true || b.sleeping === true; }

// Idempotent. Adds wx,wy,wz, invM, invI, _bbUid, _bbKey, _bbT, _bbIsl.
export function ensureBody(b, params = BB_PARAMS) {
  if (b._bbUid === undefined) b._bbUid = uidCounter++;
  if (b.wx === undefined) b.wx = 0;
  if (b.wy === undefined) b.wy = 0;
  if (b.wz === undefined) b.wz = 0;
  if (b._bbT === undefined) b._bbT = 0;
  if (b._bbIsl === undefined) b._bbIsl = -1;
  const pinned = b.pinned === true;
  if (b._bbKind !== b.kind || b._bbR !== b.r || b._bbPin !== pinned || b._bbParams !== params) {
    b._bbKind = b.kind; b._bbR = b.r; b._bbPin = pinned; b._bbParams = params;
    b._bbKey = b.kind + '|' + b.r;
    if (pinned) {
      b.invM = 0; b.invI = 0;
      b.vx = 0; b.vy = 0; b.vz = 0; b.wx = 0; b.wy = 0; b.wz = 0;
    } else {
      b.invM = 1 / massOf(b.kind, params);
      b.invI = 1 / inertiaOf(b.kind, b.r, params);
    }
  }
  return b;
}

// |v| + r*|w| in/s
export function pointSpeed(b) {
  const vx = b.vx, vy = b.vy, vz = b.vz;
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;
  return Math.sqrt(vx * vx + vy * vy + vz * vz) + b.r * Math.sqrt(wx * wx + wy * wy + wz * wz);
}

// Speed-dependent restitution; 0 below restitutionThreshold.
export function restitutionFor(approachSpeed, params = BB_PARAMS) {
  const v = approachSpeed;
  if (!(v >= params.restitutionThreshold)) return 0;
  let e;
  if (params.eSpeedExp === 0) e = params.e0;
  else e = 1 - (1 - params.e0) * Math.pow(v / params.eRefSpeed, params.eSpeedExp);
  return e < params.eMin ? params.eMin : (e > params.eMax ? params.eMax : e);
}

// Relax spin toward rolling without slip on a surface with unit normal n: w_roll = (n x v)/r.
export function syncRollingOnSurface(b, nx, ny, nz, h, params = BB_PARAMS) {
  const inv = 1 / b.r;
  const rx = (ny * b.vz - nz * b.vy) * inv;
  const ry = (nz * b.vx - nx * b.vz) * inv;
  const rz = (nx * b.vy - ny * b.vx) * inv;
  let k = params.rollSyncRate * h;
  if (k > 1) k = 1;
  const wx = b.wx || 0, wy = b.wy || 0, wz = b.wz || 0;
  b.wx = wx + (rx - wx) * k;
  b.wy = wy + (ry - wy) * k;
  b.wz = wz + (rz - wz) * k;
}

// w *= 1/(1 + angularDamping*h)
export function dampSpin(b, h, params = BB_PARAMS) {
  const f = 1 / (1 + params.angularDamping * h);
  b.wx = (b.wx || 0) * f;
  b.wy = (b.wy || 0) * f;
  b.wz = (b.wz || 0) * f;
}
