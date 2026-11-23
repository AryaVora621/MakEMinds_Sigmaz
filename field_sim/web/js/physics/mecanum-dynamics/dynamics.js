// dynamics.js - rigid-body integrator for the mecanum chassis (SI units: m, kg, s, N, N*m, rad).
// Frame: body x = forward, y = LEFT, omega = counter-clockwise (+). Twist = {vx, vy, omega}.
// EOM: M * [dvx, dvy, domega] = [fx + m*omega*vy, fy - m*omega*vx, mz]  (Coriolis uses chassis mass m only).
// M = diag(m, m, Izz) + sum_i (Jw/r^2) row_i^T row_i  (reflected wheel inertia).
import { inverseKinematics, wheelRow } from './kinematics.js';
import { wheelTorque } from './motor.js';
import { wheelForces } from './forces.js';

const TWO_PI = 2 * Math.PI;

/** 3x3 row-major mass matrix (kg, kg, kg*m^2 on the diagonal). */
export function buildMassMatrix(p) {
  const M = new Float64Array(9);
  M[0] = p.massKg;
  M[4] = p.massKg;
  M[8] = p.yawInertiaKgM2;
  const jr = p.wheelSpinInertiaKgM2 / (p.wheelRadiusM * p.wheelRadiusM);
  for (const w of p.wheels) {
    const row = wheelRow(w);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) M[3 * i + j] += jr * row[i] * row[j];
  }
  return M;
}

/** Solve M x = b (3x3) with partial-pivot Gaussian elimination. Throws if singular. */
export function solve3(M, b) {
  const a = [
    [M[0], M[1], M[2], b[0]],
    [M[3], M[4], M[5], b[1]],
    [M[6], M[7], M[8], b[2]],
  ];
  let scale = 0;
  for (let i = 0; i < 9; i++) scale = Math.max(scale, Math.abs(M[i]));
  const tol = 1e-12 * (scale || 1);
  for (let c = 0; c < 3; c++) {
    let piv = c;
    for (let r = c + 1; r < 3; r++) if (Math.abs(a[r][c]) > Math.abs(a[piv][c])) piv = r;
    if (Math.abs(a[piv][c]) < tol) throw new Error('solve3: singular matrix');
    if (piv !== c) { const t = a[c]; a[c] = a[piv]; a[piv] = t; }
    for (let r = c + 1; r < 3; r++) {
      const f = a[r][c] / a[c][c];
      for (let k = c; k < 4; k++) a[r][k] -= f * a[c][k];
    }
  }
  const x = [0, 0, 0];
  for (let r = 2; r >= 0; r--) {
    let s = a[r][3];
    for (let k = r + 1; k < 3; k++) s -= a[r][k] * x[k];
    x[r] = s / a[r][r];
  }
  return x;
}

/** Acceleration of the twist for given wheel powers, plus per-wheel telemetry. */
export function derivative(p, M, twist, powers) {
  const rim = inverseKinematics(p.wheels, twist);
  const torques = new Array(4);
  const cur = new Array(4);
  for (let i = 0; i < 4; i++) {
    const t = wheelTorque(p, powers[i], rim[i]);
    torques[i] = t.torqueNm;
    cur[i] = t.currentA;
  }
  const f = wheelForces(p, twist, torques);
  const m = p.massKg;
  const q = [f.fx + m * twist.omega * twist.vy, f.fy - m * twist.omega * twist.vx, f.mz];
  const acc = solve3(M, q);
  const telemetry = new Array(4);
  for (let i = 0; i < 4; i++) {
    const w = f.perWheel[i];
    telemetry[i] = {
      power: Math.max(-1, Math.min(1, powers[i])),
      rimSpeedMps: rim[i],
      torqueNm: torques[i],
      currentA: cur[i],
      driveN: w.driveN,
      rollerN: w.rollerN,
      rollingN: w.rollingN,
      rollerSpeedMps: w.rollerSpeedMps,
      tractionLimited: w.tractionLimited,
    };
  }
  return { ax: acc[0], ay: acc[1], alpha: acc[2], telemetry };
}

/** Advance the twist by dt with explicit-Euler substeps (<= maxSubstepS). Inputs are not mutated. */
export function stepTwist(p, M, twist, powers, dt) {
  let tw = { vx: twist.vx, vy: twist.vy, omega: twist.omega };
  if (!(dt > 0) || !Number.isFinite(dt)) {
    return { twist: tw, telemetry: derivative(p, M, tw, powers).telemetry };
  }
  const n = Math.max(1, Math.ceil(dt / p.maxSubstepS - 1e-9));
  const h = dt / n;
  let telemetry = null;
  for (let s = 0; s < n; s++) {
    const d = derivative(p, M, tw, powers);
    if (s === 0) telemetry = d.telemetry;
    tw = { vx: tw.vx + h * d.ax, vy: tw.vy + h * d.ay, omega: tw.omega + h * d.alpha };
  }
  return { twist: tw, telemetry };
}

/** Body-frame impulse {jx, jy (N*s), jz (N*m*s)} -> new twist = twist + M^-1 J. */
export function applyImpulse(p, M, twist, impulse) {
  const dv = solve3(M, [impulse.jx, impulse.jy, impulse.jz]);
  return { vx: twist.vx + dv[0], vy: twist.vy + dv[1], omega: twist.omega + dv[2] };
}

/** Impulse (jx, jy) applied at body point (px, py): jz = px*jy - py*jx. */
export function impulseAtPoint(px, py, jx, jy) {
  return { jx, jy, jz: px * jy - py * jx };
}

function wrapAngle(a) {
  let r = a % TWO_PI;
  if (r > Math.PI) r -= TWO_PI;
  else if (r <= -Math.PI) r += TWO_PI;
  return r;
}

/** Exact SE(2) exponential for a constant body twist over dt. Units-agnostic. Heading wrapped to (-pi, pi]. */
export function integratePose(pose, twist, dt) {
  const { x, y, h } = pose;
  const th = twist.omega * dt;
  let dx, dy;
  if (Math.abs(th) < 1e-9) {
    dx = twist.vx * dt;
    dy = twist.vy * dt;
  } else {
    const sn = Math.sin(th) / twist.omega;
    const cs = (1 - Math.cos(th)) / twist.omega;
    dx = twist.vx * sn - twist.vy * cs;
    dy = twist.vx * cs + twist.vy * sn;
  }
  const c = Math.cos(h), s = Math.sin(h);
  return { x: x + c * dx - s * dy, y: y + s * dx + c * dy, h: wrapAngle(h + th) };
}
