// kinematics.js - pure mecanum kinematics (no DOM, no three.js, no imports).
// Units: SI (metres, seconds, rad). Body frame: x = robot forward, y = robot LEFT, omega = counter-clockwise (+).
// Twist = {vx, vy, omega}. Wheels = [{x, y, sigma, k}] in spec order FL, FR, BL, BR (metres).
//   sigma_i = roller sign (FL=-1, FR=+1, BL=+1, BR=-1 for the standard X layout), k_i = sigma_i*x_i - y_i (yaw lever, m).
// Rim speed s_i = [1, sigma_i, k_i] . [vx, vy, omega]   (m/s, = wheel radius * wheel angular speed).
// Sources: Gfrerrer 2008 / Diegel 2002 (mecanum kinematics); WPILib MecanumDriveKinematics (pseudo-inverse FK,
// desaturate); gm0 mecanum drive page; arXiv 2510.10273 Eq.1 (speed polytope). No tunable constants live here.

const kOf = (w) => (w.k !== undefined && w.k !== null ? w.k : w.sigma * w.x - w.y);

export function wheelRow(w) {
  return [1, w.sigma, kOf(w)];
}

export function inverseKinematics(wheels, twist, out = new Array(4)) {
  const { vx, vy, omega } = twist;
  for (let i = 0; i < wheels.length; i++) {
    const w = wheels[i];
    out[i] = vx + w.sigma * vy + kOf(w) * omega;
  }
  return out;
}

// Solve 3x3 A q = b with Gaussian elimination, partial pivoting. A is row-major length 9 (copied).
function solve3x3(A, b) {
  const m = [A[0], A[1], A[2], b[0], A[3], A[4], A[5], b[1], A[6], A[7], A[8], b[2]];
  let scale = 0;
  for (let i = 0; i < 9; i++) scale = Math.max(scale, Math.abs(A[i]));
  const tol = 1e-12 * (scale || 1);
  for (let c = 0; c < 3; c++) {
    let p = c;
    for (let r = c + 1; r < 3; r++) if (Math.abs(m[r * 4 + c]) > Math.abs(m[p * 4 + c])) p = r;
    if (Math.abs(m[p * 4 + c]) <= tol) throw new Error('forwardKinematics: singular wheel matrix');
    if (p !== c) for (let j = 0; j < 4; j++) { const t = m[c * 4 + j]; m[c * 4 + j] = m[p * 4 + j]; m[p * 4 + j] = t; }
    for (let r = c + 1; r < 3; r++) {
      const f = m[r * 4 + c] / m[c * 4 + c];
      for (let j = c; j < 4; j++) m[r * 4 + j] -= f * m[c * 4 + j];
    }
  }
  const q = [0, 0, 0];
  for (let r = 2; r >= 0; r--) {
    let s = m[r * 4 + 3];
    for (let j = r + 1; j < 3; j++) s -= m[r * 4 + j] * q[j];
    q[r] = s / m[r * 4 + r];
  }
  return q;
}

export function forwardKinematics(wheels, rimSpeeds) {
  const N = new Array(9).fill(0);
  const rhs = [0, 0, 0];
  for (let i = 0; i < wheels.length; i++) {
    const row = wheelRow(wheels[i]);
    for (let a = 0; a < 3; a++) {
      rhs[a] += row[a] * rimSpeeds[i];
      for (let c = 0; c < 3; c++) N[a * 3 + c] += row[a] * row[c];
    }
  }
  const q = solve3x3(N, rhs);
  return { vx: q[0], vy: q[1], omega: q[2] };
}

// Velocity of the wheel contact point in the body frame: c = (vx - omega*y, vy + omega*x).
export function contactVelocity(w, twist) {
  return { cx: twist.vx - twist.omega * w.y, cy: twist.vy + twist.omega * w.x };
}

export function rollerAxleDir(w) {
  const n = Math.sqrt(1 + w.sigma * w.sigma);
  return [1 / n, w.sigma / n];
}

export function rollerFreeDir(w) {
  const n = Math.sqrt(1 + w.sigma * w.sigma);
  return [-w.sigma / n, 1 / n];
}

// Roller surface speed relative to the hub (m/s): (c - s*xhat) . d with rim speed s = cx + sigma*cy,
// which reduces to sqrt(1+sigma^2) * cy. Zero in pure forward, sqrt2*vy in pure strafe (45 deg rollers).
export function rollerSpeed(w, twist) {
  const { cx, cy } = contactVelocity(w, twist);
  const s = cx + w.sigma * cy;
  const d = rollerFreeDir(w);
  return (cx - s) * d[0] + cy * d[1];
}

// WPILib-style desaturate ('max') or gm0 conservative ('gm0', needs terms [[a,b,c],...]).
export function desaturate(values, limit = 1, mode = 'max', terms = null) {
  if (mode === 'gm0') {
    if (!Array.isArray(terms) || terms.length !== values.length) {
      throw new Error("desaturate: mode 'gm0' requires a terms array with one [a,b,c] per value");
    }
    let denom = limit;
    for (const t of terms) denom = Math.max(denom, Math.abs(t[0]) + Math.abs(t[1]) + Math.abs(t[2]));
    const scale = limit / denom;
    return { values: values.map((v) => v * scale), scale };
  }
  if (mode !== 'max') throw new Error(`desaturate: unknown mode '${mode}'`);
  let realMax = 0;
  for (const v of values) realMax = Math.max(realMax, Math.abs(v));
  const scale = realMax > limit ? limit / realMax : 1;
  return { values: values.map((v) => v * scale), scale };
}

export function commandToWheelPowers(wheels, cmd, opts = {}) {
  const lm = opts.lateralMultiplier ?? 1;
  const ts = opts.turnScale ?? 1;
  const mode = opts.mode ?? 'max';
  let kMax = opts.kMax;
  if (kMax === undefined || kMax === null) {
    kMax = 0;
    for (const w of wheels) kMax = Math.max(kMax, Math.abs(kOf(w)));
  }
  const fwd = cmd.fwd || 0, str = cmd.strafe || 0, trn = cmd.turn || 0;
  const raw = [];
  const terms = [];
  for (const w of wheels) {
    const a = fwd;
    const b = str * lm * w.sigma;
    const c = kMax > 0 ? (trn * ts * kOf(w)) / kMax : 0;
    raw.push(a + b + c);
    terms.push([a, b, c]);
  }
  const r = desaturate(raw, 1, mode, mode === 'gm0' ? terms : null);
  return { powers: r.values, scale: r.scale };
}

export function maxSpeedAtHeading(wheels, thetaRad, omega = 0, rimMax) {
  const c = Math.cos(thetaRad), s = Math.sin(thetaRad);
  let best = Infinity;
  for (const w of wheels) {
    const den = Math.abs(c + w.sigma * s);
    if (den < 1e-12) continue;
    best = Math.min(best, (rimMax - Math.abs(kOf(w) * omega)) / den);
  }
  if (!Number.isFinite(best)) return 0;
  return Math.max(0, best);
}

export function twistIsFeasible(wheels, twist, rimMax) {
  const lim = rimMax * (1 + 1e-9);
  const s = inverseKinematics(wheels, twist);
  for (let i = 0; i < wheels.length; i++) if (Math.abs(s[i]) > lim) return false;
  return true;
}
