// Mecanum/omni/traction kinematics (SI: m, m/s, rad/s). Body frame: +x fwd, +y left, om>0 CCW.
// Wheel order FL, FR, BL, BR. cfg is the DrivetrainConfig from geometry.js. No imports.

/** Velocity of body point `pos` [m] given body twist {vx,vy,om}: [vx - om*y, vy + om*x] m/s. */
export function contactVelocity(body, pos) {
  return [body.vx - body.om * pos[1], body.vy + body.om * pos[0]];
}

// First axis with nonzero coupling; returns null if none.
function driveAxis(w) {
  for (const a of w.axes) if (Math.abs(a.coupling) > 1e-12) return a;
  return null;
}

/** Ideal wheel surface speeds u_i = (v_c . d_i)/c_i [m/s] for body twist. */
export function idealWheelSpeeds(body, cfg, out = null) {
  const u = out || new Float64Array(4);
  for (let i = 0; i < 4; i++) {
    const w = cfg.wheels[i];
    const a = driveAxis(w);
    if (!a) { u[i] = 0; continue; }
    const vcx = body.vx - body.om * w.pos[1];
    const vcy = body.vy + body.om * w.pos[0];
    u[i] = (vcx * a.dir[0] + vcy * a.dir[1]) / a.coupling;
  }
  return u;
}

/**
 * Stick -> wheel powers in [-1,1]. Twist = (fwd, left, turn*turnInputScale/rhoM);
 * speeds normalised by max(1, max|u|). Reproduces FTC FL=f-l-w, FR=f+l+w, BL=f+l-w, BR=f-l+w.
 */
export function mixWheelPowers(fwd, left, turn, cfg, out = null) {
  const p = out || new Float64Array(4);
  const om = turn * cfg.driver.turnInputScale / cfg.driver.rhoM;
  idealWheelSpeeds({ vx: fwd, vy: left, om }, cfg, p);
  let m = 1;
  for (let i = 0; i < 4; i++) m = Math.max(m, Math.abs(p[i]));
  for (let i = 0; i < 4; i++) p[i] /= m;
  return p;
}

/** Least-squares body twist {vx,vy,om} from 4 surface speeds [m/s] (3x3 normal equations). */
export function twistFromWheelSpeeds(u, cfg) {
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  const b = [0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const w = cfg.wheels[i];
    const a = driveAxis(w);
    if (!a) continue;
    const dx = a.dir[0] / a.coupling, dy = a.dir[1] / a.coupling;
    const row = [dx, dy, -w.pos[1] * dx + w.pos[0] * dy];
    for (let r = 0; r < 3; r++) {
      b[r] += row[r] * u[i];
      for (let c = 0; c < 3; c++) A[r][c] += row[r] * row[c];
    }
  }
  // Tiny Tikhonov term keeps unobservable twists (e.g. strafe on traction wheels) at 0.
  for (let r = 0; r < 3; r++) A[r][r] += 1e-9;
  // Gaussian elimination with partial pivoting.
  for (let c = 0; c < 3; c++) {
    let piv = c;
    for (let r = c + 1; r < 3; r++) if (Math.abs(A[r][c]) > Math.abs(A[piv][c])) piv = r;
    if (piv !== c) { [A[c], A[piv]] = [A[piv], A[c]]; [b[c], b[piv]] = [b[piv], b[c]]; }
    const d = A[c][c];
    for (let r = c + 1; r < 3; r++) {
      const f = A[r][c] / d;
      for (let k = c; k < 3; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  const x = [0, 0, 0];
  for (let r = 2; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < 3; k++) s -= A[r][k] * x[k];
    x[r] = s / A[r][r];
  }
  return { vx: x[0], vy: x[1], om: x[2] };
}
