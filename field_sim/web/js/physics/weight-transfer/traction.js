// Weight transfer: mecanum traction limit using per-wheel normal loads.
// Units: lbm, in, s. Forces lbm*in/s^2; torque lbm*in^2/s^2. Wheel order FL, FR, BL, BR.
// Provenance:
//  - X-pattern mecanum kinematics FL = vf - vl - k*om etc., k = a+b: from robot geometry (derived here).
//  - Equal-force-share pseudo-inverse split: derived here (assumed model).
//  - Contact force = forceFactor*|f_i|, forceFactor = 1/cos(rollerAngle) = sqrt(2) at 45 deg (derived here).
//  - Friction limit |F| <= mu*Fz (Coulomb); mu is a placeholder in params.js (order of magnitude
//    from https://rsms.me/etc/car-physics/), measure on tiles.
//  - Bisection tolerances come from params.limits.bisectIters / searchMaxInS2.
import { G, wheelGeometry, forceFactor } from './params.js';
import { wheelLoads, tipAccels } from './loads.js';

const S = [-1, 1, 1, -1];
const R = [-1, 1, -1, 1];

const geomCache = new WeakMap();
function geom(params) {
  let g = geomCache.get(params);
  if (!g) { g = wheelGeometry(params); geomCache.set(params, g); }
  return g;
}

/** Drive-direction force per wheel (lbm*in/s^2) for body demand Fx, Fy (lbm*in/s^2), Mz (lbm*in^2/s^2). */
export function mecanumWheelForces(params, Fx, Fy, Mz) {
  const k = geom(params).k;
  const out = [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) out[i] = Fx / 4 + (S[i] * Fy) / 4 + (R[i] * Mz) / (4 * k);
  return out;
}

/** Per-wheel utilisation |F_i| / (mu*Fz_i) at body accel (ax, ay, alpha). opts {lifted, az} */
export function wheelUtil(params, cg, ax, ay, alpha, opts = {}) {
  const loads = wheelLoads(params, cg, ax, ay, opts);
  const M = cg.massLb;
  const f = mecanumWheelForces(params, M * ax, M * ay, cg.Izz * alpha);
  const ff = forceFactor(params);
  const mu = params.traction.mu * (params.traction.muScale ?? 1);
  const lifted = opts.lifted;
  const util = [0, 0, 0, 0];
  let maxUtil = 0;
  for (let i = 0; i < 4; i++) {
    const mag = Math.abs(f[i]);
    const fz = loads.n[i] * M * loads.gEff;
    let u;
    if ((lifted && lifted[i]) || !(loads.n[i] > 0) || !(fz > 0)) u = mag > 1e-9 ? Infinity : 0;
    else u = (ff * mag) / (mu * fz);
    util[i] = u;
    if (u > maxUtil) maxUtil = u;
  }
  return { util, loads, maxUtil };
}

function evalAt(params, cg, s, cmd, fix, opts) {
  const w = wheelUtil(params, cg, fix.ax + s * cmd.ax, fix.ay + s * cmd.ay, fix.alpha + s * cmd.alpha, opts);
  let ok = w.maxUtil <= 1 && w.loads.mode !== 'tipping';
  if (ok && opts.antiTip) {
    const m = params.tip && params.tip.antiTipMarginIn != null ? params.tip.antiTipMarginIn : 0;
    if (w.loads.margin < m) ok = false;
  }
  w.ok = ok;
  return w;
}

/** Largest scale in [0,1] with applied = fixed + s*cmd feasible. */
export function tractionScale(params, cg, accelCmd, accelFixed, opts = {}) {
  const o = opts || {};
  const e1 = evalAt(params, cg, 1, accelCmd, accelFixed, o);
  const slip = e1.util.map((u) => u > 1);
  if (e1.ok) return { scale: 1, sliding: false, slip, util: e1.util, loads: e1.loads };
  const e0 = evalAt(params, cg, 0, accelCmd, accelFixed, o);
  if (!e0.ok) return { scale: 0, sliding: true, slip, util: e0.util, loads: e0.loads };
  let lo = 0, hi = 1, best = e0;
  const iters = params.limits?.bisectIters ?? 24;
  for (let i = 0; i < iters; i++) {
    const mid = 0.5 * (lo + hi);
    const e = evalAt(params, cg, mid, accelCmd, accelFixed, o);
    if (e.ok) { lo = mid; best = e; } else hi = mid;
  }
  return { scale: lo, sliding: false, slip, util: best.util, loads: best.loads };
}

/** Limit a body-velocity step [vf,vl,om]. Centripetal part fixed, commanded part scaled. */
export function limitBodyVelocity(params, cg, prevBodyV, wantBodyV, dt, opts = {}) {
  if (!(dt > 0)) {
    const w = wheelLoads(params, cg, 0, 0);
    return {
      bodyV: [prevBodyV[0], prevBodyV[1], prevBodyV[2]],
      accel: { ax: 0, ay: 0, alpha: 0 },
      scale: 1, sliding: false, slip: [false, false, false, false],
      util: [0, 0, 0, 0], loads: w,
    };
  }
  const vfm = 0.5 * (prevBodyV[0] + wantBodyV[0]);
  const vlm = 0.5 * (prevBodyV[1] + wantBodyV[1]);
  const omm = 0.5 * (prevBodyV[2] + wantBodyV[2]);
  const cmd = {
    ax: (wantBodyV[0] - prevBodyV[0]) / dt,
    ay: (wantBodyV[1] - prevBodyV[1]) / dt,
    alpha: (wantBodyV[2] - prevBodyV[2]) / dt,
  };
  const fix = { ax: -vlm * omm, ay: vfm * omm, alpha: 0 };
  const r = tractionScale(params, cg, cmd, fix, opts);
  const s = r.scale;
  return {
    bodyV: [
      prevBodyV[0] + s * (wantBodyV[0] - prevBodyV[0]),
      prevBodyV[1] + s * (wantBodyV[1] - prevBodyV[1]),
      prevBodyV[2] + s * (wantBodyV[2] - prevBodyV[2]),
    ],
    accel: { ax: fix.ax + s * cmd.ax, ay: fix.ay + s * cmd.ay, alpha: s * cmd.alpha },
    scale: s, sliding: r.sliding, slip: r.slip, util: r.util, loads: r.loads,
  };
}

/** Max accel (in/s^2) along a direction at zero velocity. dir {ax,ay} unit. */
function maxAlong(params, cg, dx, dy) {
  const hiMax = params.limits?.searchMaxInS2 ?? 2 * G;
  const iters = params.limits?.bisectIters ?? 24;
  const ok = (m) => {
    const w = wheelUtil(params, cg, dx * m, dy * m, 0);
    return w.maxUtil <= 1 && w.loads.mode !== 'tipping';
  };
  if (ok(hiMax)) return hiMax;
  let lo = 0, hi = hiMax;
  for (let i = 0; i < iters; i++) {
    const mid = 0.5 * (lo + hi);
    if (ok(mid)) lo = mid; else hi = mid;
  }
  return lo;
}

export function loadAwareLimits(params, cg) {
  const maxAccelFwd = maxAlong(params, cg, 1, 0);
  const maxBrake = maxAlong(params, cg, -1, 0);
  const maxAccelLat = maxAlong(params, cg, 0, 1);
  return { maxAccelFwd, maxBrake, maxAccelLat, maxCentripetal: maxAccelLat, ...tipAccels(params, cg) };
}
