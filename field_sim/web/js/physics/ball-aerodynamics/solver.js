// solver.js - drag/spin-aware aim solver (ESM, pure, deterministic). Inches, radians, in/s.
// Bisection on the signed range residual, then secant refinement.
import { AERO } from './params.js';
import { crossHeight } from './trajectory.js';

export function launchVector(speed, azRad, elevRad) {
  const c = Math.cos(elevRad);
  return [speed * c * Math.cos(azRad), speed * c * Math.sin(azRad), speed * Math.sin(elevRad)];
}

// Closed-form drag-free parabola speed (in/s), or null if the target is unreachable at this elevation.
export function dragFreeSpeed(p0, target, elevRad, g = AERO.g) {
  const d = Math.hypot(target[0] - p0[0], target[1] - p0[1]);
  const dz = target[2] - p0[2];
  const c = Math.cos(elevRad);
  const den = 2 * c * c * (d * Math.tan(elevRad) - dz);
  if (!(den > 0)) return null;
  return Math.sqrt(g * d * d / den);
}

function setup(req) {
  const { p0, target, model } = req;
  const dx = target[0] - p0[0], dy = target[1] - p0[1];
  return {
    p0, target, model,
    az: req.azRad ?? Math.atan2(dy, dx),
    dist: Math.hypot(dx, dy),
    descending: req.descending ?? true,
    tol: req.tol ?? AERO.solver.tol,
    maxIter: req.maxIter ?? AERO.solver.maxIter,
    opts: { h: req.h ?? AERO.solver.h, tMax: AERO.solver.tMax, slip: req.slip ?? AERO.lift.spinSlip },
  };
}

// Residual R = horizontal range at crossing minus horizontal range to target (in).
function residual(c, speed, elev) {
  const init = { pos: c.p0, vel: launchVector(speed, c.az, elev) };
  const r = crossHeight(init, c.model, c.target[2], { ...c.opts, descending: c.descending });
  if (!r) {
    // No crossing: never reached the height (short) or still airborne at tMax (long). Apex tells which.
    let apex = c.p0[2];
    const vz = init.vel[2];
    if (vz > 0) apex += vz * vz / (2 * c.model.g);
    return { R: apex < c.target[2] ? -c.dist - 1 : c.dist + 1, t: NaN, apexZ: apex, none: true };
  }
  const R = Math.hypot(r.pos[0] - c.p0[0], r.pos[1] - c.p0[1]) - c.dist;
  return { R, t: r.t, apexZ: r.apexZ, none: false };
}

// Root-find on [lo,hi] with a sign change: bisection first, then secant kept inside the bracket.
function findRoot(f, lo, hi, flo, fhi, tol, maxIter) {
  const fine = tol * 0.1;
  let iters = 0;
  for (; iters < maxIter && iters < 12; iters++) {
    const mid = 0.5 * (lo + hi), fm = f(mid);
    if (Math.abs(fm.R) < fine) return { x: mid, fx: fm, iters: iters + 1 };
    if ((fm.R < 0) === (flo.R < 0)) { lo = mid; flo = fm; } else { hi = mid; fhi = fm; }
  }
  let x0 = lo, f0 = flo, x1 = hi, f1 = fhi;
  let best = Math.abs(f0.R) < Math.abs(f1.R) ? { x: x0, fx: f0 } : { x: x1, fx: f1 };
  for (; iters < maxIter; iters++) {
    if (Math.abs(best.fx.R) < fine) break;
    const den = f1.R - f0.R;
    let x = Math.abs(den) > 1e-14 ? x1 - f1.R * (x1 - x0) / den : 0.5 * (x0 + x1);
    if (!(x > Math.min(x0, x1) && x < Math.max(x0, x1))) x = 0.5 * (x0 + x1);
    const fx = f(x);
    if (Math.abs(fx.R) < Math.abs(best.fx.R)) best = { x, fx };
    if ((fx.R < 0) === (f0.R < 0)) { x0 = x; f0 = fx; } else { x1 = x; f1 = fx; }
  }
  return { x: best.x, fx: best.fx, iters };
}

export function solveSpeed(req) {
  const c = setup(req);
  const vMin = req.vMin ?? AERO.solver.vMin, vMax = req.vMax ?? AERO.solver.vMax;
  const f = v => residual(c, v, req.elevRad);
  const flo = f(vMin), fhi = f(vMax);
  if ((flo.R < 0) === (fhi.R < 0)) {
    return { ok: false, speed: NaN, t: NaN, apexZ: NaN, missIn: Math.min(Math.abs(flo.R), Math.abs(fhi.R)), iters: 2, reason: 'unreachable' };
  }
  const r = findRoot(f, vMin, vMax, flo, fhi, c.tol, c.maxIter);
  const ok = !r.fx.none && Math.abs(r.fx.R) < c.tol;
  const out = { ok, speed: r.x, t: r.fx.t, apexZ: r.fx.apexZ, missIn: r.fx.R, iters: r.iters + 2 };
  if (!ok) out.reason = 'no_convergence';
  return out;
}

export function solveElevation(req) {
  const c = setup(req);
  const speed = req.speed;
  const deg = Math.PI / 180;
  const e0 = AERO.solver.elevMinDeg, e1 = AERO.solver.elevMaxDeg, es = AERO.solver.elevStepDeg;
  const f = e => residual(c, speed, e);
  const roots = [];
  let prevE = e0 * deg, prev = f(prevE);
  for (let d = e0 + es; d <= e1 + 1e-9; d += es) {
    const e = d * deg, cur = f(e);
    if ((prev.R < 0) !== (cur.R < 0) && !(prev.none && cur.none)) {
      const r = findRoot(f, prevE, e, prev, cur, c.tol, c.maxIter);
      if (!r.fx.none && Math.abs(r.fx.R) < c.tol) roots.push({ e: r.x, fx: r.fx });
    }
    prevE = e; prev = cur;
  }
  if (roots.length === 0) return { ok: false, low: null, high: null, reason: 'unreachable' };
  const mk = r => ({ elevRad: r.e, t: r.fx.t, missIn: r.fx.R });
  return { ok: true, low: mk(roots[0]), high: roots.length > 1 ? mk(roots[roots.length - 1]) : null };
}

// Flies the drag-free closed-form speed through the model; reports the miss (in) versus the target.
export function dragFreeMiss(req) {
  const c = setup(req);
  const speed = dragFreeSpeed(c.p0, c.target, req.elevRad, c.model.g);
  if (speed === null) return { speed: null, missIn: Infinity, landing: null };
  const r = crossHeight({ pos: c.p0, vel: launchVector(speed, c.az, req.elevRad) }, c.model, c.target[2], { ...c.opts, descending: true });
  if (!r) return { speed, missIn: Infinity, landing: null };
  const landing = r.pos;
  return { speed, missIn: Math.hypot(landing[0] - c.target[0], landing[1] - c.target[1]), landing };
}
