// Fitters that turn real robot measurements into sim constants. Units: inches, seconds, radians, kg.
// All tunables come from params.js (FIT, CMAES). Sources:
//  - feedforward form u = kS*sgn(v) + kV*v + kA*a: standard DC-motor feedforward (PRIOR; WPILib SysId internals not found).
//  - ramp fit and its rate (+0.1/s up to 0.9): Road Runner v1.0 tuning docs (SOURCED rr.brott.dev/docs/v1-0/tuning/).
//  - zero-power decel fit: our own definition (Pedro's formula was not found). ASSUMED.
//  - restitution e = sqrt(hBounce/hDrop) with fall heights (centre height minus radius): energy argument, PRIOR.
import { FIT, M_PER_IN } from './params.js';
import { BallWorld, BALL, BALL_KINDS } from '../../balls.js';
import { solveLeastSquares } from './linalg.js';
import { cmaes } from './cmaes.js';

// The ball-bounce-rolling physics owner moved the effective restitution into SURFACES[*].eRef (balls.js reads SURFACES, and
// BALL.restitution* are only copies taken at load). To keep the fit meaningful we also write the matching SURFACES entry.
// Loaded defensively so this file still works if that module moves.
let SURFACES = null;
try { ({ SURFACES } = await import('../ball-bounce-rolling/params.js')); } catch (_) { SURFACES = null; }
const SURFACE_OF_FIELD = { restitutionFloor: 'floor', restitutionWall: 'wall', restitutionBall: 'ball', restitutionRobot: 'robot', restitutionHive: 'hive' };

const finite = (x) => Number.isFinite(x);

function movingAverage(v, win) {
  const n = v.length, half = Math.floor(win / 2), out = new Array(n);
  for (let i = 0; i < n; i++) {
    const lo = Math.max(0, i - half), hi = Math.min(n - 1, i + half);
    let s = 0;
    for (let j = lo; j <= hi; j++) s += v[j];
    out[i] = s / (hi - lo + 1);
  }
  return out;
}

function checkSeries(s, names) {
  for (const k of names) if (!Array.isArray(s[k]) || s[k].length === 0) throw new TypeError('fit: series ' + k + ' missing or empty');
  const n = s[names[0]].length;
  for (const k of names) if (s[k].length !== n) throw new TypeError('fit: series length mismatch');
}

/**
 * OLS fit of u = kS*sgn(v) + kV*v + kA*a.
 * v is smoothed with a centred moving average (opts.smoothWindow); a (if not supplied) is the central difference of the
 * smoothed v. Samples with |v| < minVel are dropped, as are samples within trimStartS after any |du| > 0.05 jump.
 * @param {{t:number[],u:number[],v:number[],a?:number[]}} s u dimensionless power (-1..1), v in/s, a in/s^2
 * @returns {{kS:number,kV:number,kA:number,r2:number,rmse:number,n:number}} kV in 1/(in/s), kA in 1/(in/s^2)
 */
export function fitFeedforward(s, opts = {}) {
  checkSeries(s, ['t', 'u', 'v']);
  const minVel = opts.minVel ?? FIT.minVelInS;
  const trim = opts.trimStartS ?? FIT.trimStartS;
  const win = opts.smoothWindow ?? FIT.smoothWindow;
  const N = s.t.length;
  const vs = movingAverage(s.v, win);
  const jumpT = [];
  for (let i = 1; i < N; i++) if (Math.abs(s.u[i] - s.u[i - 1]) > 0.05) jumpT.push(s.t[i]);
  const A = [], y = [];
  let jp = 0, lastJump = -Infinity;
  for (let i = 0; i < N; i++) {
    while (jp < jumpT.length && jumpT[jp] <= s.t[i]) { lastJump = jumpT[jp]; jp++; }
    let a;
    if (s.a) a = s.a[i];
    else if (i > 0 && i < N - 1) a = (vs[i + 1] - vs[i - 1]) / (s.t[i + 1] - s.t[i - 1]);
    else continue;
    if (!finite(a) || !finite(vs[i]) || !finite(s.u[i])) continue;
    if (Math.abs(vs[i]) < minVel) continue;
    if (s.t[i] - lastJump < trim) continue;
    A.push([Math.sign(vs[i]), vs[i], a]);
    y.push(s.u[i]);
  }
  if (A.length < 3) throw new Error('fitFeedforward: fewer than 3 usable samples');
  const r = solveLeastSquares(A, y);
  return { kS: r.beta[0], kV: r.beta[1], kA: r.beta[2], r2: r.r2, rmse: r.rmse, n: A.length };
}

/**
 * Road Runner v1.0 style ramp fit: OLS u = kS + kV*v on samples with v >= minVel.
 * @param {{t:number[],u:number[],v:number[]}} s
 * @returns {{kS:number,kV:number,r2:number,rmse:number,n:number}}
 */
export function fitRamp(s, opts = {}) {
  checkSeries(s, ['t', 'u', 'v']);
  const minVel = opts.minVel ?? FIT.minVelInS;
  const A = [], y = [];
  for (let i = 0; i < s.t.length; i++) {
    if (!finite(s.v[i]) || !finite(s.u[i]) || s.v[i] < minVel) continue;
    A.push([1, s.v[i]]); y.push(s.u[i]);
  }
  if (A.length < 2) throw new Error('fitRamp: fewer than 2 usable samples');
  const r = solveLeastSquares(A, y);
  return { kS: r.beta[0], kV: r.beta[1], r2: r.r2, rmse: r.rmse, n: A.length };
}

/**
 * Zero-power (coast) deceleration: OLS v = v0 - d*t over samples with |v| >= minVel (in/s). Time is taken relative to
 * the first sample of s.t. decel d is reported positive (in/s^2). Our own definition (ASSUMED).
 * @param {{t:number[],v:number[]}} s
 * @returns {{decel:number,v0:number,r2:number,n:number}}
 */
export function fitZeroPowerDecel(s, opts = {}) {
  checkSeries(s, ['t', 'v']);
  const minVel = opts.minVel ?? FIT.decelMinVelInS;
  const t0 = s.t[0];
  const A = [], y = [];
  for (let i = 0; i < s.t.length; i++) {
    if (!finite(s.v[i]) || Math.abs(s.v[i]) < minVel) continue;
    A.push([1, s.t[i] - t0]); y.push(s.v[i]);
  }
  if (A.length < 2) throw new Error('fitZeroPowerDecel: fewer than 2 usable samples');
  const r = solveLeastSquares(A, y);
  const sign = r.beta[0] >= 0 ? 1 : -1;      // report a positive decel for either direction of travel
  return { decel: -r.beta[1] * sign, v0: r.beta[0], r2: r.r2, n: A.length };
}

/**
 * Map a feedforward fit to MecanumDrive parameters.
 * vmax = (1-kS)/kV (in/s). tau = kA/kV (s). With match 't90' the real first-order response reaches 90% of vmax at
 * tau*ln(10); the sim's rate limit reaches 0.9*vmax at 0.9*vmax/maxAccel, so maxAccel = 0.9*vmax/(tau*ln 10).
 * With match 'initial' maxAccel = (1-kS)/kA (the initial slope at full power).
 * @returns {{maxSpeed:number,maxAccel:number,tau:number}} in/s, in/s^2, s
 */
export function feedforwardToDrive(ff, opts = {}) {
  const match = opts.match ?? FIT.accelMatch;
  const { kS, kV, kA } = ff;
  if (!(kV > 0) || !(kA > 0)) throw new RangeError('feedforwardToDrive: kV and kA must be positive');
  const maxSpeed = (1 - kS) / kV;
  const tau = kA / kV;
  const maxAccel = match === 'initial' ? (1 - kS) / kA : 0.9 * maxSpeed / (tau * Math.LN10);
  return { maxSpeed, maxAccel, tau };
}

function meanSd(a) {
  const n = a.length;
  const m = a.reduce((x, y) => x + y, 0) / n;
  const sd = n > 1 ? Math.sqrt(a.reduce((x, y) => x + (y - m) * (y - m), 0) / (n - 1)) : 0;
  return { m, sd, se: sd / Math.sqrt(n) };
}

/**
 * Coefficient of restitution from drop/bounce heights (both as FALL heights: centre height minus radius, in inches).
 * e_i = sqrt(hBounce/hDrop); sigma_e/e = 0.5*sqrt((sb/hb)^2 + (sd/hd)^2).
 * @returns {{e:number,sd:number,se:number,n:number,perTrial:{e:number,sigmaE:number}[]}} e is the mean, sd the sample sd, se = sd/sqrt(n)
 */
export function restitutionFromHeights(trials) {
  if (!Array.isArray(trials) || trials.length === 0) throw new TypeError('restitutionFromHeights: no trials');
  const perTrial = trials.map(t => {
    if (!(t.hDrop > 0) || !(t.hBounce >= 0)) throw new RangeError('restitutionFromHeights: need hDrop > 0 and hBounce >= 0');
    const e = Math.sqrt(t.hBounce / t.hDrop);
    const rb = t.hBounce > 0 ? (t.sigmaBounce ?? 0) / t.hBounce : 0;
    const rd = (t.sigmaDrop ?? 0) / t.hDrop;
    return { e, sigmaE: e * 0.5 * Math.sqrt(rb * rb + rd * rd) };
  });
  const st = meanSd(perTrial.map(p => p.e));
  return { e: st.m, sd: st.sd, se: st.se, n: trials.length, perTrial };
}

/** Restitution from speeds: e = |vOut/vIn|. */
export function restitutionFromVelocities(trials) {
  if (!Array.isArray(trials) || trials.length === 0) throw new TypeError('restitutionFromVelocities: no trials');
  const es = trials.map(t => {
    if (!(Math.abs(t.vIn) > 0)) throw new RangeError('restitutionFromVelocities: vIn must be nonzero');
    return Math.abs(t.vOut / t.vIn);
  });
  const st = meanSd(es);
  return { e: st.m, sd: st.sd, se: st.se, n: es.length };
}

// Sim drop used by fitRestitutionBySim: pollen released at rest, centre at hFall + r, 1/60 s steps (as the existing
// drop test in tests_js/sim_physics.test.mjs). Returns the rise of the first rebound (centre peak minus r), in inches.
function simFirstRebound(hFall) {
  const bounds = { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 };
  const w = new BallWorld({ bounds });
  const r = BALL_KINDS.pollen.radiusIn;
  const b = w.addBall('pollen', [70, 70, hFall + r], [0, 0, 0], 'pollen');
  const DT = 1 / 60, MAX_STEPS = 60 * 8;
  let impacted = false, peak = 0;
  for (let i = 0; i < MAX_STEPS; i++) {
    const prevVz = b.vz;
    w.step(DT);
    if (!impacted && prevVz < 0 && b.vz > 0) impacted = true;
    if (impacted) {
      peak = Math.max(peak, b.z);
      if (b.vz < 0) break;       // past the first peak
    }
  }
  return impacted ? peak - b.r : 0;
}

/**
 * Find the BALL[field] value whose simulated first rebound matches a measured e: golden-section search minimising
 * (simRise/hFall - e^2)^2 over [lo,hi]. BALL[field] (and the matching SURFACES[*].eRef the sim actually reads, when present) is restored in a finally block.
 * The sim's restitution may also depend on impact speed; the returned value is the reference value that reproduces eMeasured
 * for a drop from hFallIn, and simE is the realised sqrt(rise/hFall).
 * @param {number} hFallIn fall height in inches (centre height minus radius)
 * @param {number} eMeasured measured restitution
 * @returns {{value:number,simE:number,iterations:number}} simE = sqrt(simRise/hFall) at the returned value
 */
export function fitRestitutionBySim(hFallIn, eMeasured, opts = {}) {
  if (!(hFallIn > 0)) throw new RangeError('fitRestitutionBySim: hFallIn must be positive');
  const field = opts.field ?? 'restitutionFloor';
  const lo0 = opts.lo ?? 0.05, hi0 = opts.hi ?? 0.95, tol = opts.tol ?? 1e-3;
  if (!(field in BALL)) throw new Error('fitRestitutionBySim: unknown BALL field ' + field);
  const saved = BALL[field];
  const surf = SURFACES && SURFACES[SURFACE_OF_FIELD[field]] ? SURFACES[SURFACE_OF_FIELD[field]] : null;
  const savedSurf = surf ? surf.eRef : null;
  const setField = (val) => { BALL[field] = val; if (surf) surf.eRef = val; };
  let iterations = 0;
  try {
    const cost = (val) => {
      setField(val);
      const e2 = simFirstRebound(hFallIn) / hFallIn;
      return (e2 - eMeasured * eMeasured) ** 2;
    };
    const gr = (Math.sqrt(5) - 1) / 2;
    let a = lo0, b = hi0;
    let c = b - gr * (b - a), d = a + gr * (b - a);
    let fc = cost(c), fd = cost(d);
    while (b - a > tol) {
      iterations++;
      if (fc < fd) { b = d; d = c; fd = fc; c = b - gr * (b - a); fc = cost(c); }
      else { a = c; c = d; fc = fd; d = a + gr * (b - a); fd = cost(d); }
    }
    const value = (a + b) / 2;
    setField(value);
    const simE = Math.sqrt(Math.max(simFirstRebound(hFallIn), 0) / hFallIn);
    return { value, simE, iterations };
  } finally {
    BALL[field] = saved;
    if (surf) surf.eRef = savedSurf;
  }
}

/** Cd = 2*m*k_m/(rho*A), k_m = k/0.0254 (1/m), A = pi*(r*0.0254)^2. k in 1/in, m in kg, r in in. */
export function dragKToCd(k, massKg, radiusIn, rho = FIT.rhoAirKgM3) {
  const km = k / M_PER_IN, A = Math.PI * (radiusIn * M_PER_IN) ** 2;
  return 2 * massKg * km / (rho * A);
}

/** Inverse of dragKToCd: k (1/in) from Cd. */
export function cdToDragK(cd, massKg, radiusIn, rho = FIT.rhoAirKgM3) {
  const A = Math.PI * (radiusIn * M_PER_IN) ** 2;
  return cd * rho * A / (2 * massKg) * M_PER_IN;
}

// RK4 of dv/dt = -k|v|v - g z_hat in the (x,z) plane. State [x,z,vx,vz]. g in in/s^2 from BALL.g (SOURCED in balls.js).
function deriv(s, k, g, out) {
  const sp = Math.hypot(s[2], s[3]);
  out[0] = s[2]; out[1] = s[3];
  out[2] = -k * sp * s[2]; out[3] = -k * sp * s[3] - g;
}

function simulateDrag(p, ts, g, dt) {
  const [k, x0, z0, vx0, vz0] = p;
  const t0 = ts[0];
  let s = [x0, z0, vx0, vz0], t = 0;
  const xs = new Array(ts.length), zs = new Array(ts.length);
  xs[0] = x0; zs[0] = z0;
  const k1 = [0, 0, 0, 0], k2 = [0, 0, 0, 0], k3 = [0, 0, 0, 0], k4 = [0, 0, 0, 0], tmp = [0, 0, 0, 0];
  for (let i = 1; i < ts.length; i++) {
    const target = ts[i] - t0;
    while (t < target) {
      const h = Math.min(dt, target - t);   // last partial step lands exactly on the sample time
      deriv(s, k, g, k1);
      for (let j = 0; j < 4; j++) tmp[j] = s[j] + 0.5 * h * k1[j];
      deriv(tmp, k, g, k2);
      for (let j = 0; j < 4; j++) tmp[j] = s[j] + 0.5 * h * k2[j];
      deriv(tmp, k, g, k3);
      for (let j = 0; j < 4; j++) tmp[j] = s[j] + h * k3[j];
      deriv(tmp, k, g, k4);
      for (let j = 0; j < 4; j++) s[j] += h / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
      t += h;
    }
    xs[i] = s[0]; zs[i] = s[1];
  }
  return { xs, zs };
}

// Quadratic least-squares x(t) ~ c0 + c1*dt + c2*dt^2 over the first points; returns value and slope at dt = 0.
function quadStart(t, y, npts) {
  const n = Math.min(npts, t.length);
  if (n < 3) {
    const v = n > 1 ? (y[1] - y[0]) / (t[1] - t[0]) : 0;
    return { p0: y[0], v0: v };
  }
  const A = [], b = [];
  for (let i = 0; i < n; i++) { const d = t[i] - t[0]; A.push([1, d, d * d]); b.push(y[i]); }
  const r = solveLeastSquares(A, b);
  return { p0: r.beta[0], v0: r.beta[1] };
}

/**
 * Fit a drag coefficient (and the launch state) to a ball flight in the (x,z) plane.
 * Model: dv/dt = -k|v|v - g z_hat, RK4 at FIT.dragDt, parameters [k,x0,z0,vx0,vz0] at the first sample; minimise the sum of
 * squared position residuals with CMA-ES. The search runs in a normalised box [0,1]^5 (k in [0,0.01] 1/in, x0/z0 within +-10 in
 * and vx0/vz0 within +-200 in/s of a quadratic-polyfit guess from the first 5 points) so a single sigma0 suits all axes.
 * @param {{t:number[],x:number[],z:number[]}} track seconds and inches
 * @returns {{k:number,x0:number,z0:number,vx0:number,vz0:number,rmse:number,evals:number}} k in 1/in, rmse in inches
 */
export function fitDrag(track, opts = {}) {
  const N = track.t.length;
  if (N < 5 || track.x.length !== N || track.z.length !== N) throw new TypeError('fitDrag: need >= 5 matching samples');
  const kInit = opts.kInit ?? 0.0004;
  const gx = quadStart(track.t, track.x, 5), gz = quadStart(track.t, track.z, 5);
  const guess = [kInit, gx.p0, gz.p0, gx.v0, gz.v0];
  const lower = [0, guess[1] - 10, guess[2] - 10, guess[3] - 200, guess[4] - 200];
  const upper = [0.01, guess[1] + 10, guess[2] + 10, guess[3] + 200, guess[4] + 200];
  const toP = (u) => u.map((v, i) => lower[i] + v * (upper[i] - lower[i]));
  const u0 = guess.map((v, i) => (v - lower[i]) / (upper[i] - lower[i]));
  const g = BALL.g;
  const cost = (u) => {
    const sim = simulateDrag(toP(u), track.t, g, FIT.dragDt);
    let ss = 0;
    for (let i = 0; i < N; i++) {
      const dx = sim.xs[i] - track.x[i], dz = sim.zs[i] - track.z[i];
      ss += dx * dx + dz * dz;
    }
    return finite(ss) ? ss : 1e30;
  };
  const res = cmaes(cost, u0, {
    lower: [0, 0, 0, 0, 0], upper: [1, 1, 1, 1, 1], seed: opts.seed ?? 1, maxEvals: opts.maxEvals,
  });
  const p = toP(res.x);
  return { k: p[0], x0: p[1], z0: p[2], vx0: p[3], vz0: p[4], rmse: Math.sqrt(res.f / (2 * N)), evals: res.evals };
}
