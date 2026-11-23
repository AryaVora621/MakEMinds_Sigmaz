// calibrate.js - fit the plant constants from logged robot traces, and compare sim vs real traces.
// Model per axis (robot frame): a = alpha*e - beta*v - F*sign(v), with e = kV*c (effort),
//   tau = 1/beta, vFree = alpha/beta, Vss = vFree - F*tau (steady speed at e = 1).
// FLOAT samples (all wheel powers 0, e == 0 and the plant coasts without the -v/tau term) fit F only.
// Units: in, in/s, rad, rad/s, s. Pure functions, no DOM.

const TAU = 2 * Math.PI;
const sgn = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);
const wrapPi = (a) => { let r = (a + Math.PI) % TAU; if (r < 0) r += TAU; return r - Math.PI; };
const NAN_FIT = { tau: NaN, vFree: NaN, vss: NaN, coulomb: NaN, rms: NaN, n: 0 };

/** Solves the n x n system A x = b by Gauss elimination with partial pivoting; null if singular. */
function solve(A, b) {
  const n = b.length;
  const M = A.map((r, i) => [...r, b[i]]);
  let scale = 0;
  for (let i = 0; i < n; i++) scale = Math.max(scale, Math.abs(M[i][i]));
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) <= 1e-9 * scale || scale === 0) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = M[i][n];
    for (let k = i + 1; k < n; k++) s -= M[i][k] * x[k];
    x[i] = s / M[i][i];
  }
  return x;
}

/** Least squares min |y - X theta|^2 via normal equations; theta or null. */
function lsq(X, y) {
  const m = X[0].length;
  const A = Array.from({ length: m }, () => new Array(m).fill(0)), b = new Array(m).fill(0);
  for (let i = 0; i < X.length; i++) {
    for (let r = 0; r < m; r++) { b[r] += X[i][r] * y[i]; for (let c = 0; c < m; c++) A[r][c] += X[i][r] * X[i][c]; }
  }
  return solve(A, b);
}

/**
 * Fits one axis from samples [{t, e, v}] (time s, effort e = kV*c, robot-frame velocity v). Accelerations come from central
 * differences of v. Samples with |v| < 1e-3 (stiction) are skipped. If every e is 0 only F is fitted (FLOAT).
 * opts.coulomb: fix F and fit (alpha, beta) only. If the 3-parameter fit is singular (e.g. constant effort), F falls back to
 * opts.coulomb ?? 0.
 */
export function fitAxis(samples, opts = {}) {
  const rows = [];
  for (let k = 1; k < samples.length - 1; k++) {
    const a = samples[k - 1], b = samples[k], c = samples[k + 1];
    const dt = c.t - a.t;
    if (!(dt > 0) || Math.abs(b.v) < 1e-3) continue;
    rows.push({ e: b.e, v: b.v, s: sgn(b.v), acc: (c.v - a.v) / dt });
  }
  if (rows.length < 3) return { ...NAN_FIT, n: rows.length };
  const resid = (alpha, beta, F) => {
    let ss = 0;
    for (const r of rows) { const d = r.acc - (alpha * r.e - beta * r.v - F * r.s); ss += d * d; }
    return Math.sqrt(ss / rows.length);
  };
  const allFloat = rows.every((r) => r.e === 0);
  if (allFloat) {
    let sum = 0;
    for (const r of rows) sum += -r.acc * r.s;
    const F = sum / rows.length;
    let ss = 0;
    for (const r of rows) { const d = r.acc + F * r.s; ss += d * d; }
    return { tau: NaN, vFree: NaN, vss: NaN, coulomb: F, rms: Math.sqrt(ss / rows.length), n: rows.length };
  }
  let th = null, F;
  if (opts.coulomb === undefined) {
    th = lsq(rows.map((r) => [r.e, -r.v, -r.s]), rows.map((r) => r.acc));
    if (th) F = th[2];
  }
  if (!th) {
    F = opts.coulomb ?? 0;
    const t2 = lsq(rows.map((r) => [r.e, -r.v]), rows.map((r) => r.acc + F * r.s));
    if (!t2) return { ...NAN_FIT, n: rows.length };
    th = [t2[0], t2[1]];
  }
  const [alpha, beta] = th;
  const tau = 1 / beta, vFree = alpha / beta;
  return { tau, vFree, vss: vFree - F * tau, coulomb: F, rms: resid(alpha, beta, F), n: rows.length };
}

function wheelVecs(xVel, yVel) {
  const thF = ((Math.atan2(-yVel, xVel) % TAU) + TAU) % TAU;
  const a = [Math.cos(thF), Math.sin(thF)], b = [Math.cos(TAU - thF), Math.sin(TAU - thF)];
  return { v: [a, b, b, a], cxD: 4 * Math.abs(a[0]), cyD: 4 * Math.abs(a[1]) };
}

function effortsOf(p, W) {
  let fx = 0, fy = 0;
  for (let i = 0; i < 4; i++) { fx += p[i] * W.v[i][0]; fy += p[i] * W.v[i][1]; }
  return [fx / W.cxD, fy / W.cyD, ((p[2] + p[3]) - (p[0] + p[1])) / 4];
}

/**
 * Fits tau, Vss and Coulomb decel per axis from a logged trace [{t, x, y, h, p:[LF,LR,RF,RR], V?}] (field pose, rad).
 * Robot-frame velocities by central differences of the pose; effort e_k = kV * mean(c_{k-1}, c_k) because powers are zero-order
 * held between samples. FLOAT stretches (all powers 0 at k-1 and k) fit F; powered stretches then fit tau and Vss with that F.
 * params is the resolved params (used for wheel vectors, nominal voltage and fall-back coulomb values).
 */
export function fitFromTrace(trace, params) {
  const W = wheelVecs(params.mecanum.xVelocity, params.mecanum.yVelocity);
  const nomV = params.plant.plantNominalV;
  const n = trace.length;
  const vel = new Array(n).fill(null);
  for (let k = 1; k < n - 1; k++) {
    const a = trace[k - 1], b = trace[k], c = trace[k + 1];
    const dt = c.t - a.t;
    if (!(dt > 0)) continue;
    const vx = (c.x - a.x) / dt, vy = (c.y - a.y) / dt, w = wrapPi(c.h - a.h) / dt;
    const co = Math.cos(b.h), si = Math.sin(b.h);
    vel[k] = [vx * co + vy * si, -vx * si + vy * co, w];
  }
  // samples where the plant is traction-limited (friction circle) violate the linear model: drop them
  const aLim = params.plant.aTractionMax * 0.97, hs = params.plant.halfSumIn;
  const sat = new Array(n).fill(false);
  for (let k = 2; k < n - 2; k++) {
    if (!vel[k - 1] || !vel[k + 1]) continue;
    const dt = trace[k + 1].t - trace[k - 1].t;
    const a0 = vel[k + 1][0] - vel[k - 1][0], a1 = vel[k + 1][1] - vel[k - 1][1], a2 = vel[k + 1][2] - vel[k - 1][2];
    if (Math.hypot(a0, a1, a2 * hs) / dt > aLim) for (let q = k - 2; q <= k + 2; q++) sat[q] = true;
  }
  const eff = trace.map((s) => effortsOf(s.p, W));
  const isFloat = trace.map((s) => s.p.every((q) => q === 0));
  const out = { rms: {} };
  const names = ['X', 'Y', 'W'], rkeys = ['x', 'y', 'w'];
  const defF = [params.plant.coulombX, params.plant.coulombY, params.plant.coulombW];
  for (let j = 0; j < 3; j++) {
    const powered = [], floats = [];
    for (let k = 1; k < n; k++) {
      if (!vel[k] || sat[k]) continue;
      if (isFloat[k] && isFloat[k - 1]) { floats.push({ t: trace[k].t, e: 0, v: vel[k][j] }); continue; }
      const kV = trace[k].V !== undefined ? trace[k].V / nomV : 1;
      powered.push({ t: trace[k].t, e: kV * 0.5 * (eff[k - 1][j] + eff[k][j]), v: vel[k][j] });
    }
    // central-difference acceleration needs neighbours of the same regime; fit contiguous runs separately
    const runs = (arr) => {
      const res = []; let cur = [];
      for (const s of arr) {
        if (cur.length && s.t - cur[cur.length - 1].t > 1.5 * (trace[1].t - trace[0].t) + 1e-9) { res.push(cur); cur = []; }
        cur.push(s);
      }
      if (cur.length) res.push(cur);
      return res;
    };
    const fitRuns = (arr, opts) => {
      const rows = runs(arr).filter((r) => r.length >= 3);
      return rows.length ? mergeFits(rows.map((r) => fitAxis(r, opts))) : { ...NAN_FIT };
    };
    const fF = floats.length >= 3 ? fitRuns(floats) : NAN_FIT;
    const F = Number.isFinite(fF.coulomb) ? fF.coulomb : undefined;
    let fit = fitRuns(powered, F !== undefined ? { coulomb: F } : {});
    if (!Number.isFinite(fit.tau)) fit = fitRuns(powered, { coulomb: defF[j] });
    const coul = Number.isFinite(fit.coulomb) ? fit.coulomb : defF[j];
    out['tau' + names[j]] = fit.tau;
    out['coulomb' + names[j]] = F !== undefined ? F : coul;
    out[['xVelocity', 'yVelocity', 'omegaMax'][j]] = fit.vss;
    out.rms[rkeys[j]] = fit.rms;
  }
  return out;
}

/** Weighted (by n) merge of per-run fits. */
function mergeFits(fits) {
  const good = fits.filter((f) => Number.isFinite(f.tau) || Number.isFinite(f.coulomb));
  if (!good.length) return { ...NAN_FIT };
  let n = 0;
  const acc = { tau: 0, vFree: 0, vss: 0, coulomb: 0, rms2: 0 };
  const has = { tau: 0, vFree: 0, vss: 0, coulomb: 0 };
  for (const f of good) {
    n += f.n;
    for (const k of ['tau', 'vFree', 'vss', 'coulomb']) if (Number.isFinite(f[k])) { acc[k] += f[k] * f.n; has[k] += f.n; }
    acc.rms2 += f.rms * f.rms * f.n;
  }
  const r = { n, rms: Math.sqrt(acc.rms2 / n) };
  for (const k of ['tau', 'vFree', 'vss', 'coulomb']) r[k] = has[k] ? acc[k] / has[k] : NaN;
  return r;
}

function interp(tr, t) {
  if (t <= tr[0].t) return tr[0];
  const last = tr[tr.length - 1];
  if (t >= last.t) return last;
  let lo = 0, hi = tr.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tr[m].t <= t) lo = m; else hi = m; }
  const a = tr[lo], b = tr[hi], f = (t - a.t) / (b.t - a.t || 1);
  return { t, x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, h: a.h + wrapPi(b.h - a.h) * f };
}

function endTime(tr, eps) {
  let end = tr[0].t;
  for (let k = 1; k < tr.length; k++) {
    const dt = tr[k].t - tr[k - 1].t;
    if (dt > 0 && Math.hypot(tr[k].x - tr[k - 1].x, tr[k].y - tr[k - 1].y) / dt > eps) end = tr[k].t;
  }
  return end - tr[0].t;
}

/**
 * Compares a sim trace with a real trace, both [{t,x,y,h}], aligned in time at their first samples. The real trace is
 * linearly interpolated at each sim time inside the overlap. endTimeDelta = sim motion-end time - real motion-end time
 * (last time speed exceeds opts.stopEps, default 1 in/s). opts.alignPose: translate/rotate nothing, but subtract the initial
 * pose offset when true.
 */
export function compareTraces(sim, real, opts = {}) {
  if (!sim.length || !real.length) return { rmsPosIn: NaN, maxPosIn: NaN, rmsHeadRad: NaN, endTimeDelta: NaN, n: 0 };
  const s0 = sim[0].t, r0 = real[0].t;
  const rs = real.map((q) => ({ ...q, t: q.t - r0 }));
  const dur = rs[rs.length - 1].t;
  const ox = opts.alignPose ? sim[0].x - real[0].x : 0, oy = opts.alignPose ? sim[0].y - real[0].y : 0;
  const oh = opts.alignPose ? wrapPi(sim[0].h - real[0].h) : 0;
  let ss = 0, mx = 0, hs = 0, n = 0;
  for (const s of sim) {
    const t = s.t - s0;
    if (t > dur + 1e-9) break;
    const q = interp(rs, t);
    const d = Math.hypot(s.x - (q.x + ox), s.y - (q.y + oy));
    ss += d * d; if (d > mx) mx = d;
    const dh = wrapPi(s.h - (q.h + oh)); hs += dh * dh; n++;
  }
  const eps = opts.stopEps ?? 1;
  return {
    rmsPosIn: n ? Math.sqrt(ss / n) : NaN, maxPosIn: mx, rmsHeadRad: n ? Math.sqrt(hs / n) : NaN,
    endTimeDelta: endTime(sim, eps) - endTime(real, eps), n,
  };
}

/** Parses CSV with header t,x,y,h,p0,p1,p2,p3[,V] (h in rad, any column order, '#' comments and blank lines skipped). */
export function parseTraceCsv(text) {
  const lines = String(text).split(/\r?\n/).map((l) => l.trim()).filter((l) => l && !l.startsWith('#'));
  if (!lines.length) return [];
  const head = lines[0].split(',').map((s) => s.trim().toLowerCase());
  const col = (n) => head.indexOf(n);
  const ix = { t: col('t'), x: col('x'), y: col('y'), h: col('h'), p: [0, 1, 2, 3].map((i) => col('p' + i)), V: col('v') };
  if ([ix.t, ix.x, ix.y, ix.h, ...ix.p].some((i) => i < 0)) throw new Error('trace CSV header must contain t,x,y,h,p0,p1,p2,p3[,V]');
  const out = [];
  for (let i = 1; i < lines.length; i++) {
    const f = lines[i].split(',');
    const num = (c) => parseFloat(f[c]);
    const row = { t: num(ix.t), x: num(ix.x), y: num(ix.y), h: num(ix.h), p: ix.p.map(num) };
    if (ix.V >= 0 && Number.isFinite(num(ix.V))) row.V = num(ix.V);
    if (Number.isFinite(row.t) && Number.isFinite(row.x) && Number.isFinite(row.y) && Number.isFinite(row.h) && row.p.every(Number.isFinite)) out.push(row);
  }
  return out;
}
