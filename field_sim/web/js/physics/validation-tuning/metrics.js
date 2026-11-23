// Track and trajectory metrics (dependency-free). Lengths in inches, angles in rad, time in s.
// Tracks are {t:number[], x:number[], y:number[], h?:number[]}. Points are [x,y] or [x,y,z].
import { REAL } from './params.js';

// numpy assert_allclose form: |actual-desired| <= atol + rtol*|desired|. numpy defaults rtol 1e-7, atol 0.
export function allclose(actual, desired, opts = {}) {
  const rtol = opts.rtol ?? 1e-7;
  const atol = opts.atol ?? 0;
  const A = Array.isArray(actual) ? actual : [actual];
  const D = Array.isArray(desired) ? desired : [desired];
  if (A.length !== D.length) throw new TypeError('length mismatch: ' + A.length + ' vs ' + D.length);
  let pass = true, maxErr = 0, worstIndex = -1, allowedAtWorst = 0, worstExcess = -Infinity;
  for (let i = 0; i < A.length; i++) {
    const err = Math.abs(A[i] - D[i]);
    const allowed = atol + rtol * Math.abs(D[i]);
    const ok = err <= allowed; // false for NaN
    if (!ok) pass = false;
    const excess = Number.isNaN(err) ? Infinity : err - allowed;
    if (excess > worstExcess) {
      worstExcess = excess;
      worstIndex = i;
      allowedAtWorst = allowed;
      maxErr = err;
    }
  }
  if (A.length === 0) return { pass: true, maxErr: 0, worstIndex: -1, allowedAtWorst: 0 };
  // maxErr should be the largest absolute error overall
  let m = 0;
  for (let i = 0; i < A.length; i++) {
    const e = Math.abs(A[i] - D[i]);
    if (Number.isNaN(e)) { m = NaN; break; }
    if (e > m) m = e;
  }
  return { pass, maxErr: m, worstIndex, allowedAtWorst };
}

function pairCheck(a, b) {
  if (a.length !== b.length) throw new TypeError('length mismatch');
  if (a.length === 0) throw new TypeError('empty input');
}

export function rmse(a, b) {
  pairCheck(a, b);
  let s = 0;
  for (let i = 0; i < a.length; i++) s += (a[i] - b[i]) ** 2;
  return Math.sqrt(s / a.length);
}

export function maxAbsErr(a, b) {
  pairCheck(a, b);
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i] - b[i]));
  return m;
}

function dist(p, q) {
  let s = 0;
  const n = Math.max(p.length, q.length);
  for (let i = 0; i < n; i++) {
    const d = (p[i] ?? 0) - (q[i] ?? 0);
    s += d * d;
  }
  return Math.sqrt(s);
}

function needPoints(P, Q) {
  if (!Array.isArray(P) || !Array.isArray(Q) || P.length === 0 || Q.length === 0) throw new TypeError('need non-empty point lists');
}

// Discrete Frechet distance (Eiter & Mannila 1994; recurrence PRIOR). Iterative two-row DP.
export function discreteFrechet(P, Q) {
  needPoints(P, Q);
  const q = Q.length;
  let prev = new Float64Array(q);
  let cur = new Float64Array(q);
  prev[0] = dist(P[0], Q[0]);
  for (let j = 1; j < q; j++) prev[j] = Math.max(prev[j - 1], dist(P[0], Q[j]));
  for (let i = 1; i < P.length; i++) {
    cur[0] = Math.max(prev[0], dist(P[i], Q[0]));
    for (let j = 1; j < q; j++) {
      cur[j] = Math.max(Math.min(prev[j], prev[j - 1], cur[j - 1]), dist(P[i], Q[j]));
    }
    [prev, cur] = [cur, prev];
  }
  return prev[q - 1];
}

// Hausdorff distance: ignores order.
export function hausdorff(P, Q) {
  needPoints(P, Q);
  const dir = (A, B) => {
    let worst = 0;
    for (const a of A) {
      let best = Infinity;
      for (const b of B) best = Math.min(best, dist(a, b));
      if (best > worst) worst = best;
    }
    return worst;
  };
  return Math.max(dir(P, Q), dir(Q, P));
}

// Dynamic time warping: minimum SUM of point distances along a monotone warp path. Not a metric.
export function dtw(P, Q) {
  needPoints(P, Q);
  const q = Q.length;
  let prev = new Float64Array(q);
  let cur = new Float64Array(q);
  prev[0] = dist(P[0], Q[0]);
  for (let j = 1; j < q; j++) prev[j] = prev[j - 1] + dist(P[0], Q[j]);
  for (let i = 1; i < P.length; i++) {
    cur[0] = prev[0] + dist(P[i], Q[0]);
    for (let j = 1; j < q; j++) cur[j] = dist(P[i], Q[j]) + Math.min(prev[j], prev[j - 1], cur[j - 1]);
    [prev, cur] = [cur, prev];
  }
  return prev[q - 1];
}

// Wrap an angle to (-pi, pi].
export function wrapAngle(a) {
  const TWO_PI = 2 * Math.PI;
  let r = a % TWO_PI;
  if (r > Math.PI) r -= TWO_PI;
  else if (r <= -Math.PI) r += TWO_PI;
  return r;
}

function checkTrack(tr) {
  if (!tr || !Array.isArray(tr.t) || !Array.isArray(tr.x) || !Array.isArray(tr.y)) throw new TypeError('bad track');
  if (tr.t.length === 0 || tr.t.length !== tr.x.length || tr.t.length !== tr.y.length) throw new TypeError('bad track lengths');
  if (tr.h && tr.h.length !== tr.t.length) throw new TypeError('bad track heading length');
}

// Linear interpolation by time (clamped at the ends); heading uses the shortest arc.
export function resampleByTime(track, tQuery) {
  checkTrack(track);
  const { t, x, y, h } = track;
  const n = t.length;
  const out = { t: tQuery.slice(), x: [], y: [] };
  if (h) out.h = [];
  let k = 0;
  for (const tq of tQuery) {
    if (tq <= t[0] || n === 1) {
      out.x.push(x[0]); out.y.push(y[0]); if (h) out.h.push(h[0]);
      continue;
    }
    if (tq >= t[n - 1]) {
      out.x.push(x[n - 1]); out.y.push(y[n - 1]); if (h) out.h.push(h[n - 1]);
      continue;
    }
    if (k > 0 && t[k] > tq) k = 0; // query went backwards
    while (k < n - 2 && t[k + 1] < tq) k++;
    const dt = t[k + 1] - t[k];
    const f = dt > 0 ? (tq - t[k]) / dt : 0;
    out.x.push(x[k] + f * (x[k + 1] - x[k]));
    out.y.push(y[k] + f * (y[k + 1] - y[k]));
    if (h) out.h.push(h[k] + f * wrapAngle(h[k + 1] - h[k]));
  }
  return out;
}

// Closed-form 2D Kabsch (PRIOR): find R(theta), t minimising |Q - (R P + t)|.
export function alignSE2(P, Q) {
  if (P.length !== Q.length) throw new TypeError('length mismatch');
  needPoints(P, Q);
  const n = P.length;
  let pbx = 0, pby = 0, qbx = 0, qby = 0;
  for (let i = 0; i < n; i++) { pbx += P[i][0]; pby += P[i][1]; qbx += Q[i][0]; qby += Q[i][1]; }
  pbx /= n; pby /= n; qbx /= n; qby /= n;
  let s = 0, c = 0;
  for (let i = 0; i < n; i++) {
    const px = P[i][0] - pbx, py = P[i][1] - pby, qx = Q[i][0] - qbx, qy = Q[i][1] - qby;
    s += px * qy - py * qx;
    c += px * qx + py * qy;
  }
  const theta = Math.atan2(s, c);
  const ct = Math.cos(theta), st = Math.sin(theta);
  return { theta, tx: qbx - (ct * pbx - st * pby), ty: qby - (st * pbx + ct * pby) };
}

// Estimate samples inside the reference time span, paired with the resampled reference.
function pairTracks(est, ref) {
  checkTrack(est);
  checkTrack(ref);
  const t0 = ref.t[0], t1 = ref.t[ref.t.length - 1];
  const idx = [];
  for (let i = 0; i < est.t.length; i++) if (est.t[i] >= t0 && est.t[i] <= t1) idx.push(i);
  if (idx.length === 0) throw new TypeError('tracks do not overlap in time');
  const tq = idx.map((i) => est.t[i]);
  const r = resampleByTime(ref, tq);
  const e = {
    t: tq,
    x: idx.map((i) => est.x[i]),
    y: idx.map((i) => est.y[i]),
  };
  if (est.h) e.h = idx.map((i) => est.h[i]);
  return { e, r };
}

// Absolute trajectory error (est vs ref resampled at est.t). align=false by default.
export function ate(est, ref, opts = {}) {
  const { e, r } = pairTracks(est, ref);
  const n = e.t.length;
  let al = { theta: 0, tx: 0, ty: 0 };
  if (opts.align) {
    al = alignSE2(e.x.map((v, i) => [v, e.y[i]]), r.x.map((v, i) => [v, r.y[i]]));
  }
  const ct = Math.cos(al.theta), st = Math.sin(al.theta);
  let s2 = 0, sum = 0, mx = 0;
  for (let i = 0; i < n; i++) {
    const px = ct * e.x[i] - st * e.y[i] + al.tx;
    const py = st * e.x[i] + ct * e.y[i] + al.ty;
    const d = Math.hypot(r.x[i] - px, r.y[i] - py);
    s2 += d * d; sum += d; if (d > mx) mx = d;
  }
  return { rmse: Math.sqrt(s2 / n), mean: sum / n, max: mx, n, align: al };
}

// SE(2) helpers: pose = [x, y, theta]
function inv([x, y, th]) {
  const c = Math.cos(th), s = Math.sin(th);
  return [-(c * x + s * y), -(-s * x + c * y), -th];
}
function mul([x1, y1, t1], [x2, y2, t2]) {
  const c = Math.cos(t1), s = Math.sin(t1);
  return [x1 + c * x2 - s * y2, y1 + s * x2 + c * y2, t1 + t2];
}

// Relative pose error (Sturm et al. form, PRIOR) over interval delta (default REAL.rpeDeltaS = 1.0 s).
export function rpe(est, ref, opts = {}) {
  const delta = opts.delta ?? REAL.rpeDeltaS; // s
  const { e, r } = pairTracks(est, ref);
  const n = e.t.length;
  const P = e.t.map((_, i) => [e.x[i], e.y[i], e.h ? e.h[i] : 0]);
  const Q = e.t.map((_, i) => [r.x[i], r.y[i], r.h ? r.h[i] : 0]);
  let st = 0, sr = 0, cnt = 0;
  let j = 0;
  for (let i = 0; i < n; i++) {
    if (j < i) j = i;
    while (j < n && e.t[j] < e.t[i] + delta) j++;
    if (j >= n) break;
    const dQ = mul(inv(Q[i]), Q[j]);
    const dP = mul(inv(P[i]), P[j]);
    const E = mul(inv(dQ), dP);
    st += E[0] * E[0] + E[1] * E[1];
    const w = wrapAngle(E[2]);
    sr += w * w;
    cnt++;
  }
  if (cnt === 0) return { transRmse: NaN, rotRmse: NaN, n: 0 };
  return { transRmse: Math.sqrt(st / cnt), rotRmse: Math.sqrt(sr / cnt), n: cnt };
}

// All track metrics in one call. Shape metrics compare sim samples to the real track resampled at sim times.
export function compareTracks(sim, real, opts = {}) {
  const { e, r } = pairTracks(sim, real);
  const P = e.x.map((v, i) => [v, e.y[i]]);
  const Q = r.x.map((v, i) => [v, r.y[i]]);
  return {
    ate: ate(sim, real, { align: false }),
    ateAligned: ate(sim, real, { align: true }),
    rpe: rpe(sim, real, opts),
    frechet: discreteFrechet(P, Q),
    hausdorff: hausdorff(P, Q),
    dtw: dtw(P, Q),
    durationS: e.t[e.t.length - 1] - e.t[0],
  };
}
