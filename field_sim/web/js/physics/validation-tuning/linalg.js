// Small dense linear algebra for the validation-tuning calibration code. Dependency-free, browser-safe.
// Units: unitless (callers carry the units of their data). Nothing here is a tunable constant.
//
// solveLeastSquares: Householder QR (PRIOR: Golub & Van Loan, Matrix Computations, ch. 5.2; not fetched this session).
// symEig: cyclic Jacobi eigenvalue iteration for symmetric matrices (PRIOR: Golub & Van Loan ch. 8.5). Fine for n <= ~10.

const RANK_TOL = 1e-12;   // ASSUMED: relative pivot threshold below which a column is treated as linearly dependent
const JACOBI_MAX_SWEEPS = 100; // ASSUMED: convergence is quadratic, 100 sweeps is far more than needed

/**
 * Ordinary least squares min ||A beta - y||_2 by Householder QR.
 * @param {number[][]} A m rows by n columns, m >= n
 * @param {number[]} y length m
 * @returns {{beta:number[], residuals:number[], rss:number, r2:number, rmse:number}}
 *   residuals[i] = y[i] - A[i].beta; r2 = 1 - rss/tss (tss about mean(y); 1 when tss = 0 and rss = 0); rmse = sqrt(rss/m).
 */
export function solveLeastSquares(A, y) {
  const m = A.length;
  if (!Array.isArray(y) || y.length !== m) throw new TypeError('solveLeastSquares: y length must equal A rows');
  if (m === 0) throw new TypeError('solveLeastSquares: empty system');
  const n = A[0].length;
  if (n === 0 || m < n) throw new TypeError('solveLeastSquares: need rows >= columns >= 1');
  const R = A.map(row => {
    if (row.length !== n) throw new TypeError('solveLeastSquares: ragged matrix');
    return Float64Array.from(row);
  });
  const b = Float64Array.from(y);
  let scale = 0;
  for (let j = 0; j < n; j++) {
    let s = 0;
    for (let i = 0; i < m; i++) s += R[i][j] * R[i][j];
    scale = Math.max(scale, Math.sqrt(s));
  }
  const v = new Float64Array(m);
  for (let k = 0; k < n; k++) {
    let norm2 = 0;
    for (let i = k; i < m; i++) norm2 += R[i][k] * R[i][k];
    const norm = Math.sqrt(norm2);
    if (!(norm > RANK_TOL * scale) || norm === 0) throw new Error('solveLeastSquares: rank deficient at column ' + k);
    const alpha = R[k][k] > 0 ? -norm : norm;
    for (let i = k; i < m; i++) v[i] = R[i][k];
    v[k] -= alpha;
    let vn2 = 0;
    for (let i = k; i < m; i++) vn2 += v[i] * v[i];
    if (vn2 === 0) continue;
    for (let j = k; j < n; j++) {
      let dot = 0;
      for (let i = k; i < m; i++) dot += v[i] * R[i][j];
      const f = 2 * dot / vn2;
      for (let i = k; i < m; i++) R[i][j] -= f * v[i];
    }
    let dotb = 0;
    for (let i = k; i < m; i++) dotb += v[i] * b[i];
    const fb = 2 * dotb / vn2;
    for (let i = k; i < m; i++) b[i] -= fb * v[i];
  }
  const beta = new Array(n).fill(0);
  for (let k = n - 1; k >= 0; k--) {
    let s = b[k];
    for (let j = k + 1; j < n; j++) s -= R[k][j] * beta[j];
    beta[k] = s / R[k][k];
  }
  const residuals = new Array(m);
  let rss = 0, ymean = 0;
  for (let i = 0; i < m; i++) ymean += y[i];
  ymean /= m;
  let tss = 0;
  for (let i = 0; i < m; i++) {
    let pred = 0;
    for (let j = 0; j < n; j++) pred += A[i][j] * beta[j];
    const r = y[i] - pred;
    residuals[i] = r;
    rss += r * r;
    tss += (y[i] - ymean) * (y[i] - ymean);
  }
  const r2 = tss > 0 ? 1 - rss / tss : (rss === 0 ? 1 : 0);
  return { beta, residuals, rss, r2, rmse: Math.sqrt(rss / m) };
}

/**
 * Symmetric eigendecomposition by cyclic Jacobi rotations.
 * @param {number[][]} C symmetric n x n
 * @returns {{values:number[], vectors:number[][]}} sorted by descending eigenvalue.
 *   Convention: vectors[i] is the unit eigenvector belonging to values[i], so C*vectors[i] = values[i]*vectors[i].
 */
export function symEig(C) {
  const n = C.length;
  if (n === 0) throw new TypeError('symEig: empty matrix');
  const A = C.map(r => {
    if (r.length !== n) throw new TypeError('symEig: matrix must be square');
    return Float64Array.from(r);
  });
  const V = [];
  for (let i = 0; i < n; i++) { V.push(new Float64Array(n)); V[i][i] = 1; }
  for (let sweep = 0; sweep < JACOBI_MAX_SWEEPS; sweep++) {
    let off = 0, diag = 0;
    for (let i = 0; i < n; i++) {
      diag += A[i][i] * A[i][i];
      for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
    }
    if (off <= 1e-30 * Math.max(diag, 1e-300) || off === 0) break;
    for (let p = 0; p < n - 1; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = A[p][q];
        if (apq === 0) continue;
        const theta = (A[q][q] - A[p][p]) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) {           // A <- A J (columns p, q)
          const akp = A[k][p], akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {           // A <- J^T A (rows p, q)
          const apk = A[p][k], aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {           // V <- V J ; column j of V is eigenvector j
          const vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const idx = Array.from({ length: n }, (_, i) => i).sort((a, b) => A[b][b] - A[a][a]);
  return {
    values: idx.map(i => A[i][i]),
    vectors: idx.map(i => Array.from({ length: n }, (_, k) => V[k][i])),
  };
}
