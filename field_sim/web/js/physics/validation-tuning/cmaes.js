// CMA-ES (covariance matrix adaptation evolution strategy), minimisation, seeded and deterministic.
// Defaults follow Hansen, "The CMA Evolution Strategy: A Tutorial", arXiv:1604.00772, Table 1 (SOURCED).
// Box handling is NOT from the tutorial (ASSUMED project choice): each sample is clamped into [lower, upper] and the
// repaired point is the one evaluated and used for the updates. Tolerances and budget come from params.js CMAES.
import { CMAES } from './params.js';
import { mulberry32, gaussian } from './rng.js';
import { symEig } from './linalg.js';

/**
 * @param {(x:number[])=>number} f objective to minimise
 * @param {number[]} x0 start point (clamped into the box when bounds are given)
 * @param {{sigma0?:number, lower?:number[], upper?:number[], maxEvals?:number, tolFun?:number, tolX?:number, seed?:number, lambda?:number}} [opts]
 * @returns {{x:number[], f:number, evals:number, iterations:number, stop:'tolFun'|'tolX'|'maxEvals'}}
 *   x/f are the best point ever evaluated. Deterministic for a given seed (default seed 1).
 */
export function cmaes(f, x0, opts = {}) {
  const n = x0.length;
  if (n < 1) throw new TypeError('cmaes: x0 must be non-empty');
  const lower = opts.lower ?? null, upper = opts.upper ?? null;
  if ((lower && lower.length !== n) || (upper && upper.length !== n)) throw new TypeError('cmaes: bounds length mismatch');
  const clamp = (x) => {
    for (let i = 0; i < n; i++) {
      if (lower && x[i] < lower[i]) x[i] = lower[i];
      if (upper && x[i] > upper[i]) x[i] = upper[i];
    }
    return x;
  };
  let sigma;
  if (opts.sigma0 !== undefined) sigma = opts.sigma0;
  else if (lower && upper) {
    let s = 0;
    for (let i = 0; i < n; i++) s += upper[i] - lower[i];
    sigma = CMAES.sigma0Frac * (s / n);
  } else sigma = CMAES.sigma0Frac;
  if (!(sigma > 0)) throw new RangeError('cmaes: sigma0 must be positive');
  const maxEvals = opts.maxEvals ?? CMAES.maxEvalsFactor * n * n;
  const tolFun = opts.tolFun ?? CMAES.tolFun;
  const tolX = opts.tolX ?? CMAES.tolX;
  const randn = gaussian(mulberry32(opts.seed ?? 1));

  // Strategy parameters, Table 1.
  const lambda = opts.lambda ?? (4 + Math.floor(3 * Math.log(n)));
  const mu = Math.floor(lambda / 2);
  let w = [];
  for (let i = 1; i <= mu; i++) w.push(Math.log((lambda + 1) / 2) - Math.log(i));
  const wsum = w.reduce((a, b) => a + b, 0);
  w = w.map(v => v / wsum);
  const mueff = 1 / w.reduce((a, b) => a + b * b, 0);
  const cs = (mueff + 2) / (n + mueff + 5);
  const ds = 1 + 2 * Math.max(0, Math.sqrt((mueff - 1) / (n + 1)) - 1) + cs;
  const cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n);
  const c1 = 2 / ((n + 1.3) * (n + 1.3) + mueff);
  const cmu = Math.min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) * (n + 2) + mueff));
  const chiN = Math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n));

  let m = clamp(x0.slice());
  let C = [];
  for (let i = 0; i < n; i++) { C.push(new Array(n).fill(0)); C[i][i] = 1; }
  let B = C.map(r => r.slice());      // B[i] = eigenvector i
  let D = new Array(n).fill(1);       // sqrt eigenvalues
  const pc = new Array(n).fill(0), ps = new Array(n).fill(0);

  let evals = 0, iter = 0;
  let bestX = m.slice(), bestF = f(m.slice());
  evals++;
  const hist = [];
  const histLen = 10 + Math.ceil(30 * n / lambda);
  let stop = 'maxEvals';

  while (evals < maxEvals) {
    iter++;
    const pop = [];
    for (let k = 0; k < lambda; k++) {
      const z = Array.from({ length: n }, () => randn());
      const y = new Array(n).fill(0);
      for (let j = 0; j < n; j++) {
        const dz = D[j] * z[j];
        for (let i = 0; i < n; i++) y[i] += B[j][i] * dz;
      }
      const x = clamp(m.map((mi, i) => mi + sigma * y[i]));
      const yEff = x.map((xi, i) => (xi - m[i]) / sigma);   // repaired step actually taken
      const fx = f(x.slice());
      evals++;
      if (fx < bestF) { bestF = fx; bestX = x.slice(); }
      pop.push({ x, y: yEff, f: fx });
    }
    pop.sort((a, b) => a.f - b.f);

    const yw = new Array(n).fill(0);
    for (let k = 0; k < mu; k++) for (let i = 0; i < n; i++) yw[i] += w[k] * pop[k].y[i];
    m = m.map((mi, i) => mi + sigma * yw[i]);   // c_m = 1
    clamp(m);

    // C^{-1/2} yw = B D^-1 B^T yw
    const t = new Array(n).fill(0);
    for (let j = 0; j < n; j++) {
      let dot = 0;
      for (let i = 0; i < n; i++) dot += B[j][i] * yw[i];
      t[j] = dot / D[j];
    }
    const inv = new Array(n).fill(0);
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) inv[i] += B[j][i] * t[j];
    for (let i = 0; i < n; i++) ps[i] = (1 - cs) * ps[i] + Math.sqrt(cs * (2 - cs) * mueff) * inv[i];
    const psn = Math.sqrt(ps.reduce((a, b) => a + b * b, 0));
    const hsig = psn / Math.sqrt(1 - Math.pow(1 - cs, 2 * iter)) / chiN < 1.4 + 2 / (n + 1) ? 1 : 0;
    for (let i = 0; i < n; i++) pc[i] = (1 - cc) * pc[i] + hsig * Math.sqrt(cc * (2 - cc) * mueff) * yw[i];
    const dh = (1 - hsig) * cc * (2 - cc);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j <= i; j++) {
        let rmu = 0;
        for (let k = 0; k < mu; k++) rmu += w[k] * pop[k].y[i] * pop[k].y[j];
        const v = (1 - c1 - cmu) * C[i][j] + c1 * (pc[i] * pc[j] + dh * C[i][j]) + cmu * rmu;
        C[i][j] = v; C[j][i] = v;
      }
    }
    sigma *= Math.exp((cs / ds) * (psn / chiN - 1));

    const eig = symEig(C);
    B = eig.vectors;
    D = eig.values.map(v => Math.sqrt(Math.max(v, 1e-300)));

    // Termination.
    hist.push(pop[0].f);
    if (hist.length > histLen) hist.shift();
    const fmax = Math.max(...hist, ...pop.map(p => p.f)), fmin = Math.min(...hist, ...pop.map(p => p.f));
    if (hist.length >= histLen && fmax - fmin < tolFun) { stop = 'tolFun'; break; }
    let maxStd = 0;
    for (let i = 0; i < n; i++) maxStd = Math.max(maxStd, sigma * Math.sqrt(Math.max(C[i][i], 0)));
    if (maxStd < tolX) { stop = 'tolX'; break; }
  }
  return { x: bestX, f: bestF, evals, iterations: iter, stop };
}
