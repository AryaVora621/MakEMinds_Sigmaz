// Statistics helpers (dependency-free). Source tags per spec section 2.
import { STAT } from './params.js';

function needArr(a, minLen = 1) {
  if (!Array.isArray(a) && !ArrayBuffer.isView(a)) throw new TypeError('expected an array');
  if (a.length < minLen) throw new TypeError('need at least ' + minLen + ' values');
  for (let i = 0; i < a.length; i++) if (!Number.isFinite(a[i])) throw new TypeError('non-finite value at ' + i);
}

export function mean(a) {
  needArr(a);
  let s = 0;
  for (const v of a) s += v;
  return s / a.length;
}

// Sample variance (n-1 denominator).
export function variance(a) {
  needArr(a, 2);
  const m = mean(a);
  let s = 0;
  for (const v of a) s += (v - m) * (v - m);
  return s / (a.length - 1);
}

export function std(a) {
  return Math.sqrt(variance(a));
}

export function median(a) {
  return quantile(a, 0.5);
}

// Quantile, Hyndman-Fan type 7 (linear interpolation of order statistics).
export function quantile(a, q) {
  needArr(a);
  if (!(q >= 0 && q <= 1)) throw new TypeError('q must be in [0,1]');
  const s = Array.from(a).sort((x, y) => x - y);
  const h = (s.length - 1) * q;
  const lo = Math.floor(h);
  const hi = Math.min(lo + 1, s.length - 1);
  return s[lo] + (h - lo) * (s[hi] - s[lo]);
}

// Normal CDF via Abramowitz & Stegun 7.1.26 erf (|err| < 1.5e-7, PRIOR).
function erf(x) {
  const sign = x < 0 ? -1 : 1;
  const ax = Math.abs(x);
  const t = 1 / (1 + 0.3275911 * ax);
  const y =
    1 -
    (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-ax * ax);
  return sign * y;
}

export function normalCdf(x) {
  if (Number.isNaN(x)) throw new TypeError('x is NaN');
  return 0.5 * (1 + erf(x / Math.SQRT2));
}

// Normal quantile via Acklam's rational approximation (rel err about 1.15e-9, PRIOR).
export function normalQuantile(p) {
  if (!(p > 0 && p < 1)) throw new TypeError('p must be in (0,1)');
  const a = [-3.969683028665376e1, 2.209460984245205e2, -2.759285104469687e2, 1.38357751867269e2, -3.066479806614716e1, 2.506628277459239];
  const b = [-5.447609879822406e1, 1.615858368580409e2, -1.556989798598866e2, 6.680131188771972e1, -1.328068155288572e1];
  const c = [-7.784894002430293e-3, -3.223964580411365e-1, -2.400758277161838, -2.549732539343734, 4.374664141464968, 2.938163982698783];
  const d = [7.784695709041462e-3, 3.224671290700398e-1, 2.445134137142996, 3.754408661907416];
  const pLow = 0.02425;
  let q, r;
  if (p < pLow) {
    q = Math.sqrt(-2 * Math.log(p));
    return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
  }
  if (p > 1 - pLow) {
    q = Math.sqrt(-2 * Math.log(1 - p));
    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
  }
  q = p - 0.5;
  r = q * q;
  return ((((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q) / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1);
}

function checkKN(k, n) {
  if (!Number.isInteger(n) || n <= 0) throw new TypeError('n must be a positive integer');
  if (!Number.isInteger(k) || k < 0 || k > n) throw new TypeError('k must be an integer in [0,n]');
}

// Wilson score interval (SOURCED PMC2706447 / Brown, Cai & DasGupta 2001).
export function wilson(k, n, z = STAT.z95) {
  checkKN(k, n);
  const p = k / n;
  const z2 = z * z;
  const denom = 1 + z2 / n;
  const centre = p + z2 / (2 * n);
  const half = z * Math.sqrt((p * (1 - p)) / n + z2 / (4 * n * n));
  let lo = (centre - half) / denom;
  let hi = (centre + half) / denom;
  if (k === 0) lo = 0;
  if (k === n) hi = 1;
  return { p, lo: Math.max(0, lo), hi: Math.min(1, hi) };
}

// Agresti-Coull interval (SOURCED PMC2706447).
export function agrestiCoull(k, n, z = STAT.z95) {
  checkKN(k, n);
  const z2 = z * z;
  const nn = n + z2;
  const pp = (k + z2 / 2) / nn;
  const half = z * Math.sqrt((pp * (1 - pp)) / nn);
  return { p: k / n, lo: Math.max(0, pp - half), hi: Math.min(1, pp + half) };
}

// Two-proportion pooled z test (PRIOR), two-sided p.
export function twoProportionZ(k1, n1, k2, n2) {
  checkKN(k1, n1);
  checkKN(k2, n2);
  const p1 = k1 / n1;
  const p2 = k2 / n2;
  const pp = (k1 + k2) / (n1 + n2);
  const se = Math.sqrt(pp * (1 - pp) * (1 / n1 + 1 / n2));
  if (se === 0) return { z: 0, p: 1 };
  const z = (p1 - p2) / se;
  return { z, p: 2 * (1 - normalCdf(Math.abs(z))) };
}

// Sample size per group for a two-proportion test (PRIOR). (0.5, 0.6) -> 385.
export function sampleSizeTwoProportions(p1, p2, opts = {}) {
  const alpha = opts.alpha ?? STAT.alpha;
  const power = opts.power ?? STAT.power;
  if (!(p1 >= 0 && p1 <= 1 && p2 >= 0 && p2 <= 1)) throw new TypeError('p1,p2 must be in [0,1]');
  if (p1 === p2) throw new TypeError('p1 and p2 must differ');
  const za = alpha === STAT.alpha ? STAT.z95 : normalQuantile(1 - alpha / 2);
  const zb = power === STAT.power ? STAT.zPower80 : normalQuantile(power);
  const n = ((za + zb) ** 2 * (p1 * (1 - p1) + p2 * (1 - p2))) / ((p1 - p2) ** 2);
  return Math.ceil(n - 1e-9);
}

// ln Gamma (Lanczos, g=7, n=9).
function lgamma(x) {
  const g = [0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313, -176.61502916214059, 12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7];
  if (x < 0.5) return Math.log(Math.PI / Math.abs(Math.sin(Math.PI * x))) - lgamma(1 - x);
  x -= 1;
  let s = g[0];
  const t = x + 7.5;
  for (let i = 1; i < 9; i++) s += g[i] / (x + i);
  return 0.5 * Math.log(2 * Math.PI) + (x + 0.5) * Math.log(t) - t + Math.log(s);
}

// Continued fraction for the incomplete beta (modified Lentz).
function betacf(x, a, b) {
  const TINY = 1e-300;
  const qab = a + b, qap = a + 1, qam = a - 1;
  let c = 1;
  let d = 1 - (qab * x) / qap;
  if (Math.abs(d) < TINY) d = TINY;
  d = 1 / d;
  let h = d;
  for (let m = 1; m <= 500; m++) {
    const m2 = 2 * m;
    let aa = (m * (b - m) * x) / ((qam + m2) * (a + m2));
    d = 1 + aa * d; if (Math.abs(d) < TINY) d = TINY;
    c = 1 + aa / c; if (Math.abs(c) < TINY) c = TINY;
    d = 1 / d;
    h *= d * c;
    aa = (-(a + m) * (qab + m) * x) / ((a + m2) * (qap + m2));
    d = 1 + aa * d; if (Math.abs(d) < TINY) d = TINY;
    c = 1 + aa / c; if (Math.abs(c) < TINY) c = TINY;
    d = 1 / d;
    const del = d * c;
    h *= del;
    if (Math.abs(del - 1) < 1e-14) break;
  }
  return h;
}

// Regularised incomplete beta I_x(a,b).
export function regIncBeta(x, a, b) {
  if (!(a > 0 && b > 0)) throw new TypeError('a,b must be > 0');
  if (!(x >= 0 && x <= 1)) throw new TypeError('x must be in [0,1]');
  if (x === 0 || x === 1) return x;
  const bt = Math.exp(lgamma(a + b) - lgamma(a) - lgamma(b) + a * Math.log(x) + b * Math.log(1 - x));
  if (x < (a + 1) / (a + b + 2)) return (bt * betacf(x, a, b)) / a;
  return 1 - (bt * betacf(1 - x, b, a)) / b;
}

export function studentTCdf(t, df) {
  if (!(df > 0)) throw new TypeError('df must be > 0');
  if (Number.isNaN(t)) throw new TypeError('t is NaN');
  if (!Number.isFinite(t)) return t > 0 ? 1 : 0;
  const x = df / (df + t * t);
  const p = 0.5 * regIncBeta(x, df / 2, 0.5);
  return t > 0 ? 1 - p : p;
}

// Welch two-sample t test, two-sided p.
export function welchT(a, b) {
  needArr(a, 2);
  needArr(b, 2);
  const na = a.length, nb = b.length;
  const va = variance(a), vb = variance(b);
  const se2 = va / na + vb / nb;
  const diff = mean(a) - mean(b);
  if (se2 === 0) return { t: diff === 0 ? 0 : Math.sign(diff) * Infinity, df: na + nb - 2, p: diff === 0 ? 1 : 0 };
  const t = diff / Math.sqrt(se2);
  const df = (se2 * se2) / ((va / na) ** 2 / (na - 1) + (vb / nb) ** 2 / (nb - 1));
  const p = regIncBeta(df / (df + t * t), df / 2, 0.5);
  return { t, df, p };
}

// Two-sample Kolmogorov-Smirnov, asymptotic p.
export function ksTwoSample(a, b) {
  needArr(a);
  needArr(b);
  const sa = Array.from(a).sort((x, y) => x - y);
  const sb = Array.from(b).sort((x, y) => x - y);
  const na = sa.length, nb = sb.length;
  let i = 0, j = 0, d = 0;
  while (i < na && j < nb) {
    const v = Math.min(sa[i], sb[j]);
    while (i < na && sa[i] <= v) i++;
    while (j < nb && sb[j] <= v) j++;
    d = Math.max(d, Math.abs(i / na - j / nb));
  }
  const ne = (na * nb) / (na + nb);
  const lambda = (Math.sqrt(ne) + 0.12 + 0.11 / Math.sqrt(ne)) * d;
  let p = 0;
  if (lambda < 0.2) p = 1;
  else {
    let sign = 1;
    for (let k = 1; k <= 100; k++) {
      const term = sign * Math.exp(-2 * k * k * lambda * lambda);
      p += term;
      sign = -sign;
      if (Math.abs(term) < 1e-12) break;
    }
    p = Math.min(1, Math.max(0, 2 * p));
  }
  return { d, p };
}

// Accept iff the Wilson interval lies inside [pMin, pMax] (either bound optional).
export function acceptRate(k, n, opts = {}) {
  const ci = wilson(k, n, opts.z ?? STAT.z95);
  const okLo = opts.pMin === undefined || ci.lo >= opts.pMin;
  const okHi = opts.pMax === undefined || ci.hi <= opts.pMax;
  return { pass: okLo && okHi, ci };
}
