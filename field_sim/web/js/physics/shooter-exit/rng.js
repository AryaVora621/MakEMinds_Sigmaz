// Seeded RNG, Gaussian sampler and normal CDF for shot-dispersion work. No DOM, no deps.

// mulberry32: small 32-bit seeded PRNG. Returns a function giving floats in [0,1).
export function mulberry32(seed) {
  let a = (seed >>> 0) || 0;
  return function () {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Box-Muller N(0,1); the second value of each pair is cached.
export function makeGaussian(rng) {
  let spare = null;
  return function () {
    if (spare !== null) { const s = spare; spare = null; return s; }
    let u = rng();
    while (u <= 1e-300) u = rng();          // avoid log(0)
    const v = rng();
    const r = Math.sqrt(-2 * Math.log(u)), th = 2 * Math.PI * v;
    spare = r * Math.sin(th);
    return r * Math.cos(th);
  };
}

// erf via Abramowitz & Stegun formula 7.1.26 (|error| < 1.5e-7).
export function erf(x) {
  if (Number.isNaN(x)) return NaN;
  const sign = x < 0 ? -1 : 1, ax = Math.abs(x);
  const p = 0.3275911, a1 = 0.254829592, a2 = -0.284496736, a3 = 1.421413741, a4 = -1.453152027, a5 = 1.061405429;
  const t = 1 / (1 + p * ax);
  const y = 1 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * Math.exp(-ax * ax);
  return sign * y;
}

// Standard normal CDF Phi(x) = 0.5*(1 + erf(x/sqrt2)).
export function normCdf(x) {
  if (x === Infinity) return 1;
  if (x === -Infinity) return 0;
  return 0.5 * (1 + erf(x / Math.SQRT2));
}
