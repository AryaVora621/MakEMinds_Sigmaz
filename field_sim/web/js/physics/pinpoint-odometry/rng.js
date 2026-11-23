// rng.js - seeded mulberry32 PRNG with Box-Muller gaussian. Pure ES module, no DOM.
// Deterministic for a given seed; never touches Math.random.

export function makeRng(seed = 12345) {
  let s = (seed >>> 0) || 0;
  let spare = null;
  function next() {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  function gauss() {
    if (spare !== null) { const g = spare; spare = null; return g; }
    let u = next();
    if (u < 1e-12) u = 1e-12; // avoid log(0)
    const v = next();
    const r = Math.sqrt(-2 * Math.log(u));
    spare = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  }
  return { next, gauss };
}
