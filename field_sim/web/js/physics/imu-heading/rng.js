// rng.js - seeded RNG for the imu-heading area. No DOM, no deps.
// mulberry32 core; string seeds hashed with FNV-1a 32-bit; Box-Muller gauss() with cached spare.

/** FNV-1a 32-bit hash of a string -> uint32. */
export function fnv1a(str) {
  let h = 0x811c9dc5;
  const s = String(str);
  for (let i = 0; i < s.length; i++) {
    h ^= s.charCodeAt(i);
    h = Math.imul(h, 0x01000193);
  }
  return h >>> 0;
}

/**
 * createRng(seed) -> {seed, next, gauss, normal, fork}
 * next() in [0,1); gauss() ~ N(0,1); fork(label) gives a deterministic independent child stream.
 */
export function createRng(seed) {
  const seed32 = (typeof seed === 'string' ? fnv1a(seed) : (Number(seed) | 0)) >>> 0;
  let a = seed32;
  let spare = null;

  function next() {
    a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  function gauss() {
    if (spare !== null) {
      const s = spare;
      spare = null;
      return s;
    }
    let u1 = next();
    while (u1 <= 0) u1 = next();
    const u2 = next();
    const r = Math.sqrt(-2 * Math.log(u1));
    const th = 2 * Math.PI * u2;
    spare = r * Math.sin(th);
    return r * Math.cos(th);
  }

  function normal(mu, sigma) {
    return mu + sigma * gauss();
  }

  function fork(label) {
    return createRng((fnv1a(label) ^ Math.imul(seed32, 0x9E3779B1)) >>> 0);
  }

  return { seed: seed32, next, gauss, normal, fork };
}
