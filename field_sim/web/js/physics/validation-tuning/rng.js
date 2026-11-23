// Seeded RNG helpers. mulberry32 matches tests_js/sim_physics.test.mjs seededRng bit for bit.
export function mulberry32(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Standard normal generator (Box-Muller); the second value of each pair is cached.
export function gaussian(rng) {
  let spare = null;
  return () => {
    if (spare !== null) {
      const v = spare;
      spare = null;
      return v;
    }
    let u = rng();
    while (u <= 0) u = rng();
    const v = rng();
    const r = Math.sqrt(-2 * Math.log(u));
    spare = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  };
}

// Read a numeric seed from process.env[name] (node only); otherwise return fallback.
export function seedFromEnv(name, fallback) {
  try {
    const raw = globalThis.process?.env?.[name];
    if (raw === undefined || raw === '') return fallback;
    const n = Number(raw);
    return Number.isFinite(n) ? n : fallback;
  } catch (_) {
    return fallback;
  }
}
