// timing.js - deterministic RNG, loop-period jitter, delayed command queue, power quantisation.
// Pure ES module, no DOM, no Math.random. Units: seconds.
// All timing values come from Params['timing'] (see params.js; each is tagged [ASSUMED]/[SRC] there).

/** Seeded 32-bit PRNG (mulberry32). Returns a function giving floats in [0,1). */
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

/** Standard normal sample via Box-Muller. */
export function gaussian(rng) {
  let u = rng();
  if (u < 1e-12) u = 1e-12; // avoid log(0)
  const v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function clamp(x, lo, hi) { return x < lo ? lo : (x > hi ? hi : x); }

/** OpMode loop clock: next loop start = t + clamp(mean + sd*N(0,1), min, max) seconds. */
export class LoopClock {
  constructor(timing, rng) {
    this.timing = timing;
    this.rng = rng;
    this.lastPeriodS = timing.loopPeriodMeanS;
  }
  next(t) {
    const tm = this.timing;
    const p = clamp(tm.loopPeriodMeanS + tm.loopJitterSdS * gaussian(this.rng),
      tm.loopPeriodMinS, tm.loopPeriodMaxS);
    this.lastPeriodS = p;
    return t + p;
  }
}

/** FIFO queue ordered by applyAt (stable for equal times). */
export class CommandQueue {
  constructor() { this._items = []; this._seq = 0; }
  push(applyAtS, cmd) {
    const item = { t: applyAtS, seq: this._seq++, cmd };
    const a = this._items;
    // binary search for insertion point after all entries with t <= applyAtS (stable)
    let lo = 0, hi = a.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (a[mid].t <= applyAtS) lo = mid + 1; else hi = mid;
    }
    a.splice(lo, 0, item);
  }
  popDue(t) {
    const a = this._items;
    let n = 0;
    while (n < a.length && a[n].t <= t + 1e-12) n++;
    return a.splice(0, n).map((e) => e.cmd);
  }
  clear() { this._items.length = 0; }
  get size() { return this._items.length; }
}

/** Quantise power in [-1,1] to `levels` steps per unit (e.g. 32767 [ASSUMED]). NaN -> 0. */
export function quantizePower(p, levels) {
  if (!Number.isFinite(p)) return p === Infinity ? 1 : (p === -Infinity ? -1 : 0);
  const c = clamp(p, -1, 1);
  const r = Math.round(c * levels) / levels;
  return r === 0 ? 0 : r; // normalise -0
}
