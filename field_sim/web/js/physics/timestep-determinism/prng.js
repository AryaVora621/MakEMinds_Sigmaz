// prng.js - deterministic PRNG (sfc32 seeded by splitmix32), named streams, random trap.
// Only integer ops (Math.imul, |0, >>>0) and + - * / are used, so output is identical across JS engines.
// Source: bryc/code jshash/PRNGs.md (https://github.com/bryc/code/blob/master/jshash/PRNGs.md).
import { TS } from './params.js';

const U32 = 4294967296;

// splitmix32 -> () => uint32
export function splitmix32(seed) {
  let a = seed | 0;
  return function () {
    a = (a + 0x9e3779b9) | 0;
    let t = a ^ (a >>> 16);
    t = Math.imul(t, 0x21f0aaad);
    t = t ^ (t >>> 15);
    t = Math.imul(t, 0x735a2d97);
    return (t ^ (t >>> 15)) >>> 0;
  };
}

// FNV-1a 32-bit over UTF-16 code units of a string.
export function fnv1a32(str, seed = 0x811c9dc5) {
  let h = seed >>> 0;
  for (let i = 0; i < str.length; i++) {
    h ^= str.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h >>> 0;
}

function makeRng(seed, s0, s1, s2, s3) {
  let a = s0 | 0, b = s1 | 0, c = s2 | 0, d = s3 | 0;
  function nextU32() {
    const t = (((a + b) | 0) + d) | 0;
    d = (d + 1) | 0;
    a = b ^ (b >>> 9);
    b = (c + (c << 3)) | 0;
    c = (c << 21) | (c >>> 11);
    c = (c + t) | 0;
    return t >>> 0;
  }
  const rng = {
    seed: seed >>> 0,
    nextU32,
    next() { return nextU32() / U32; },
    range(lo, hi) { return lo + (hi - lo) * (nextU32() / U32); },
    int(lo, hiExclusive) { return lo + Math.floor((nextU32() / U32) * (hiExclusive - lo)); },
    // Irwin-Hall: sum of 12 uniforms - 6 has mean 0, variance 1; only + - needed.
    gaussian(mean = 0, sd = 1) {
      let s = 0;
      for (let i = 0; i < 12; i++) s += nextU32() / U32;
      return mean + sd * (s - 6);
    },
    stream(name) {
      return createRng((rng.seed ^ fnv1a32(String(name))) >>> 0);
    },
    getState() { return [a >>> 0, b >>> 0, c >>> 0, d >>> 0]; },
    setState(s) {
      if (!s || s.length !== 4) throw new Error('rng state must be 4 uint32');
      a = s[0] | 0; b = s[1] | 0; c = s[2] | 0; d = s[3] | 0;
    },
  };
  return rng;
}

export function createRng(seed = TS.DEFAULT_SEED) {
  const sm = splitmix32(seed);
  const rng = makeRng(seed, sm(), sm(), sm(), sm());
  for (let i = 0; i < 15; i++) rng.nextU32(); // bryc: discard 15 outputs after seeding
  return rng;
}

// Replace Math.random with a counting (or throwing) wrapper. restore() puts the original back.
export function installRandomTrap({ throwOnCall = false } = {}) {
  const original = Math.random;
  let n = 0;
  Math.random = function () {
    n++;
    if (throwOnCall) throw new Error('Math.random called inside deterministic sim');
    return original.call(Math);
  };
  return {
    count() { return n; },
    restore() { Math.random = original; },
  };
}
