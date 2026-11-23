// Mini property-based testing (dependency-free; no fast-check available). Generators are {generate(rng), shrink(v)}.
// shrink(v) returns an iterable of strictly "simpler" candidates, most aggressive first.
// Numbers follow the sim: inches, s, rad.
import { BallWorld, BALL } from '../../balls.js';
import { PROP } from './params.js';
import { mulberry32, seedFromEnv } from './rng.js';
import { createMonitor } from './invariants.js';
import { FIELD_BOUNDS } from './node_env.js';

const DEFAULT_SEED = 0x5eed;   // ASSUMED: fixed default so runs never depend on the clock

function bisectToward(v, target) {
  const out = [];
  if (v === target) return out;
  out.push(target);
  let d = (v - target) / 2;
  for (let k = 0; k < 40 && Math.abs(d) > 1e-9; k++, d /= 2) out.push(v - d);
  return out;
}

export const gen = {
  /** Uniform double in [min,max]; shrinks by bisecting toward the bound nearest 0. */
  double({ min, max }) {
    const target = Math.min(max, Math.max(min, 0));
    return { generate: rng => min + (max - min) * rng(), shrink: v => bisectToward(v, target) };
  },
  /** Uniform integer in [min,max]; shrinks toward the bound nearest 0. */
  integer({ min, max }) {
    const target = Math.min(max, Math.max(min, 0));
    return {
      generate: rng => min + Math.floor(rng() * (max - min + 1)),
      shrink: v => [...new Set(bisectToward(v, target).map(Math.round))].filter(x => x !== v && x >= min && x <= max),
    };
  },
  constant(v) { return { generate: () => v, shrink: () => [] }; },
  /** Pick one of the values; shrinks toward earlier entries. */
  oneOf(...vals) {
    return { generate: rng => vals[Math.floor(rng() * vals.length)], shrink: v => vals.slice(0, Math.max(0, vals.indexOf(v))) };
  },
  /** Object with one generator per key; shrinks one field at a time. */
  record(shape) {
    const keys = Object.keys(shape);
    return {
      generate: rng => { const o = {}; for (const k of keys) o[k] = shape[k].generate(rng); return o; },
      shrink: function* (v) { for (const k of keys) for (const c of shape[k].shrink(v[k])) yield { ...v, [k]: c }; },
    };
  },
  /** Array of g; shrinks by dropping elements (down to minLength) and then by shrinking one element at a time. */
  array(g, { minLength = 0, maxLength = 10 } = {}) {
    return {
      generate: rng => { const n = minLength + Math.floor(rng() * (maxLength - minLength + 1)); return Array.from({ length: n }, () => g.generate(rng)); },
      shrink: function* (v) {
        if (v.length > minLength) {
          yield v.slice(0, Math.max(minLength, v.length >> 1));
          for (let i = 0; i < v.length; i++) yield v.filter((_, j) => j !== i);
        }
        for (let i = 0; i < v.length; i++) for (const c of g.shrink(v[i])) yield v.map((x, j) => (j === i ? c : x));
      },
    };
  },
};

function runPred(pred, v) {
  try { const r = pred(v); return r === false ? { ok: false, error: 'predicate returned false' } : { ok: true }; }
  catch (e) { return { ok: false, error: e && e.message ? e.message : String(e) }; }
}

/** Run numRuns random cases. Seed: env PROP_SEED if set, else opts.seed, else 0x5eed. numRuns: opts, else env PROP_RUNS, else PROP.numRunsCI. */
export function forAll(arb, predicate, opts = {}) {
  const seed = seedFromEnv(PROP.seedEnv, opts.seed ?? DEFAULT_SEED);
  const numRuns = opts.numRuns ?? seedFromEnv(PROP.runsEnv, PROP.numRunsCI);
  const maxShrink = opts.maxShrinkSteps ?? PROP.maxShrinkSteps;
  const rng = mulberry32(seed);
  for (let run = 0; run < numRuns; run++) {
    const value = arb.generate(rng);
    const r = runPred(predicate, value);
    if (r.ok) continue;
    // greedy shrink: accept the first failing candidate, restart from it; at most maxShrink accepted steps
    let cur = value, err = r.error, accepted = 0, tries = 0;
    const triesCap = maxShrink * 20;
    outer: while (accepted < maxShrink) {
      for (const cand of arb.shrink(cur)) {
        if (++tries > triesCap) break outer;
        const rr = runPred(predicate, cand);
        if (!rr.ok) { cur = cand; err = rr.error; accepted++; continue outer; }
      }
      break;
    }
    return { ok: false, runs: run + 1, seed, failure: { run, value, shrunk: cur, error: err } };
  }
  return { ok: true, runs: numRuns, seed };
}

export function assertProperty(arb, predicate, opts = {}) {
  const r = forAll(arb, predicate, opts);
  if (!r.ok) {
    throw new Error(`property failed (seed ${r.seed}, run ${r.failure.run}): ${r.failure.error}\n  original: ${JSON.stringify(r.failure.value)}\n  shrunk:   ${JSON.stringify(r.failure.shrunk)}`);
  }
}

// ---------------------------------------------------------------- standard properties

const BR = BALL.r;
const BX = { min: FIELD_BOUNDS.minX + BR + 1, max: FIELD_BOUNDS.maxX - BR - 1 };
const BY = { min: FIELD_BOUNDS.minY + BR + 1, max: FIELD_BOUNDS.maxY - BR - 1 };
const VC = 600 / Math.sqrt(3);   // per-axis bound so that |v| <= 600 in/s (spec randomDrop)
const ballArb = gen.record({
  x: gen.double(BX), y: gen.double(BY), z: gen.double({ min: 2, max: 80 }),
  vx: gen.double({ min: -VC, max: VC }), vy: gen.double({ min: -VC, max: VC }), vz: gen.double({ min: -VC, max: VC }),
});

// Drop balls skipping any whose centre would start overlapping an earlier one (random placement would otherwise
// create overlaps that are an artefact of the test, not the sim).
function placeBalls(world, list, skip) {
  const placed = [];
  for (const b of list) {
    if (placed.some(p => Math.hypot(p.x - b.x, p.y - b.y, p.z - b.z) < 2 * BR + 0.5)) continue;
    if (skip && skip(b)) continue;
    placed.push(b);
    world.addBall('pollen', [b.x, b.y, b.z], [b.vx || 0, b.vy || 0, b.vz || 0], 'pollen');
  }
  return placed.length;
}

/** n in 1..20 balls, random pose and velocity (|v| <= 600 in/s), dt from {1/30,1/60,1/120,1/240}, 2 s. All invariants incl. energy. */
export const randomDropArb = gen.record({
  dt: gen.oneOf(1 / 60, 1 / 30, 1 / 120, 1 / 240),
  balls: gen.array(ballArb, { minLength: 1, maxLength: 20 }),
});

export function randomDropProperty(v) {
  const world = new BallWorld({ bounds: { ...FIELD_BOUNDS } }, null);
  placeBalls(world, v.balls);
  const mon = createMonitor(world, { closed: true });
  const steps = Math.round(2 / v.dt);
  for (let i = 0; i < steps; i++) { world.step(v.dt); mon.step(); }
  const rep = mon.report();
  if (!rep.ok) throw new Error('invariant violated: ' + JSON.stringify(rep.violations[0]));
  return true;
}

/** Robot (9 x 9 in half sizes, 14 in tall) driving at 0..80 in/s on a random heading through 1..20 resting balls. Non-energy invariants. */
export const randomPushArb = gen.record({
  speed: gen.double({ min: 0, max: 80 }),
  heading: gen.double({ min: -Math.PI, max: Math.PI }),
  balls: gen.array(gen.record({ x: gen.double(BX), y: gen.double(BY) }), { minLength: 1, maxLength: 20 }),
});

const PUSH_S = 1.5;   // s of driving, ASSUMED (robot travels at most 120 in, centred on the field)
export function randomPushProperty(v) {
  const dt = 1 / 60, dims = { halfX: 9, halfY: 9, height: 14 };
  const world = new BallWorld({ bounds: { ...FIELD_BOUNDS } }, null);
  const dx = Math.cos(v.heading), dy = Math.sin(v.heading);
  const x0 = 72 - dx * v.speed * PUSH_S / 2, y0 = 72 - dy * v.speed * PUSH_S / 2;
  // keep balls clear of the robot's starting footprint (a ball inside the box at t=0 is a test artefact)
  placeBalls(world, v.balls.map(b => ({ x: b.x, y: b.y, z: BR })), b => Math.hypot(b.x - x0, b.y - y0) < 9 * Math.SQRT2 + BR + 1);
  const mon = createMonitor(world, { closed: false });
  const steps = Math.round(PUSH_S / dt);
  for (let i = 0; i < steps; i++) {
    const t = i * dt;
    world.setRobot({ x: x0 + dx * v.speed * t, y: y0 + dy * v.speed * t, h: v.heading, vx: dx * v.speed, vy: dy * v.speed, om: 0 }, dims);
    world.step(dt); mon.step();
  }
  const rep = mon.report();
  if (!rep.ok) throw new Error('invariant violated: ' + JSON.stringify(rep.violations[0]));
  return true;
}
