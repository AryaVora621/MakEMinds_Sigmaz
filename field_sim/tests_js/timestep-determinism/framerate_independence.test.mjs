import test from 'node:test';
import assert from 'node:assert/strict';
import { TS } from '../../web/js/physics/timestep-determinism/params.js';
import { createRng } from '../../web/js/physics/timestep-determinism/prng.js';
import { approach, integrateBodyTwist } from '../../web/js/physics/timestep-determinism/integrators.js';
import { FixedClock } from '../../web/js/physics/timestep-determinism/fixedclock.js';
import { createSimLoop } from '../../web/js/physics/timestep-determinism/simloop.js';

// Toy sim (duplicated from replay.test.mjs on purpose: no shared helper files).
function createSim(seed) {
  const rng = createRng(seed).stream('shoot');
  const pose = { x: 0, y: 0, h: 0 };
  const st = { vx: 0, om: 0, ball: { x: 0, y: 0 } };
  let vt = 0, ot = 0;
  return {
    stepTick({ h, isControl, input }) {
      if (isControl) {
        vt = (input?.axes?.fwd ?? 0) * 50; // [in/s]
        ot = (input?.axes?.turn ?? 0) * 2; // [rad/s]
        st.ball.x += rng.gaussian() * 0.01; // [in]
        st.ball.y += rng.gaussian() * 0.01;
      }
      st.vx = approach(st.vx, vt, 100, 200, h);
      st.om = approach(st.om, ot, 20, 40, h);
      integrateBodyTwist(pose, st.vx, 0, st.om, h);
    },
    snapshot() { return { pose: { ...pose }, vx: st.vx, om: st.om, ball: { ...st.ball } }; },
  };
}

// Wall-time input script, piecewise constant, with changes only at control-tick boundaries
// (multiples of 1/60 s). The loop evaluates it at the sim time of the control tick (tick*h) so that
// display rate cannot change which input a tick sees.
function script(tSeconds) {
  if (tSeconds < 1.0) return { axes: { fwd: 0.6, turn: 0.1 }, buttons: {} };
  if (tSeconds < 2.5) return { axes: { fwd: -0.2, turn: 0.8 }, buttons: {} };
  return { axes: { fwd: 1, turn: -0.4 }, buttons: {} };
}

function drive(frameDts, { backlog = 'drop', untilTick = 1200 } = {}) {
  const sim = createSim(TS.DEFAULT_SEED);
  const hashes = new Map();
  const loop = createSimLoop({
    clock: new FixedClock({ backlog }),
    stepTick: sim.stepTick,
    snapshot: sim.snapshot,
    inputSource: (tick) => script(tick * TS.PHYSICS_DT),
    recorder: { record() {}, recordHash: (t, h) => hashes.set(t, h.exact) },
  });
  let guard = 0;
  for (const dt of frameDts) {
    loop.frame(dt);
    if (loop.tick >= untilTick || ++guard > 1e6) break;
  }
  return { hashes, loop };
}

const constantFrames = (rate) => ({ *[Symbol.iterator]() { for (;;) yield 1 / rate; } });

test('exact hashes at 5 s sim time match across 30, 60 and 144 Hz display rates', () => {
  const results = [30, 60, 144].map((rate) => drive(constantFrames(rate)).hashes);
  const ref = results[0].get(1200);
  assert.ok(ref, 'hash at tick 1200 recorded');
  for (const h of results) {
    for (const t of [240, 480, 720, 960, 1200]) assert.equal(h.get(t), results[0].get(t), `tick ${t}`);
  }
});

test('heavy-load frame pattern (drop policy) gives the same tick-indexed state as smooth frames', () => {
  const rng = createRng(777);
  const heavy = { *[Symbol.iterator]() { for (;;) yield rng.range(0, 0.3); } };
  const smooth = drive(constantFrames(60)).hashes;
  const loaded = drive(heavy, { backlog: 'drop' });
  assert.ok(loaded.loop.tick >= 1200);
  for (const t of [240, 480, 720, 960, 1200]) assert.equal(loaded.hashes.get(t), smooth.get(t), `tick ${t}`);
});

test('runTicks headless matches the frame-driven hash', () => {
  const sim = createSim(TS.DEFAULT_SEED);
  const hashes = new Map();
  const loop = createSimLoop({
    stepTick: sim.stepTick,
    snapshot: sim.snapshot,
    inputSource: (tick) => script(tick * TS.PHYSICS_DT),
    recorder: { record() {}, recordHash: (t, h) => hashes.set(t, h.exact) },
  });
  loop.runTicks(1200);
  assert.equal(hashes.get(1200), drive(constantFrames(144)).hashes.get(1200));
});

test.skip('TODO: app.js wiring (loop.frame replaces simUpdate(dt)) is the integrator\'s job', () => {});
