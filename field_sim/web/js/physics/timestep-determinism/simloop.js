// simloop.js - loop orchestration (timestep-determinism, spec section 4).
// Owns step 1a/1b (input fetch + record on control ticks) and step 9 (tick++ and periodic hash).
// Steps 2-8 are the caller's stepTick. No Date/performance/Math.random reads here.
import { TS } from './params.js';
import { hashBoth } from './statehash.js';
import { FixedClock } from './fixedclock.js';
import { lerpSnapshot } from './interp.js';

export function createSimLoop({
  clock = new FixedClock(),
  stepTick,
  snapshot,
  inputSource = () => undefined,
  recorder = null,
  hashEvery = TS.HASH_EVERY_TICKS,
  angleKeys = [],
}) {
  const loop = {
    clock,
    lastInput: undefined,
    prev: snapshot(),
    curr: null,
    get tick() { return clock.tick; },
    get simTime() { return clock.simTime; },
  };
  loop.curr = loop.prev;

  function oneTick() {
    const tick = clock.tick;
    const isControl = clock.isControlTick(tick);
    if (isControl) {
      const input = inputSource(tick) ?? loop.lastInput;
      if (recorder) recorder.record(tick, input);
      loop.lastInput = input;
    }
    loop.prev = loop.curr;
    stepTick({
      tick,
      h: clock.h,
      controlDt: clock.h * clock.controlDivider,
      isControl,
      input: loop.lastInput,
    });
    loop.curr = snapshot();
    clock.onStep();
    if (recorder && hashEvery > 0 && clock.tick % hashEvery === 0) {
      recorder.recordHash(clock.tick, hashBoth(loop.curr));
    }
  }

  loop.frame = function frame(frameDt) {
    const { steps, alpha, droppedSeconds } = clock.advance(frameDt);
    for (let i = 0; i < steps; i++) oneTick();
    const render = lerpSnapshot(loop.prev, loop.curr, alpha, { angleKeys });
    return { steps, alpha, droppedSeconds, render };
  };

  // Headless: run exactly n ticks, no cap, accumulator untouched.
  loop.runTicks = function runTicks(n) {
    for (let i = 0; i < n; i++) oneTick();
  };

  return loop;
}
