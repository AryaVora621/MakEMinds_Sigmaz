// replay.js - headless record / replay verification (spec section 6). Duck-typed sim factory, no P2 dependency.
// createSim(seed) -> { stepTick({tick,h,controlDt,isControl,input}), snapshot() }.
// Per-tick order (spec section 4): on control ticks take input; stepTick; tick++; hash when tick % hashEvery == 0.
import { TS } from './params.js';
import { hashBoth } from './statehash.js';
import { quantizeInput, InputRecorder } from './inputlog.js';

export function runReplay(log, {
  createSim, hz = TS.PHYSICS_HZ, controlDivider = TS.CONTROL_DIVIDER,
  stopOnDesync = true, compare = 'exact',
} = {}) {
  const sim = createSim(log.seed);
  const h = 1 / hz;
  const controlDt = h * controlDivider;
  let firstDesync = null;
  const check = (tick, actual) => {
    if (firstDesync) return;
    const exp = tick === 0 ? log.initialHash : log.hashAt(tick);
    if (!exp) return;
    if (exp[compare] !== actual[compare]) {
      firstDesync = { tick, kind: compare, expected: exp[compare], actual: actual[compare] };
    }
  };
  let last = hashBoth(sim.snapshot());
  check(0, last);
  let input = log.inputAt(0);
  let tick = 0;
  while (tick < log.endTick && !(firstDesync && stopOnDesync)) {
    const isControl = tick % controlDivider === 0;
    if (isControl) input = log.inputAt(tick);
    sim.stepTick({ tick, h, controlDt, isControl, input });
    tick++;
    if (log.hashAt(tick) || tick === log.endTick) {
      last = hashBoth(sim.snapshot());
      check(tick, last);
    }
  }
  return { ok: firstDesync === null, ticks: tick, firstDesync, finalHash: last };
}

export function recordRun({
  createSim, seed, schema, inputForTick, ticks,
  hz = TS.PHYSICS_HZ, controlDivider = TS.CONTROL_DIVIDER, hashEvery = TS.HASH_EVERY_TICKS,
}) {
  const sim = createSim(seed);
  const h = 1 / hz;
  const controlDt = h * controlDivider;
  const rec = new InputRecorder({ schema, seed, initialHash: hashBoth(sim.snapshot()) });
  let input = quantizeInput({}, schema);
  for (let tick = 0; tick < ticks;) {
    const isControl = tick % controlDivider === 0;
    if (isControl) {
      input = quantizeInput(inputForTick(tick), schema);
      rec.record(tick, input);
    }
    sim.stepTick({ tick, h, controlDt, isControl, input });
    tick++;
    if (tick % hashEvery === 0) rec.recordHash(tick, hashBoth(sim.snapshot()));
  }
  const log = rec.finish(ticks);
  log.hz = hz;
  log.controlDivider = controlDivider;
  return log;
}
