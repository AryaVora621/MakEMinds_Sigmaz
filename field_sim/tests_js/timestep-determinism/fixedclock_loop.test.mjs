import test from 'node:test';
import assert from 'node:assert/strict';
import { TS } from '../../web/js/physics/timestep-determinism/params.js';
import { FixedClock } from '../../web/js/physics/timestep-determinism/fixedclock.js';
import { createSimLoop } from '../../web/js/physics/timestep-determinism/simloop.js';
import { lerp, wrapAngle, lerpAngle, lerpPose, lerpSnapshot } from '../../web/js/physics/timestep-determinism/interp.js';

const H = 1 / 240;

test('frames at 30, 60 and 144 Hz summing to 1 s give tick 240 (+-1)', () => {
  for (const rate of [30, 60, 144]) {
    const c = new FixedClock();
    for (let f = 0; f < rate; f++) {
      const { steps } = c.advance(1 / rate);
      for (let i = 0; i < steps; i++) c.onStep();
    }
    assert.ok(Math.abs(c.tick - 240) <= 1, `rate ${rate}: tick ${c.tick}`);
  }
});

test('a 1.0 s frame clamps to 0.25 s and caps at 32 steps with drop policy', () => {
  const c = new FixedClock({ backlog: 'drop' });
  const r = c.advance(1.0);
  assert.equal(r.steps, TS.MAX_STEPS_PER_FRAME);
  assert.ok(c.acc < H, `acc ${c.acc}`);
  assert.ok(r.droppedSeconds > 0);
  assert.ok(r.droppedSeconds <= 0.25);
});

test('carry policy keeps the backlog', () => {
  const c = new FixedClock({ backlog: 'carry' });
  const r = c.advance(0.25);
  assert.equal(r.steps, 32);
  assert.ok(c.acc > 20 * H, `acc ${c.acc}`);
  const r2 = c.advance(0);
  assert.equal(r2.steps, 28); // backlog (60 - 32 ticks, +-1 rounding) is paid out next frame, not dropped
  assert.ok(c.acc < H);
});

test('negative / NaN frame dt is treated as 0', () => {
  const c = new FixedClock();
  assert.equal(c.advance(-1).steps, 0);
  assert.equal(c.advance(NaN).steps, 0);
  assert.equal(c.acc, 0);
});

test('alpha stays in [0,1] and simTime == tick*h', () => {
  const c = new FixedClock();
  let s = 12345;
  for (let i = 0; i < 500; i++) {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    const { steps, alpha } = c.advance((s / 4294967296) * 0.3);
    assert.ok(alpha >= 0 && alpha <= 1);
    for (let k = 0; k < steps; k++) c.onStep();
    assert.equal(c.simTime, c.tick * c.h);
  }
});

test('isControlTick follows the divider', () => {
  const c = new FixedClock();
  assert.ok(c.isControlTick(0));
  assert.ok(c.isControlTick(4));
  assert.ok(!c.isControlTick(3));
});

test('createSimLoop: input on control ticks only, ordered steps, periodic hashes', () => {
  const inputTicks = [], stepTicks = [], recorded = [], hashes = [];
  let n = 0;
  const loop = createSimLoop({
    stepTick: (ctx) => { stepTicks.push(ctx.tick); n++; assert.equal(ctx.isControl, ctx.tick % 4 === 0); },
    snapshot: () => ({ n }),
    inputSource: (t) => { inputTicks.push(t); return { t }; },
    recorder: { record: (t) => recorded.push(t), recordHash: (t, h) => hashes.push([t, h]) },
  });
  loop.runTicks(500);
  assert.equal(loop.tick, 500);
  assert.deepEqual(stepTicks, Array.from({ length: 500 }, (_, i) => i));
  assert.equal(inputTicks.length, 125);
  assert.ok(inputTicks.every((t) => t % 4 === 0));
  assert.deepEqual(recorded, inputTicks);
  assert.deepEqual(hashes.map((x) => x[0]), [240, 480]);
  assert.match(hashes[0][1].exact, /^[0-9a-f]{8}$/);
});

test('createSimLoop.frame runs steps and returns interpolated render', () => {
  let x = 0;
  const loop = createSimLoop({ stepTick: () => { x += 1; }, snapshot: () => ({ x }) });
  const r = loop.frame(2.5 * H);
  assert.equal(r.steps, 2);
  assert.ok(r.alpha >= 0 && r.alpha <= 1);
  assert.ok(r.render.x >= 1 && r.render.x <= 2);
});

test('interp helpers', () => {
  assert.equal(lerp(0, 10, 0.25), 2.5);
  assert.ok(Math.abs(wrapAngle(3 * Math.PI) - Math.PI) < 1e-12);
  assert.ok(wrapAngle(-Math.PI) > 0); // range is (-pi, pi]
  const p = lerpPose({ x: 0, y: 0, h: 3.1 }, { x: 2, y: 4, h: -3.1 }, 0.5);
  assert.equal(p.x, 1);
  assert.equal(p.y, 2);
  assert.ok(Math.cos(p.h) < -0.999);
});

test('lerpSnapshot heading crosses +-pi along the short arc', () => {
  const prev = { x: 0, h: 3.1, nested: { heading: 3.1, yaw: -3.1 }, arr: [0, 2], name: 'a' };
  const curr = { x: 10, h: -3.1, nested: { heading: -3.1, yaw: 3.1 }, arr: [2, 4], name: 'b' };
  const snapCopy = JSON.stringify(prev);
  const r = lerpSnapshot(prev, curr, 0.5);
  assert.equal(r.x, 5);
  assert.ok(Math.cos(r.h) < -0.999, `h ${r.h}`);
  assert.ok(Math.cos(r.nested.heading) < -0.999);
  assert.ok(Math.cos(r.nested.yaw) < -0.999);
  assert.deepEqual(r.arr, [1, 3]);
  assert.equal(r.name, 'b');
  assert.equal(JSON.stringify(prev), snapCopy);
  const r0 = lerpSnapshot(null, curr, 0.5);
  assert.deepEqual(r0, curr);
  assert.notEqual(r0, curr);
  assert.ok(Math.abs(lerpAngle(3.1, -3.1, 0.5)) > 3.14);
});
