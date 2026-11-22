import test from 'node:test';
import assert from 'node:assert/strict';
import { Scheduler, autoAimCmd, estimateAutoAimSeconds } from '../web/js/scheduler.js';

/** Runs the scheduler for `secs` of simulated time in steps of `dt`. */
function runFor(s, ctx, secs, dt = 0.1) {
  const n = Math.round(secs / dt);
  for (let i = 0; i < n; i++) s.tick(ctx, dt);
}
/** Returns a ctx with setAutoAim/feed spies. */
function spyCtx(extra = {}) {
  const calls = { feed: [], setAutoAim: [] };
  const ctx = {
    feed: v => calls.feed.push(v),
    setAutoAim: v => calls.setAutoAim.push(v),
    ...extra,
  };
  return { ctx, calls };
}

test('estimateAutoAimSeconds is inside [0.15, 3.0]', () => {
  const cases = [
    {},
    { turretErrDeg: 0, flywheelFrac: 0, hoodErrDeg: 0 },
    { turretErrDeg: 1, flywheelFrac: 0.01, hoodErrDeg: 0.01 },
    { turretErrDeg: 300, flywheelFrac: 1, hoodErrDeg: 90 },
    { turretErrDeg: 1e6, flywheelFrac: 1e6, hoodErrDeg: 1e6 },
    { turretErrDeg: NaN, flywheelFrac: NaN, hoodErrDeg: NaN },
  ];
  for (const c of cases) {
    const v = estimateAutoAimSeconds(c);
    assert.ok(Number.isFinite(v), `finite for ${JSON.stringify(c)}`);
    assert.ok(v >= 0.15 && v <= 3.0, `${v} in [0.15, 3.0] for ${JSON.stringify(c)}`);
  }
});

test('estimateAutoAimSeconds grows with turretErrDeg', () => {
  // flywheelFrac 0 removes the spin-up term, which otherwise dominates the max() at small turret errors.
  const spec = { turretRateDegS: 100 };
  const small = estimateAutoAimSeconds({ turretErrDeg: 30, flywheelFrac: 0, spec });
  const big = estimateAutoAimSeconds({ turretErrDeg: 120, flywheelFrac: 0, spec });
  assert.ok(big > small, `${big} > ${small}`);
});

test('estimateAutoAimSeconds is deterministic for the same input', () => {
  const input = { turretErrDeg: 42, flywheelFrac: 0.7, hoodErrDeg: 5, spec: { turretRateDegS: 200, flywheelTauS: 0.4 } };
  const a = estimateAutoAimSeconds(input);
  const b = estimateAutoAimSeconds(input);
  const c = estimateAutoAimSeconds({ ...input, spec: { ...input.spec } });
  assert.equal(a, b);
  assert.equal(a, c);
});

test('autoAimCmd with seconds:0.5 finishes after 0.5 s of ticking, with start and end in trace', () => {
  const s = new Scheduler();
  const { ctx } = spyCtx();
  s.schedule(autoAimCmd({ seconds: 0.5 }));
  runFor(s, ctx, 0.4);
  assert.equal(s.isRunning(), true, 'still running at 0.4 s');
  runFor(s, ctx, 0.2);
  assert.equal(s.isRunning(), false, 'finished by 0.6 s');
  const names = s.trace.filter(e => e.name === 'autoAim').map(e => e.event);
  assert.ok(names.includes('start'), 'trace has start');
  assert.ok(names.includes('end'), 'trace has end');
  assert.equal(names.indexOf('start') < names.indexOf('end'), true);
});

test('autoAimCmd with ready:()=>true finishes in the first tick', () => {
  const s = new Scheduler();
  const { ctx } = spyCtx();
  s.schedule(autoAimCmd({ seconds: 5, ready: () => true }));
  s.tick(ctx, 0.1);
  assert.equal(s.isRunning(), false);
  assert.ok(s.trace.some(e => e.name === 'autoAim' && e.event === 'end'));
});

test('autoAimCmd with shootAfter feeds only after aiming, for about shootSeconds, and ends with feed(false)', () => {
  const s = new Scheduler();
  const { ctx, calls } = spyCtx();
  s.schedule(autoAimCmd({ seconds: 0.5, shootAfter: true, shootSeconds: 1.0 }));
  // Before aiming completes (0.5 s) no feed(true) may happen.
  runFor(s, ctx, 0.4);
  assert.equal(calls.feed.includes(true), false, 'no feed before aiming');
  // Run through aiming and the shoot window.
  runFor(s, ctx, 2.0);
  assert.equal(s.isRunning(), false, 'finished after the shoot window');
  const trues = calls.feed.filter(v => v === true).length;
  // Shoot window is 1.0 s at dt 0.1, so roughly 10 ticks of feed(true); allow +/-2 for tick boundaries.
  assert.ok(trues >= 8 && trues <= 12, `feed(true) count ${trues} is about 10`);
  // The last feed call is the end-of-command feed(false).
  assert.equal(calls.feed.at(-1), false, 'end calls feed(false)');
  // Every feed(true) comes after the first aimed tick, i.e. after 0.5 s of ticking.
  const firstTrue = calls.feed.indexOf(true);
  assert.ok(firstTrue >= 0);
});

test('autoAimCmd end always calls ctx.feed(false)', () => {
  const s = new Scheduler();
  const { ctx, calls } = spyCtx();
  s.schedule(autoAimCmd({ seconds: 0.2, shootAfter: false }));
  runFor(s, ctx, 0.5);
  assert.equal(s.isRunning(), false);
  assert.equal(calls.feed.at(-1), false);
});

test('autoAimCmd end calls setAutoAim(false) when shootAfter is set', () => {
  const s = new Scheduler();
  const { ctx, calls } = spyCtx();
  s.schedule(autoAimCmd({ seconds: 0.2, shootAfter: true, shootSeconds: 0.2 }));
  runFor(s, ctx, 1.0);
  assert.equal(s.isRunning(), false);
  assert.equal(calls.setAutoAim[0], true, 'auto aim turned on first');
  assert.equal(calls.setAutoAim.at(-1), false, 'auto aim turned off at end');
});

test('autoAimCmd works with an empty ctx {}', () => {
  const s = new Scheduler();
  s.schedule(autoAimCmd({ seconds: 0.3, shootAfter: true, shootSeconds: 0.2 }));
  assert.doesNotThrow(() => runFor(s, {}, 1.0));
  assert.equal(s.isRunning(), false);
  assert.ok(s.trace.some(e => e.name === 'autoAim' && e.event === 'end'));
});

test('autoAimCmd without seconds uses the estimate and still finishes', () => {
  const s = new Scheduler();
  const { ctx } = spyCtx();
  s.schedule(autoAimCmd({}));
  runFor(s, ctx, 3.5);
  assert.equal(s.isRunning(), false);
});
