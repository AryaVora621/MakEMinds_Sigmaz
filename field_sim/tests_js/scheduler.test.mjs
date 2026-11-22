import test from 'node:test';
import assert from 'node:assert/strict';
import { Scheduler, waitCmd, instantCmd, runCmd, waitUntilCmd, sequential, parallel, race, deadline, repeat, conditional } from '../web/js/scheduler.js';

const mk = () => { const errs = []; return { errs, ctx: { onError: (e, c) => errs.push([e.message, c.name]) } }; };
function runFor(s, ctx, secs, dt = 0.1) { const n = Math.round(secs / dt); for (let i = 0; i < n; i++) s.tick(ctx, dt); }
/** logs end calls with interrupted flag */
function probe(name, cmd, log) {
  const e = cmd.end; cmd.name = name;
  cmd.end = (c, i) => { log.push(`${name}:${i ? 'int' : 'end'}`); e?.(c, i); };
  return cmd;
}

test('wait finishes after its duration', () => {
  const s = new Scheduler(), { ctx } = mk(); s.schedule(waitCmd(0.5));
  runFor(s, ctx, 0.4); assert.equal(s.isRunning(), true);
  runFor(s, ctx, 0.1); assert.equal(s.isRunning(), false);
});
test('instant runs action once and ends in the same tick', () => {
  let n = 0; const s = new Scheduler(), { ctx } = mk(); s.schedule(instantCmd(() => n++));
  s.tick(ctx, 0.02); assert.equal(n, 1); assert.equal(s.isRunning(), false);
});
test('runCmd runs until isFinished', () => {
  let n = 0; const s = new Scheduler(), { ctx } = mk(); s.schedule(runCmd(() => n++, () => n >= 3));
  runFor(s, ctx, 1); assert.equal(n, 3); assert.equal(s.isRunning(), false);
});
test('waitUntil ends on predicate, or on timeout', () => {
  let ok = false; const s = new Scheduler(), { ctx } = mk();
  const w = waitUntilCmd(() => ok, 1); s.schedule(w);
  runFor(s, ctx, 0.5); assert.equal(s.isRunning(), true); ok = true; s.tick(ctx, 0.1); assert.equal(s.isRunning(), false); assert.equal(w.timedOut, false);
  const s2 = new Scheduler(), w2 = waitUntilCmd(() => false, 0.3); s2.schedule(w2); runFor(s2, ctx, 0.5);
  assert.equal(s2.isRunning(), false); assert.equal(w2.timedOut, true);
});
test('sequential runs children in order and sums time', () => {
  const log = [], s = new Scheduler(), { ctx } = mk();
  s.schedule(sequential(probe('a', waitCmd(0.2), log), probe('b', waitCmd(0.3), log)));
  runFor(s, ctx, 0.4); assert.deepEqual(log, ['a:end']); assert.equal(s.isRunning(), true);
  runFor(s, ctx, 0.2); assert.deepEqual(log, ['a:end', 'b:end']); assert.equal(s.isRunning(), false);
});
test('zero-length commands: sequential runs through them in a single tick', () => {
  let n = 0; const s = new Scheduler(), { ctx } = mk();
  s.schedule(sequential(instantCmd(() => n++), waitCmd(0), instantCmd(() => n++), sequential(), instantCmd(() => n++)));
  s.tick(ctx, 0.02); assert.equal(n, 3); assert.equal(s.isRunning(), false);
});
test('empty groups finish immediately', () => {
  for (const g of [sequential(), parallel(), race()]) { const s = new Scheduler(), { ctx } = mk(); s.schedule(g); s.tick(ctx, 0.02); assert.equal(s.isRunning(), false); }
});
test('parallel ends when ALL children end', () => {
  const log = [], s = new Scheduler(), { ctx } = mk();
  s.schedule(parallel(probe('a', waitCmd(0.2), log), probe('b', waitCmd(0.5), log)));
  runFor(s, ctx, 0.3); assert.deepEqual(log, ['a:end']); assert.equal(s.isRunning(), true);
  runFor(s, ctx, 0.2); assert.deepEqual(log, ['a:end', 'b:end']); assert.equal(s.isRunning(), false);
});
test('race ends on first and interrupts the rest exactly once', () => {
  const log = [], s = new Scheduler(), { ctx } = mk();
  s.schedule(race(probe('slow', waitCmd(1), log), probe('fast', waitCmd(0.2), log), probe('mid', waitCmd(0.5), log)));
  runFor(s, ctx, 0.3);
  assert.equal(s.isRunning(), false);
  assert.deepEqual(log, ['fast:end', 'slow:int', 'mid:int']); // winner first, then losers in order
});
test('deadline ends when the deadline command ends; others interrupted', () => {
  const log = [], s = new Scheduler(), { ctx } = mk();
  s.schedule(deadline(probe('dl', waitCmd(0.3), log), probe('long', waitCmd(5), log), probe('quick', waitCmd(0.1), log)));
  runFor(s, ctx, 0.4);
  assert.equal(s.isRunning(), false);
  assert.deepEqual(log, ['quick:end', 'dl:end', 'long:int']);
});
test('deadline: others finishing early does not end the group', () => {
  const s = new Scheduler(), { ctx } = mk(); s.schedule(deadline(waitCmd(0.5), waitCmd(0.1)));
  runFor(s, ctx, 0.3); assert.equal(s.isRunning(), true); runFor(s, ctx, 0.3); assert.equal(s.isRunning(), false);
});
test('cancelAll interrupts nested commands once, in nested order', () => {
  const log = [], s = new Scheduler(), { ctx } = mk();
  s.schedule(sequential(probe('x', waitCmd(0.1), log), parallel(probe('p1', waitCmd(5), log), probe('p2', waitCmd(5), log))));
  runFor(s, ctx, 0.3); s.cancelAll(ctx); s.cancelAll(ctx);
  assert.deepEqual(log, ['x:end', 'p1:int', 'p2:int']); assert.equal(s.isRunning(), false);
});
test('nested groups: sequential(parallel(race(...)))', () => {
  const log = [], s = new Scheduler(), { ctx } = mk();
  s.schedule(sequential(
    parallel(race(probe('r1', waitCmd(0.1), log), probe('r2', waitCmd(9), log)), probe('w', waitCmd(0.3), log)),
    probe('after', waitCmd(0.1), log)));
  runFor(s, ctx, 0.6);
  assert.deepEqual(log, ['r1:end', 'r2:int', 'w:end', 'after:end']); assert.equal(s.isRunning(), false);
});
test('repeat runs n times; zero-length repeat terminates', () => {
  let n = 0; const s = new Scheduler(), { ctx } = mk();
  s.schedule(repeat(sequential(instantCmd(() => n++), waitCmd(0.1)), 3)); runFor(s, ctx, 1);
  assert.equal(n, 3); assert.equal(s.isRunning(), false);
  let m = 0; const s2 = new Scheduler(); s2.schedule(repeat(instantCmd(() => m++), 50)); s2.tick(ctx, 0.02);
  assert.equal(m, 50); assert.equal(s2.isRunning(), false);
});
test('conditional picks branch at start time', () => {
  const hit = []; let flag = false; const s = new Scheduler(), { ctx } = mk();
  s.schedule(sequential(instantCmd(() => { flag = true; }), conditional(() => flag, instantCmd(() => hit.push('a')), instantCmd(() => hit.push('b')))));
  s.tick(ctx, 0.02); assert.deepEqual(hit, ['a']);
  const s2 = new Scheduler(); flag = false; s2.schedule(conditional(() => flag, instantCmd(() => hit.push('a')))); s2.tick(ctx, 0.02);
  assert.equal(s2.isRunning(), false); assert.deepEqual(hit, ['a']);
});
test('exception in initialize/execute/isFinished is reported and the command ends, never hangs', () => {
  const log = [], { errs, ctx } = mk(), s = new Scheduler();
  s.schedule(sequential(
    probe('bad-init', { initialize() { throw new Error('i'); } }, log),
    probe('bad-exec', { execute() { throw new Error('e'); }, isFinished: () => false }, log),
    probe('bad-fin', { isFinished() { throw new Error('f'); } }, log),
    probe('ok', waitCmd(0.1), log)));
  runFor(s, ctx, 0.5);
  assert.equal(s.isRunning(), false);
  assert.deepEqual(errs.map(e => e[0]), ['i', 'e', 'f']);
  assert.deepEqual(log, ['bad-init:int', 'bad-exec:int', 'bad-fin:int', 'ok:end']); // end exactly once each
});
test('exception inside parallel child does not break siblings; exception in end() is contained', () => {
  const { errs, ctx } = mk(), s = new Scheduler(); let done = false;
  s.schedule(parallel({ name: 'boom', execute() { throw new Error('x'); } }, { name: 'endboom', isFinished: () => true, end() { throw new Error('y'); } }, sequential(waitCmd(0.2), instantCmd(() => { done = true; }))));
  runFor(s, ctx, 0.5); assert.equal(done, true); assert.equal(s.isRunning(), false); assert.deepEqual(errs.map(e => e[0]).sort(), ['x', 'y']);
});
test('works without ctx.onError', () => {
  const s = new Scheduler(); s.schedule({ initialize() { throw new Error('z'); } }); s.tick({}, 0.1); assert.equal(s.isRunning(), false);
});
test('trace records start/end/interrupt with node ids and times', () => {
  const s = new Scheduler(), { ctx } = mk();
  const a = waitCmd(0.1); a.node = 'A'; const b = waitCmd(5); b.node = 'B'; const r = race(a, b); r.node = 'R';
  s.schedule(r); runFor(s, ctx, 0.3);
  const ev = s.trace.map(e => `${e.node}:${e.event}`);
  assert.deepEqual(ev, ['R:start', 'A:start', 'B:start', 'A:end', 'B:interrupt', 'R:end']);
  assert.ok(s.trace.every(e => typeof e.t === 'number' && 'name' in e));
  assert.ok(Math.abs(s.trace.find(e => e.node === 'A' && e.event === 'end').t - 0.1) < 1e-9);
  assert.ok(s.time > 0.29 && s.time < 0.31);
});
test('scheduling from a finished command works; deterministic repeat runs give equal traces', () => {
  const run = () => { const s = new Scheduler(), { ctx } = mk(); s.schedule(sequential(waitCmd(0.2), parallel(waitCmd(0.1), waitCmd(0.3)))); runFor(s, ctx, 1); return JSON.stringify(s.trace); };
  assert.equal(run(), run());
});
