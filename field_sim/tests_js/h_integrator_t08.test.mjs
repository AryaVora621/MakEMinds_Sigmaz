import test from 'node:test';
import assert from 'node:assert/strict';
import { Scheduler, waitCmd, instantCmd, runCmd, sequential, parallel, race, deadline, repeat, conditional } from '../web/js/scheduler.js';

const ctx = {}, dt = 0.1, near = (a, b) => assert.ok(Math.abs(a - b) < 1e-6, `${a} != ${b}`);
const runUntilIdle = s => { let n = 0; while (s.isRunning() && n < 1000) { s.tick(ctx, dt); n++; } };
/** Records 'name:end' or 'name:int' on every end call. */
function probe(name, cmd, log) {
  const e = cmd.end; cmd.name = name;
  cmd.end = (c, i) => { log.push(`${name}:${i ? 'int' : 'end'}`); e?.(c, i); };
  return cmd;
}
test('sequential(parallel(wait1, wait2), instant): instant runs at 2s', () => {
  const s = new Scheduler(); let at = -1;
  s.schedule(sequential(parallel(waitCmd(1), waitCmd(2)), instantCmd(() => { at = s.time; })));
  for (let i = 0; i < 19; i++) s.tick(ctx, dt);
  assert.equal(at, -1);
  s.tick(ctx, dt); near(at, 2.0);
  assert.equal(s.isRunning(), false);
});
test('race ends at shorter child and interrupts the longer one', () => {
  const s = new Scheduler(), log = [];
  s.schedule(race(probe('short', waitCmd(0.5), log), probe('long', waitCmd(2), log)));
  runUntilIdle(s); near(s.time, 0.5);
  assert.deepEqual(log, ['short:end', 'long:int']);
});
test('deadline ends when the deadline command ends; others interrupted', () => {
  const s = new Scheduler(), log = [];
  s.schedule(deadline(probe('dl', waitCmd(1), log), probe('forever', runCmd(() => {}, () => false), log)));
  runUntilIdle(s); near(s.time, 1.0);
  assert.deepEqual(log, ['dl:end', 'forever:int']);
});
test('3-level nesting: instant runs at 0.5s after parallel(seq(0.3,0.2), 0.4)', () => {
  const s = new Scheduler(); let at = -1;
  s.schedule(sequential(parallel(sequential(waitCmd(0.3), waitCmd(0.2)), waitCmd(0.4)), instantCmd(() => { at = s.time; })));
  runUntilIdle(s); near(at, 0.5);
});
test('empty groups finish in the first tick and do not hang', () => {
  for (const make of [() => sequential(), () => parallel(), () => race(), () => deadline(waitCmd(0))]) {
    const s = new Scheduler(); s.schedule(make()); s.tick(ctx, dt);
    assert.equal(s.isRunning(), false);
  }
});
test('repeat(instant, 3) runs three times; repeat(wait 0.2, 3) takes 0.6s', () => {
  let n = 0; const s1 = new Scheduler();
  s1.schedule(repeat(instantCmd(() => n++), 3)); s1.tick(ctx, dt);
  assert.equal(n, 3); assert.equal(s1.isRunning(), false);
  const s2 = new Scheduler(); s2.schedule(repeat(waitCmd(0.2), 3));
  runUntilIdle(s2); near(s2.time, 0.6);
});
test('conditional picks the true branch, and the false branch otherwise', () => {
  const ran = [];
  const mk = p => conditional(() => p, instantCmd(() => ran.push('a')), instantCmd(() => ran.push('b')));
  const s1 = new Scheduler(); s1.schedule(mk(true)); s1.tick(ctx, dt);
  const s2 = new Scheduler(); s2.schedule(mk(false)); s2.tick(ctx, dt);
  assert.deepEqual(ran, ['a', 'b']);
});
/* REPORT (t08): ran `node --test tests_js/h_integrator_t08.test.mjs` (Node v24.14.0): 7 pass, 0 fail.
 * Read-only use of web/js/scheduler.js; no bug found, no FAILING-BY-DESIGN finding. Not covered: onError paths. */
