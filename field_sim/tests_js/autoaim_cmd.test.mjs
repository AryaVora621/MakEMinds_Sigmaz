import test from 'node:test';
import assert from 'node:assert/strict';
import { makeAutoAim, aimReady, AUTOAIM_DEFAULTS } from '../web/js/autoaim_cmd.js';

const DT = 0.05;
const GOOD = { rpm: 3000, turretDeg: 10, hoodDeg: 30, feasible: true };
const SETTLED = { rpm: 3000, turretDeg: 10, hoodDeg: 30 };
const INSTANT = { rpm: Infinity, turretDeg: Infinity, hoodDeg: Infinity };
const SLOW = { rpm: 100, turretDeg: 1, hoodDeg: 1 }; // max change per second

/**
 * Fake host with first-order plants. Each apply(t) moves every plant toward t by
 * rates[k] * dt (dt is the test step DT, since apply() receives no dt).
 * target: object | null | fn(callIndex) -> target | null
 */
function fakeHost({
  target = GOOD,
  start = SETTLED,
  rates = INSTANT,
  hopperEmpty = () => false,
  failShoot = false,
} = {}) {
  const plant = { ...start };
  const calls = { transfer: [], apply: [], shoot: [], targetReads: 0, fails: [], logs: [] };
  const host = {
    target() {
      const i = calls.targetReads++;
      return typeof target === 'function' ? target(i) : target;
    },
    current: () => ({ ...plant }),
    apply(t) {
      calls.apply.push(t);
      for (const k of ['rpm', 'turretDeg', 'hoodDeg']) {
        const maxStep = rates[k] * DT;
        const diff = t[k] - plant[k];
        plant[k] += Math.max(-maxStep, Math.min(maxStep, diff));
      }
    },
    transfer: on => { calls.transfer.push(on); },
    shoot: args => {
      calls.shoot.push(args);
      if (failShoot) throw new Error('jam');
    },
    hopperEmpty: () => hopperEmpty(),
  };
  const opts = { host, onFail: r => calls.fails.push(r), onLog: m => calls.logs.push(m) };
  return { host, plant, calls, opts };
}

/** Advances by a whole number of DT steps so timing boundaries are exact. */
function runFor(cmd, secs) {
  const n = Math.round(secs / DT);
  for (let i = 0; i < n; i++) cmd.update(DT);
}

test('aimReady: rpm, turret and hood tolerance boundaries are strict', () => {
  const t = { rpm: 3000, turretDeg: 0, hoodDeg: 0, feasible: true };
  // rpm: relative tolerance 0.04 (strict)
  assert.equal(aimReady({ rpm: 3000 * 1.0399, turretDeg: 0, hoodDeg: 0 }, t), true);
  assert.equal(aimReady({ rpm: 3000 * 1.041, turretDeg: 0, hoodDeg: 0 }, t), false);
  assert.equal(aimReady({ rpm: 3000 * 0.959, turretDeg: 0, hoodDeg: 0 }, t), false);
  // turret: 2 deg (strict)
  assert.equal(aimReady({ rpm: 3000, turretDeg: 1.99, hoodDeg: 0 }, t), true);
  assert.equal(aimReady({ rpm: 3000, turretDeg: 2, hoodDeg: 0 }, t), false);
  assert.equal(aimReady({ rpm: 3000, turretDeg: -2, hoodDeg: 0 }, t), false);
  // hood: 1.5 deg (strict)
  assert.equal(aimReady({ rpm: 3000, turretDeg: 0, hoodDeg: 1.49 }, t), true);
  assert.equal(aimReady({ rpm: 3000, turretDeg: 0, hoodDeg: 1.5 }, t), false);
  // infeasible, low rpm, missing inputs
  assert.equal(aimReady({ rpm: 3000, turretDeg: 0, hoodDeg: 0 }, { ...t, feasible: false }), false);
  assert.equal(aimReady({ rpm: 100, turretDeg: 0, hoodDeg: 0 }, { rpm: 100, turretDeg: 0, hoodDeg: 0, feasible: true }), false);
  assert.equal(aimReady({ rpm: 3000, turretDeg: 0, hoodDeg: 0 }, null), false);
  assert.equal(aimReady(null, t), false);
});

test('defaults mirror the sim thresholds', () => {
  assert.equal(AUTOAIM_DEFAULTS.rpmTolFrac, 0.04);
  assert.equal(AUTOAIM_DEFAULTS.turretTolDeg, 2);
  assert.equal(AUTOAIM_DEFAULTS.minRpm, 300);
  assert.equal(AUTOAIM_DEFAULTS.timeoutSec, 4);
});

test('start() turns transfer on when transferOnStart is set; ready after settle; stop() turns it off', () => {
  const f = fakeHost();
  const cmd = makeAutoAim({ ...f.opts, transferOnStart: true });
  cmd.start();
  assert.deepEqual(f.calls.transfer, [true]);
  runFor(cmd, 0.1);
  assert.equal(cmd.done(), true);
  assert.equal(cmd.status().result, 'ready');
  assert.equal(f.calls.shoot.length, 0);
  cmd.stop();
  assert.deepEqual(f.calls.transfer, [true, false]);
  cmd.stop();
  assert.deepEqual(f.calls.transfer, [true, false]);
});

test('done only after settleSec of continuous readiness; a flicker resets the timer', () => {
  // Update index 1 returns no solution, so readiness drops for one frame.
  const f = fakeHost({ target: i => (i === 1 ? null : GOOD) });
  const cmd = makeAutoAim(f.opts);
  cmd.start();
  runFor(cmd, 0.15); // updates 0..2: ready, flicker, ready (0.05 s held)
  assert.equal(cmd.done(), false);
  assert.ok(Math.abs(cmd.status().readySec - 0.05) < 1e-9);
  runFor(cmd, 0.05); // update 3: 0.1 s held
  assert.equal(cmd.done(), true);
  assert.equal(cmd.status().result, 'ready');
});

test('times out at the 4 s default when plants are too slow; no throw, onFail once, no shot', () => {
  const f = fakeHost({ start: { rpm: 0, turretDeg: 0, hoodDeg: 0 }, rates: SLOW });
  const cmd = makeAutoAim({ ...f.opts, shootAfter: true });
  cmd.start();
  assert.doesNotThrow(() => runFor(cmd, 3.5));
  assert.equal(cmd.done(), false);
  assert.doesNotThrow(() => runFor(cmd, 1.0));
  assert.equal(cmd.done(), true);
  assert.equal(cmd.status().result, 'timeout');
  assert.equal(f.calls.fails.length, 1);
  assert.match(f.calls.fails[0], /^timeout after 4s \(not on target\)$/);
  assert.equal(f.calls.shoot.length, 0);
  assert.equal(cmd.status().fired, false);
});

test('null or infeasible target never becomes ready and times out', () => {
  const nullCase = fakeHost({ target: null });
  const nullCmd = makeAutoAim(nullCase.opts);
  nullCmd.start();
  runFor(nullCmd, 4.5);
  assert.equal(nullCmd.status().result, 'timeout');
  assert.equal(nullCmd.status().ready, false);
  assert.equal(nullCmd.status().readySec, 0);
  assert.match(nullCase.calls.fails[0], /no solution/);
  assert.equal(nullCase.calls.apply.length, 0); // nothing to drive without a target

  const infeasible = fakeHost({ target: { ...GOOD, feasible: false } });
  const infCmd = makeAutoAim(infeasible.opts);
  infCmd.start();
  runFor(infCmd, 4.5);
  assert.equal(infCmd.status().result, 'timeout');
  assert.match(infeasible.calls.fails[0], /infeasible shot/);
});

test('tracking: apply receives the newly read target on every update', () => {
  const targets = [];
  const f = fakeHost({
    target: i => {
      const t = { rpm: 3000, turretDeg: 10 + i * 0.5, hoodDeg: 30, feasible: true };
      targets.push(t);
      return t;
    },
  });
  const cmd = makeAutoAim({ ...f.opts, track: true });
  cmd.start();
  runFor(cmd, 0.5); // 10 updates; target moves 0.5 deg per update, stays within tolerance
  assert.equal(cmd.done(), true);
  assert.equal(cmd.status().tracking, true);
  assert.equal(f.calls.apply.length, 10);
  assert.equal(targets.length, 10);
  f.calls.apply.forEach((applied, k) => {
    assert.deepEqual(applied, targets[k]);
    assert.equal(applied.turretDeg, 10 + k * 0.5);
  });
  assert.equal(f.calls.shoot.length, 0);
});

test('shootAfter fires exactly once on ready and finishes when hopperEmpty returns true', () => {
  let empty = false;
  const f = fakeHost({ hopperEmpty: () => empty });
  const cmd = makeAutoAim({ ...f.opts, shootAfter: true });
  cmd.start();
  runFor(cmd, 0.1); // ready on update 1, fires there
  assert.deepEqual(f.calls.shoot, [{ all: true }]);
  assert.equal(cmd.status().phase, 'firing');
  assert.equal(cmd.done(), false);
  runFor(cmd, 0.5);
  assert.equal(cmd.done(), false);
  empty = true;
  runFor(cmd, 0.05);
  assert.equal(cmd.done(), true);
  assert.equal(cmd.status().phase, 'finished');
  assert.equal(cmd.status().fired, true);
  assert.equal(cmd.status().result, 'ready');
  runFor(cmd, 1);
  assert.equal(f.calls.shoot.length, 1);
});

test('shootAfter completes after 6 s of firing when hopperEmpty never reports true', () => {
  const f = fakeHost({ hopperEmpty: () => false });
  const cmd = makeAutoAim({ ...f.opts, shootAfter: true });
  cmd.start();
  runFor(cmd, 0.1);
  assert.equal(f.calls.shoot.length, 1);
  runFor(cmd, 5.5);
  assert.equal(cmd.done(), false);
  runFor(cmd, 1.0);
  assert.equal(cmd.done(), true);
  assert.equal(cmd.status().reason, 'fire timeout');
  assert.equal(f.calls.shoot.length, 1);
});

test('shootAfter does not fire on timeout unless fireOnTimeout is set', () => {
  const f = fakeHost({ target: { ...GOOD, turretDeg: 50 }, start: SETTLED, rates: SLOW });
  const cmd = makeAutoAim({ ...f.opts, shootAfter: true });
  cmd.start();
  runFor(cmd, 4.5);
  assert.equal(f.calls.shoot.length, 0);
  assert.equal(cmd.status().result, 'timeout');
  assert.equal(cmd.status().firedOnTimeout, false);

  const g = fakeHost({ target: { ...GOOD, turretDeg: 50 }, rates: SLOW, hopperEmpty: () => true });
  const cmd2 = makeAutoAim({ ...g.opts, shootAfter: true, fireOnTimeout: true });
  cmd2.start();
  runFor(cmd2, 4.5);
  assert.equal(g.calls.shoot.length, 1);
  assert.equal(cmd2.status().firedOnTimeout, true);
});

test('stop() before ready turns transfer off and reports stopped; keepTransfer keeps it on', () => {
  const f = fakeHost({ target: null });
  const cmd = makeAutoAim({ ...f.opts, transferOnStart: true });
  cmd.start();
  runFor(cmd, 0.5);
  const applied = f.calls.apply.length;
  cmd.stop();
  assert.equal(cmd.done(), true);
  assert.equal(cmd.status().result, 'stopped');
  assert.deepEqual(f.calls.transfer, [true, false]);
  runFor(cmd, 1); // updates after stop are ignored
  assert.equal(f.calls.apply.length, applied);

  const g = fakeHost();
  const cmd2 = makeAutoAim({ ...g.opts, keepTransfer: true, transferOnStart: true });
  cmd2.start();
  cmd2.stop();
  assert.deepEqual(g.calls.transfer, [true]);
});

test('custom timeoutSec is respected', () => {
  const slowOneSec = fakeHost({ start: { rpm: 0, turretDeg: 0, hoodDeg: 0 }, rates: SLOW });
  const cmd1 = makeAutoAim({ ...slowOneSec.opts, timeoutSec: 1 });
  cmd1.start();
  runFor(cmd1, 0.95);
  assert.equal(cmd1.done(), false);
  runFor(cmd1, 0.05);
  assert.equal(cmd1.done(), true);
  assert.equal(cmd1.status().result, 'timeout');
  assert.match(slowOneSec.calls.fails[0], /^timeout after 1s/);

  const slowTwoHalf = fakeHost({ start: { rpm: 0, turretDeg: 0, hoodDeg: 0 }, rates: SLOW });
  const cmd2 = makeAutoAim({ ...slowTwoHalf.opts, timeoutSec: 2.5 });
  cmd2.start();
  runFor(cmd2, 2.45);
  assert.equal(cmd2.done(), false);
  runFor(cmd2, 0.05);
  assert.equal(cmd2.done(), true);
  assert.match(slowTwoHalf.calls.fails[0], /^timeout after 2\.5s/);
});

test('host exceptions are contained and reported as logs', () => {
  const cmd = makeAutoAim({
    host: {
      target() { throw new Error('x'); },
      current() { throw new Error('y'); },
      apply() {},
      transfer() { throw new Error('t'); },
      shoot() {},
    },
    timeoutSec: 0.2,
  });
  assert.doesNotThrow(() => { cmd.start(); cmd.update(0.1); cmd.update(0.1); cmd.stop(); });
  assert.equal(cmd.done(), true);
});

test('a throwing shoot() is reported through onFail and finishes the command', () => {
  const f = fakeHost({ hopperEmpty: () => true, failShoot: true });
  const cmd = makeAutoAim({ ...f.opts, shootAfter: true });
  cmd.start();
  assert.doesNotThrow(() => runFor(cmd, 0.3));
  assert.equal(cmd.done(), true);
  assert.equal(f.calls.fails.length, 1);
  assert.match(f.calls.fails[0], /shoot failed/);
});
