// t09: autoAim track/timeout contract through buildCommandTree.
// KNOWN-BUG: COMMAND_DEFS.autoAim (web/js/autos.js ~line 41) declares only `shootAfter`, so the planner
// schema has no `track` or `timeout` fields. The compile case (~line 442) forwards n.args wholesale, so
// the adapter call may keep them, but the declared contract does not. The KNOWN-BUG test is marked todo
// so the suite stays green; it should flip to a hard assert once the def declares track and timeout.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as A from '../web/js/autos.js';
import { Scheduler } from '../web/js/scheduler.js';

function autoWithAim(args) {
  const a = A.newAuto({ name: 'AA', alliance: 'blue', start: { x: 20, y: 20, h: 0 } });
  A.addNode(a, null, { type: 'command', name: 'autoAim', args });
  return a;
}

// Runs the tree for a short time and returns every argument object the adapter received for autoAim.
function compileAndRun(auto) {
  const calls = [];
  const ad = {
    autoAim: args => { calls.push(args); },
    time: () => 0, ballCount: () => 0, custom: () => {},
  };
  const s = new Scheduler();
  s.schedule(A.buildCommandTree(auto, ad));
  const ctx = { onError: e => { throw e; } };
  for (let i = 0; i < 4 && s.isRunning(); i++) s.tick(ctx, 0.05);
  return calls;
}

test('autoAim shootAfter alone is preserved through compile', () => {
  const calls = compileAndRun(autoWithAim({ shootAfter: true }));
  assert.equal(calls.length, 1);
  assert.equal(calls[0].shootAfter, true);
});

test('autoAim track and timeout survive compile into the adapter call', () => {
  const calls = compileAndRun(autoWithAim({ shootAfter: true, track: true, timeout: 2.5 }));
  assert.equal(calls.length, 1);
  assert.equal(calls[0].shootAfter, true);
  assert.equal(calls[0].track, true);
  assert.equal(calls[0].timeout, 2.5);
});

test('KNOWN-BUG: COMMAND_DEFS.autoAim declares track and timeout args', { todo: true }, () => {
  const declared = A.COMMAND_DEFS.autoAim.args;
  assert.ok('track' in declared, 'track not declared');
  assert.ok('timeout' in declared, 'timeout not declared');
});
