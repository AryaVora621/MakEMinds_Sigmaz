// t13: parallel group runs a command while a path follows.
// Pre-order numbering: root sequential n0 -> parallel n1 -> {follow P1 n2, intakeOn n3, aimOn n4}; then shoot n5.
// run1() is called each tick and calls run2/run3/run4, so the follow start and intakeOn share the parallel construct.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { exportAutoJava } from '../web/js/autoexport.js';

const P = (x, y) => ({ x, y });
const cmd = (name) => ({ type: 'command', name, args: {} });
const auto = {
  name: 'T13', alliance: 'blue', start: { x: 24, y: 120, h: Math.PI / 2 },
  paths: { P1: { segments: [{ type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'constant', start: 0 } }], markers: [] } },
  root: { type: 'sequential', children: [
    { type: 'parallel', children: [{ type: 'follow', path: 'P1' }, cmd('intakeOn'), cmd('aimOn')] },
    cmd('shoot'),
  ] },
};
const j = exportAutoJava(auto, { className: 'T13Auto' });

// Extract the body text of one emitted state method, from its banner comment to the next banner.
function stateBody(src, n) {
  const start = src.indexOf(`// state ${n}:`);
  assert.ok(start >= 0, `state ${n} emitted`);
  const next = src.indexOf('    // state ', start + 5);
  return src.slice(start, next < 0 ? undefined : next);
}

test('parallel state holds the follow start and intakeOn in the same run method', () => {
  const par = stateBody(j, 1);
  assert.match(par, /parallel \(3 branches/);
  assert.match(par, /run2\(\)/);
  assert.match(par, /run3\(\)/);
  assert.match(par, /run4\(\)/);
  // follow start is in state 2, intakeOn statement in state 3; both are reached from run1
  assert.match(stateBody(j, 2), /robot\.auto\.followPath\(path_P1, true, FOLLOW_TIMEOUT_S\)/);
  assert.match(stateBody(j, 3), /robot\.intake\.forward\(\);/);
});

test('follower waits on isBusy before the parallel ends, and shoot is gated by that', () => {
  const follow = stateBody(j, 2);
  assert.match(follow, /if \(robot\.auto\.isBusyOrTimeout\(\)\) \{\n\s+return false;/);
  // root sequential: case 0 (parallel) must return false before case 1 (shoot) runs
  const root = stateBody(j, 0);
  const c0 = root.indexOf('case 0:'), c1 = root.indexOf('case 1:');
  assert.ok(c0 >= 0 && c1 > c0, 'root has case 0 then case 1');
  const gate = root.slice(c0, c1);
  assert.match(gate, /if \(!run1\(\)\) \{\n\s+return false;/);
  assert.match(root.slice(c1), /run5\(\)/);
  assert.match(stateBody(j, 5), /robot\.feedIfReady\(true\)|shoot/);
});

test('loop ticks the robot each pass (follower.update lives in AutoDrivebase)', () => {
  assert.match(j, /while \(opModeIsActive\(\)\) \{[\s\S]*robot\.update\(\);/);
  const drive = readFileSync(new URL('../../TeamCode/src/main/java/commands/autonomous/AutoDrivebase.java', import.meta.url), 'utf8');
  assert.match(drive, /follower\.update\(\);/);
});

test('parallel completion condition uses an all-flags boolean, not a single branch', () => {
  assert.match(stateBody(j, 1), /boolean all = true;/);
  assert.match(j, /private final boolean\[\] n1_flags = new boolean\[3\]/);
});
