// t14: race and deadline semantics in the exported statemachine Java (web/js/autoexport.js emitStateMachine).
// Read-only evidence for the cancel path: TeamCode/src/main/java/commands/autonomous/AutoDrivebase.java
// cancelMove() -> holdPoint() -> follower.holdPoint(pose) (lines 116-121, 188-190), which replaces path following.
import test from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, checkJavaStructure } from '../web/js/autoexport.js';

const P = (x, y) => ({ x, y });
const auto = root => ({
  name: 'T14', alliance: 'blue', start: { x: 24, y: 120, h: Math.PI / 2 },
  paths: { P1: { segments: [{ type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'constant', start: 0 } }], markers: [] } },
  root,
});
const gen = root => exportAutoJava(auto(root), { className: 'T14' });
// Body of the state method runN(), up to the matching cancelN() declaration.
const runBody = (j, n) => {
  const m = new RegExp(`private boolean run${n}\\(\\) \\{([\\s\\S]*?)\\n    \\}\\n\\n    private void cancel${n}`).exec(j);
  return m ? m[1] : '';
};
const cancelBody = (j, n) => {
  const m = new RegExp(`private void cancel${n}\\(\\) \\{([\\s\\S]*?)\\n    \\}`).exec(j);
  return m ? m[1] : '';
};

test('(a) race[follow P1, wait 0.5]: first finisher ends group, loser cancelled', () => {
  const j = gen({ type: 'race', children: [{ type: 'follow', path: 'P1' }, { type: 'wait', seconds: 0.5 }] });
  assert.equal(checkJavaStructure(j).ok, true);
  // Group state is node 0; follow is node 1, wait is node 2.
  const run0 = runBody(j, 0);
  assert.match(run0, /if \(winner < 0 && run1\(\)\) \{\n\s+winner = 0;/);
  assert.match(run0, /if \(winner < 0 && run2\(\)\) \{\n\s+winner = 1;/);
  assert.match(run0, /if \(winner >= 0\) \{[\s\S]*return true;/);
  // Loser cancel: when wait wins, the follow (child 0) is cancelled through cancel1 -> cancelMove.
  assert.match(run0, /if \(winner != 0\) \{\n\s+cancel1\(\);/);
  assert.match(cancelBody(j, 1), /robot\.auto\.cancelMove\(\);/);
  // Group-level cancel also cancels both children (idempotent: each checks started/done).
  assert.match(cancelBody(j, 0), /cancel1\(\);\n\s+cancel2\(\);/);
  // Known semantics: the group does not return true until a child returns true, so a run that
  // is never won keeps going; the 0.5 s wait always wins before any path needs longer. Not a bug.
});

test('(b) deadline[follow P1 (idx 0), intakeOn, wait 5]: follow end ends group and cancels others', () => {
  const j = gen({ type: 'deadline', deadline: 0, children: [
    { type: 'follow', path: 'P1' }, { type: 'command', name: 'intakeOn', args: {} }, { type: 'wait', seconds: 5 },
  ] });
  assert.equal(checkJavaStructure(j).ok, true);
  const run0 = runBody(j, 0);
  // Branch 0 (follow = node 1) decides: when it returns true the group sets its flag and returns true.
  assert.match(run0, /if \(run1\(\)\) \{\n\s+n0_flags\[0\] = true;/);
  // Others (intakeOn = node 2, wait = node 3) are cancelled only if not already finished.
  assert.match(run0, /if \(!n0_flags\[1\]\) \{\n\s+cancel2\(\);/);
  assert.match(run0, /if \(!n0_flags\[2\]\) \{\n\s+cancel3\(\);/);
  assert.match(run0, /cancel3\(\);[\s\S]*?return true;/);
  // Follow branch cancels its path only if it has not finished (done flag guards cancelMove).
  assert.match(cancelBody(j, 1), /if \(n1_started && !n1_done\) \{\n\s+robot\.auto\.cancelMove\(\);/);
  // Non-deadline branches are only run while the deadline has not ended.
  assert.match(run0, /if \(!n0_flags\[1\] && run2\(\)\)/);
  assert.match(run0, /if \(!n0_flags\[2\] && run3\(\)\)/);
  // NOTE: intakeOn is an instant command with an empty cancel2, so the intake stays on after a
  // deadline ends. Same as sequential intakeOn with no intakeOff; flagged as a semantic note, not asserted as a bug.
});

test('(c) invalid deadline index falls back to branch 0 without throwing', () => {
  for (const bad of [9, -1, 1.5, 'x', null]) {
    let j;
    assert.doesNotThrow(() => { j = gen({ type: 'deadline', deadline: bad, children: [{ type: 'follow', path: 'P1' }, { type: 'wait', seconds: 1 }] }); }, `deadline=${bad}`);
    assert.equal(checkJavaStructure(j).ok, true, `deadline=${bad}`);
    assert.match(j, /deadline \(branch 0 decides/, `deadline=${bad}`);
    assert.match(j, /if \(run1\(\)\) \{\n\s+n0_flags\[0\] = true;/, `deadline=${bad}`);
  }
});
