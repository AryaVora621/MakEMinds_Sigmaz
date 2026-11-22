import test from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, checkJavaStructure } from '../web/js/autoexport.js';

// t12: sequential order of exported follow + commands (Pedro 2.1.0 compatibility).
const P = (x, y) => ({ x, y });
const cmd = (name, args) => ({ type: 'command', name, args });
const auto = (root) => ({
  name: 'Seq Order Auto', alliance: 'blue', start: { x: 24, y: 120, h: Math.PI / 2 },
  paths: {
    a: { segments: [{ type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'constant', start: 0 } }], markers: [] },
    b: { segments: [{ type: 'line', points: [P(48, 96), P(40, 40)], heading: { mode: 'constant', start: 0 } }], markers: [] },
  },
  root,
});
const ROOT = { type: 'sequential', children: [
  { type: 'follow', path: 'a' }, cmd('shoot', { seconds: 1.5 }), { type: 'follow', path: 'b' }, cmd('intakeOff'),
] };

test('sequential follow A, shoot, follow B are emitted in order', () => {
  const j = exportAutoJava(auto(ROOT), { className: 'SeqOrder' });
  const iA = j.indexOf('robot.auto.followPath(path_a');
  const iB = j.indexOf('robot.auto.followPath(path_b');
  // shoot handling: the exporter labels the shoot state "// state N: shoot" and calls robot.feedIfReady there
  const iShoot = j.search(/\/\/ state \d+: shoot/);
  assert.ok(iA >= 0, 'follow A emitted');
  assert.ok(iB >= 0, 'follow B emitted');
  assert.ok(iShoot >= 0, 'shoot handling emitted');
  assert.ok(iA < iShoot, `follow A (${iA}) precedes shoot (${iShoot})`);
  assert.ok(iShoot < iB, `shoot (${iShoot}) precedes follow B (${iB})`);
});

test('sequential root passes the structure checker with no problems', () => {
  const j = exportAutoJava(auto(ROOT), { className: 'SeqOrder' });
  const r = checkJavaStructure(j);
  assert.equal(typeof r.ok, 'boolean');
  assert.ok(Array.isArray(r.problems));
  assert.deepEqual(r.problems, []);
  assert.equal(r.ok, true);
});

test('no chain callback APIs appear (Pedro 2.1.0 chain callbacks never fire)', () => {
  const j = exportAutoJava(auto(ROOT), { className: 'SeqOrder' });
  assert.doesNotMatch(j, /addParametricCallback/);
  assert.doesNotMatch(j, /addCallback/);
});
