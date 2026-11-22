import { test } from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, checkJavaStructure, KNOWN_COMMANDS } from '../web/js/autoexport.js';

const P = (x, y) => ({ x, y });
const base = (root) => ({
  name: 'AA Test', alliance: 'blue', start: { x: 24, y: 120, h: 0 },
  paths: { p1: { segments: [{ type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'tangent' } }], markers: [] } },
  root,
});
const gen = (root) => exportAutoJava(base(root), { className: 'TestAuto' });
const seq = (children) => ({ id: 's', type: 'sequential', children });
const follow = { id: 'f', type: 'follow', path: 'p1' };
const aaShoot = { id: 'a', type: 'command', name: 'autoAim', args: { shootAfter: true } };
const aaPlain = { id: 'b', type: 'autoAim', shootAfter: false, seconds: 2.5 };
const shoot = { id: 'sh', type: 'command', name: 'shoot' };

test('command-shape autoAim with shootAfter emits AutoAimLogic state, import and feed', () => {
  const j = gen(seq([follow, aaShoot]));
  assert.ok(checkJavaStructure(j).ok, checkJavaStructure(j).problems.join('; '));
  assert.match(j, /import commands\.autonomous\.AutoAimLogic;/);
  assert.match(j, /import robot.RobotSettings;/);
  assert.match(j, /new AutoAimLogic\(RobotSettings.AutoAim.TIMEOUT_S, true, RobotSettings.AutoAim.TRANSFER_ON_START\)/);
  assert.match(j, /feedIfReady\(true\)/);
  assert.match(j, /RobotSettings.AutoAim.SHOOT_AFTER_MAX_S/);
});

test('planner-shape autoAim (node.type) without shootAfter never feeds and uses node.seconds', () => {
  const j = gen(seq([follow, aaPlain]));
  assert.ok(checkJavaStructure(j).ok, checkJavaStructure(j).problems.join('; '));
  assert.match(j, /new AutoAimLogic\(2\.5, false, RobotSettings.AutoAim.TRANSFER_ON_START\)/);
  assert.doesNotMatch(j, /feedIfReady/);
  assert.doesNotMatch(j, /SHOOT_AFTER_MAX_S/);
});

test('autoAim state is timed, not an instant call, and has a cancel method', () => {
  const j = gen(seq([follow, aaShoot]));
  assert.match(j, /private boolean run\d+\(\) \{/);
  assert.match(j, /private void cancel\d+\(\) \{/);
  assert.doesNotMatch(j, /custom_autoAim/);
});

test('no AutoAim import when the tree has no autoAim node', () => {
  const j = gen(seq([follow, shoot]));
  assert.doesNotMatch(j, /AutoAim/);
  assert.match(j, /feedIfReady\(true\)/);
});

test('autoAim inside a parallel branch stays non-blocking (no AutoAim.run call)', () => {
  const j = gen({ id: 'par', type: 'parallel', children: [follow, aaShoot] });
  assert.ok(checkJavaStructure(j).ok, checkJavaStructure(j).problems.join('; '));
  assert.doesNotMatch(j, /AutoAim\.run\(/);
});

// ---- command-shape and marker-shape checks (added to cover the autoAim command contract) ----
const markerAuto = (markerCmd) => ({
  name: 'AA Marker', alliance: 'blue', start: { x: 24, y: 120, h: 0 },
  paths: { p1: { segments: [{ type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'tangent' } }],
    markers: [{ t: 0.5, command: markerCmd }] } },
  root: seq([follow]),
});

test('command autoAim without shootAfter: AutoAimLogic state, no feedIfReady', () => {
  const j = gen(seq([follow, { type: 'command', name: 'autoAim', args: { shootAfter: false } }]));
  assert.ok(checkJavaStructure(j).ok, checkJavaStructure(j).problems.join('; '));
  assert.match(j, /new AutoAimLogic\(RobotSettings.AutoAim.TIMEOUT_S, false, RobotSettings.AutoAim.TRANSFER_ON_START\)/);
  assert.doesNotMatch(j, /feedIfReady/);
});

test('command autoAim with shootAfter:true emits feedIfReady(true)', () => {
  const j = gen(seq([follow, { type: 'command', name: 'autoAim', args: { shootAfter: true } }]));
  assert.ok(checkJavaStructure(j).ok, checkJavaStructure(j).problems.join('; '));
  assert.match(j, /feedIfReady\(true\)/);
});

test('planner node shape {type:"autoAim", shootAfter:true} matches the command shape', () => {
  const cmdJ = gen(seq([follow, { type: 'command', name: 'autoAim', args: { shootAfter: true } }]));
  const nodeJ = gen(seq([follow, { type: 'autoAim', shootAfter: true }]));
  assert.equal(nodeJ, cmdJ);
});

test('path marker autoAim emits startAutoAim(', () => {
  const j = exportAutoJava(markerAuto({ type: 'command', name: 'autoAim', args: { shootAfter: true } }), { className: 'TestAuto' });
  assert.ok(checkJavaStructure(j).ok, checkJavaStructure(j).problems.join('; '));
  assert.match(j, /startAutoAim\(/);
});

test('an auto with no autoAim does not import or mention AutoAimCommand', () => {
  const j = gen(seq([follow, shoot]));
  assert.doesNotMatch(j, /import commands\.autonomous\.AutoAim/);
  assert.doesNotMatch(j, /AutoAimCommand/);
});

test('autoAim is listed in KNOWN_COMMANDS', () => {
  assert.ok(KNOWN_COMMANDS.includes('autoAim'));
});
