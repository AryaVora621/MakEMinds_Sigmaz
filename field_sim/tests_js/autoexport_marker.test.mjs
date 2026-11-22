import { test } from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, checkJavaStructure } from '../web/js/autoexport.js';

const P = (x, y) => ({ x, y });
const fixture = (markerCmd) => ({
  name: 'Marker Test', alliance: 'blue', start: { x: 24, y: 120, h: 0 },
  paths: { p1: { segments: [{ type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'tangent' } }],
    markers: [{ t: 0.5, command: markerCmd }] } },
  root: { type: 'sequential', children: [{ type: 'follow', path: 'p1' }] },
});
const gen = (markerCmd) => exportAutoJava(fixture(markerCmd), { className: 'MarkerAuto' });
const ok = j => { const r = checkJavaStructure(j); assert.deepEqual(r.problems, []); assert.equal(r.ok, true); };

test('autoAim path marker emits startAutoAim and a per-tick update call', () => {
  const j = gen({ type: 'command', name: 'autoAim', args: { shootAfter: true } });
  ok(j);
  assert.match(j, /startAutoAim\(true\);/);
  assert.match(j, /autoAimCmd\.update\(\)/);
  assert.match(j, /private void startAutoAim\(boolean shootAfter\)/);
  assert.match(j, /import commands\.autonomous\.AutoAimLogic;/);
});

test('autoAim marker without shootAfter passes false', () => {
  const j = gen({ type: 'command', name: 'autoAim' });
  ok(j);
  assert.match(j, /startAutoAim\(false\);/);
});

test('planner-shape autoAim marker is normalised the same way', () => {
  const j = gen({ type: 'autoAim', shootAfter: true });
  ok(j);
  assert.match(j, /startAutoAim\(true\);/);
  assert.match(j, /autoAimCmd\.update\(\)/);
});

test('no autoAim helper or loop line when no autoAim marker is used', () => {
  const j = gen({ type: 'command', name: 'intakeOn' });
  ok(j);
  assert.doesNotMatch(j, /startAutoAim/);
  assert.doesNotMatch(j, /autoAimCmd/);
});
