import test from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, checkJavaStructure, exportWarnings, locateMarker, segmentWeights } from '../web/js/autoexport.js';

const P = (x, y) => ({ x, y });
const seg = (pts, type = 'line') => ({ type, points: pts, heading: { mode: 'tangent' } });
const base = (root, paths) => ({ name: 'Cmp', alliance: 'blue', start: { x: 24, y: 120, h: 0 }, paths: paths || { a: { segments: [seg([P(0, 0), P(10, 0)])] }, b: { segments: [seg([P(10, 0), P(20, 0)])] } }, root });

test('segmentWeights: line with >2 points counts endpoints only (matches emitted BezierLine)', () => {
  assert.deepEqual(segmentWeights([seg([P(0, 0), P(30, 40), P(60, 0)])]), [60]);
  assert.ok(segmentWeights([seg([P(0, 0), P(30, 40), P(60, 0)], 'curve')])[0] > 99);
});
test('locateMarker: end of path maps to end of last segment, also with zero weights', () => {
  assert.deepEqual(locateMarker(1, [0, 0, 0]), { seg: 2, t: 1 });
  assert.deepEqual(locateMarker(1, [0, 0]), { seg: 1, t: 1 });
  assert.equal(locateMarker(0.5, []), null);
  assert.deepEqual(locateMarker(1, [100, 50]), { seg: 1, t: 1 });
});
test('marker on a zero-segment path does not fire on the first poll', () => {
  const a = base({ type: 'follow', path: 'e' }, { e: { segments: [], markers: [{ t: 0.5, command: { type: 'command', name: 'intakeOn' } }] } });
  const j = exportAutoJava(a);
  assert.ok(!/chainIdx > -1/.test(j));
});
test('exportWarnings: concurrent follows, no-timeout waitUntil, unknown condition, bad deadline', () => {
  const w = exportWarnings(base({ type: 'race', children: [{ type: 'follow', path: 'a' }, { type: 'follow', path: 'b' }, { type: 'waitUntil', condition: 'mystery' }] }));
  const codes = w.map(x => x.code);
  for (const c of ['concurrent-follow', 'no-timeout', 'unknown-condition']) assert.ok(codes.includes(c), c);
  assert.ok(exportWarnings(base({ type: 'deadline', deadline: 9, children: [{ type: 'follow', path: 'a' }] })).some(x => x.code === 'bad-deadline'));
});
test('exportWarnings: one follow plus commands in parallel is clean; sequential follows are clean', () => {
  assert.equal(exportWarnings(base({ type: 'parallel', children: [{ type: 'follow', path: 'a' }, { type: 'command', name: 'intakeOn' }, { type: 'wait', seconds: 1 }] })).length, 0);
  assert.equal(exportWarnings(base({ type: 'sequential', children: [{ type: 'follow', path: 'a' }, { type: 'follow', path: 'b' }] })).length, 0);
});
test('warnings land in the Java header and the output still passes the structure check', () => {
  const j = exportAutoJava(base({ type: 'parallel', children: [{ type: 'follow', path: 'a' }, { type: 'follow', path: 'b' }] }));
  assert.match(j, /EXPORT WARNINGS[\s\S]*\[concurrent-follow\]/);
  assert.equal(checkJavaStructure(j).ok, true);
});
test('custom command names that sanitize to the same method emit one stub only', () => {
  const j = exportAutoJava(base({ type: 'sequential', children: [{ type: 'command', name: 'a-b' }, { type: 'command', name: 'a_b' }] }));
  assert.equal((j.match(/private void custom_a_b\(\)/g) || []).length, 1);
});
