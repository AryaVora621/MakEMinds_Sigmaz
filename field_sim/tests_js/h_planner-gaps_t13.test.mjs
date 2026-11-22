// t13: pointer-event path editing. Read-only findings (no source edited):
// pathui.js has no DOM listeners (ed.pointer down/move/up only). app.js wires pointer events on mapCv/window.
// No pointercancel, setPointerCapture, pointerId checks or touch-action CSS exist, so the cancel and
// second-pointer cases are KNOWN-BUG tests with {todo: true}.
import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const mk = () => createPathEditor({ mx: x => x, my: y => y, getK: () => 1, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0 });
const ptr = (pointerType, pointerId = 1) => ({ button: 0, pointerType, pointerId, isPrimary: pointerId === 1 });
const endPt = ed => ed.route.segments[0].points.at(-1), valid = ed => !P.validateRoute(ed.route).some(i => i.severity === 'error');
function setup() {
  const ed = mk(); ed.active = true; ed.tool = 'line';
  ed.route.start = { x: 24, y: 24, h: 0 }; P.addLine(ed.route, 100, 24);
  return ed;
}
const near = (p, x, y) => assert.ok(Math.hypot(p.x - x, p.y - y) < 3, `expected near (${x},${y}), got (${p.x},${p.y})`);

test('touch drag moves a waypoint and the route validates', () => {
  const ed = setup(); const p0 = endPt(ed);
  ed.pointer('down', p0.x, p0.y, ptr('touch')); assert.ok(ed.drag, 'touch down starts a drag');
  ed.pointer('move', 110, 30, ptr('touch')); ed.pointer('up', 110, 30, ptr('touch'));
  near(endPt(ed), 110, 30); assert.ok(valid(ed));
});

test('pen drag moves a waypoint the same way as touch', () => {
  const ed = setup(); const p0 = endPt(ed);
  ed.pointer('down', p0.x, p0.y, ptr('pen')); assert.ok(ed.drag, 'pen down starts a drag');
  ed.pointer('move', 90, 40, ptr('pen')); ed.pointer('up', 90, 40, ptr('pen'));
  near(endPt(ed), 90, 40); assert.ok(valid(ed));
});

test('a touch drag released with up leaves the route valid', () => {
  const ed = setup(); const p0 = endPt(ed);
  ed.pointer('down', p0.x, p0.y, ptr('touch')); ed.pointer('move', 120, 60, ptr('touch'));
  ed.pointer('up', 120, 60, ptr('touch'));
  assert.equal(ed.drag, null, 'up ends the drag'); assert.ok(valid(ed));
});

test('KNOWN-BUG: pointercancel ends the drag and the route still validates', { todo: true }, () => {
  const ed = setup(); const p0 = endPt(ed);
  ed.pointer('down', p0.x, p0.y, ptr('touch')); ed.pointer('move', 110, 30, ptr('touch'));
  ed.pointer('cancel', 110, 30, ptr('touch'));   // no cancel type in pathui; host has no pointercancel listener
  assert.equal(ed.drag, null, 'cancel must end the drag'); assert.ok(valid(ed));
});

test('KNOWN-BUG: a second pointer does not start a second drag', { todo: true }, () => {
  const ed = setup(); const p0 = endPt(ed);
  ed.pointer('down', p0.x, p0.y, ptr('touch', 1)); const first = ed.drag;
  ed.pointer('down', p0.x, p0.y, ptr('touch', 2));
  assert.strictEqual(ed.drag, first, 'second pointerId must not replace the active drag');
});

test('KNOWN-BUG: moves from a second pointer do not move the waypoint', { todo: true }, () => {
  const ed = setup(); const p0 = endPt(ed);
  ed.pointer('down', p0.x, p0.y, ptr('touch', 1)); ed.pointer('move', 60, 60, ptr('touch', 2));
  near(endPt(ed), p0.x, p0.y); ed.pointer('up', 60, 60, ptr('touch', 2));
});
