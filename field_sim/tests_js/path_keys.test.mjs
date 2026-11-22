import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor, handleKey } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const mk = () => {
  const ed = createPathEditor({ mx: x => x * 5, my: y => 720 - y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0, library: P.createPathLibrary(null), toast: () => {} });
  ed.active = true; ed.grid = 0; ed.magnet = false; return ed;
};
const click = (ed, x, y, e = {}) => { const r = ed.pointer('down', x, y, { button: 0, ...e }); ed.pointer('up', x, y, e); return r; };
/** Start at (10,10) and three connected curve waypoints along y=10: segments 0, 1, 2 end at x = 30, 50, 70. */
const build = () => {
  const ed = mk(); ed.route.start.x = 10; ed.route.start.y = 10;
  click(ed, 30, 10); click(ed, 50, 10); click(ed, 70, 10);
  assert.equal(ed.route.segments.length, 3);
  return ed;
};
const key = (ed, k, extra = {}) => handleKey(ed, { key: k, targetTag: 'BODY', ...extra });
const ends = ed => ed.route.segments.map(s => s.points.at(-1).x);
const snap = ed => JSON.stringify(ed.route);

test('Delete removes the selected waypoint; undo restores it; redo reapplies it', () => {
  const ed = build();
  ed.selPt = { seg: 1, idx: ed.route.segments[1].points.length - 1 };   // the waypoint at x = 50
  assert.equal(key(ed, 'Delete'), true);
  assert.deepEqual(ends(ed), [30, 70], 'waypoint at x = 50 is gone');
  assert.equal(ed.selPt, null);
  assert.equal(key(ed, 'z', { ctrlKey: true }), true);
  assert.deepEqual(ends(ed), [30, 50, 70], 'undo restores it');
  assert.equal(handleKey(ed, { key: 'y', ctrlKey: true, targetTag: 'BODY' }), true);
  assert.deepEqual(ends(ed), [30, 70], 'Ctrl+Y redoes the delete');
  ed.undo(); assert.deepEqual(ends(ed), [30, 50, 70]);
  assert.equal(handleKey(ed, { key: 'Z', ctrlKey: true, shiftKey: true, targetTag: 'BODY' }), true);
  assert.deepEqual(ends(ed), [30, 70], 'Ctrl+Shift+Z redoes the delete');
});

test('Backspace also deletes the selected waypoint; the start point is refused', () => {
  const ed = build();
  ed.selPt = { seg: 2, idx: ed.route.segments[2].points.length - 1 };
  assert.equal(key(ed, 'Backspace'), true);
  assert.deepEqual(ends(ed), [30, 50]);
  const before = snap(ed); ed.selPt = { seg: 0, idx: 0 };
  assert.equal(key(ed, 'Delete'), true);
  assert.equal(snap(ed), before, 'start point cannot be deleted');
});

test('no action when the key comes from an INPUT, SELECT or TEXTAREA', () => {
  const ed = build(); ed.selPt = { seg: 1, idx: ed.route.segments[1].points.length - 1 };
  const before = snap(ed);
  assert.equal(handleKey(ed, { key: 'Delete', targetTag: 'INPUT' }), false);
  assert.equal(handleKey(ed, { key: 'Backspace', targetTag: 'TEXTAREA' }), false);
  assert.equal(handleKey(ed, { key: 'z', ctrlKey: true, targetTag: 'INPUT' }), false, 'Ctrl+Z inside a field is the field\'s own undo');
  assert.equal(handleKey(ed, { key: 'Escape', targetTag: 'SELECT' }), false);
  assert.equal(snap(ed), before);
  assert.equal(ed.selPt !== null, true);
});

test('Escape deselects the path and the waypoint', () => {
  const ed = build(); ed.selPt = { seg: 1, idx: 2 };
  assert.equal(key(ed, 'Escape'), true);
  assert.equal(ed.sel, -1); assert.equal(ed.selPt, null);
});

test('keys do nothing when the editor is not live', () => {
  const ed = build(); ed.active = false; ed.selPt = { seg: 1, idx: 2 };
  assert.equal(key(ed, 'Delete'), false); assert.equal(ed.route.segments.length, 3);
});

test('each panel action pushes exactly one undo entry (undo returns to before, redo to after)', () => {
  const cases = [
    ['addrot', 0], ['addzone', 0], ['split', 0], ['addctl', 0], ['eye', 0], ['lock', 0], ['up', 1], ['down', 0], ['del', 1], ['delseg', 0],
  ];
  for (const [act, sel] of cases) {
    const ed = build(); ed.sel = sel; ed.selPt = null;
    const pre = snap(ed); ed.act(act);
    const post = snap(ed);
    assert.notEqual(post, pre, act + ' changes the route');
    assert.equal(ed.undo(), true, act + ' has an undo entry');
    assert.equal(snap(ed), pre, act + ': one undo returns to the state before');
    assert.equal(ed.redo(), true, act + ' has a redo entry');
    assert.equal(snap(ed), post, act + ': one redo returns to the state after');
  }
});

test('deleting a rotation target: click selects its dot, Delete removes it, undo restores it', () => {
  const ed = build();
  P.addRotation(ed.route, 0, 0.5, 0); ed.commit();
  const q = P.bezierPoint(ed.route.segments[0].points, 0.5);
  const segs = ed.route.segments.length;
  click(ed, q.x, q.y);
  assert.deepEqual(ed.rotSel, { seg: 0, k: 0 }, 'clicking the rotation dot selects it');
  assert.equal(ed.route.segments.length, segs, 'selecting a dot adds no waypoint');
  assert.equal(key(ed, 'Delete'), true);
  assert.equal(ed.route.segments[0].rotations.length, 0, 'rotation target removed');
  ed.undo(); assert.equal(ed.route.segments[0].rotations.length, 1, 'undo restores it');
  ed.redo(); assert.equal(ed.route.segments[0].rotations.length, 0, 'redo removes it again');
});

test('deleting a selected event marker removes it in one undo entry', () => {
  const ed = build();
  const name = Object.keys(P.EVENT_PRESETS)[0];
  P.addEvent(ed.route, 0, 0.5, name); ed.commit();
  ed.evSel = { seg: 0, i: 0 };
  assert.equal(key(ed, 'Delete'), true);
  assert.equal((ed.route.segments[0].callbacks || []).length, 0, 'event removed');
  ed.undo(); assert.equal(ed.route.segments[0].callbacks.length, 1, 'undo restores the event');
});

test('Escape also clears the rotation and event selections', () => {
  const ed = build(); ed.rotSel = { seg: 0, k: 0 }; ed.evSel = { seg: 0, i: 0 };
  key(ed, 'Escape'); assert.equal(ed.rotSel, null); assert.equal(ed.evSel, null);
});
