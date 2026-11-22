import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const mk = (extra = {}) => { const ed = createPathEditor({ mx: x => x * 5, my: y => 720 - y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0, library: P.createPathLibrary(null), toast: () => {}, ...extra }); ed.active = true; return ed; };
const click = (ed, x, y, e = {}) => { const r = ed.pointer('down', x, y, { button: 0, ...e }); ed.pointer('up', x, y, e); return r; };
const stub = () => { const calls = []; const ctx = new Proxy({ calls, measureText: () => ({ width: 40 }) }, { get: (t, k) => (k in t ? t[k] : (...a) => { calls.push([k, ...a]); }), set: () => true }); return ctx; };
const withView = (v, fn) => { const o = { ...P.PLAN_VIEW }; Object.assign(P.PLAN_VIEW, v); try { return fn(); } finally { Object.assign(P.PLAN_VIEW, o); P.PLAN_VIEW.openPath = o.openPath; } };

test('addWaypoint appends a connected cubic with handles at 1/3 and 2/3', () => {
  const r = P.newRoute({ start: { x: 10, y: 10 } }); const i = P.addWaypoint(r, 40, 10);
  assert.equal(i, 0); assert.equal(r.segments[0].type, 'curve'); assert.deepEqual(r.segments[0].points, [{ x: 10, y: 10 }, { x: 20, y: 10 }, { x: 30, y: 10 }, { x: 40, y: 10 }]);
  P.addWaypoint(r, 40, 40); assert.deepEqual(r.segments[1].points[0], { x: 40, y: 10 }, 'connected to the previous end');
});

test('insertWaypointAt splits the nearest segment; refuses far clicks and the ends', () => {
  const r = P.newRoute({ start: { x: 10, y: 10 } }); P.addWaypoint(r, 70, 10);
  assert.equal(P.insertWaypointAt(r, 40, 12, 4), 0); assert.equal(r.segments.length, 2);
  assert.ok(Math.abs(r.segments[0].points.at(-1).x - 40) < 1.5); assert.deepEqual(r.segments[1].points[0], r.segments[0].points.at(-1));
  assert.equal(P.insertWaypointAt(r, 40, 30, 4), -1, 'too far');
  assert.equal(P.insertWaypointAt(r, 10.1, 10, 4), -1, 'at the start anchor: no insert');
});

test('moveAnchor carries the handles of both neighbouring segments; control points move alone', () => {
  const r = P.newRoute({ start: { x: 0, y: 0 } }); P.addWaypoint(r, 30, 0); P.addWaypoint(r, 60, 0);
  P.moveAnchor(r, 0, 3, 30, 10);
  assert.deepEqual(r.segments[0].points[2], { x: 20, y: 10 }, 'incoming handle follows'); assert.deepEqual(r.segments[1].points[1], { x: 40, y: 10 }, 'outgoing handle follows');
  assert.deepEqual(r.segments[1].points[0], { x: 30, y: 10 });
  P.moveAnchor(r, 0, 1, 12, 5); assert.deepEqual(r.segments[0].points[0], { x: 0, y: 0 }); assert.deepEqual(r.segments[0].points[1], { x: 12, y: 5 });
});

test('alliance view bounds: red = x 0..72, blue = x 72..144, full = all; clamp and inView', () => {
  assert.deepEqual([P.viewBounds('red').x0, P.viewBounds('red').x1], [0, 72]); assert.deepEqual([P.viewBounds('blue').x0, P.viewBounds('blue').x1], [72, 144]);
  const f = P.viewBounds('blue', true); assert.equal(f.full, true); assert.deepEqual([f.x0, f.x1, f.y0, f.y1], [0, 144, 0, 144]);
  const b = P.viewBounds('red'); assert.deepEqual(P.clampToView(b, 100, 200), { x: 72, y: 144 }); assert.equal(P.inView(b, 10, 10), true); assert.equal(P.inView(b, 100, 10), false);
});

test('editor: click empty field appends a connected waypoint with handles; clicking a path inserts; right-click and Delete remove', () => {
  const ed = mk(); ed.route.start.x = 20; ed.route.start.y = 20; ed.grid = 0; ed.magnet = false;
  assert.equal(click(ed, 80, 20), true); assert.equal(ed.route.segments.length, 1); assert.equal(ed.route.segments[0].points.length, 4);
  click(ed, 80, 70); assert.equal(ed.route.segments.length, 2); assert.deepEqual(ed.route.segments[1].points[0], ed.route.segments[0].points.at(-1));
  click(ed, 50, 20.5); assert.equal(ed.route.segments.length, 3, 'click on the first path inserts a waypoint (split)');
  const anchor = ed.route.segments[0].points.at(-1);
  ed.pointer('down', anchor.x, anchor.y, { button: 2 }); ed.pointer('up', anchor.x, anchor.y, {});
  assert.equal(ed.route.segments.length, 2, 'right-click removes the waypoint');
  const e2 = ed.route.segments.at(-1).points.at(-1); click(ed, e2.x, e2.y); assert.ok(ed.selPt, 'anchor selected'); ed.hover = null;
  assert.equal(ed.key({ key: 'Delete' }), true); assert.equal(ed.route.segments.length, 1, 'Delete removes the selected waypoint');
});

test('dragging an anchor carries its handles', () => {
  const ed = mk(); ed.route.start.x = 20; ed.route.start.y = 20; ed.grid = 0; ed.magnet = false; click(ed, 80, 20); click(ed, 120, 20);
  const a = ed.route.segments[0].points.at(-1), before = { ...ed.route.segments[1].points[1] };
  ed.pointer('down', a.x, a.y, { button: 0 }); ed.pointer('move', a.x, a.y + 12, {}); ed.pointer('up', a.x, a.y + 12, {});
  assert.ok(Math.abs(ed.route.segments[1].points[1].y - (before.y + 12)) < 1e-9, 'outgoing handle moved with the anchor');
});

test('Auto tab: the map never places or drags a robot; clicking a path opens it; empty clicks are swallowed', () => {
  const ed = mk(); ed.active = false;
  const route = P.newRoute({ start: { x: 100, y: 20 } }); P.addWaypoint(route, 130, 60);
  let opened = null;
  withView({ autoVisible: true, items: [{ id: 'p1', route, selected: false, markers: [] }], start: { x: 100, y: 20, h: 0 }, alliance: 'blue', openPath: id => { opened = id; } }, () => {
    assert.equal(ed.editing, true, 'host sees editing=true on the Auto tab so it routes pointer events here');
    assert.equal(ed.pointer('down', 5, 120, { button: 0 }), true, 'empty click swallowed (no robot placement)'); assert.equal(opened, null);
    assert.equal(ed.pointer('down', 5, 120, { button: 2 }), true, 'right-click swallowed (no robot turn)');
    assert.equal(ed.pointer('down', 115, 40, { button: 0 }), true); assert.equal(opened, 'p1');
    assert.equal(ed.pointer('down', 100, 20, { button: 0, ctrlKey: true }), false, 'ctrl-click still copies a Pose (host)');
    ed.draw(stub());
  });
  assert.equal(ed.editing, false);
});

test('Auto tab with a path open: edits are limited to the alliance half, Full field lifts it', () => {
  const ed = mk(); ed.active = false; ed.route.start.x = 100; ed.route.start.y = 20; ed.grid = 0; ed.magnet = false; ed.editing = true;
  let toasts = 0; const ed2 = ed;
  withView({ autoVisible: true, items: [], start: null, alliance: 'blue', fullField: false }, () => {
    assert.equal(ed2.bounds().x0, 72);
    assert.equal(click(ed2, 30, 50), true); assert.equal(ed2.route.segments.length, 0, 'red half is not editable for a blue auto');
    click(ed2, 110, 50); assert.equal(ed2.route.segments.length, 1);
    ed2.pointer('down', 110, 50, { button: 0 }); ed2.pointer('move', 20, 50, {}); ed2.pointer('up', 20, 50, {});
    assert.equal(ed2.route.segments[0].points.at(-1).x, 72, 'drag clamps at the half boundary');
    P.PLAN_VIEW.fullField = true; click(ed2, 30, 90); assert.equal(ed2.route.segments.length, 2, 'full field allows the other half');
    P.PLAN_VIEW.alliance = 'red'; P.PLAN_VIEW.fullField = false; assert.equal(ed2.bounds().x1, 72);
  });
  void toasts;
});

test('drawing: PathPlanner colours, robot outline only at start/end/selected and preview, never the live robot', () => {
  const ed = mk(); ed.grid = 0; ed.magnet = false; ed.route.start.x = 20; ed.route.start.y = 20; click(ed, 80, 20); click(ed, 80, 80);
  const c = stub(); ed.draw(c);
  const fills = c.calls.filter(x => x[0] === 'fillRect').length, strokes = c.calls.filter(x => x[0] === 'strokeRect').length;
  assert.equal(strokes, 3, 'start, goal and the selected waypoint outlines (no other robot)');
  assert.equal(fills, 0, 'PathPlanner robot outlines are stroke only: no fillRect, no translucent boxes');
  ed.selPt = null; const c2 = stub(); ed.draw(c2); assert.equal(c2.calls.filter(x => x[0] === 'strokeRect').length, 2, 'start and goal only');
  ed.playPose = { x: 50, y: 50, h: 0 }; const c3 = stub(); ed.draw(c3); assert.equal(c3.calls.filter(x => x[0] === 'strokeRect').length, 3, 'preview adds one outline');
});
