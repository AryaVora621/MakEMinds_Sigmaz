import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor, waypointRows, applyWaypointEdit, insertWaypointAfter, deleteWaypoint } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const near = (a, b, eps = 1e-6) => assert.ok(Math.abs(a - b) < eps, `${a} != ${b}`);
// 3 anchors: start (10,20,0 deg), (30,40,90 deg), (50,60,-90 deg)
const mkRoute = () => {
  const r = P.newRoute({ name: 'T', alliance: 'red', start: { x: 10, y: 20, h: 0 } });
  P.addLine(r, 30, 40); P.addLine(r, 50, 60);
  P.setEndHeading(r, 0, Math.PI / 2); P.setEndHeading(r, 1, -Math.PI / 2);
  return r;
};

test('waypointRows on a 3-point route gives x, y and heading per anchor', () => {
  const rows = waypointRows(mkRoute());
  assert.equal(rows.length, 3);
  assert.equal(rows[0].start, true);
  assert.deepEqual([rows[0].x, rows[0].y], [10, 20]); near(rows[0].deg, 0);
  assert.equal(rows[1].start, false); assert.equal(rows[1].seg, 0); assert.equal(rows[1].idx, 1);
  assert.deepEqual([rows[1].x, rows[1].y], [30, 40]); near(rows[1].deg, 90);
  assert.deepEqual([rows[2].x, rows[2].y], [50, 60]); near(rows[2].deg, -90);
});

test('waypointRows on an empty route is just the start point', () => {
  const rows = waypointRows(P.newRoute({ start: { x: 5, y: 6, h: Math.PI } }));
  assert.equal(rows.length, 1); near(rows[0].deg, 180);
});

test('applyWaypointEdit x clamps to the field and carries the shared join', () => {
  const r = mkRoute(), rows = waypointRows(r);
  const res = applyWaypointEdit(r, rows[1], 'x', 200);
  assert.equal(res.ok, true); assert.equal(res.value, P.FIELD);
  assert.equal(r.segments[0].points.at(-1).x, P.FIELD);
  assert.equal(r.segments[1].points[0].x, P.FIELD, 'next segment starts where this one ends');
  assert.equal(waypointRows(r)[1].x, P.FIELD);
});

test('applyWaypointEdit y clamps at 0 and the start row moves route.start', () => {
  const r = mkRoute(), rows = waypointRows(r);
  assert.equal(applyWaypointEdit(r, rows[0], 'y', -5).ok, true);
  assert.equal(r.start.y, 0); assert.equal(r.segments[0].points[0].y, 0);
  assert.equal(applyWaypointEdit(r, rows[2], 'y', 30).ok, true);
  assert.equal(r.segments[1].points.at(-1).y, 30);
});

test('applyWaypointEdit heading sets the waypoint heading and wraps to +-180', () => {
  const r = mkRoute(), rows = waypointRows(r);
  const a = applyWaypointEdit(r, rows[1], 'deg', 45);
  assert.equal(a.ok, true); near(a.value, 45);
  near(P.resolveHeadings(r)[0].h1, Math.PI / 4); near(waypointRows(r)[1].deg, 45);
  applyWaypointEdit(r, rows[2], 'deg', 270);
  near(waypointRows(r)[2].deg, -90);
  applyWaypointEdit(r, rows[0], 'deg', 30);
  near(r.start.h, Math.PI / 6);
});

test('applyWaypointEdit refuses bad input, tangent headings and locked paths without changing the route', () => {
  const r = mkRoute(), rows = waypointRows(r), before = JSON.stringify(r);
  assert.equal(applyWaypointEdit(r, rows[1], 'x', 'abc').ok, false);
  assert.equal(JSON.stringify(r), before);
  r.segments[0].heading = { mode: 'tangent', reversed: false };
  assert.equal(applyWaypointEdit(r, rows[1], 'deg', 10).ok, false);
  assert.equal(r.segments[0].heading.mode, 'tangent');
  r.segments[1].locked = true;
  const locked = waypointRows(r)[2];
  assert.equal(locked.locked, true);
  assert.equal(applyWaypointEdit(r, locked, 'x', 10).ok, false);
  assert.equal(r.segments[1].points.at(-1).x, 50);
});

test('insertWaypointAfter splits the following path; after the last anchor it appends one', () => {
  const r = mkRoute(), rows = waypointRows(r);
  const at = insertWaypointAfter(r, rows[1]);
  assert.deepEqual(at, { seg: 1, idx: 1 });
  assert.equal(r.segments.length, 3);
  const nr = waypointRows(r);
  assert.equal(nr.length, 4);
  assert.deepEqual([nr[2].x, nr[2].y], [40, 50], 'midpoint of the 30,40 -> 50,60 path');
  const end = insertWaypointAfter(r, nr[3]);
  assert.equal(r.segments.length, 4);
  assert.equal(end.seg, 3); assert.equal(waypointRows(r).length, 5);
  assert.equal(r.segments[3].points.at(-1).x, 62, 'appended 12 in past the last anchor');
});

test('insertWaypointAfter on the start of an empty route appends a segment', () => {
  const r = P.newRoute({ start: { x: 10, y: 20, h: 0 } });
  const at = insertWaypointAfter(r, waypointRows(r)[0]);
  assert.deepEqual(at, { seg: 0, idx: 3 }, 'appended path is a cubic Bezier, so its end is index 3');
  assert.equal(r.segments.length, 1);
  assert.deepEqual(r.segments[0].points.at(-1), { x: 22, y: 20 });
});

test('deleteWaypoint cannot delete the start point and removes an end anchor', () => {
  const r = mkRoute(), rows = waypointRows(r);
  assert.equal(deleteWaypoint(r, rows[0]), false);
  assert.equal(r.segments.length, 2);
  assert.equal(deleteWaypoint(r, rows[1]), true);
  assert.equal(r.segments.length, 1);
  assert.deepEqual(r.segments[0].points.at(-1), { x: 50, y: 60 });
});

test('the panel renders one numeric x, y and heading row per anchor', () => {
  const ed = createPathEditor({ mx: x => x * 5, my: y => y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0 });
  P.addLine(ed.route, 60, 30); P.addLine(ed.route, 70, 40);
  const html = ed.panelHtml();
  assert.equal((html.match(/data-wrow="1"/g) || []).length, 3);
  assert.equal((html.match(/data-f="wx"/g) || []).length, 3);
  assert.equal((html.match(/data-f="wy"/g) || []).length, 3);
  assert.equal((html.match(/data-f="wh"/g) || []).length, 3);
  assert.equal((html.match(/data-wact="ins"/g) || []).length, 3);
  assert.equal((html.match(/data-wact="del"/g) || []).length, 3);
});
