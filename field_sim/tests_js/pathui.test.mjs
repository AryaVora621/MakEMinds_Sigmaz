import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const mk = () => createPathEditor({ mx: x => x * 5, my: y => y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0 });
const click = (ed, x, y, e = {}) => { ed.pointer('down', x, y, { button: 0, ...e }); ed.pointer('up', x, y, e); };
const sleep = ms => new Promise(r => setTimeout(r, ms));

test('clicking adds snapped paths; grid and magnet snapping; undo/redo', () => {
  const ed = mk(); ed.active = true; ed.grid = 6; ed.tool = 'line';
  click(ed, 50.4, 30.2); assert.deepEqual(ed.route.segments[0].points.at(-1), { x: 48, y: 30 });
  click(ed, 49.5, 31); assert.equal(ed.route.segments.length, 2); assert.deepEqual(ed.route.segments[1].points.at(-1), { x: 48, y: 30 }, 'magnet snaps to the existing point');
  ed.undo(); assert.equal(ed.route.segments.length, 1); ed.redo(); assert.equal(ed.route.segments.length, 2);
  click(ed, 70.7, 70.2, { altKey: true }); assert.deepEqual(ed.route.segments[2].points.at(-1), { x: 70.7, y: 70.2 });
});

test('dragging a heading handle sets the waypoint heading (snapped to 15 deg)', () => {
  const ed = mk(); ed.active = true; ed.tool = 'line'; ed.route.start.h = 0; click(ed, 60, 24);
  const end = ed.route.segments[0].points.at(-1);
  const L = 24 / 5; // handle sits 24 px from the waypoint
  const res = P.resolveHeadings(ed.route)[0];
  ed.pointer('down', end.x + Math.cos(res.h1) * L, end.y + Math.sin(res.h1) * L, { button: 0 });
  assert.equal(ed.drag.kind, 'hdg');
  ed.pointer('move', end.x, end.y + 10, {}); ed.pointer('up', end.x, end.y + 10, {});
  assert.ok(Math.abs(P.resolveHeadings(ed.route)[0].h1 - Math.PI / 2) < 1e-6);
  ed.undo(); assert.ok(Math.abs(P.resolveHeadings(ed.route)[0].h1) < 1e-6);
});

test('event markers: add from the palette, drag along the path, right-click deletes', () => {
  const ed = mk(); ed.active = true; ed.tool = 'line'; ed.route.start = { x: 24, y: 24, h: Math.PI / 2 }; click(ed, 100, 24);   // start pinned to the red-half default these clicks were written for
  ed.sel = 0; assert.ok(P.addEvent(ed.route, 0, 0.5, 'intakeOn') >= 0);
  const ev = P.routeEvents(ed.route)[0];
  ed.pointer('down', ev.x, ev.y, { button: 0 }); assert.equal(ed.drag.kind, 'evt');
  ed.pointer('move', 90, 24, {}); ed.pointer('up', 90, 24, {});
  assert.ok(ed.route.segments[0].callbacks[0].t > 0.8);
  const ev2 = P.routeEvents(ed.route)[0]; ed.pointer('down', ev2.x, ev2.y, { button: 2 }); ed.pointer('up', ev2.x, ev2.y, {});
  assert.equal(ed.route.segments[0].callbacks.length, 0);
});

test('panel HTML has inline SVG icons, tooltips, snap controls and scrubber', () => {
  const ed = mk(); ed.active = true; ed.tool = 'line'; click(ed, 100, 24); P.addEvent(ed.route, 0, 0.5, 'shootAll');
  ed.open.add('settings'); ed.open.add('events');
  const h = ed.panelHtml();
  assert.match(h, /<svg class="pp-ico"/); assert.match(h, /title="Undo \(Ctrl\+Z\)"/); assert.match(h, /data-f="grid"/); assert.match(h, /data-f="scrub"/); assert.match(h, /class="pp-tick"/);
  ed.sel = 0; assert.match(ed.panelHtml(), /data-evadd="gateOpen"/);
  assert.match(h, /data-sec="waypoints"/); assert.match(h, /data-act="addev"/);
});

test('playback uses the planned trajectory (fallback model) with seek and step', async () => {
  const ed = mk(); ed.active = true; ed.tool = 'line'; ed.route.start = { x: 24, y: 24, h: Math.PI / 2 }; click(ed, 100, 24);   // start pinned to the red-half default these clicks were written for
  await sleep(300); assert.ok(ed.traj && ed.traj.length > 5, 'trajectory planned');
  const T = ed.traj.at(-1).t; ed.seek(T / 2); assert.ok(ed.playPose.x > 24 && ed.playPose.x < 100);
  ed.stepBy(0.05); assert.ok(ed.time > T / 2);
  ed.seek(0); ed.play(); let n = 0; while (ed.playing && n++ < 2000) ed.step(0.05);
  assert.equal(ed.playing, false); assert.ok(Math.abs(ed.playPose.x - 100) < 0.5);
});

test('path library: new / rename / duplicate / delete keep the project list in sync', () => {
  const lib = P.createPathLibrary(null);
  const ed = createPathEditor({ mx: x => x, my: y => y, getK: () => 1, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0, library: lib });
  ed.active = true; ed.tool = 'line'; click(ed, 90, 24);
  assert.deepEqual(lib.names(), ['Path 1']); assert.equal(lib.get('Path 1').segments.length, 1);
  const n2 = lib.create('Second'); assert.equal(lib.active(), 'Second'); assert.equal(ed.route.name, 'Second'); assert.equal(ed.route.segments.length, 0, 'editor switched to the new empty path');
  lib.setActive('Path 1'); assert.equal(ed.route.segments.length, 1, 'switching loads the stored path');
  assert.ok(lib.rename('Path 1', 'Left')); assert.equal(ed.route.name, 'Left');
  const d = lib.duplicate('Left'); assert.equal(d, 'Left copy'); assert.equal(ed.route.name, 'Left copy');
  lib.remove('Left copy'); assert.ok(lib.names().includes(ed.route.name)); assert.equal(lib.names().length, 2);
});
