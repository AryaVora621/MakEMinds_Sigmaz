import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

// The robot (ed.playPose, the ghost pose during playback/preview) is display-only in the planner:
// no pointer down/move/up sequence may start a drag of it or move it. Only waypoints, control
// points, the start marker, rotation dots and event diamonds are draggable.

const mk = (extra = {}) => createPathEditor({ mx: x => x, my: y => y, getK: () => 1, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0, ...extra });
const snap = ed => JSON.stringify({ playPose: ed.playPose, start: ed.route.start, segs: ed.route.segments, time: ed.time, progress: ed.progress, playing: ed.playing });

function robotDragAttempt(ed, at) {
  ed.pointer('down', at.x, at.y, { button: 0 });
  assert.equal(ed.drag, null, 'pointer down on the robot must not start a drag');
  ed.pointer('move', at.x + 30, at.y + 20, {});
  ed.pointer('up', at.x + 30, at.y + 20, {});
  assert.equal(ed.drag, null, 'no drag after up');
}

test('path tab: pointer down/move/up on the robot pose neither drags nor moves it', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select';
  ed.route.start = { x: 24, y: 24, h: 0 };
  P.addLine(ed.route, 100, 24);
  ed.playPose = { x: 60, y: 100, h: 0.7 };
  const before = snap(ed);
  robotDragAttempt(ed, ed.playPose);
  assert.equal(snap(ed), before, 'robot pose, route and playback state unchanged');
});

test('path tab: robot pose while playing is not draggable and stepping keeps it display-only', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select';
  P.addLine(ed.route, 100, 24);
  ed.playPose = { x: 80, y: 60, h: 1.2 };
  ed.playing = true;
  robotDragAttempt(ed, ed.playPose);
  assert.deepEqual(ed.playPose, { x: 80, y: 60, h: 1.2 });
  assert.equal(ed.playing, true);
});

test('path tab: a robot-sized footprint next to a waypoint still drags only the waypoint, never the robot', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select';
  P.addLine(ed.route, 100, 24);
  const end = ed.route.segments[0].points.at(-1);
  ed.playPose = { x: end.x + 40, y: end.y + 40, h: 0 };
  const robot = { ...ed.playPose };
  ed.pointer('down', end.x, end.y, { button: 0 });
  assert.ok(ed.drag, 'waypoint is draggable');
  ed.pointer('move', end.x + 5, end.y + 5, {});
  assert.deepEqual(ed.playPose, robot, 'robot pose unchanged while the waypoint moves');
  ed.pointer('up', end.x + 5, end.y + 5, {});
  assert.equal(ed.playPose, null, 'a committed route edit clears the preview (existing commit() behaviour), it does not move the robot');
});

test('auto tab overview (no path open): pointer down on the robot pose does not drag or move it', () => {
  const V = P.PLAN_VIEW;
  const saved = { autoVisible: V.autoVisible, items: V.items, openPath: V.openPath };
  try {
    V.autoVisible = true; V.items = []; V.openPath = null;
    const ed = mk({ library: P.createPathLibrary(null) });
    ed.playPose = { x: 60, y: 100, h: 0.3 };
    const before = snap(ed);
    const handled = ed.pointer('down', 60, 100, { button: 0 });
    assert.equal(handled, true, 'overview swallows the press so the host cannot place the robot');
    assert.equal(ed.drag, null);
    ed.pointer('move', 70, 110, {}); ed.pointer('up', 70, 110, {});
    assert.equal(ed.drag, null);
    assert.equal(snap(ed), before);
  } finally { V.autoVisible = saved.autoVisible; V.items = saved.items; V.openPath = saved.openPath; }
});

test('auto tab with a path opened for editing: robot pose is still not draggable', () => {
  const V = P.PLAN_VIEW;
  const saved = { autoVisible: V.autoVisible, items: V.items, openPath: V.openPath };
  try {
    V.autoVisible = true; V.items = []; V.openPath = null;
    const ed = mk(); ed.editing = true; ed.tool = 'select';
    P.addLine(ed.route, 100, 24);
    ed.playPose = { x: 60, y: 100, h: 0 };
    const before = snap(ed);
    robotDragAttempt(ed, ed.playPose);
    assert.equal(snap(ed), before);
  } finally { V.autoVisible = saved.autoVisible; V.items = saved.items; V.openPath = saved.openPath; }
});
