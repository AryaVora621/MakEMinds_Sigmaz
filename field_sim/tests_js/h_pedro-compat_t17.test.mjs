// t17: .pp round trip of an auto's paths, markers and command-group tree.
// The .pp format carries lines (geometry, heading, callbacks) and a flat sequence of path/wait items only.
// Auto markers are mapped to segment callbacks (single-segment paths, so t is the same scale).
import test from 'node:test';
import assert from 'node:assert/strict';
import * as A from '../web/js/autos.js';
import { wrapAngle } from '../web/js/pathing.js';
import { exportPP, ppToRoute } from '../web/js/ppformat.js';

const line = (a, b, h0, h1) => ({ type: 'line', points: [a, b], heading: { mode: 'linear', start: h0, end: h1, reversed: false }, callbacks: [], label: '' });
const curve = (a, c, b, h0, h1) => ({ type: 'curve', points: [a, c, b], heading: { mode: 'linear', start: h0, end: h1, reversed: false }, callbacks: [], label: '' });

function sample() {
  const a = A.newAuto({ name: 'T17', alliance: 'blue', start: { x: 20.1234567, y: 20.7654321, h: 0.123456789 } });
  a.paths.p1 = { segments: [line({ x: 20.1234567, y: 20.7654321 }, { x: 60.333333, y: 20.5 }, 0.123456789, 1.5707963)],
    markers: [{ t: 0.5, command: { type: 'command', name: 'shoot', args: { count: 2 } } }] };
  a.paths.p2 = { segments: [curve({ x: 60.333333, y: 20.5 }, { x: 80.25, y: 40.125 }, { x: 60.0, y: 60.9 }, 1.5707963, 3.0)],
    markers: [{ t: 0.25, command: { type: 'command', name: 'intakeOn', args: {} } }] };
  A.addNode(a, null, { type: 'follow', path: 'p1' });
  A.addNode(a, null, { type: 'parallel', children: [{ type: 'command', name: 'intakeOn', args: {} }, { type: 'follow', path: 'p2' }] });
  A.addNode(a, null, { type: 'race', children: [{ type: 'wait', seconds: 1 }, { type: 'command', name: 'park', args: {} }] });
  return { a: A.ensureIds(a) };
}
const near = (x, y, msg = '') => assert.ok(Math.abs(x - y) < 1e-6, `${msg} ${x} vs ${y}`);
const angNear = (x, y) => near(wrapAngle(x - y), 0, 'angle');

// Auto -> .pp route: segments in path order, markers become segment callbacks.
function toPP(a, order) {
  const segments = order.flatMap(id => a.paths[id].segments.map(s => ({ ...s, callbacks: a.paths[id].markers.map(m => ({ t: m.t, name: m.command.name, args: m.command.args })) })));
  return exportPP({ name: a.name, alliance: a.alliance, start: a.start, segments }, { timestamp: 'T' });
}
const order = ['p1', 'p2'];

test('path geometry and headings survive the .pp round trip within 1e-6', () => {
  const { a } = sample(); const { route, warnings } = ppToRoute(toPP(a, order));
  assert.deepEqual(warnings, []); assert.equal(route.segments.length, 2);
  near(route.start.x, a.start.x); near(route.start.y, a.start.y); angNear(route.start.h, a.start.h);
  order.forEach((id, i) => {
    const s0 = a.paths[id].segments[0], s1 = route.segments[i];
    assert.equal(s1.type, s0.type); s0.points.forEach((p, j) => { near(s1.points[j].x, p.x, `${id}.x${j}`); near(s1.points[j].y, p.y, `${id}.y${j}`); });
    angNear(s1.heading.start, s0.heading.start); angNear(s1.heading.end, s0.heading.end);
  });
});

test('markers survive as .pp callbacks with name, t and args', () => {
  const { a } = sample(); const { route } = ppToRoute(toPP(a, order));
  const back = order.map((id, i) => ({ id, markers: route.segments[i].callbacks.map(c => ({ t: c.t, command: { type: 'command', name: c.name, args: c.args ?? {} } })) }));
  order.forEach((id, i) => {
    assert.equal(back[i].markers.length, a.paths[id].markers.length, id);
    a.paths[id].markers.forEach((m, k) => { near(back[i].markers[k].t, m.t); assert.equal(back[i].markers[k].command.name, m.command.name); assert.deepEqual(back[i].markers[k].command.args, m.command.args); });
  });
});

// gap: the .pp sequence only holds {kind:'path'|'wait'} items. Parallel, race, sequential and command nodes
// have no .pp representation, so the tree cannot be rebuilt from the file. Expected to fail until a format extension exists.
test('command-group structure survives .pp round trip', { todo: 'gap: .pp sequence has no parallel/race/sequential groups or command nodes' }, () => {
  const { a } = sample(); const pp = JSON.parse(toPP(a, order));
  // Only information the file holds: the flat sequence of paths.
  const carried = { type: 'sequential', children: pp.sequence.map(s => ({ type: 'follow', path: pp.lines.find(l => l.id === s.lineId).name })) };
  assert.deepEqual(carried, a.root);
});
