import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';
import { routeToPP, ppToRoute, exportPP, DEFAULT_COLOR } from '../web/js/ppformat.js';

function demo() {
  const r = P.newRoute({ name: 'Demo', start: { x: 24, y: 24, h: Math.PI / 2 } });
  P.addLine(r, 24, 72); P.addCurve(r, 60, 96, 24, 96); P.addLine(r, 100, 96);
  r.segments[0].heading = { mode: 'linear', start: Math.PI / 2, end: Math.PI, reversed: false };
  r.segments[2].heading = { mode: 'constant', start: 0.3, reversed: false };
  r.segments[1].heading = { mode: 'tangent', reversed: true };
  r.segments[1].color = '#112233'; r.segments[0].label = 'Intake'; r.segments[2].speed = 30; r.segments[2].callbacks = [{ t: 0.5, name: 'shoot' }];
  return r;
}
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-5, `${a} vs ${b}`);

test('export uses the Visualizer keys', () => {
  const o = routeToPP(demo(), { timestamp: 'T' });
  assert.equal(o.version, '2.0.0'); assert.equal(o.startPoint.headingDeg, 90); assert.equal(o.lines.length, 3);
  assert.deepEqual(o.lines[1].controlPoints, [{ x: 24, y: 96 }]); assert.deepEqual(o.lines[1].endPoint, { x: 60, y: 96 });
  assert.deepEqual(o.lines[1].heading, { type: 'tangential', reverse: true }); assert.equal(o.lines[0].heading.endDeg, 180);
  assert.deepEqual(o.sequence[0], { kind: 'path', lineId: o.lines[0].id }); assert.equal(o.lines[0].color, DEFAULT_COLOR);
});
test('round trip preserves geometry, headings, colours, extras', () => {
  const a = demo(), { route: b, warnings } = ppToRoute(exportPP(a));
  assert.deepEqual(warnings, []); assert.equal(b.segments.length, 3); near(b.start.h, a.start.h);
  b.segments.forEach((s, i) => { assert.equal(s.type, a.segments[i].type); s.points.forEach((p, j) => { near(p.x, a.segments[i].points[j].x); near(p.y, a.segments[i].points[j].y); }); });
  const ra = P.resolveHeadings(a), rb = P.resolveHeadings(b); ra.forEach((h, i) => { near(h.h0, rb[i].h0); near(h.h1, rb[i].h1); });
  assert.equal(b.segments[1].color, '#112233'); assert.equal(b.segments[0].label, 'Intake'); assert.equal(b.segments[2].speed, 30); assert.equal(b.segments[2].callbacks[0].name, 'shoot');
  assert.equal(b.segments[1].heading.reversed, true);
});
test('importer ignores unknown keys and respects sequence order', () => {
  const pp = { version: '2.0.0', startPoint: { x: 10, y: 10, headingDeg: 0 }, junk: 1, shapes: [{}], robot: {},
    lines: [{ id: 'a', endPoint: { x: 50, y: 10 }, controlPoints: [], heading: { type: 'constant', degrees: 0 }, extra: 1 }, { id: 'b', endPoint: { x: 50, y: 50 }, controlPoints: [{ x: 60, y: 30 }], heading: { type: 'tangential', reverse: false } }],
    sequence: [{ kind: 'path', lineId: 'b' }, { kind: 'wait', durationMs: 500 }, { kind: 'path', lineId: 'a' }] };
  const { route, warnings } = ppToRoute(pp);
  assert.equal(route.segments[0].type, 'curve'); assert.deepEqual(route.segments[1].points[0], { x: 50, y: 50 }); assert.ok(warnings.some(w => /wait/.test(w)));
});
test('importer validates numerics and surfaces warnings', () => {
  const { route, warnings } = ppToRoute({ startPoint: { x: 10, y: 10 }, lines: [{ endPoint: { x: 'a', y: 1 } }, { endPoint: { x: 200, y: 20 }, heading: { type: 'piecewise' } }, null] });
  assert.equal(route.segments.length, 1); assert.ok(warnings.length >= 3); assert.equal(route.segments[0].heading.mode, 'tangent');
});
test('importer rejects non-JSON and tolerates a missing start', () => {
  assert.throws(() => ppToRoute('{nope'), /Invalid/); assert.throws(() => ppToRoute('[]'), /object/);
  const { route, warnings } = ppToRoute({ lines: [] }); assert.equal(route.segments.length, 0); assert.ok(warnings.some(w => /startPoint/.test(w)));
});
test('compound path is flattened with a warning; heading type inferred from keys', () => {
  const { route, warnings } = ppToRoute({ startPoint: { x: 1, y: 1, headingDeg: 45 }, lines: [{ kind: 'compound', segments: [{ endPoint: { x: 5, y: 5 }, controlPoints: [] }, { endPoint: { x: 9, y: 9 }, controlPoints: [], heading: { startDeg: 0, endDeg: 90 } }] }] });
  assert.equal(route.segments.length, 1); assert.ok(warnings.some(w => /compound/.test(w)));
  const r = ppToRoute({ startPoint: { x: 1, y: 1 }, lines: [{ endPoint: { x: 5, y: 5 }, heading: { startDeg: 0, endDeg: 90 } }] }).route; near(r.segments[0].heading.end, Math.PI / 2);
});
