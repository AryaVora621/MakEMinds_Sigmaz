// Obstacle accuracy tests for validateRoute (web/js/pathing.js).
// Obstacle shape accepted: axis-aligned rectangle {minX,minY,maxX,maxY,name}. Circles are NOT supported (no radius field).
// Method: validateRoute checks sampleRoute(route, 1), i.e. one sample per inch, and warns when a sample lies inside a box.
// KNOWN-BUG (reported as TODO, does not fail the run): a corner clip whose chord is shorter than ~1 in can fall between
// samples and go unreported. Example: line (40,60.5)->(60.5,40) clips box [50,60]x[50,60] for ~0.7 in; no sample lands inside.
import test from 'node:test';
import assert from 'node:assert/strict';
import { newRoute, addLine, validateRoute } from '../web/js/pathing.js';

const box = { minX: 50, minY: 50, maxX: 60, maxY: 60, name: 'box' };
const passes = issues => issues.filter(i => /passes through/.test(i.msg));
function line(x0, y0, x1, y1) { const r = newRoute({ start: { x: x0, y: y0, h: 0 } }); addLine(r, x1, y1); return r; }

test('(a) line clearly outside the obstacle: no pass-through issue', () => {
  assert.equal(passes(validateRoute(line(10, 10, 130, 10), [box])).length, 0); // well below
  assert.equal(passes(validateRoute(line(10, 120, 130, 120), [box])).length, 0); // well above
});

test('(b) line just beside the obstacle (1.5 in clearance): no pass-through issue', () => {
  assert.equal(passes(validateRoute(line(10, 61.5, 130, 61.5), [box])).length, 0); // above the top edge
  assert.equal(passes(validateRoute(line(48.5, 10, 48.5, 130), [box])).length, 0); // left of the west edge
});

test('(c) line through the centre: exactly one warn per segment', () => {
  const w = passes(validateRoute(line(20, 55, 100, 55), [box]));
  assert.equal(w.length, 1); assert.equal(w[0].severity, 'warn'); assert.equal(w[0].seg, 0);
  // two segments, both through the box: one warn each (segment 0 and segment 1)
  const r = line(20, 55, 100, 55); addLine(r, 20, 55);
  const w2 = passes(validateRoute(r, [box]));
  assert.equal(w2.length, 2); assert.deepEqual(w2.map(i => i.seg).sort(), [0, 1]);
});

test('(d) segment clipping a corner with a ~2.8 in chord is flagged', () => {
  // line x + y = 102 enters box [50,60]^2 near corner (50,50) and exits through the far sides
  assert.equal(passes(validateRoute(line(40, 62, 62, 40), [box])).length, 1);
});

test('KNOWN-BUG: corner clip shorter than the 1 in sample step is missed', { todo: 'validateRoute samples every 1 in; a ~0.7 in corner chord can fall between samples' }, () => {
  assert.equal(passes(validateRoute(line(40, 60.5, 60.5, 40), [box])).length, 1);
});
