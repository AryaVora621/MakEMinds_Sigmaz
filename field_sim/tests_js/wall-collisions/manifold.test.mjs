import test from 'node:test';
import assert from 'node:assert/strict';
import { makeParams } from '../../web/js/physics/wall-collisions/params.js';
import { robotPolygon, transformPolygon } from '../../web/js/physics/wall-collisions/geometry.js';
import {
  prepareObstacles, collidePolygonBox, collidePolygonCircle, collidePolygonBounds, collideRobot, maxPenetration
} from '../../web/js/physics/wall-collisions/manifold.js';

const P = makeParams({});
const poly = robotPolygon(P.robot);
const MARGIN = P.solver.speculativeDistance; // 0.7874 in
const world = (x, y, h) => transformPolygon(poly, x, y, h);
const BOUNDS = { minX: 0, minY: 0, maxX: 144, maxY: 144 };
const prep = (boxes = [], circles = []) => prepareObstacles({ bounds: BOUNDS, boxes, circles }, P);
const near = (a, b, t = 1e-6) => Math.abs(a - b) < t;

test('face-on contact with +x bounds: 2 points, normal (-1,0)', () => {
  const ms = collidePolygonBounds(world(135.3, 72, 0), BOUNDS, MARGIN);
  assert.equal(ms.length, 1);
  const m = ms[0];
  assert.equal(m.key, 'bounds:1');
  assert.equal(m.kind, 'bounds');
  assert.equal(m.points.length, 2);
  assert.ok(near(m.normal[0], -1) && near(m.normal[1], 0));
  for (const pt of m.points) assert.ok(near(pt.s, -0.3, 1e-9), `s=${pt.s}`);
});

test('robot far from walls: no bounds manifold', () => {
  assert.equal(collidePolygonBounds(world(72, 72, 0), BOUNDS, MARGIN).length, 0);
});

test('robot rotated 30 deg against a box face: 1 point', () => {
  const box = { minX: 50, minY: 0, maxX: 70, maxY: 100, kind: 'hive_frame' };
  const pb = prep([box]).boxes[0];
  const reach = 9 * (Math.cos(Math.PI / 6) + Math.sin(Math.PI / 6));
  const m = collidePolygonBox(world(50 - reach + 0.2, 50, Math.PI / 6), pb, MARGIN, P);
  assert.ok(m);
  assert.equal(m.points.length, 1);
  assert.ok(near(Math.hypot(m.normal[0], m.normal[1]), 1, 1e-9));
  assert.ok(m.points[0].s < 0);
});

test('robot face vs box face overlap: 2 points with s < 0, normal (-1,0)', () => {
  const box = { minX: 50, minY: 0, maxX: 70, maxY: 100 };
  const pb = prep([box]).boxes[0];
  const m = collidePolygonBox(world(50 - 9 + 0.3, 50, 0), pb, MARGIN, P);
  assert.ok(m);
  assert.equal(m.points.length, 2);
  assert.ok(near(m.normal[0], -1, 1e-6) && near(m.normal[1], 0, 1e-6));
  for (const pt of m.points) assert.ok(pt.s < 0, `s=${pt.s}`);
});

test('separated beyond the margin gives null', () => {
  const box = { minX: 50, minY: 0, maxX: 70, maxY: 100 };
  const pb = prep([box]).boxes[0];
  assert.equal(collidePolygonBox(world(50 - 9 - 5, 50, 0), pb, MARGIN, P), null);
  assert.equal(collideRobot(world(50 - 9 - 5, 50, 0), prep([box]), MARGIN, P).length, 0);
});

test('circle: 1 point, normal points toward the robot centre', () => {
  const circ = { x: 60, y: 50, r: 5 };
  const w = world(46.2, 50, 0);
  const m = collidePolygonCircle(w, circ, MARGIN);
  assert.ok(m);
  assert.equal(m.points.length, 1);
  const n = m.normal;
  assert.ok(near(Math.hypot(n[0], n[1]), 1, 1e-9));
  const p = m.points[0].p;
  assert.ok(n[0] * (w.cx - p[0]) + n[1] * (w.cy - p[1]) > 0, 'normal toward robot');
  assert.ok(near(n[0], -1, 1e-6));
  assert.ok(m.points[0].s < 0);
});

test('circle far away: null', () => {
  assert.equal(collidePolygonCircle(world(10, 50, 0), { x: 60, y: 50, r: 5 }, MARGIN), null);
});

test('point ids are stable under 1e-4 in perturbations', () => {
  const box = { minX: 50, minY: 0, maxX: 70, maxY: 100 };
  const pb = prep([box]).boxes[0];
  const a = collidePolygonBox(world(41.3, 50, 0), pb, MARGIN, P);
  const b = collidePolygonBox(world(41.3 + 1e-4, 50 - 1e-4, 1e-5), pb, MARGIN, P);
  assert.deepEqual(a.points.map((p) => p.id), b.points.map((p) => p.id));
  const c = collidePolygonBounds(world(135.3, 72, 0), BOUNDS, MARGIN)[0];
  const d = collidePolygonBounds(world(135.3 + 1e-4, 72 + 1e-4, 0), BOUNDS, MARGIN)[0];
  assert.deepEqual(c.points.map((p) => p.id), d.points.map((p) => p.id));
});

test('rLocal is the anchor in the robot frame', () => {
  const h = 0.7, x = 135, y = 70;
  const ms = collideRobot(world(x, y, h), prep(), MARGIN, P);
  assert.ok(ms.length > 0);
  for (const m of ms) for (const pt of m.points) {
    const dx = pt.p[0] - x, dy = pt.p[1] - y, c = Math.cos(h), s = Math.sin(h);
    assert.ok(near(pt.rLocal[0], c * dx + s * dy, 1e-9));
    assert.ok(near(pt.rLocal[1], -s * dx + c * dy, 1e-9));
  }
});

test('prepareObstacles flags internal face and corners on adjacent boxes, no input mutation', () => {
  const A = { minX: 20, minY: 60, maxX: 30, maxY: 70 };
  const B = { minX: 30, minY: 60, maxX: 40, maxY: 70 };
  const input = { bounds: BOUNDS, boxes: [A, B], circles: [] };
  const pr = prepareObstacles(input, P);
  assert.equal(input.boxes[0].poly, undefined);
  // A: faces 0 bottom,1 right,2 top,3 left. Right face (x=30) is shared with B.
  assert.equal(pr.boxes[0].internalFace[1], true);
  assert.equal(pr.boxes[0].internalFace[3], false);
  assert.equal(pr.boxes[0].internalFace[0], false);
  // A's right-side corners 1 (30,60) and 2 (30,70) lie on B's boundary.
  assert.equal(pr.boxes[0].internalCorner[1], true);
  assert.equal(pr.boxes[0].internalCorner[2], true);
  assert.equal(pr.boxes[0].internalCorner[0], false);
  // B's left face is internal.
  assert.equal(pr.boxes[1].internalFace[3], true);
  assert.equal(pr.boxes[1].internalFace[1], false);
});

test('collideRobot orders bounds, boxes, circles and uses keys', () => {
  const box = { minX: 120, minY: 60, maxX: 130, maxY: 80 };
  const circ = { x: 138, y: 100, r: 3 };
  const ms = collideRobot(world(135, 80, 0), prep([box], [circ]), MARGIN, P);
  const keys = ms.map((m) => m.key);
  assert.ok(keys.length >= 2);
  const rank = (k) => (k.startsWith('bounds') ? 0 : k.startsWith('box') ? 1 : 2);
  const ranks = keys.map(rank);
  assert.deepEqual(ranks, [...ranks].sort((a, b) => a - b));
  assert.ok(keys.includes('bounds:1'));
});

test('maxPenetration', () => {
  assert.equal(maxPenetration([]), 0);
  assert.equal(maxPenetration([{ points: [{ s: 0.4 }, { s: 0.1 }] }]), 0);
  assert.ok(near(maxPenetration([{ points: [{ s: -0.3 }, { s: -0.7 }] }, { points: [{ s: -0.1 }] }]), 0.7));
});

test('degenerate: no NaN with robot centred exactly on a circle centre', () => {
  const m = collidePolygonCircle(world(60, 50, 0), { x: 60, y: 50, r: 2 }, MARGIN);
  if (m) {
    assert.ok(m.normal.every(Number.isFinite));
    assert.ok(m.points.every((p) => Number.isFinite(p.s) && p.p.every(Number.isFinite)));
  }
});

// Round-2 regression: robot sliding up the x = 60 faces of two stacked boxes. Its +y face just crossed the
// seam y = 70 (by 0.005 in) while pressing 0.01 in into the x face. Before the fix the robot's +y face won
// SAT against box B (sep -0.005 > -0.01) and produced the ghost normal (0,-1) on B's internal bottom face.
test('seam: robot face against an internal box face yields the box x-face normal, not a ghost', () => {
  const A = { minX: 60, minY: 30, maxX: 80, maxY: 70, kind: 'hive_frame' };
  const B = { minX: 60, minY: 70, maxX: 80, maxY: 110, kind: 'hive_frame' };
  const pr = prep([A, B]);
  const yTop = 70.005; // robot top face y (in)
  const ms = collideRobot(world(60 + 0.01 - 9, yTop - 9, 0), pr, MARGIN, P);
  for (const m of ms) {
    assert.ok(Math.abs(m.normal[1]) <= 1e-9, `ghost normal ${m.normal} key ${m.key}`);
    for (const pt of m.points) assert.ok(Number.isFinite(pt.s));
  }
  const mb = ms.find((m) => m.key === 'box:1');
  assert.ok(mb, 'box B still reports its x-face contact');
  assert.ok(near(mb.normal[0], -1));
});
