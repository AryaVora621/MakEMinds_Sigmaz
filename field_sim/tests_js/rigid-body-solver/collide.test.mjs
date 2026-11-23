import test, { beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import {
  resetBodyIds, makeBox, makePolygon, makeCircle, computeMassData, boxInertia, collide, PARAMS, makeRot,
} from '../../web/js/physics/rigid-body-solver/index.js';

const H = 0.2286; // m, 9 in
const OPTS = {
  speculativeDistance: PARAMS.contact.speculativeDistance,
  faceRelTol: PARAMS.collide.faceRelTol,
  faceAbsTolFrac: PARAMS.collide.faceAbsTolFrac,
};
const xf = (x, y, a = 0) => ({ p: { x, y }, q: makeRot(a) });
const boxes = () => [makeBox(H, H), makeBox(H, H)];

beforeEach(() => resetBodyIds());

test('face-to-face overlap of 0.01 m gives 2 points, normal A->B, separation -0.01, points midway', () => {
  const [a, b] = boxes();
  const m = collide(a, xf(0, 0), b, xf(2 * H - 0.01, 0), OPTS);
  assert.equal(m.pointCount, 2);
  assert.ok(Math.abs(m.normal.x - 1) < 1e-9 && Math.abs(m.normal.y) < 1e-9);
  for (const p of m.points) {
    assert.ok(Math.abs(p.separation + 0.01) < 1e-6);
    // A's right face at x = H, B's left face at x = H - 0.01; midway = H - 0.005
    assert.ok(Math.abs(p.point.x - (H - 0.005)) < 1e-6);
  }
  const ys = m.points.map((p) => p.point.y).sort((p, q) => p - q);
  assert.ok(Math.abs(ys[0] + H) < 1e-6 && Math.abs(ys[1] - H) < 1e-6);
});

test('normal flips sign when A and B are swapped', () => {
  const [a, b] = boxes();
  const m = collide(b, xf(2 * H - 0.01, 0), a, xf(0, 0), OPTS);
  assert.equal(m.pointCount, 2);
  assert.ok(m.normal.x < -0.999);
});

test('box rotated 45 degrees corner-first against a face gives 1 point', () => {
  const [a, b] = boxes();
  const diag = H * Math.SQRT2;
  const m = collide(a, xf(0, 0), b, xf(H + diag - 0.01, 0, Math.PI / 4), OPTS);
  assert.equal(m.pointCount, 1);
  assert.ok(m.points[0].separation < 0 && m.points[0].separation > -0.02);
});

test('gap of 0.03 m gives 0 points', () => {
  const [a, b] = boxes();
  assert.equal(collide(a, xf(0, 0), b, xf(2 * H + 0.03, 0), OPTS).pointCount, 0);
});

test('gap of 0.01 m gives 2 speculative points with separation +0.01', () => {
  const [a, b] = boxes();
  const m = collide(a, xf(0, 0), b, xf(2 * H + 0.01, 0), OPTS);
  assert.equal(m.pointCount, 2);
  for (const p of m.points) assert.ok(Math.abs(p.separation - 0.01) < 1e-6);
});

test('feature ids are identical after a 0.001 m translation', () => {
  const [a, b] = boxes();
  const m1 = collide(a, xf(0, 0), b, xf(2 * H - 0.01, 0.002), OPTS);
  const m2 = collide(a, xf(0, 0), b, xf(2 * H - 0.01, 0.003), OPTS);
  assert.equal(m1.pointCount, 2);
  assert.equal(m2.pointCount, 2);
  assert.deepEqual(m1.points.map((p) => p.id).sort(), m2.points.map((p) => p.id).sort());
});

test('polygon vs circle, face region: normal is the face normal', () => {
  const box = makeBox(0.5, 0.5);
  const c = makeCircle(0.1);
  const m = collide(box, xf(0, 0), c, xf(0.5 + 0.1 - 0.01, 0.2), OPTS);
  assert.equal(m.pointCount, 1);
  assert.ok(Math.abs(m.normal.x - 1) < 1e-9 && Math.abs(m.normal.y) < 1e-9);
  assert.ok(Math.abs(m.points[0].separation + 0.01) < 1e-6);
});

test('polygon vs circle, vertex region: normal points from the corner to the centre', () => {
  const box = makeBox(0.5, 0.5);
  const c = makeCircle(0.1);
  const m = collide(box, xf(0, 0), c, xf(0.56, 0.56), OPTS); // corner distance 0.0849 < r = 0.1
  assert.equal(m.pointCount, 1);
  const s = Math.SQRT1_2;
  assert.ok(Math.abs(m.normal.x - s) < 1e-9 && Math.abs(m.normal.y - s) < 1e-9);
  assert.ok(Math.abs(m.points[0].separation - (Math.hypot(0.06, 0.06) - 0.1)) < 1e-9);
});

test('circle vs polygon equals polygon vs circle with the normal negated', () => {
  const box = makeBox(0.5, 0.5);
  const c = makeCircle(0.1);
  const pc = collide(box, xf(0, 0), c, xf(0.59, 0.2), OPTS);
  const cp = collide(c, xf(0.59, 0.2), box, xf(0, 0), OPTS);
  assert.equal(cp.pointCount, 1);
  assert.ok(Math.abs(cp.normal.x + pc.normal.x) < 1e-9 && Math.abs(cp.normal.y + pc.normal.y) < 1e-9);
  assert.ok(Math.abs(cp.points[0].separation - pc.points[0].separation) < 1e-9);
});

test('circle vs circle: normal, separation and midpoint', () => {
  const a = makeCircle(0.2), b = makeCircle(0.1);
  const m = collide(a, xf(0, 0), b, xf(0.29, 0), OPTS);
  assert.equal(m.pointCount, 1);
  assert.ok(Math.abs(m.normal.x - 1) < 1e-12);
  assert.ok(Math.abs(m.points[0].separation + 0.01) < 1e-9);
  assert.ok(Math.abs(m.points[0].point.x - (0.2 - 0.005)) < 1e-9);
});

test('makePolygon builds a CCW convex hull and drops interior and duplicate points', () => {
  const p = makePolygon([[1, 1], [-1, -1], [1, -1], [-1, 1], [0, 0], [0, 0]]);
  assert.equal(p.count, 4);
  let area2 = 0;
  for (let i = 0; i < p.count; i++) {
    const a = p.vertices[i], b = p.vertices[(i + 1) % p.count];
    area2 += a.x * b.y - a.y * b.x;
  }
  assert.ok(area2 > 0, 'CCW winding has positive signed area');
  assert.ok(Math.abs(area2 / 2 - 4) < 1e-9);
  assert.ok(Math.abs(p.centroid.x) < 1e-9 && Math.abs(p.centroid.y) < 1e-9);
  for (const n of p.normals) assert.ok(Math.abs(Math.hypot(n.x, n.y) - 1) < 1e-9);
});

test('makePolygon throws with fewer than 3 distinct points', () => {
  assert.throws(() => makePolygon([[0, 0], [1, 0], [1, 0]]));
});

test('computeMassData on a box matches boxInertia', () => {
  const box = makeBox(0.3, 0.2);
  const md = computeMassData(box, 1);
  assert.ok(Math.abs(md.mass - 0.6 * 0.4) < 1e-9);
  assert.ok(Math.abs(md.inertia - boxInertia(md.mass, 0.3, 0.2)) < 1e-9);
});
