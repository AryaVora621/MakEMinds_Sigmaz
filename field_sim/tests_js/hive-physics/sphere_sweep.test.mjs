// Swept-sphere CCD tests (hive-physics P4). Inches. Analytic expectations, tolerance 1e-9 in t.
import test from 'node:test';
import assert from 'node:assert/strict';
import { sweepSphereTriangle, sphereOverlapsTriangle } from '../../web/js/physics/hive-physics/spheresweep.js';
import { TriMesh } from '../../web/js/hivemesh.js';

// Triangle in plane z = 0: a(0,0,0) b(10,0,0) c(0,10,0); unit normal +z. Edge bits: 1=ab, 2=bc, 4=ca. Vertex bits: 1=a,2=b,4=c.
const A = [0, 0, 0], B = [10, 0, 0], C = [0, 10, 0], N = [0, 0, 1];
const sweep = (p0, d, r, eb = 7, vb = 7) => {
  const out = new Float64Array(6);
  const t = sweepSphereTriangle(p0, d, r, A, B, C, N, eb, vb, out);
  return { t, out };
};
const close = (a, b, tol = 1e-9) => Math.abs(a - b) <= tol;

test('face TOI: sphere falling onto the face touches at centre height r', () => {
  const { t, out } = sweep([2, 2, 5], [0, 0, -10], 1);
  assert.ok(close(t, 0.4), 't ' + t);
  assert.ok(close(out[0], 0) && close(out[1], 0) && close(out[2], 1), 'normal');
  assert.ok(close(out[3], 2) && close(out[4], 2) && close(out[5], 0), 'point');
});

test('face TOI from below reports the downward normal', () => {
  const { t, out } = sweep([2, 2, -5], [0, 0, 10], 1);
  assert.ok(close(t, 0.4));
  assert.ok(close(out[2], -1));
});

test('edge TOI: sphere sliding in the face plane side catches the edge ab at the analytic time', () => {
  // centre at height z = 0.5, moving +y from y = -5; touches the edge when sqrt(y^2 + 0.5^2) = r = 1
  const { t, out } = sweep([5, -5, 0.5], [0, 10, 0], 1);
  const expect = (5 - Math.sqrt(0.75)) / 10;
  assert.ok(close(t, expect), 't ' + t + ' vs ' + expect);
  assert.ok(close(out[0], 0) && close(out[1], -Math.sqrt(0.75)) && close(out[2], 0.5), 'normal ' + [...out]);
  assert.ok(close(out[3], 5) && close(out[4], 0) && close(out[5], 0), 'point');
});

test('vertex TOI: sphere approaching vertex a along -x..+x hits the vertex sphere analytically', () => {
  const { t, out } = sweep([-5, 0, 0.5], [10, 0, 0], 1, 0, 1);   // only vertex a active
  const expect = (5 - Math.sqrt(0.75)) / 10;
  assert.ok(close(t, expect), 't ' + t);
  assert.ok(close(out[0], -Math.sqrt(0.75)) && close(out[2], 0.5));
  assert.ok(close(out[3], 0) && close(out[4], 0) && close(out[5], 0));
});

test('inactive edges and vertices are skipped (no ghost hit)', () => {
  assert.equal(sweep([5, -5, 0.5], [0, 10, 0], 1, 0, 0).t, -1);
  assert.equal(sweep([-5, 0, 0.5], [10, 0, 0], 1, 0, 0).t, -1);
});

test('miss returns -1; too short a move returns -1', () => {
  assert.equal(sweep([50, 50, 5], [0, 0, -10], 1).t, -1);
  assert.equal(sweep([2, 2, 5], [0, 0, -1], 1).t, -1);
});

test('start-overlapping triangle is ignored (discrete contacts own it)', () => {
  assert.equal(sphereOverlapsTriangle([2, 2, 0.5], 1, A, B, C), true);
  assert.equal(sphereOverlapsTriangle([2, 2, 1.5], 1, A, B, C), false);
  assert.equal(sweep([2, 2, 0.5], [0, 0, -3], 1).t, -1);
});

test('TriMesh.sweepSphere catches an edge that the centre segment misses (1.67 in/step)', () => {
  const mesh = new TriMesh([0, 0, 0, 10, 0, 0, 0, 10, 0], new Uint32Array([0, 1, 2]), 2.5);
  const p0 = [-0.5, 2, 1.5], p1 = [-0.5, 2, 1.5 - 1.67];
  assert.equal(mesh.sweepSegment(p0, p1).hit, false, 'centre segment passes outside the triangle');
  const h = mesh.sweepSphere(p0, p1, 1);
  assert.equal(h.hit, true);
  // edge ca along y at x = 0: touches when sqrt(0.5^2 + z^2) = 1 -> z = sqrt(0.75)
  const expect = (1.5 - Math.sqrt(0.75)) / 1.67;
  assert.ok(close(h.t, expect, 1e-9), 't ' + h.t + ' vs ' + expect);
  assert.ok(close(h.normal[0], -0.5, 1e-9) && close(h.normal[2], Math.sqrt(0.75), 1e-9));
  assert.ok(close(h.point[0], 0, 1e-9) && close(h.point[1], 2, 1e-9));
  assert.ok(Array.isArray(h.center) && close(h.center[2], 1.5 - expect * 1.67, 1e-9));
});

test('TriMesh.sweepSphere: no hit -> {hit:false, t:1}; face hit and start-overlap skipped', () => {
  const mesh = new TriMesh([0, 0, 0, 10, 0, 0, 0, 10, 0], new Uint32Array([0, 1, 2]), 2.5);
  const miss = mesh.sweepSphere([40, 40, 40], [40, 40, 38], 1);
  assert.equal(miss.hit, false); assert.equal(miss.t, 1);
  const face = mesh.sweepSphere([2, 2, 5], [2, 2, -5], 1);
  assert.equal(face.hit, true);
  assert.ok(close(face.t, 0.4, 1e-9));
  const ov = mesh.sweepSphere([2, 2, 0.5], [2, 2, -3], 1);
  assert.equal(ov.hit, false);
});
