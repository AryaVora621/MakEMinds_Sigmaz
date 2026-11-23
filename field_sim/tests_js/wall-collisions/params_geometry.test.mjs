import test from 'node:test';
import assert from 'node:assert/strict';
import { WALL_PARAMS, makeParams, materialFor, UNITS } from '../../web/js/physics/wall-collisions/params.js';
import { robotPolygon, transformPolygon, boxPolygon, polygonAabb, bodyMass, cross2 } from '../../web/js/physics/wall-collisions/geometry.js';

test('WALL_PARAMS is deep-frozen', () => {
  assert.ok(Object.isFrozen(WALL_PARAMS));
  assert.ok(Object.isFrozen(WALL_PARAMS.robot));
  assert.ok(Object.isFrozen(WALL_PARAMS.materials.default));
  assert.throws(() => { 'use strict'; WALL_PARAMS.robot.halfX = 1; }, TypeError);
});

test('spec constants present with documented values', () => {
  assert.equal(WALL_PARAMS.robot.halfX, 9);
  assert.equal(WALL_PARAMS.robot.massLb, 31.4);
  assert.equal(WALL_PARAMS.solver.substeps, 4);
  assert.equal(WALL_PARAMS.solver.contactHertz, 30);
  assert.equal(WALL_PARAMS.solver.dampingRatio, 10);
  assert.equal(WALL_PARAMS.restitution.threshold, 10);
  assert.equal(WALL_PARAMS.materials.default.mu, 0.62);
  assert.equal(WALL_PARAMS.materials.default.e, 0.08);
  assert.ok(Math.abs(UNITS.N_PER_LB_IN_S2 - 0.45359237 * 0.0254) < 1e-9);
});

test('makeParams returns a mutable merged copy', () => {
  const P = makeParams({ solver: { substeps: 8 }, materials: { default: { e: 0 } } });
  assert.equal(P.solver.substeps, 8);
  assert.equal(P.solver.contactHertz, 30);
  assert.equal(P.materials.default.e, 0);
  P.robot.halfX = 10; // mutable
  assert.equal(WALL_PARAMS.robot.halfX, 9);
});

test('makeParams validates', () => {
  assert.throws(() => makeParams({ materials: { default: { e: 1.5 } } }), RangeError);
  assert.throws(() => makeParams({ solver: { substeps: 0 } }), RangeError);
  assert.throws(() => makeParams({ solver: { substeps: 1.5 } }), RangeError);
  assert.throws(() => makeParams({ robot: { massLb: 0 } }), RangeError);
  assert.throws(() => makeParams({ robot: { chamfer: 9 } }), RangeError);
  assert.throws(() => makeParams({ materials: { default: { mu: -1 } } }), RangeError);
});

test('materialFor falls back for unknown kinds and fills hertz/zeta', () => {
  const P = makeParams({});
  const m = materialFor(P, 'no_such_kind');
  assert.equal(m.mu, 0.62);
  assert.equal(m.e, 0.08);
  assert.equal(m.hertz, P.solver.contactHertz);
  assert.equal(m.zeta, P.solver.dampingRatio);
  assert.equal(materialFor(P, 'hive_frame').mu, 0.62);
});

test('robotPolygon: 4 CCW verts, unit outward normals', () => {
  const p = robotPolygon({ halfX: 9, halfY: 9 });
  assert.equal(p.verts.length, 4);
  let area2 = 0;
  for (let i = 0; i < 4; i++) {
    const a = p.verts[i], b = p.verts[(i + 1) % 4];
    area2 += cross2(a[0], a[1], b[0], b[1]);
    const n = p.normals[i];
    assert.ok(Math.abs(Math.hypot(n[0], n[1]) - 1) < 1e-12);
    const mx = 0.5 * (a[0] + b[0]), my = 0.5 * (a[1] + b[1]);
    assert.ok(n[0] * mx + n[1] * my > 0, 'outward');
  }
  assert.ok(area2 > 0, 'CCW');
  assert.ok(Math.abs(area2 / 2 - 324) < 1e-9);
  assert.ok(Math.abs(p.reach - 9 * Math.SQRT2) < 1e-9);
});

test('robotPolygon: chamfer 2 gives 8 verts; bumper grows extents', () => {
  const p = robotPolygon({ halfX: 9, halfY: 9, chamfer: 2 });
  assert.equal(p.verts.length, 8);
  const q = robotPolygon({ halfX: 9, halfY: 9, bumper: 1 });
  assert.equal(q.hx, 10);
  assert.equal(q.hy, 10);
});

test('transformPolygon rotates and translates', () => {
  const p = robotPolygon({ halfX: 9, halfY: 9 });
  const w = transformPolygon(p, 10, 20, Math.PI / 2);
  // local (9,-9) -> R90 -> (9, 9) -> +centre
  assert.ok(Math.abs(w.verts[0][0] - 19) < 1e-9 && Math.abs(w.verts[0][1] - 29) < 1e-9);
  const a = polygonAabb(w);
  assert.ok(Math.abs(a.minX - 1) < 1e-9 && Math.abs(a.maxY - 29) < 1e-9);
});

test('bodyMass: plate inertia 1695.6 lb*in^2 for defaults', () => {
  const m = bodyMass(makeParams({}));
  assert.equal(m.m, 31.4);
  assert.ok(Math.abs(m.I - 1695.6) < 0.1, `I=${m.I}`);
  assert.ok(Math.abs(m.invM * m.m - 1) < 1e-12 && Math.abs(m.invI * m.I - 1) < 1e-12);
});

test('boxPolygon: CCW from (minX,minY), outward axis normals', () => {
  const b = boxPolygon({ minX: 1, minY: 2, maxX: 5, maxY: 9 });
  assert.deepEqual(b.verts, [[1, 2], [5, 2], [5, 9], [1, 9]]);
  assert.deepEqual(b.normals.map((n) => [n[0] + 0, n[1] + 0]), [[0, -1], [1, 0], [0, 1], [-1, 0]]);
});
