// cellstack + params tests (hive-physics P1). Geometry in inches; tolerances in inches.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { HIVE_PHYS, hivePhysParams } from '../../web/js/physics/hive-physics/params.js';
import { cellFrame, stackPositions, ballRadiusIn, ballMassKg, restLocal } from '../../web/js/physics/hive-physics/cellstack.js';

const scene = JSON.parse(readFileSync(new URL('../../web/data/scene.json', import.meta.url), 'utf8'));
const states = scene.constants.hives.red.states;
const cellA = states.audience_up.cells.audience;
const cellF = states.far_up.cells.far;
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

test('cellFrame: unit, orthogonal axes; sensible dimensions', () => {
  for (const cell of [cellA, cellF]) {
    const f = cellFrame(cell);
    assert.ok(Math.abs(len(f.uFloor) - 1) < 1e-9);
    assert.ok(Math.abs(len(f.nFloor) - 1) < 1e-9);
    assert.ok(Math.abs(dot(f.uFloor, f.nFloor)) < 1e-9);
    assert.equal(f.uFloor[0], 0);
    assert.equal(f.nFloor[0], 0);
    assert.ok(Math.abs(f.width - 20.04) < 0.01, 'width ' + f.width);
    assert.ok(f.floorLen > 5 && f.wallHeight > 5);
  }
});

test('cellFrame: uFloor points from back wall to opening; nFloor points into the cell', () => {
  const f = cellFrame(cellA);
  // opening normal of the audience cell is (0,-0.866,0.5); uFloor must have a negative y component (toward opening)
  assert.ok(dot(f.uFloor, cellA.opening_normal) > 0.5);
  assert.ok(f.nFloor[2] > 0, 'nFloor up for the up cell');
});

test('7 pollen fit in row 0 of the red audience cell, 8th opens row 1', () => {
  const r = ballRadiusIn('pollen');
  const p7 = stackPositions(cellA, Array(7).fill(r));
  assert.ok(p7.every((p) => p.row === 0 && p.layer === 0));
  const p8 = stackPositions(cellA, Array(8).fill(r));
  assert.equal(p8[7].row, 1);
});

test('30 mixed balls: no overlap, inside side walls, above floor, in front of back wall', () => {
  const radii = [];
  for (let i = 0; i < 30; i++) radii.push(i % 3 === 0 ? ballRadiusIn('nectar') : ballRadiusIn('pollen'));
  for (const cell of [cellA, cellF]) {
    const f = cellFrame(cell);
    const ps = stackPositions(cell, radii);
    assert.equal(ps.length, 30);
    for (let i = 0; i < ps.length; i++) {
      const r = radii[i], c = ps[i].pos;
      assert.ok(c.every(Number.isFinite));
      assert.ok(c[0] >= f.x0 + r - 1e-6 && c[0] <= f.x1 - r + 1e-6, 'x inside walls');
      const d = [0, c[1] - f.origin[1], c[2] - f.origin[2]];
      const s = dot(d, f.uFloor), h = dot(d, f.nFloor);
      if (!ps[i].overflow) {
        assert.ok(h >= r - 1e-6, 'height above floor ' + h);
        assert.ok(s >= r - 1e-6, 'in front of back wall ' + s);
      }
      for (let j = 0; j < i; j++) {
        if (ps[i].overflow || ps[j].overflow) continue;
        assert.ok(dist(c, ps[j].pos) >= r + radii[j] - 1e-6, `balls ${i},${j} overlap`);
      }
    }
  }
});

test('stackPositions is deterministic and pure', () => {
  const radii = [1.7995, 1.391, 1.391, 1.7995, 1.391, 1.391, 1.391, 1.391, 1.391, 1.391];
  const a = JSON.stringify(stackPositions(cellA, radii));
  const b = JSON.stringify(stackPositions(cellA, radii));
  assert.equal(a, b);
  const snap = JSON.stringify(cellA);
  stackPositions(cellA, radii);
  assert.equal(JSON.stringify(cellA), snap, 'cell not mutated');
});

test('hivePhysParams returns a deep copy and never mutates HIVE_PHYS', () => {
  const before = JSON.stringify(HIVE_PHYS);
  const p = hivePhysParams({ tipModel: 'torque', stack: { rowNestFactor: 0.866 }, inertiaSearchKgM2: [1, 2] });
  assert.equal(p.tipModel, 'torque');
  assert.equal(p.stack.rowNestFactor, 0.866);
  assert.equal(p.stack.layerRise, HIVE_PHYS.stack.layerRise, 'sibling key preserved');
  assert.deepEqual(p.inertiaSearchKgM2, [1, 2]);
  p.ballMassKg.pollen = 99; p.mesh.activeEdges = false;
  assert.equal(JSON.stringify(HIVE_PHYS), before);
  assert.notEqual(hivePhysParams().ballMassKg, HIVE_PHYS.ballMassKg);
});

test('params carry the spec values and a source tag per key', () => {
  assert.equal(HIVE_PHYS.ballRadiusIn.pollen, 1.391);
  assert.equal(HIVE_PHYS.ballRadiusIn.nectar, 1.7995);
  assert.equal(HIVE_PHYS.drainPastCentreDeg, 9.79);
  assert.equal(HIVE_PHYS.mesh.activeEdgeCos, 0.996195);
  assert.equal(HIVE_PHYS.tipModel, 'rule');
  assert.equal(HIVE_PHYS.motionModel, 'dynamic');
  assert.ok(HIVE_PHYS.source && typeof HIVE_PHYS.source === 'object');
  assert.ok(Math.abs(Math.asin(0.17) * 180 / Math.PI - HIVE_PHYS.drainPastCentreDeg) < 0.01);
});

test('type mapping: unknown/undefined -> pollen, nectar-like -> nectar', () => {
  assert.equal(ballRadiusIn(undefined), HIVE_PHYS.ballRadiusIn.pollen);
  assert.equal(ballRadiusIn('banana'), HIVE_PHYS.ballRadiusIn.pollen);
  assert.equal(ballRadiusIn('nectar'), HIVE_PHYS.ballRadiusIn.nectar);
  assert.equal(ballMassKg('big_nectar'), HIVE_PHYS.ballMassKg.nectar);
  assert.equal(ballMassKg('pollen'), HIVE_PHYS.ballMassKg.pollen);
});

test('restLocal inverts the world rotation about the pivot', () => {
  const pivot = [59.25, 72, 43.95], ang = -30 * Math.PI / 180;
  const local = [5, -3];
  const y = local[0] * Math.cos(ang) - local[1] * Math.sin(ang), z = local[0] * Math.sin(ang) + local[1] * Math.cos(ang);
  const r = restLocal([0, pivot[1] + y, pivot[2] + z], pivot, ang);
  assert.ok(Math.abs(r[0] - 5) < 1e-9 && Math.abs(r[1] + 3) < 1e-9);
});
