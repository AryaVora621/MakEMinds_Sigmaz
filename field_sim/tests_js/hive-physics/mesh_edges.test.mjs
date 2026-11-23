// Active-edge (anti-ghost) mesh tests (hive-physics P4). Inches; hive LOCAL frame = world - pivot at CAD pose.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { TriMesh, parseHiveCollider } from '../../web/js/hivemesh.js';
import { FEAT, closestPtTriFeature, weldVertices, computeActiveMasks, featureActive } from '../../web/js/physics/hive-physics/meshedges.js';
import { cellFrame } from '../../web/js/physics/hive-physics/cellstack.js';

const scene = JSON.parse(readFileSync(new URL('../../web/data/scene.json', import.meta.url), 'utf8'));
const bin = readFileSync(new URL('../../web/data/hive_collider.bin', import.meta.url));
const raw = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const meshes = { red: new TriMesh(raw.red.verts, raw.red.tris) };

// Quad (0,0)-(10,10) in z = 0 split along the diagonal 0-2.
const QV = [0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0], QT = new Uint32Array([0, 1, 2, 0, 2, 3]);
const out = () => new Float64Array(7 * 4);

test('quad: sphere over the shared diagonal -> one contact with the face normal when activeEdges, two when not', () => {
  const on = new TriMesh(QV, QT, 2.5, { activeEdges: true });
  const off = new TriMesh(QV, QT, 2.5, { activeEdges: false });
  const o1 = out(), o2 = out();
  const n1 = on.sphereContacts(5, 5, 0.5, 1, o1, 4);
  const n2 = off.sphereContacts(5, 5, 0.5, 1, o2, 4);
  assert.equal(n1, 1);
  assert.equal(n2, 2);
  const nrm = [o1[0], o1[1], o1[2]];
  assert.ok(nrm[2] > 0.9999, 'normal ' + nrm);
  assert.ok(Math.abs(o1[3] - 0.5) < 1e-9, 'depth r - 0.5');
});

test('quad: a sphere touching the diagonal from below also gets the face normal (oriented to the centre)', () => {
  const on = new TriMesh(QV, QT, 2.5, { activeEdges: true });
  const o = out();
  assert.equal(on.sphereContacts(5, 5, -0.5, 1, o, 4), 1);
  assert.ok(o[2] < -0.9999);
});

test('single triangle: a boundary edge stays active and yields the edge normal', () => {
  const m = new TriMesh([0, 0, 0, 10, 0, 0, 0, 10, 0], new Uint32Array([0, 1, 2]), 2.5, { activeEdges: true });
  const o = out();
  assert.equal(m.sphereContacts(5, -0.5, 0.3, 1, o, 4), 1);
  const l = Math.hypot(0.5, 0.3);
  assert.ok(Math.abs(o[1] + 0.5 / l) < 1e-9 && Math.abs(o[2] - 0.3 / l) < 1e-9, 'normal ' + [o[0], o[1], o[2]]);
  assert.ok(Math.abs(o[3] - (1 - l)) < 1e-9);
});

test('activeEdges=false is the legacy output and activeEdges default is on', () => {
  const dflt = new TriMesh(QV, QT);
  assert.equal(dflt.opts.activeEdges, true);
  assert.equal(dflt.opts.activeEdgeCos, 0.996195);
  const off = new TriMesh(QV, QT, 2.5, { activeEdges: false });
  assert.equal(off.opts.activeEdges, false);
});

test('computeActiveMasks: coplanar diagonal inactive, boundary edges active, counts', () => {
  const nrm = new Float64Array([0, 0, 1, 0, 0, 1]);
  const r = computeActiveMasks(new Float64Array(QV), QT, nrm, { cosThreshold: 0.996195, weldTol: 1e-4 });
  assert.equal(r.totalEdges, 5);
  assert.equal(r.activeEdges, 4);
  // tri 0 = (0,1,2): edges 01 (boundary), 12 (boundary), 20 (diagonal, inactive)
  assert.equal(r.edgeMask[0], 0b011);
  // tri 1 = (0,2,3): edges 02 (diagonal, inactive), 23 (boundary), 30 (boundary)
  assert.equal(r.edgeMask[1], 0b110);
});

test('computeActiveMasks: a 90 degree fold keeps its crease active', () => {
  const V = new Float64Array([0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10]);   // two tris sharing edge 0-1: one in z=0 plane, one in y=0 plane
  const T = new Uint32Array([0, 1, 2, 0, 1, 3]);
  const nrm = new Float64Array([0, 0, 1, 0, -1, 0]);
  const r = computeActiveMasks(V, T, nrm, {});
  assert.equal(r.totalEdges, 5);
  assert.equal(r.activeEdges, 5);
});

test('weldVertices merges duplicates within tolerance to the lowest index', () => {
  const v = new Float64Array([0, 0, 0, 1, 0, 0, 0, 0, 1e-6, 5, 5, 5]);
  const c = weldVertices(v, 1e-4);
  assert.equal(c[0], 0); assert.equal(c[2], 0); assert.equal(c[1], 1); assert.equal(c[3], 3);
});

test('closestPtTriFeature classifies face, edge and vertex regions', () => {
  const o = new Float64Array(3);
  const f = (x, y, z) => closestPtTriFeature(x, y, z, 0, 0, 0, 10, 0, 0, 0, 10, 0, o);
  assert.equal(f(2, 2, 3), FEAT.FACE);
  assert.equal(f(5, -2, 0), FEAT.E01);
  assert.equal(f(-2, 5, 0), FEAT.E20);
  assert.equal(f(8, 8, 0), FEAT.E12);
  assert.equal(f(-2, -2, 0), FEAT.V0);
  assert.equal(f(15, -2, 0), FEAT.V1);
  assert.equal(f(-2, 15, 0), FEAT.V2);
  assert.equal(featureActive(FEAT.FACE, 0, 0), true);
  assert.equal(featureActive(FEAT.E01, 0, 7), false);
  assert.equal(featureActive(FEAT.E01, 1, 0), true);
  assert.equal(featureActive(FEAT.V2, 0, 4), true);
  assert.equal(featureActive(FEAT.V2, 7, 3), false);
});

test('real red hive mesh: some edges are suppressed but not all', () => {
  const m = meshes.red;
  assert.ok(m.totalEdgeCount > 0);
  assert.ok(m.activeEdgeCount > 0);
  assert.ok(m.activeEdgeCount < m.totalEdgeCount, `active ${m.activeEdgeCount} of ${m.totalEdgeCount}`);
  assert.ok(m.edgeMask instanceof Uint8Array && m.vertMask instanceof Uint8Array);
  assert.equal(m.edgeMask.length, m.nTris);
  console.log(`[report] red mesh: ${m.activeEdgeCount} active / ${m.totalEdgeCount} edges, ${m.nTris} tris`);
});

test('a sphere resting on the up-cell floor gets a normal within 2 deg of the floor normal', () => {
  const g = scene.groups.hive_red, piv = g.pivot;
  const cells = scene.constants.hives.red.states[g.cad_pose_state].cells;
  const up = Object.values(cells).find((c) => c.up);
  const f = cellFrame(up);
  const r = 1.391;
  // centre of the floor, 0.1 in into the floor surface, in the hive local frame (world - pivot)
  const fc = up.floor_corners;
  const mid = [0, 1, 2].map((k) => (fc[0][k] + fc[1][k] + fc[2][k] + fc[3][k]) / 4);
  const c = [mid[0] - piv[0] + f.nFloor[0] * (r - 0.1), mid[1] - piv[1] + f.nFloor[1] * (r - 0.1), mid[2] - piv[2] + f.nFloor[2] * (r - 0.1)];
  for (const opts of [true, false]) {
    const m = meshes.red;
    const saved = m.opts.activeEdges; m.opts.activeEdges = opts;
    const o = new Float64Array(7 * 4);
    const n = m.sphereContacts(c[0], c[1], c[2], r, o, 4);
    m.opts.activeEdges = saved;
    assert.ok(n >= 1, 'floor contact found');
    const dotn = o[0] * f.nFloor[0] + o[1] * f.nFloor[1] + o[2] * f.nFloor[2];
    const ang = Math.acos(Math.min(1, Math.max(-1, dotn))) * 180 / Math.PI;
    assert.ok(ang < 2, `activeEdges=${opts}: normal ${ang.toFixed(3)} deg off floor normal`);
  }
});
