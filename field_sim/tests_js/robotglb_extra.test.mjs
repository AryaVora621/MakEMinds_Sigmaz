import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from '../web/vendor/three.module.js';
import { loadRobotGLB, animateCADParts } from '../web/js/robotglb.js';
import { buildRobot } from '../web/js/robotmodel.js';

const SPEC = JSON.parse(readFileSync(new URL('../web/data/robot_spec.json', import.meta.url), 'utf8'));
const Z = { orientation: { upAxis: 'Z', forwardAxis: '+X' } };

// Z-up metre CAD: 0.381 x 0.375 x 0.2 chassis, a turret node (origin on its axis), plus a protruding intake box.
function box(sx, sy, sz) {
  const p = [];
  for (let i = 0; i < 8; i++) p.push((i & 1 ? 1 : -1) * sx / 2, (i & 2 ? 1 : -1) * sy / 2, (i & 4 ? 1 : -1) * sz / 2);
  return p;
}
const IDX = [0, 1, 3, 0, 3, 2, 4, 7, 5, 4, 6, 7, 0, 5, 1, 0, 4, 5, 2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3];
function build({ sparse = false, ext = {}, intake = true } = {}) {
  const boxes = [box(0.381, 0.375, 0.2), box(0.1, 0.1, 0.05), box(0.15, 0.3, 0.05)];
  const chunks = [], bufferViews = [], accessors = [], meshes = [];
  let off = 0;
  const addView = u8 => { chunks.push(u8); bufferViews.push({ buffer: 0, byteOffset: off, byteLength: u8.length }); off += u8.length; return bufferViews.length - 1; };
  boxes.forEach((b, i) => {
    const f = new Float32Array(b);
    if (sparse && i === 0) {                          // chassis: no dense data, 8 sparse overrides
      const iv = addView(new Uint8Array(new Uint16Array([0, 1, 2, 3, 4, 5, 6, 7]).buffer)), vv = addView(new Uint8Array(f.buffer));
      accessors.push({ componentType: 5126, count: 8, type: 'VEC3', sparse: { count: 8, indices: { bufferView: iv, componentType: 5123 }, values: { bufferView: vv } } });
    } else accessors.push({ bufferView: addView(new Uint8Array(f.buffer)), componentType: 5126, count: 8, type: 'VEC3' });
    accessors.push({ bufferView: addView(new Uint8Array(new Uint16Array(IDX).buffer)), componentType: 5123, count: 36, type: 'SCALAR' });
    meshes.push({ primitives: [{ attributes: { POSITION: accessors.length - 2 }, indices: accessors.length - 1 }] });
  });
  const bin = new Uint8Array(off); let o = 0; for (const c of chunks) { bin.set(c, o); o += c.length; }
  const nodes = [
    { name: 'Robot', children: [1, 2, ...(intake ? [3] : [])] },
    { name: 'Chassis', mesh: 0 },
    { name: 'Turret Assembly', translation: [0, 0, 0.15], children: [4] },
    { name: 'Intake', mesh: 2, translation: [0.3, 0, 0] },
    { name: 'Turret Plate', mesh: 1 },
  ];
  return { asset: { version: '2.0' }, scene: 0, scenes: [{ nodes: [0] }], nodes, meshes, accessors, bufferViews, buffers: [{ byteLength: bin.length, uri: 'data:application/octet-stream;base64,' + Buffer.from(bin).toString('base64') }], ...ext };
}

test('sparse accessors are decoded (no fallback)', async () => {
  const m = await loadRobotGLB(THREE, build({ sparse: true }), SPEC, { ...Z, units: 'm' });
  assert.ok(m, loadRobotGLB.lastReason);
  const b = new THREE.Box3().setFromObject(m.group.getObjectByName('Chassis#0'));
  assert.ok(Math.abs(b.max.x - b.min.x - 15) < 0.01, 'chassis geometry present and 15 in wide');
});

test('draco/meshopt and unknown required extensions are rejected with a reason', async () => {
  for (const e of [{ extensionsRequired: ['KHR_draco_mesh_compression'] }, { extensionsUsed: ['EXT_meshopt_compression'] }, { extensionsRequired: ['VENDOR_weird'] }]) {
    assert.equal(await loadRobotGLB(THREE, build({ ext: e }), SPEC, {}), null);
    assert.match(loadRobotGLB.lastReason, /draco|meshopt|extension/i);
  }
});

test('fit ignores the protruding intake: chassis is 15 in wide, centred, on the floor', async () => {
  const m = await loadRobotGLB(THREE, build(), SPEC, Z);
  assert.ok(m, loadRobotGLB.lastReason);
  assert.equal(m.units, 'm');
  const b = new THREE.Box3().setFromObject(m.group.getObjectByName('Chassis#0'));
  assert.ok(Math.abs(b.max.x + b.min.x) < 1e-3 && Math.abs(b.max.y + b.min.y) < 1e-3, 'chassis centred');
  assert.ok(Math.abs(b.max.x - b.min.x - 15) < 0.01);
  assert.ok(Math.abs(b.min.z) < 1e-6);
  assert.ok(m.parts.intake && m.parts.turret);
});

test('negative or NaN scale is rejected, unknown units warn', async () => {
  assert.equal(await loadRobotGLB(THREE, build(), SPEC, { scale: -1 }), null);
  assert.equal(await loadRobotGLB(THREE, build(), SPEC, { scale: NaN }), null);
  const m = await loadRobotGLB(THREE, build(), SPEC, { ...Z, units: 'ft' });
  assert.ok(m.warnings.some(w => /unknown units/.test(w)));
});

test('animateCADParts rotates the turret about local z and restores the rest pose', async () => {
  const m = await loadRobotGLB(THREE, build(), SPEC, Z);
  const t = m.parts.turret;
  assert.equal(animateCADParts(THREE, m.parts, { turretDeg: 90 }), 1);
  const dir = new THREE.Vector3(1, 0, 0).applyQuaternion(t.quaternion);
  assert.ok(Math.abs(dir.y - 1) < 1e-6 && Math.abs(dir.x) < 1e-6);
  animateCADParts(THREE, m.parts, { turretDeg: 0 });
  assert.ok(t.quaternion.angleTo(new THREE.Quaternion()) < 1e-9);
  assert.equal(animateCADParts(THREE, null, {}), 0);
});

test('findMovingParts: missing lists only turret/hood/flywheel', async () => {
  const m = await loadRobotGLB(THREE, build({ intake: false }), SPEC, Z);
  assert.deepEqual([...m.parts.missing].sort(), ['flywheel', 'hood']);
  assert.equal(m.parts.intake, null);
});

test('buildRobot().dispose() frees geometries without throwing', () => {
  const r = buildRobot(THREE, SPEC, {});
  let n = 0; r.group.traverse(o => { if (o.geometry) o.geometry.addEventListener('dispose', () => n++); });
  r.dispose();
  assert.ok(n > 10);
});
