// RESULT (node --test tests_js/h_robot-model_a12.test.mjs): tests 4, suites 0, pass 4, fail 0, cancelled 0, skipped 0, todo 0, duration_ms 108.0205
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../web/vendor/three.module.js';
import { parseGLB, readGLBContainer } from '../web/js/robotglb.js';

// Synthetic GLB: three box meshes, Z-up, metre units, one BIN chunk. Chassis is 0.381 x 0.375 x 0.2 m.
const BOXES = [
  { name: 'Chassis', size: [0.381, 0.375, 0.2], t: [0, 0, 0] },
  { name: 'Turret', size: [0.1, 0.1, 0.05], t: [0, 0, 0.15] },
  { name: 'Flywheel', size: [0.08, 0.08, 0.02], t: [0, 0, 0.2] },
];
const QUADS = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]];
const near = (a, b, t = 1e-5) => assert.ok(Math.abs(a - b) <= t, `${a} vs ${b}`);

function boxData(size) {
  const pos = new Float32Array(24), idx = [];
  for (let i = 0; i < 8; i++) {
    const s = [i & 1 ? 1 : -1, i & 2 ? 1 : -1, i & 4 ? 1 : -1];
    for (let k = 0; k < 3; k++) pos[i * 3 + k] = s[k] * size[k] / 2;
  }
  for (const [a, b, c, d] of QUADS) idx.push(a, b, c, a, c, d);
  return { pos, idx: Uint16Array.from(idx) };
}

// Builds a .glb in memory: 12-byte header, JSON chunk padded with spaces, BIN chunk padded to 4 bytes.
function buildGLB() {
  const chunks = [], bufferViews = [], accessors = [], meshes = [], nodes = [];
  let off = 0;
  BOXES.forEach((b, i) => {
    const { pos, idx } = boxData(b.size);
    const pb = new Uint8Array(pos.buffer), ib = new Uint8Array(idx.buffer);
    chunks.push(pb, ib);
    bufferViews.push({ buffer: 0, byteOffset: off, byteLength: pb.length, target: 34962 });
    accessors.push({ bufferView: 2 * i, componentType: 5126, count: 8, type: 'VEC3' });
    off += pb.length;
    bufferViews.push({ buffer: 0, byteOffset: off, byteLength: ib.length, target: 34963 });
    accessors.push({ bufferView: 2 * i + 1, componentType: 5123, count: 36, type: 'SCALAR' });
    off += ib.length;
    meshes.push({ name: b.name, primitives: [{ attributes: { POSITION: 2 * i }, indices: 2 * i + 1, mode: 4 }] });
    nodes.push({ name: b.name, mesh: i, translation: b.t });
  });
  const json = { asset: { version: '2.0' }, scene: 0, scenes: [{ nodes: [0, 1, 2] }], nodes, meshes, accessors, bufferViews, buffers: [{ byteLength: off }] };
  const enc = new TextEncoder().encode(JSON.stringify(json));
  const jsonChunk = new Uint8Array(enc.length + ((4 - enc.length % 4) % 4)).fill(0x20);
  jsonChunk.set(enc);
  const total = 12 + 8 + jsonChunk.length + 8 + off;
  const out = new Uint8Array(total), dv = new DataView(out.buffer);
  dv.setUint32(0, 0x46546C67, true); dv.setUint32(4, 2, true); dv.setUint32(8, total, true);
  dv.setUint32(12, jsonChunk.length, true); dv.setUint32(16, 0x4E4F534A, true); out.set(jsonChunk, 20);
  let p = 28 + jsonChunk.length;
  dv.setUint32(p - 8, off, true); dv.setUint32(p - 4, 0x004E4942, true);
  for (const c of chunks) { out.set(c, p); p += c.length; }
  return out.buffer;
}

test('synthetic GLB container splits into JSON and a 504-byte BIN chunk', () => {
  const { json, bin } = readGLBContainer(buildGLB());
  assert.equal(json.nodes.length, 3);
  assert.equal(bin.byteLength, 3 * (96 + 72));
});

test('parseGLB returns a scene with 3 named nodes, 8 vertices and 36 indices each', async () => {
  const { scene, warnings } = await parseGLB(THREE, buildGLB());
  assert.deepEqual(scene.children.map(c => c.name), ['Chassis', 'Turret', 'Flywheel']);
  for (const node of scene.children) {
    const mesh = node.children.find(o => o.isMesh);
    assert.ok(mesh, node.name + ' has no mesh');
    assert.equal(mesh.geometry.attributes.position.count, 8, node.name + ' vertex count');
    assert.equal(mesh.geometry.index.count, 36, node.name + ' index count');
  }
  assert.deepEqual(warnings, []);
});

test('bounding boxes match the metre-sized boxes (chassis 0.381 x 0.375 x 0.2, Z-up)', async () => {
  const { scene } = await parseGLB(THREE, buildGLB());
  scene.updateMatrixWorld(true);
  const c = new THREE.Box3().setFromObject(scene.children[0]), s = c.getSize(new THREE.Vector3());
  near(s.x, 0.381); near(s.y, 0.375); near(s.z, 0.2);
  const b = new THREE.Box3().setFromObject(scene);
  near(b.min.x, -0.1905); near(b.max.x, 0.1905); near(b.min.y, -0.1875); near(b.max.y, 0.1875);
  near(b.min.z, -0.1); near(b.max.z, 0.21); // flywheel top at 0.2 + 0.01
});

test('truncated, bad-magic, short and overrunning input all reject', async () => {
  const full = new Uint8Array(buildGLB());
  await assert.rejects(parseGLB(THREE, full.slice(0, full.length - 10)), /truncated/);
  await assert.rejects(parseGLB(THREE, full.slice(0, 16)), /too small/);
  await assert.rejects(parseGLB(THREE, new Uint8Array(64).fill(0xab)), /not a glTF/);
  const badMagic = full.slice(); badMagic[0] = 0;
  await assert.rejects(parseGLB(THREE, badMagic), /not a glTF/);
  const overrun = full.slice(); new DataView(overrun.buffer).setUint32(12, 1e6, true);
  await assert.rejects(parseGLB(THREE, overrun), /overruns/);
});
