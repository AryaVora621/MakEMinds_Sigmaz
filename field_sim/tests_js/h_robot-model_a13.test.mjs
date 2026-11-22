// Observed: node --test tests_js/h_robot-model_a13.test.mjs -> tests 3, pass 3, fail 0, duration ~122 ms
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../web/vendor/three.module.js';
import { parseGLB, placeCAD, orientationMatrix } from '../web/js/robotglb.js';
// Robot frame is inches (robot_spec.json "units": "in"). TOL = 1 mm in inches.
const SPEC = JSON.parse(await import('node:fs').then(f => f.readFileSync(new URL('../web/data/robot_spec.json', import.meta.url), 'utf8')));
const IN = 1 / 0.0254, MM = 1 / 25.4, TOL = 0.001 * IN;
const near = (a, b, t = TOL) => assert.ok(Math.abs(a - b) <= t, `${a} vs ${b} (tol ${t})`);
const TARGET = [0.381 * IN, 0.375 * IN, 0.25 * IN];   // chassis x, y, z in inches
const BIG = [0.381, 0.375, 0.25], BIG_T = [1, 2, 3];  // metres; node offset from origin

// Box mesh (8 verts, 36 indices) packed as a Z-up GLB. size and t are in file units.
function boxGLB(size, t) {
  const [sx, sy, sz] = size.map(v => v / 2);
  const pos = new Float32Array([-sx,-sy,-sz, sx,-sy,-sz, sx,sy,-sz, -sx,sy,-sz, -sx,-sy,sz, sx,-sy,sz, sx,sy,sz, -sx,sy,sz]);
  const idx = new Uint16Array([0,1,2,0,2,3, 4,6,5,4,7,6, 0,4,5,0,5,1, 1,5,6,1,6,2, 2,6,7,2,7,3, 3,7,4,3,4,0]);
  const bin = new Uint8Array(pos.byteLength + idx.byteLength);
  bin.set(new Uint8Array(pos.buffer), 0); bin.set(new Uint8Array(idx.buffer), pos.byteLength);
  const json = { asset: { version: '2.0' }, scene: 0, scenes: [{ nodes: [0] }], nodes: [{ name: 'Chassis', mesh: 0, translation: t }],
    meshes: [{ primitives: [{ attributes: { POSITION: 0 }, indices: 1 }] }],
    accessors: [{ bufferView: 0, componentType: 5126, count: 8, type: 'VEC3' }, { bufferView: 1, componentType: 5123, count: 36, type: 'SCALAR' }],
    bufferViews: [{ buffer: 0, byteOffset: 0, byteLength: pos.byteLength }, { buffer: 0, byteOffset: pos.byteLength, byteLength: idx.byteLength }],
    buffers: [{ byteLength: bin.byteLength }] };
  const js = new TextEncoder().encode(JSON.stringify(json)), jb = new Uint8Array(Math.ceil(js.length / 4) * 4).fill(0x20); jb.set(js);
  const out = new Uint8Array(28 + jb.length + bin.length), dv = new DataView(out.buffer), o = 20 + jb.length;
  [[0, 0x46546C67], [4, 2], [8, out.length], [12, jb.length], [16, 0x4E4F534A], [o, bin.length], [o + 4, 0x004E4942]].forEach(([at, v]) => dv.setUint32(at, v, true));
  out.set(jb, 20); out.set(bin, o + 8);
  return out;
}
async function place(size, t, opts = { orientation: { upAxis: 'Z', forwardAxis: '+X' } }) {
  const { scene, warnings } = await parseGLB(THREE, boxGLB(size, t));
  return placeCAD(THREE, scene, SPEC, opts, warnings);
}
// footprint and height match the target, bottom on the floor, XY centred
function fit(R) {
  const b = R.bounds;
  near(b.size[0], TARGET[0]); near(b.size[1], TARGET[1]); near(b.size[2], TARGET[2]);
  near(b.min[2], 0); near((b.min[0] + b.max[0]) / 2, 0); near((b.min[1] + b.max[1]) / 2, 0);
}

test('metre Z-up GLB offset by (1,2,3) m fits the 0.381 x 0.375 x 0.25 m frame within 1 mm', async () => {
  const R = await place(BIG, BIG_T);
  assert.equal(R.units, 'm'); near(R.scale, IN, 1e-9); fit(R);
  near(SPEC.chassis.footprint[1][0] - SPEC.chassis.footprint[0][0], TARGET[0]); // spec footprint: same 15.0 x 14.76 in
});

test('same model in mm (x1000, no units option) is detected as mm and fits identically', async () => {
  const A = await place(BIG, BIG_T), B = await place(BIG.map(v => v * 1000), BIG_T.map(v => v * 1000));
  assert.equal(B.units, 'mm'); near(B.scale, MM, 1e-9); fit(B);
  for (let i = 0; i < 3; i++) { near(A.bounds.min[i], B.bounds.min[i], 1e-6); near(A.bounds.max[i], B.bounds.max[i], 1e-6); }
});

test('every mesh has a positive determinant; file Z stays the sim up axis for each forward axis', async () => {
  for (const forwardAxis of ['+X', '-X', '+Y', '-Y']) {
    const orientation = { upAxis: 'Z', forwardAxis }, R = await place(BIG, BIG_T, { orientation });
    const up = new THREE.Vector3(0, 0, 1).transformDirection(orientationMatrix(THREE, orientation));
    near(up.z, 1, 1e-9); near(up.x, 0, 1e-9); near(up.y, 0, 1e-9);
    const dets = []; R.group.traverse(o => { if (o.isMesh) dets.push(o.matrixWorld.determinant()); });
    assert.ok(dets.length >= 1, `${forwardAxis}: no meshes`);
    dets.forEach(d => assert.ok(d > 0, `${forwardAxis}: det ${d}`));
    near(R.bounds.size[2], TARGET[2]); near(R.bounds.min[2], 0);
  }
});
