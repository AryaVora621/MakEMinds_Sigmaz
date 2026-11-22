// extract_hive_mesh.mjs - build the hive collision mesh from the real CAD triangles in scene.json/scene.bin.
//   node tools_js/extract_hive_mesh.mjs          (from field_sim/)  -> web/data/hive_collider.bin
//
// What is kept: every hive_red / hive_blue instance whose `sub` is rib (side plates), skin (top/bottom/back glass
// panels), tube or churro (cross members). Fasteners, cable ties, bearings, dampers, brackets, plugs and
// decoration are ignored. (The static `hive_frame` group is not part of the moving hive.)
// Triangles are transformed to world space with the instance matrix (CAD pose) and stored in the hive's LOCAL
// frame = world position minus the pivot (rotation left at the CAD pose; the physics rotates by angle-cadAngle
// about +x). Vertices are welded (1e-3 in), degenerate triangles removed, and in dense parts (>5000 tris, the ribs)
// faces below MIN_AREA sq in are dropped (fillet/hole tessellation; the ball radius is 1.4 in so the gaps are irrelevant).
// Colliders are double-sided; per-triangle normals are derived by hivemesh.js TriMesh when loading.
//
// BINARY FORMAT (little endian, every block 4-byte aligned):
//   char[4] 'HVC1' | u32 nHives
//   per hive: char[8] name (NUL padded) | f32 pivot[3] | u32 nVerts | u32 nTris |
//             f32 verts[nVerts*3] (local frame, inches) | u32 tris[nTris*3] (vertex indices)
import fs from 'node:fs';

const KEEP = new Set(['rib', 'skin', 'tube', 'churro']);
const MIN_AREA = 0.1, DENSE = 5000;
const root = new URL('../web/data/', import.meta.url);
const scene = JSON.parse(fs.readFileSync(new URL('scene.json', root), 'utf8'));
const bin = fs.readFileSync(new URL('scene.bin', root));
const ab = bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength);

const records = [];
const report = [];
for (const name of ['red', 'blue']) {
  const g = scene.groups['hive_' + name], piv = g.pivot;
  const map = new Map(), verts = [], tris = [];
  let dropped = 0, droppedArea = 0, keptArea = 0;
  const vid = (x, y, z) => {
    const k = Math.round(x * 1000) + ',' + Math.round(y * 1000) + ',' + Math.round(z * 1000);
    let i = map.get(k);
    if (i === undefined) { i = verts.length / 3; map.set(k, i); verts.push(x, y, z); }
    return i;
  };
  for (const inst of scene.instances) {
    if (inst.category !== 'hive_' + name || !KEEP.has(inst.sub)) continue;
    const part = scene.parts[inst.part], m = inst.m;
    for (const label of ['opaque', 'trans']) {
      const e = part[label]; if (!e) continue;
      const pos = new Float32Array(ab, e.pos, e.count * 3), W = new Float64Array(e.count * 3);
      for (let i = 0; i < e.count; i++) {
        const x = pos[3 * i], y = pos[3 * i + 1], z = pos[3 * i + 2];
        W[3 * i] = m[0] * x + m[1] * y + m[2] * z + m[3] - piv[0];
        W[3 * i + 1] = m[4] * x + m[5] * y + m[6] * z + m[7] - piv[1];
        W[3 * i + 2] = m[8] * x + m[9] * y + m[10] * z + m[11] - piv[2];
      }
      const dense = e.count / 3 > DENSE;
      for (let t = 0; t + 2 < e.count; t += 3) {
        const a = 3 * t, b = a + 3, c = a + 6;
        const ux = W[b] - W[a], uy = W[b + 1] - W[a + 1], uz = W[b + 2] - W[a + 2];
        const vx = W[c] - W[a], vy = W[c + 1] - W[a + 1], vz = W[c + 2] - W[a + 2];
        const area = 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
        if (dense && area < MIN_AREA) { dropped++; droppedArea += area; continue; }
        const i0 = vid(W[a], W[a + 1], W[a + 2]), i1 = vid(W[b], W[b + 1], W[b + 2]), i2 = vid(W[c], W[c + 1], W[c + 2]);
        if (i0 === i1 || i1 === i2 || i0 === i2 || area < 1e-9) continue;
        tris.push(i0, i1, i2); keptArea += area;
      }
    }
  }
  records.push({ name, piv, verts: Float32Array.from(verts), tris: Uint32Array.from(tris) });
  report.push(`${name}: ${verts.length / 3} verts, ${tris.length / 3} tris (dropped ${dropped} small, area ${droppedArea.toFixed(1)}; kept area ${keptArea.toFixed(0)})`);
}

let size = 8;
for (const r of records) size += 8 + 12 + 8 + r.verts.byteLength + r.tris.byteLength;
const out = Buffer.alloc(size); let o = 0;
out.write('HVC1', o, 'ascii'); o += 4; out.writeUInt32LE(records.length, o); o += 4;
for (const r of records) {
  out.write(r.name, o, 8, 'ascii'); o += 8;
  for (const p of r.piv) { out.writeFloatLE(p, o); o += 4; }
  out.writeUInt32LE(r.verts.length / 3, o); o += 4; out.writeUInt32LE(r.tris.length / 3, o); o += 4;
  Buffer.from(r.verts.buffer).copy(out, o); o += r.verts.byteLength;
  Buffer.from(r.tris.buffer).copy(out, o); o += r.tris.byteLength;
}
fs.writeFileSync(new URL('hive_collider.bin', root), out);
console.log(report.join('\n') + `\nwrote hive_collider.bin (${size} bytes)`);
