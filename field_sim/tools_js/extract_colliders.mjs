// Extracts robot-level obstacle geometry from web/data/scene.{json,bin} -> web/data/colliders.json
// Usage: node field_sim/tools_js/extract_colliders.mjs   (any cwd)
// Units: inches, Pedro frame. Geometry is taken at the CAD pose (hive tip state = scene.json groups.*.cad_pose_state),
// as WORLD matrices straight from instances[].m (no pivot offset).
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const dataDir = path.join(here, '..', 'web', 'data');
const scene = JSON.parse(fs.readFileSync(path.join(dataDir, 'scene.json'), 'utf8'));
const bin = fs.readFileSync(path.join(dataDir, 'scene.bin'));
const ab = bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.length);

const Z_CAP = 18;      // robot height: geometry above this does not block
const Z_FLAT = 1.0;    // clipped geometry that never rises above this is flat (rails/tape): drivable
const CATS = {
  perimeter_wall: 'wall', perimeter_pin: 'wall', hive_frame: 'hive_frame', hardware: 'hardware',
  flower: 'flower', artifact_tray: 'tray', hive_red: 'hive', hive_blue: 'hive', decoration: 'decoration',
};

const partTris = {};
function tris(part) {
  if (partTris[part]) return partTris[part];
  const out = [];
  for (const l of ['opaque', 'trans']) {
    const e = scene.parts[part][l]; if (!e) continue;
    out.push(new Float32Array(ab, e.pos, e.count * 3));   // triangle soup xyz (count = vertex count)
  }
  return (partTris[part] = out);
}

// clip polygon to z<=zc
function clipPoly(poly, zc) {
  const o = [];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const ain = a[2] <= zc, bin_ = b[2] <= zc;
    if (ain) o.push(a);
    if (ain !== bin_) { const t = (zc - a[2]) / (b[2] - a[2]); o.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, zc]); }
  }
  return o;
}

// Per instance: list of per-triangle low-slice bounds, split into connected-ish clusters later
function lowSlices(inst) {
  const m = inst.m, res = [];
  for (const f of tris(inst.part)) {
    for (let i = 0; i + 8 < f.length; i += 9) {
      const P = [];
      for (let v = 0; v < 3; v++) {
        const x = f[i + v * 3], y = f[i + v * 3 + 1], z = f[i + v * 3 + 2];
        P.push([m[0] * x + m[1] * y + m[2] * z + m[3], m[4] * x + m[5] * y + m[6] * z + m[7], m[8] * x + m[9] * y + m[10] * z + m[11]]);
      }
      const c = clipPoly(P, Z_CAP);
      if (c.length < 3) continue;
      let b = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9];
      for (const p of c) for (let k = 0; k < 3; k++) { if (p[k] < b[k]) b[k] = p[k]; if (p[k] > b[k + 3]) b[k + 3] = p[k]; }
      res.push(b);
    }
  }
  return res;
}

const boxesIn = (a, b, g = 0.01) => !(a[0] > b[3] + g || b[0] > a[3] + g || a[1] > b[4] + g || b[1] > a[4] + g);
const area = (b) => Math.max(1e-6, (b[3] - b[0]) * (b[4] - b[1]));
const uni = (a, b) => [Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.min(a[2], b[2]), Math.max(a[3], b[3]), Math.max(a[4], b[4]), Math.max(a[5], b[5])];

// Merge boxes: greedy, only when the union is not much bigger than the parts (keeps diagonals tight, long walls whole)
function mergeBoxes(list, slack) {
  let boxes = list.slice(), changed = true;
  while (changed) {
    changed = false;
    outer: for (let i = 0; i < boxes.length; i++) for (let j = i + 1; j < boxes.length; j++) {
      const a = boxes[i], b = boxes[j];
      if (!boxesIn(a, b)) continue;
      const u = uni(a, b);
      // overlap-aware area estimate of the pieces
      const ox = Math.max(0, Math.min(a[3], b[3]) - Math.max(a[0], b[0])), oy = Math.max(0, Math.min(a[4], b[4]) - Math.max(a[1], b[1]));
      const pieces = area(a) + area(b) - ox * oy;
      if (area(u) <= pieces * slack) { boxes[i] = u; boxes.splice(j, 1); changed = true; break outer; }
    }
  }
  return boxes;
}

const raw = {};   // kind -> [bounds]
for (const inst of scene.instances) {
  const kind = CATS[inst.category]; if (!kind || kind === 'decoration' || kind === 'hive') continue;
  const sl = lowSlices(inst);
  if (!sl.length) continue;
  // cluster triangle slices into tight xy boxes (diagonal legs become several small boxes)
  let cl = mergeBoxes(sl.map((b) => b.slice()), 1.15);
  for (const b of cl) {
    if (b[5] < Z_FLAT) continue;
    (raw[kind] = raw[kind] || []).push(b);
  }
}

const rnd = (v) => Math.round(v * 100) / 100;
const boxes = [];
const circles = [];
for (const [kind, list] of Object.entries(raw)) {
  const slack = kind === 'wall' ? 3 : 1.3;
  for (const b of mergeBoxes(list, slack)) {
    const w = b[3] - b[0], d = b[4] - b[1];
    if (w < 0.05 && d < 0.05) continue;
    boxes.push({ minX: rnd(b[0]), minY: rnd(b[1]), maxX: rnd(b[3]), maxY: rnd(b[4]), zMin: rnd(Math.max(b[2], 0)), zMax: rnd(b[5]), kind });
  }
}

// play-area bounds from the glass inner faces / rail inner faces in scene constants (cross-checked with wall geometry)
const gw = scene.constants.field.walls.glass_inner_faces;
const bounds = { minX: gw.x_low, minY: gw.y_low, maxX: gw.x_high, maxY: gw.y_high };

// Drop boxes lying wholly outside (or flush with/behind) the play-area wall faces: walls are handled by `bounds`
const eps = 0.0;
const kept = boxes.filter((b) => {
  if (b.kind === 'wall') return false;                         // walls == bounds
  if (b.kind === 'hardware' && Math.max(b.maxX - b.minX, b.maxY - b.minY) < 3) return false;   // screws/fasteners
  if (b.kind === 'tray') return false;                         // trays sit outside the field (x<0 / x>144)
  return b.maxX > bounds.minX && b.minX < bounds.maxX && b.maxY > bounds.minY && b.minY < bounds.maxY;
});
// Keep wall-mounted flower/hardware pieces only as far as they intrude past the wall plane (clip to the interior)
const final = [];
for (const b of kept) {
  const c = { ...b, minX: rnd(Math.max(b.minX, bounds.minX)), minY: rnd(Math.max(b.minY, bounds.minY)), maxX: rnd(Math.min(b.maxX, bounds.maxX)), maxY: rnd(Math.min(b.maxY, bounds.maxY)) };
  if (c.maxX - c.minX < 0.02 || c.maxY - c.minY < 0.02) continue;
  final.push(c);
}
void eps;

const out = { units: 'inches', frame: 'pedro', hive_tip_state: Object.fromEntries(['red', 'blue'].map((a) => [a, scene.groups['hive_' + a].cad_pose_state])), z_cap: Z_CAP, bounds, boxes: final, circles };
const outPath = path.join(dataDir, 'colliders.json');
fs.writeFileSync(outPath, JSON.stringify(out));

// ---- summary ----
const counts = {};
for (const b of final) counts[b.kind] = (counts[b.kind] || 0) + 1;
console.log('bounds', bounds);
console.log('counts', counts, 'circles', circles.length, 'bytes', fs.statSync(outPath).size);
const hf = final.filter((b) => b.kind === 'hive_frame');
if (hf.length) {
  const u = hf.reduce((a, b) => ({ minX: Math.min(a.minX, b.minX), minY: Math.min(a.minY, b.minY), maxX: Math.max(a.maxX, b.maxX), maxY: Math.max(a.maxY, b.maxY) }), { minX: 1e9, minY: 1e9, maxX: -1e9, maxY: -1e9 });
  console.log('hive footprint (frame obstacles <18in) bbox', u);
  console.log('hive frame boxes:'); for (const b of hf) console.log(' ', JSON.stringify(b));
}
const hv = scene.instances.filter((i) => i.category.startsWith('hive_') && i.category !== 'hive_frame');
console.log('hive body instances:', hv.length, '(world pose at CAD tip state; lowest z of any hive vertex ~30.65 in, so body is above robot height)');
