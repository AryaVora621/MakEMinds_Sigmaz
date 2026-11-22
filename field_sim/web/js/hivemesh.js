// hivemesh.js - pure triangle-mesh collision helpers for the hive (no DOM, no three.js). Units inches.
// All coordinates handled here are in the hive LOCAL frame = world position minus the hive pivot, with the hive at
// its CAD pose (rotation about +x is applied by the caller).
import { HIVE_TIP } from './hivetip_model.js';
//
//   parseHiveCollider(arrayBuffer) -> { red:{pivot,verts,tris}, blue:{...} }   (format: tools_js/extract_hive_mesh.mjs)
//   new TriMesh(verts, tris, cellSize)   double-sided triangle soup + uniform 3D grid
//     .sphereVsMesh(center, radius, prev?) -> {hit, normal, depth, point}      deepest contact
//     .sphereContacts(cx,cy,cz,r,out,max)  -> n contacts, out = flat [nx,ny,nz,depth,px,py,pz]*n, deepest first
//     .sweepSegment(p0, p1)                -> {hit, t, normal, point} earliest crossing of a triangle by a segment
//   cellSensors({pivot, cells})  -> { audience:sensor, far:sensor }  inner capture volumes (convex prisms, local frame)
//   insideCell(pointLocal, sensor) -> bool
//   buildFallbackMesh({pivot, cells}) -> {verts, tris}  idealised walls from the cell corner data (when no CAD mesh given)

export function parseHiveCollider(buf) {
  if (!buf || buf.byteLength < 8) throw new Error("hive collider: buffer too short");
  const dv = new DataView(buf);
  const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
  if (magic !== 'HVC1') throw new Error('hive collider: bad magic ' + magic);
  const nH = dv.getUint32(4, true), out = {};
  let o = 8;
  for (let h = 0; h < nH; h++) {
    let name = ''; for (let i = 0; i < 8; i++) { const c = dv.getUint8(o + i); if (c) name += String.fromCharCode(c); }
    o += 8;
    const pivot = [dv.getFloat32(o, true), dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true)]; o += 12;
    const nv = dv.getUint32(o, true), nt = dv.getUint32(o + 4, true); o += 8;
    if (nv > 1e7 || nt > 1e7) throw new Error("hive collider: implausible counts for " + name);
    if (o + nv * 12 + nt * 12 > buf.byteLength) throw new Error("hive collider: truncated at hive " + name);
    const verts = new Float32Array(buf.slice(o, o + nv * 12)); o += nv * 12;
    const tris = new Uint32Array(buf.slice(o, o + nt * 12)); o += nt * 12;
    for (let i = 0; i < tris.length; i++) if (!(tris[i] < nv)) throw new Error(`hive collider ${name}: tri index ${tris[i]} >= ${nv}`);
    out[name] = { pivot, verts, tris };
  }
  return out;
}

// closest point on triangle abc to p (Ericson, Real-Time Collision Detection). Writes into out[0..2].
function closestPtTri(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz, out) {
  const abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  if (d1 <= 0 && d2 <= 0) { out[0] = ax; out[1] = ay; out[2] = az; return; }
  const bpx = px - bx, bpy = py - by, bpz = pz - bz;
  const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) { out[0] = bx; out[1] = by; out[2] = bz; return; }
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); out[0] = ax + v * abx; out[1] = ay + v * aby; out[2] = az + v * abz; return; }
  const cpx = px - cx, cpy = py - cy, cpz = pz - cz;
  const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) { out[0] = cx; out[1] = cy; out[2] = cz; return; }
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); out[0] = ax + w * acx; out[1] = ay + w * acy; out[2] = az + w * acz; return; }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    out[0] = bx + w * (cx - bx); out[1] = by + w * (cy - by); out[2] = bz + w * (cz - bz); return;
  }
  const den = 1 / (va + vb + vc), v = vb * den, w = vc * den;
  out[0] = ax + abx * v + acx * w; out[1] = ay + aby * v + acy * w; out[2] = az + abz * v + acz * w;
}

export class TriMesh {
  constructor(verts, tris, cell = 2.5) {
    const nt = (tris.length / 3) | 0;
    for (let i = 0; i < nt * 3; i++) if (!(tris[i] * 3 + 2 < verts.length)) throw new Error(`TriMesh: tri index ${tris[i]} out of range`);
    this.nTris = nt;
    this.v = Float64Array.from(verts);
    this.t = tris;
    this.nrm = new Float64Array(nt * 3);   // unit face normals (double-sided: orientation chosen per query)
    const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
    const lo = new Float64Array(nt * 3), hi = new Float64Array(nt * 3), v = this.v;
    for (let i = 0; i < nt; i++) {
      const a = tris[3 * i] * 3, b = tris[3 * i + 1] * 3, c = tris[3 * i + 2] * 3;
      const ux = v[b] - v[a], uy = v[b + 1] - v[a + 1], uz = v[b + 2] - v[a + 2];
      const wx = v[c] - v[a], wy = v[c + 1] - v[a + 1], wz = v[c + 2] - v[a + 2];
      let nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
      const l = Math.hypot(nx, ny, nz) || 1; this.nrm[3 * i] = nx / l; this.nrm[3 * i + 1] = ny / l; this.nrm[3 * i + 2] = nz / l;
      for (let k = 0; k < 3; k++) {
        const p = v[a + k], q = v[b + k], r = v[c + k];
        lo[3 * i + k] = Math.min(p, q, r); hi[3 * i + k] = Math.max(p, q, r);
        mn[k] = Math.min(mn[k], lo[3 * i + k]); mx[k] = Math.max(mx[k], hi[3 * i + k]);
      }
    }
    if (nt === 0) { mn.fill(0); mx.fill(0); }
    this.min = mn; this.max = mx; this.cell = cell;
    this.n = [0, 1, 2].map(k => Math.max(1, Math.ceil((mx[k] - mn[k]) / cell + 1e-9)));
    const [gx, gy, gz] = this.n, ncell = gx * gy * gz;
    const cnt = new Int32Array(ncell + 1);
    const range = (i, out) => {
      for (let k = 0; k < 3; k++) {
        out[k] = Math.max(0, Math.min(this.n[k] - 1, Math.floor((lo[3 * i + k] - mn[k]) / cell)));
        out[3 + k] = Math.max(0, Math.min(this.n[k] - 1, Math.floor((hi[3 * i + k] - mn[k]) / cell)));
      }
    };
    const rg = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < nt; i++) {
      range(i, rg);
      for (let z = rg[2]; z <= rg[5]; z++) for (let y = rg[1]; y <= rg[4]; y++) for (let x = rg[0]; x <= rg[3]; x++) cnt[1 + x + gx * (y + gy * z)]++;
    }
    for (let i = 0; i < ncell; i++) cnt[i + 1] += cnt[i];
    this.start = cnt; this.items = new Int32Array(cnt[ncell]);
    const fill = cnt.slice(0, ncell);
    for (let i = 0; i < nt; i++) {
      range(i, rg);
      for (let z = rg[2]; z <= rg[5]; z++) for (let y = rg[1]; y <= rg[4]; y++) for (let x = rg[0]; x <= rg[3]; x++) this.items[fill[x + gx * (y + gy * z)]++] = i;
    }
    this.stamp = new Int32Array(nt); this.stampN = 0;
    this._cp = [0, 0, 0]; this._tmp = new Float64Array(7 * 8);
  }

  _nextStamp() {   // Int32 stamp wraps negative after ~2.1e9 queries and dedupe would silently stop
    if (++this.stampN >= 0x7ffffff0) { this.stamp.fill(0); this.stampN = 1; }
    return this.stampN;
  }

  // Flat contacts of sphere (c,r) with the mesh, deepest first, at most `max` (one per triangle, so a flat wall made of
  // several triangles may report several near-identical contacts).
  sphereContacts(cx, cy, cz, r, out, max = 4) {
    const mn = this.min, mx = this.max;
    if (cx + r < mn[0] || cx - r > mx[0] || cy + r < mn[1] || cy - r > mx[1] || cz + r < mn[2] || cz - r > mx[2]) return 0;
    const cell = this.cell, [gx, gy, gz] = this.n;
    const x0 = Math.max(0, Math.floor((cx - r - mn[0]) / cell)), x1 = Math.min(gx - 1, Math.floor((cx + r - mn[0]) / cell));
    const y0 = Math.max(0, Math.floor((cy - r - mn[1]) / cell)), y1 = Math.min(gy - 1, Math.floor((cy + r - mn[1]) / cell));
    const z0 = Math.max(0, Math.floor((cz - r - mn[2]) / cell)), z1 = Math.min(gz - 1, Math.floor((cz + r - mn[2]) / cell));
    const v = this.v, t = this.t, items = this.items, start = this.start, st = this.stamp, cp = this._cp, r2 = r * r;
    const sid = this._nextStamp();
    let n = 0;
    for (let z = z0; z <= z1; z++) for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
      const ci = x + gx * (y + gy * z);
      for (let k = start[ci]; k < start[ci + 1]; k++) {
        const i = items[k]; if (st[i] === sid) continue; st[i] = sid;
        const a = t[3 * i] * 3, b = t[3 * i + 1] * 3, c = t[3 * i + 2] * 3;
        closestPtTri(cx, cy, cz, v[a], v[a + 1], v[a + 2], v[b], v[b + 1], v[b + 2], v[c], v[c + 1], v[c + 2], cp);
        const dx = cx - cp[0], dy = cy - cp[1], dz = cz - cp[2], d2 = dx * dx + dy * dy + dz * dz;
        if (!(d2 < r2)) continue;
        const d = Math.sqrt(d2); let nx, ny, nz;
        if (d > 1e-7) { nx = dx / d; ny = dy / d; nz = dz / d; }
        else { nx = this.nrm[3 * i]; ny = this.nrm[3 * i + 1]; nz = this.nrm[3 * i + 2]; }
        const depth = r - d;
        // insert sorted by depth (descending), keep `max`
        let pos = n < max ? n : max - 1;
        if (n >= max && depth <= out[(max - 1) * 7 + 3]) continue;
        while (pos > 0 && out[(pos - 1) * 7 + 3] < depth) { for (let q = 0; q < 7; q++) out[pos * 7 + q] = out[(pos - 1) * 7 + q]; pos--; }
        const o = pos * 7; out[o] = nx; out[o + 1] = ny; out[o + 2] = nz; out[o + 3] = depth; out[o + 4] = cp[0]; out[o + 5] = cp[1]; out[o + 6] = cp[2];
        if (n < max) n++;
      }
    }
    return n;
  }

  // Deepest contact. `prev` (optional [x,y,z]) orients the normal if the centre lies exactly on a face.
  sphereVsMesh(center, radius, prev) {
    const n = this.sphereContacts(center[0], center[1], center[2], radius, this._tmp, 1);
    if (!n) return { hit: false, normal: [0, 0, 0], depth: 0, point: [0, 0, 0] };
    const o = this._tmp;
    let nx = o[0], ny = o[1], nz = o[2];
    if (prev && Math.hypot(center[0] - o[4], center[1] - o[5], center[2] - o[6]) < 1e-7) {
      if (nx * (prev[0] - center[0]) + ny * (prev[1] - center[1]) + nz * (prev[2] - center[2]) < 0) { nx = -nx; ny = -ny; nz = -nz; }
    }
    return { hit: true, normal: [nx, ny, nz], depth: o[3], point: [o[4], o[5], o[6]] };
  }

  // Earliest intersection of segment p0->p1 with any triangle (two-sided). normal faces p0.
  sweepSegment(p0, p1) {
    const mn = this.min, mx = this.max, cell = this.cell, [gx, gy, gz] = this.n;
    const lox = Math.min(p0[0], p1[0]), hix = Math.max(p0[0], p1[0]), loy = Math.min(p0[1], p1[1]), hiy = Math.max(p0[1], p1[1]);
    const loz = Math.min(p0[2], p1[2]), hiz = Math.max(p0[2], p1[2]);
    if (hix < mn[0] || lox > mx[0] || hiy < mn[1] || loy > mx[1] || hiz < mn[2] || loz > mx[2]) return { hit: false, t: 1 };
    const x0 = Math.max(0, Math.floor((lox - mn[0]) / cell)), x1 = Math.min(gx - 1, Math.floor((hix - mn[0]) / cell));
    const y0 = Math.max(0, Math.floor((loy - mn[1]) / cell)), y1 = Math.min(gy - 1, Math.floor((hiy - mn[1]) / cell));
    const z0 = Math.max(0, Math.floor((loz - mn[2]) / cell)), z1 = Math.min(gz - 1, Math.floor((hiz - mn[2]) / cell));
    const v = this.v, tr = this.t, items = this.items, start = this.start, st = this.stamp, nrm = this.nrm;
    const dx = p1[0] - p0[0], dy = p1[1] - p0[1], dz = p1[2] - p0[2], sid = this._nextStamp(), eps = 1e-9;
    let best = 2, bi = -1;
    for (let z = z0; z <= z1; z++) for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
      const ci = x + gx * (y + gy * z);
      for (let k = start[ci]; k < start[ci + 1]; k++) {
        const i = items[k]; if (st[i] === sid) continue; st[i] = sid;
        const a = tr[3 * i] * 3, b = tr[3 * i + 1] * 3, c = tr[3 * i + 2] * 3;
        const e1x = v[b] - v[a], e1y = v[b + 1] - v[a + 1], e1z = v[b + 2] - v[a + 2];
        const e2x = v[c] - v[a], e2y = v[c + 1] - v[a + 1], e2z = v[c + 2] - v[a + 2];
        const px = dy * e2z - dz * e2y, py = dz * e2x - dx * e2z, pz = dx * e2y - dy * e2x;
        const det = e1x * px + e1y * py + e1z * pz;
        if (det > -1e-12 && det < 1e-12) continue;
        const inv = 1 / det, tx = p0[0] - v[a], ty = p0[1] - v[a + 1], tz = p0[2] - v[a + 2];
        const u = (tx * px + ty * py + tz * pz) * inv; if (u < -eps || u > 1 + eps) continue;
        const qx = ty * e1z - tz * e1y, qy = tz * e1x - tx * e1z, qz = tx * e1y - ty * e1x;
        const w = (dx * qx + dy * qy + dz * qz) * inv; if (w < -eps || u + w > 1 + eps) continue;
        const tt = (e2x * qx + e2y * qy + e2z * qz) * inv;
        if (tt < 0 || tt > 1 || tt >= best) continue;
        best = tt; bi = i;
      }
    }
    if (bi < 0) return { hit: false, t: 1 };
    let nx = nrm[3 * bi], ny = nrm[3 * bi + 1], nz = nrm[3 * bi + 2];
    if (nx * dx + ny * dy + nz * dz > 0) { nx = -nx; ny = -ny; nz = -nz; }
    return { hit: true, t: best, normal: [nx, ny, nz], point: [p0[0] + dx * best, p0[1] + dy * best, p0[2] + dz * best] };
  }
}

export function pointInPoly(px, py, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
    if ((yi > py) !== (yj > py) && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// Capture sensors. `cells` are the CAD-pose cell objects (absolute world coordinates), pivot is subtracted.
// A sensor is the convex prism (opening bottom A, opening top B, back-wall top D, floor back C in the y-z profile,
// extruded along x), inset by SENSOR_INSET from floor / back / lid / side walls and OPEN_INSET behind the opening plane.
export const SENSOR_INSET = 0.5, OPEN_INSET = 1.0;
export function cellSensors(hive) {
  const piv = hive.pivot, out = {};
  for (const name of Object.keys(hive.cells || {})) {
    const c = hive.cells[name]; if (!c) continue;
    const L = p => [p[0] - piv[0], p[1] - piv[1], p[2] - piv[2]];
    const A = L(c.opening_corners[0]), B = L(c.opening_corners[2]), C = L(c.floor_corners[2]), D = L(c.back_wall_corners[2]);
    const xs = c.opening_corners.map(p => p[0] - piv[0]);
    const prof = [A, B, D, C];
    const cy = (A[1] + B[1] + C[1] + D[1]) / 4, cz = (A[2] + B[2] + C[2] + D[2]) / 4;
    const planes = [];
    for (let i = 0; i < 4; i++) {
      const p = prof[i], q = prof[(i + 1) % 4];
      const ey = q[1] - p[1], ez = q[2] - p[2], l = Math.hypot(ey, ez);
      let ny = -ez / l, nz = ey / l;
      if (ny * (cy - p[1]) + nz * (cz - p[2]) < 0) { ny = -ny; nz = -nz; }
      const inset = i === 0 ? OPEN_INSET : SENSOR_INSET;
      planes.push([ny, nz, -(ny * p[1] + nz * p[2]) - inset]);   // inside when ny*y + nz*z + d >= 0
    }
    const n = c.opening_normal, ap = L(c.aim_point);
    const uy = B[1] - A[1], uz = B[2] - A[2], ul = Math.hypot(uy, uz), u = [uy / ul, uz / ul];
    const op = (c.opening_polygon || c.opening_corners).map(p => [p[0] - piv[0], (p[1] - piv[1] - A[1]) * u[0] + (p[2] - piv[2] - A[2]) * u[1]]);
    out[name] = {
      name, x0: Math.min(...xs) + SENSOR_INSET, x1: Math.max(...xs) - SENSOR_INSET, planes,
      n: [n[0], n[1], n[2]], P: ap, A, u, op,
    };
  }
  return out;
}
export function insideCell(p, s) {
  if (p[0] < s.x0 || p[0] > s.x1) return false;
  for (let i = 0; i < 4; i++) { const q = s.planes[i]; if (q[0] * p[1] + q[1] * p[2] + q[2] < 0) return false; }
  return true;
}
// Does the segment p0->p1 (local frame) cross the opening plane against its normal inside the opening polygon?
export function crossesOpening(p0, p1, s) {
  const n = s.n, P = s.P;
  const d0 = n[0] * (p0[0] - P[0]) + n[1] * (p0[1] - P[1]) + n[2] * (p0[2] - P[2]);
  const d1 = n[0] * (p1[0] - P[0]) + n[1] * (p1[1] - P[1]) + n[2] * (p1[2] - P[2]);
  if (!(d0 > 0 && d1 <= 0)) return false;
  const f = d0 / (d0 - d1);
  const hx = p0[0] + (p1[0] - p0[0]) * f, hy = p0[1] + (p1[1] - p0[1]) * f, hz = p0[2] + (p1[2] - p0[2]) * f;
  return pointInPoly(hx, (hy - s.A[1]) * s.u[0] + (hz - s.A[2]) * s.u[1], s.op);
}

// Idealised mesh from cell corner data: floor, back wall, lid, two side plates per cell (+ the gable shoulders that
// turn the rectangular profile into the pentagonal opening_polygon). Used only when no CAD mesh is supplied.
// Pollen-equivalent tip label for both ball types. Pure, no DOM.
// stats: { upPollen, upNectar } counts (missing/non-finite -> 0). Optional stats.nectarWeight (default HIVE_TIP.nectarWeight 1.75 (INFERRED); old note:
// community-note assumption from hivecycle.js HIVE_PARAMS, UNVERIFIED) and stats.tipTarget (default 8, user-supplied
// fact: 8 pollen tips a hive; the manual does not state it).
// Example: hiveTipLabel({upPollen:5, upNectar:1}) -> 'pollen 5 + nectar 1 = 6.8/8' (5 + 1.75 = 6.75 rounds to 6.8)
export function hiveTipLabel(stats) {
  stats = stats ?? {};
  const num = v => (Number.isFinite(Number(v)) ? Number(v) : 0);
  const pollen = Math.max(0, Math.floor(num(stats.upPollen)));
  const nectar = Math.max(0, Math.floor(num(stats.upNectar)));
  const nw = stats.nectarWeight === undefined ? HIVE_TIP.nectarWeight : num(stats.nectarWeight);
  const target = stats.tipTarget === undefined ? HIVE_TIP.threshold : num(stats.tipTarget);
  const total = pollen * 1 + nectar * nw;
  return `pollen ${pollen} + nectar ${nectar} = ${total.toFixed(1)}/${target}`;
}

export function buildFallbackMesh(hive) {
  const piv = hive.pivot, verts = [], tris = [];
  const V = p => { verts.push(p[0] - piv[0], p[1] - piv[1], p[2] - piv[2]); return verts.length / 3 - 1; };
  const quad = (a, b, c, d) => { const i = [a, b, c, d].map(V); tris.push(i[0], i[1], i[2], i[0], i[2], i[3]); };
  const tri = (a, b, c) => { const i = [a, b, c].map(V); tris.push(i[0], i[1], i[2]); };
  for (const name of Object.keys(hive.cells || {})) {
    const c = hive.cells[name]; if (!c) continue;
    const oc = c.opening_corners, fc = c.floor_corners, bc = c.back_wall_corners;
    quad(fc[0], fc[1], fc[2], fc[3]);          // floor
    quad(bc[0], bc[1], bc[2], bc[3]);          // back wall
    quad(oc[3], oc[2], bc[2], bc[3]);          // lid (opening top edge -> back wall top edge)
    for (const k of [0, 1]) {                  // side plates: opening-bottom, opening-top, back-top, floor-back
      const A = oc[k === 0 ? 0 : 1], B = oc[k === 0 ? 3 : 2], D = bc[k === 0 ? 3 : 2], C = fc[k === 0 ? 3 : 2];
      tri(A, B, D); tri(A, D, C);
    }
    const poly = c.opening_polygon;
    if (poly && poly.length === 5) {           // gable shoulders in the opening plane
      tri(oc[3], poly[4], poly[3]); tri(oc[2], poly[3], poly[2]);
    }
  }
  return { verts: Float32Array.from(verts), tris: Uint32Array.from(tris) };
}

// ---- Tip visual + drop timing (pure, no DOM). All constants are cosmetic, ASSUMED, TODO tune by eye. ----
export const TIP_VISUAL = { dampK: 6, releaseStartFrac: 0.5, staggerS: 0.1 };
const DAMP_NORM = 1 - (1 + TIP_VISUAL.dampK) * Math.exp(-TIP_VISUAL.dampK);
// Critically-damped settle: 0 at u=0, 1 at u=1, monotone (no overshoot), starts with zero slope like a released damper.
export function dampedEase(u) {
  const x = Math.min(1, Math.max(0, Number.isFinite(u) ? u : 0)), k = TIP_VISUAL.dampK;
  return (1 - (1 + k * x) * Math.exp(-k * x)) / DAMP_NORM;
}
// Angle of a tipping hive at time t. releasing=true while the basket is moving (balls may leave the opening).
export function hiveTipVisual({ t, tipStartT, fromDeg, toDeg, tipDurationS }) {
  const el = t - tipStartT;
  if (!(tipDurationS > 0)) {
    const done = el >= 0;
    return { angleDeg: done ? toDeg : fromDeg, progress: done ? 1 : 0, releasing: false };
  }
  if (!(el >= 0)) return { angleDeg: fromDeg, progress: 0, releasing: false };
  if (el >= tipDurationS) return { angleDeg: toDeg, progress: 1, releasing: false };
  const u = el / tipDurationS;
  return { angleDeg: fromDeg + (toDeg - fromDeg) * dampedEase(u), progress: u, releasing: true };
}
// Seconds after tip start at which each of nBalls leaves the cell (ascending). Caller clamps if it wants all inside the tip.
export function dropSchedule({ nBalls, tipDurationS, releaseStartFrac = TIP_VISUAL.releaseStartFrac, staggerS = TIP_VISUAL.staggerS }) {
  const n = Math.max(0, Math.floor(nBalls || 0));
  const f = Math.min(1, Math.max(0, releaseStartFrac)), s = Math.max(0, staggerS);
  return Array.from({ length: n }, (_, i) => f * tipDurationS + i * s);
}
// Which scheduled drops are due between prevT and t (elapsed seconds since tip start): indices in (prevT, t].
export function dropsDue(schedule, prevElapsed, elapsed) {
  const out = [];
  for (let i = 0; i < schedule.length; i++) if (schedule[i] > prevElapsed && schedule[i] <= elapsed) out.push(i);
  return out;
}
