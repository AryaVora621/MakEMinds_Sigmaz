// meshedges.js - triangle feature classification and "active edge" masks for the hive collision mesh (pure, no DOM).
// Units inches (any length unit works; tolerances below are in mesh units = inches).
//
// SOURCES
//  - Active-edge rule: Jolt Physics MeshShape (mActiveEdgeCosThresholdAngle = cos 5 deg = 0.996195): an edge shared
//    by exactly 2 triangles is active iff the dihedral angle exceeds the threshold; boundary edges (1 tri) and
//    non-manifold edges (3+ tris) are active. Medium confidence (read from memory of MeshShape.h/.cpp).
//  - Inactive-feature contacts use the face normal instead of the closest-point normal: Box2D "ghost collisions"
//    https://box2d.org/posts/2020/06/ghost-collisions/ and Bullet btInternalEdgeUtility.cpp.
//  - closestPtTriFeature: Ericson, Real-Time Collision Detection (2005) sec 5.1.5, region logic unchanged.
// LIMITATION: the hive collider is a DOUBLE-SIDED triangle soup with arbitrary winding, so the dihedral test uses
//  |n1.n2| (a fold and its mirror look the same). A concave crease whose angle exceeds the threshold is therefore
//  active (fine); only creases < 5 deg either way are suppressed. Concavity itself cannot be judged without winding.
// weldTol 1e-4 in and the quantised-key welding are ASSUMED numerical choices.

export const FEAT = { FACE: 0, V0: 1, V1: 2, V2: 3, E01: 4, E12: 5, E20: 6 };

// Closest point on triangle abc to p, plus which feature it lies on. Writes out[0..2], returns a FEAT code.
export function closestPtTriFeature(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz, out) {
  const abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  if (d1 <= 0 && d2 <= 0) { out[0] = ax; out[1] = ay; out[2] = az; return FEAT.V0; }
  const bpx = px - bx, bpy = py - by, bpz = pz - bz;
  const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) { out[0] = bx; out[1] = by; out[2] = bz; return FEAT.V1; }
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); out[0] = ax + v * abx; out[1] = ay + v * aby; out[2] = az + v * abz; return FEAT.E01; }
  const cpx = px - cx, cpy = py - cy, cpz = pz - cz;
  const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) { out[0] = cx; out[1] = cy; out[2] = cz; return FEAT.V2; }
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); out[0] = ax + w * acx; out[1] = ay + w * acy; out[2] = az + w * acz; return FEAT.E20; }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    out[0] = bx + w * (cx - bx); out[1] = by + w * (cy - by); out[2] = bz + w * (cz - bz); return FEAT.E12;
  }
  const den = 1 / (va + vb + vc), v = vb * den, w = vc * den;
  out[0] = ax + abx * v + acx * w; out[1] = ay + aby * v + acy * w; out[2] = az + abz * v + acz * w;
  return FEAT.FACE;
}

// vertex index -> canonical (lowest) index of all vertices within ~tol of it (quantised-grid key, tol in mesh units).
export function weldVertices(verts, tol = 1e-4) {
  const nv = (verts.length / 3) | 0, canon = new Int32Array(nv), map = new Map();
  const inv = 1 / (tol > 0 ? tol : 1e-4);
  for (let i = 0; i < nv; i++) {
    const key = Math.round(verts[3 * i] * inv) + ',' + Math.round(verts[3 * i + 1] * inv) + ',' + Math.round(verts[3 * i + 2] * inv);
    const f = map.get(key);
    if (f === undefined) { map.set(key, i); canon[i] = i; } else canon[i] = f;
  }
  return canon;
}

// Per-triangle bitmasks. edgeMask[t]: bit0 E01, bit1 E12, bit2 E20 (1 = active). vertMask[t]: bit0 V0, bit1 V1, bit2 V2.
// Degenerate edges (both ends weld together) stay inactive and are not counted.
export function computeActiveMasks(verts, tris, faceNormals, opts = {}) {
  const cosT = opts.cosThreshold ?? 0.996195, weldTol = opts.weldTol ?? 1e-4;
  const nt = (tris.length / 3) | 0, nv = (verts.length / 3) | 0;
  const canon = weldVertices(verts, weldTol);
  const edgeMask = new Uint8Array(nt), vertMask = new Uint8Array(nt);
  const edges = new Map();   // key -> array of slots (tri*3 + edgeIndex)
  for (let t = 0; t < nt; t++) {
    for (let e = 0; e < 3; e++) {
      const u = canon[tris[3 * t + e]], w = canon[tris[3 * t + (e + 1) % 3]];
      if (u === w) continue;
      const key = u < w ? u * nv + w : w * nv + u;
      const l = edges.get(key);
      if (l) l.push(t * 3 + e); else edges.set(key, [t * 3 + e]);
    }
  }
  const vAct = new Uint8Array(nv);
  let active = 0, total = 0;
  const mark = (slot) => { edgeMask[(slot / 3) | 0] |= 1 << (slot % 3); };
  for (const [key, slots] of edges) {
    total++;
    let act;
    if (slots.length !== 2) act = true;
    else {
      const t1 = (slots[0] / 3) | 0, t2 = (slots[1] / 3) | 0;
      const dot = faceNormals[3 * t1] * faceNormals[3 * t2] + faceNormals[3 * t1 + 1] * faceNormals[3 * t2 + 1] + faceNormals[3 * t1 + 2] * faceNormals[3 * t2 + 2];
      act = Math.abs(dot) < cosT;
    }
    if (!act) continue;
    active++;
    for (const s of slots) mark(s);
    vAct[(key / nv) | 0] = 1; vAct[key % nv] = 1;
  }
  for (let t = 0; t < nt; t++) {
    for (let k = 0; k < 3; k++) if (vAct[canon[tris[3 * t + k]]]) vertMask[t] |= 1 << k;
  }
  return { edgeMask, vertMask, activeEdges: active, totalEdges: total };
}

// Is the closest-point feature a real (active) feature of the mesh? FACE is always active.
export function featureActive(feature, edgeBits, vertBits) {
  switch (feature) {
    case FEAT.FACE: return true;
    case FEAT.V0: return (vertBits & 1) !== 0;
    case FEAT.V1: return (vertBits & 2) !== 0;
    case FEAT.V2: return (vertBits & 4) !== 0;
    case FEAT.E01: return (edgeBits & 1) !== 0;
    case FEAT.E12: return (edgeBits & 2) !== 0;
    case FEAT.E20: return (edgeBits & 4) !== 0;
    default: return true;
  }
}
