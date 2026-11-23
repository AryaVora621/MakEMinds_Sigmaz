// spheresweep.js - continuous collision (swept sphere vs triangle), pure, no DOM. Units: inches (any consistent unit).
// Equations are the standard ray-vs-offset-plane / ray-vs-cylinder / ray-vs-sphere derivations (Ericson, Real-Time
// Collision Detection 2005, ch.5 "moving sphere vs triangle"); not re-verified against the book this session.
// Sphere centre moves p(t) = p0 + t d, t in [0,1]. Inactive edges/vertices (edgeBits/vertBits, see meshedges.js)
// are skipped so a sphere cannot catch on the internal seams of a flat wall (ghost collisions).
// sweepEps = 1e-9 (barycentric / degenerate tolerance) is an ASSUMED numerical value.
import { closestPtTriFeature } from './meshedges.js';

const EPS = 1e-9;
const _cp = new Float64Array(3);

export function sphereOverlapsTriangle(p, r, a, b, c) {
  closestPtTriFeature(p[0], p[1], p[2], a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], _cp);
  const dx = p[0] - _cp[0], dy = p[1] - _cp[1], dz = p[2] - _cp[2];
  return dx * dx + dy * dy + dz * dz < r * r;
}

// Sphere vs infinite segment-cylinder of edge (a,b): earliest t in [0,1] with axial parameter in [0,1]; -1 if none.
function sweepEdge(p0, d, r, a, b, out, best) {
  const ex = b[0] - a[0], ey = b[1] - a[1], ez = b[2] - a[2];
  const mx = p0[0] - a[0], my = p0[1] - a[1], mz = p0[2] - a[2];
  const ee = ex * ex + ey * ey + ez * ez; if (ee < 1e-18) return best;
  const dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2], de = d[0] * ex + d[1] * ey + d[2] * ez;
  const me = mx * ex + my * ey + mz * ez, md = mx * d[0] + my * d[1] + mz * d[2], mm = mx * mx + my * my + mz * mz;
  const A = dd * ee - de * de, B = 2 * (ee * md - me * de), C = ee * (mm - r * r) - me * me;
  if (A < 1e-12) return best;
  const disc = B * B - 4 * A * C; if (disc < 0) return best;
  const t = (-B - Math.sqrt(disc)) / (2 * A);
  if (t < 0 || t > 1 || t >= best) return best;
  const u = ((mx + t * d[0]) * ex + (my + t * d[1]) * ey + (mz + t * d[2]) * ez) / ee;
  if (u < 0 || u > 1) return best;
  const qx = a[0] + u * ex, qy = a[1] + u * ey, qz = a[2] + u * ez;
  const nx = p0[0] + t * d[0] - qx, ny = p0[1] + t * d[1] - qy, nz = p0[2] + t * d[2] - qz, l = Math.hypot(nx, ny, nz) || 1;
  out[0] = nx / l; out[1] = ny / l; out[2] = nz / l; out[3] = qx; out[4] = qy; out[5] = qz;
  return t;
}

function sweepVertex(p0, d, r, v, out, best) {
  const mx = p0[0] - v[0], my = p0[1] - v[1], mz = p0[2] - v[2];
  const A = d[0] * d[0] + d[1] * d[1] + d[2] * d[2], B = 2 * (mx * d[0] + my * d[1] + mz * d[2]), C = mx * mx + my * my + mz * mz - r * r;
  if (A < 1e-12) return best;
  const disc = B * B - 4 * A * C; if (disc < 0) return best;
  const t = (-B - Math.sqrt(disc)) / (2 * A);
  if (t < 0 || t > 1 || t >= best) return best;
  const nx = mx + t * d[0], ny = my + t * d[1], nz = mz + t * d[2], l = Math.hypot(nx, ny, nz) || 1;
  out[0] = nx / l; out[1] = ny / l; out[2] = nz / l; out[3] = v[0]; out[4] = v[1]; out[5] = v[2];
  return t;
}

// Earliest time of impact of sphere (p0, radius r, displacement d) with triangle abc (unit face normal n).
// Returns t in [0,1] (out = normal xyz, contact point xyz) or -1 (no hit, or sphere already overlaps the triangle).
export function sweepSphereTriangle(p0, d, r, a, b, c, n, edgeBits, vertBits, out) {
  const s0 = n[0] * (p0[0] - a[0]) + n[1] * (p0[1] - a[1]) + n[2] * (p0[2] - a[2]);
  const abs0 = Math.abs(s0);
  if (abs0 < r && sphereOverlapsTriangle(p0, r, a, b, c)) return -1;   // start overlap: discrete contacts own it
  if (abs0 >= r) {
    const sigma = s0 < 0 ? -1 : 1, nd = n[0] * d[0] + n[1] * d[1] + n[2] * d[2], v = sigma * nd;
    if (v < 0) {
      const tf = (abs0 - r) / (-v);
      if (tf >= 0 && tf <= 1) {
        const qx = p0[0] + tf * d[0] - sigma * r * n[0], qy = p0[1] + tf * d[1] - sigma * r * n[1], qz = p0[2] + tf * d[2] - sigma * r * n[2];
        const e0x = b[0] - a[0], e0y = b[1] - a[1], e0z = b[2] - a[2], e1x = c[0] - a[0], e1y = c[1] - a[1], e1z = c[2] - a[2];
        const wx = qx - a[0], wy = qy - a[1], wz = qz - a[2];
        const d00 = e0x * e0x + e0y * e0y + e0z * e0z, d01 = e0x * e1x + e0y * e1y + e0z * e1z, d11 = e1x * e1x + e1y * e1y + e1z * e1z;
        const d20 = wx * e0x + wy * e0y + wz * e0z, d21 = wx * e1x + wy * e1y + wz * e1z;
        const den = d00 * d11 - d01 * d01;
        if (Math.abs(den) > 1e-18) {
          const bv = (d11 * d20 - d01 * d21) / den, bw = (d00 * d21 - d01 * d20) / den;
          if (bv >= -EPS && bw >= -EPS && bv + bw <= 1 + EPS) {
            out[0] = sigma * n[0]; out[1] = sigma * n[1]; out[2] = sigma * n[2]; out[3] = qx; out[4] = qy; out[5] = qz;
            return tf;
          }
        }
      }
    }
  }
  let best = 2;
  if (edgeBits & 1) best = sweepEdge(p0, d, r, a, b, out, best);
  if (edgeBits & 2) best = sweepEdge(p0, d, r, b, c, out, best);
  if (edgeBits & 4) best = sweepEdge(p0, d, r, c, a, out, best);
  if (vertBits & 1) best = sweepVertex(p0, d, r, a, out, best);
  if (vertBits & 2) best = sweepVertex(p0, d, r, b, out, best);
  if (vertBits & 4) best = sweepVertex(p0, d, r, c, out, best);
  return best <= 1 ? best : -1;
}
