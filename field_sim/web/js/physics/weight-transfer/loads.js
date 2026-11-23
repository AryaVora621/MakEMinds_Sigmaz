// Weight transfer: body acceleration, ZMP and per-wheel normal loads.
// Units: length in inches, time in s, accel in in/s^2 (rad/s^2 for alpha).
// Robot frame: +x forward, +y left, z up. Wheel order FL, FR, BL, BR.
// Provenance: ZMP planar form derived here from the Wikipedia ZMP vector form
// (https://en.wikipedia.org/wiki/Zero_moment_point); consistent with longitudinal
// weight transfer dW = m*a*h/L (https://en.wikipedia.org/wiki/Weight_transfer).
// The four-wheel equal-stiffness plane fit and the 'three'/'tipping' modes are
// derived here (assumed model, not sourced).
import { G, SX, SY, wheelGeometry } from './params.js';

// wheelGeometry allocates; cache per params object (params are plain, treated as immutable
// between calls; a new makeParams() result gets a new cache entry).
const geomCache = new WeakMap();
function geom(params) {
  let g = geomCache.get(params);
  if (!g) {
    g = wheelGeometry(params);
    geomCache.set(params, g);
  }
  return g;
}

/** Body accel from [vf,vl,om] at two times, midpoint centripetal terms. */
export function bodyAccel(prevBodyV, bodyV, dt) {
  if (!(dt > 0)) return { ax: 0, ay: 0, alpha: 0 };
  const vf0 = prevBodyV[0], vl0 = prevBodyV[1], om0 = prevBodyV[2];
  const vf1 = bodyV[0], vl1 = bodyV[1], om1 = bodyV[2];
  const vfm = 0.5 * (vf0 + vf1), vlm = 0.5 * (vl0 + vl1), omm = 0.5 * (om0 + om1);
  return {
    ax: (vf1 - vf0) / dt - vlm * omm,
    ay: (vl1 - vl0) / dt + vfm * omm,
    alpha: (om1 - om0) / dt,
  };
}

/** Zero-moment point on flat ground (angular momentum rate neglected). gEff = G + az. */
export function zmp(cg, ax, ay, az = 0) {
  const gEff = G + az;
  return { x: cg.x - (cg.z * ax) / gEff, y: cg.y - (cg.z * ay) / gEff };
}

function clamp01(v) { return v < 0 ? 0 : v > 1 ? 1 : v; }

/** Normal load per wheel. opts {az=0}. n sums to 1. */
export function wheelLoads(params, cg, ax, ay, opts = {}) {
  const az = opts.az || 0;
  const gEff = G + az;
  const g = geom(params);
  const a = g.a, b = g.b;
  const px = cg.x - (cg.z * ax) / gEff;
  const py = cg.y - (cg.z * ay) / gEff;
  const u = px - g.xc, w = py - g.yc;

  const n = [0, 0, 0, 0];
  let mode = 'four';
  let axis = null;

  const n0 = 0.25 + (SX[0] * u) / (4 * a) + (SY[0] * w) / (4 * b);
  const n1 = 0.25 + (SX[1] * u) / (4 * a) + (SY[1] * w) / (4 * b);
  const n2 = 0.25 + (SX[2] * u) / (4 * a) + (SY[2] * w) / (4 * b);
  const n3 = 0.25 + (SX[3] * u) / (4 * a) + (SY[3] * w) / (4 * b);

  if (n0 >= 0 && n1 >= 0 && n2 >= 0 && n3 >= 0) {
    n[0] = n0; n[1] = n1; n[2] = n2; n[3] = n3;
  } else if (Math.abs(u) <= a && Math.abs(w) <= b) {
    mode = 'three';
    // wheel diagonally opposite the ZMP lifts: sx = -sign(u), sy = -sign(w)
    const su = u >= 0 ? 1 : -1, sw = w >= 0 ? 1 : -1;
    let lift = 0;
    for (let i = 0; i < 4; i++) if (SX[i] === -su && SY[i] === -sw) { lift = i; break; }
    // remaining three indices
    const ids = [0, 0, 0];
    let c = 0;
    for (let i = 0; i < 4; i++) if (i !== lift) ids[c++] = i;
    const x1 = SX[ids[0]] * a, y1 = SY[ids[0]] * b;
    const x2 = SX[ids[1]] * a, y2 = SY[ids[1]] * b;
    const x3 = SX[ids[2]] * a, y3 = SY[ids[2]] * b;
    // Cramer on [1 1 1; x1 x2 x3; y1 y2 y3] * n = [1; u; w]
    const det = (x2 * y3 - x3 * y2) - (x1 * y3 - x3 * y1) + (x1 * y2 - x2 * y1);
    const d1 = (x2 * y3 - x3 * y2) - (u * y3 - x3 * w) + (u * y2 - x2 * w);
    const d2 = (u * y3 - x3 * w) - (x1 * y3 - x3 * y1) + (x1 * w - u * y1);
    const d3 = (x2 * w - u * y2) - (x1 * w - u * y1) + (x1 * y2 - x2 * y1);
    let m1 = d1 / det, m2 = d2 / det, m3 = d3 / det;
    if (m1 < 0 && m1 > -1e-9) m1 = 0;
    if (m2 < 0 && m2 > -1e-9) m2 = 0;
    if (m3 < 0 && m3 > -1e-9) m3 = 0;
    n[ids[0]] = m1; n[ids[1]] = m2; n[ids[2]] = m3;
  } else {
    mode = 'tipping';
    const exn = Math.abs(u) / a, eyn = Math.abs(w) / b;
    if (exn >= eyn) {
      if (u > 0) {
        axis = 'front';
        n[0] = clamp01(0.5 + w / (2 * b)); n[1] = 1 - n[0];
      } else {
        axis = 'rear';
        n[2] = clamp01(0.5 + w / (2 * b)); n[3] = 1 - n[2];
      }
    } else if (w > 0) {
      axis = 'left';
      n[0] = clamp01(0.5 + u / (2 * a)); n[2] = 1 - n[0];
    } else {
      axis = 'right';
      n[1] = clamp01(0.5 + u / (2 * a)); n[3] = 1 - n[1];
    }
  }

  const wScale = (cg.massLb * gEff) / G; // lbf of total supported weight
  const lbf = [n[0] * wScale, n[1] * wScale, n[2] * wScale, n[3] * wScale];
  const contacts = [n[0] > 0, n[1] > 0, n[2] > 0, n[3] > 0];
  const margin = Math.min(a - Math.abs(u), b - Math.abs(w));
  return { n, lbf, contacts, mode, axis, margin, zmp: { x: px, y: py }, gEff };
}

/** Static tip-over accelerations (magnitudes, in/s^2). */
export function tipAccels(params, cg) {
  const g = geom(params);
  const u0 = cg.x - g.xc, w0 = cg.y - g.yc, h = cg.z;
  return {
    tipFwd: (G * (g.a + u0)) / h,
    tipBrake: (G * (g.a - u0)) / h,
    tipLeft: (G * (g.b + w0)) / h,
    tipRight: (G * (g.b - w0)) / h,
  };
}
