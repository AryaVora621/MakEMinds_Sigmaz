// cellstack.js - resting positions of balls stacked inside a hive cell (spec Sec 4).
// Pure functions. Inches, Pedro world frame. Imports only ./params.js.
// ASSUMED (see params.js stack): square row/layer packing, no nesting; real balls roll and settle differently.
import { HIVE_PHYS } from './params.js';

const isNectar = (t) => typeof t === 'string' && /nectar/i.test(t);
export function ballRadiusIn(type, params = HIVE_PHYS) {
  return isNectar(type) ? params.ballRadiusIn.nectar : params.ballRadiusIn.pollen;
}
export function ballMassKg(type, params = HIVE_PHYS) {
  return isNectar(type) ? params.ballMassKg.nectar : params.ballMassKg.pollen;
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const norm = (a) => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];

/** Frame of a cell (world coords at the state the cell object belongs to). */
export function cellFrame(cell) {
  const on = cell.opening_normal, aim = cell.aim_point;
  const fc = cell.floor_corners.slice().sort((p, q) => dot(on, sub(q, aim)) - dot(on, sub(p, aim))); // open first
  const open = [fc[0], fc[1]], back = [fc[2], fc[3]];
  const origin = (back[0][0] <= back[1][0] ? back[0] : back[1]).slice();
  const xs = cell.opening_corners.map((p) => p[0]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs);
  const d = sub(mid(open[0], open[1]), mid(back[0], back[1]));
  const floorLen = len(d);
  const uFloor = norm([0, d[1], d[2]]);
  const bw = cell.back_wall_corners;
  const wv = sub(bw[3], bw[0]);
  const wallHeight = len(wv);
  const k = dot(wv, uFloor);
  const nFloor = norm([0, wv[1] - k * uFloor[1], wv[2] - k * uFloor[2]]);
  return { x0, x1, width: x1 - x0, origin, uFloor, nFloor, floorLen, wallHeight };
}

/** Ordered stacking of balls (radii[i], inches). Deterministic, recomputed from scratch. */
export function stackPositions(cell, radii, opts = HIVE_PHYS.stack) {
  const f = cellFrame(cell);
  const nest = opts.rowNestFactor ?? 1.0, rise = opts.layerRise ?? 1.0;
  const eps = 1e-9;
  // pass 1: assign balls to (layer, row) slots
  const items = [];
  let layer = 0, row = 0, base = 0, L = 0, used = 0, rowMax = 0, layerMax = 0, rowN = 0;
  for (const r of radii) {
    if (rowN > 0 && used + 2 * r > f.width + eps) { // close row, open next
      L += 2 * rowMax * nest; row++; used = 0; rowMax = 0; rowN = 0;
    }
    if (rowN === 0 && L > 0 && L + 2 * r > f.floorLen + eps) { // layer full
      base += 2 * layerMax * rise; layer++; row = 0; L = 0; used = 0; rowMax = 0; layerMax = 0;
    }
    items.push({ r, layer, row, L, base, used });
    used += 2 * r; rowN++;
    if (r > rowMax) rowMax = r;
    if (r > layerMax) layerMax = r;
  }
  // pass 2: centre each row, build positions
  const rowUsed = new Map();
  for (const it of items) rowUsed.set(it.layer + ':' + it.row, it.used + 2 * it.r);
  return items.map((it) => {
    const overflow = it.base + 2 * it.r > f.wallHeight + eps;
    const used2 = rowUsed.get(it.layer + ':' + it.row);
    let x = f.x0 + (f.width - used2) / 2 + it.used + it.r;
    x = Math.min(Math.max(x, f.x0 + it.r), f.x1 - it.r);
    const s = it.L + it.r;
    const h = overflow ? Math.max(it.r, f.wallHeight - it.r) : it.base + it.r;
    const u = f.uFloor, n = f.nFloor, o = f.origin;
    return { pos: [x, o[1] + s * u[1] + h * n[1], o[2] + s * u[2] + h * n[2]], row: it.row, layer: it.layer, overflow };
  });
}

/** World position -> hive-body local [ly, lz] measured at angleRad (inverse rotation about +x through pivot). */
export function restLocal(worldPos, pivot, angleRad) {
  const dy = worldPos[1] - pivot[1], dz = worldPos[2] - pivot[2];
  const c = Math.cos(-angleRad), s = Math.sin(-angleRad);
  return [dy * c - dz * s, dy * s + dz * c];
}
