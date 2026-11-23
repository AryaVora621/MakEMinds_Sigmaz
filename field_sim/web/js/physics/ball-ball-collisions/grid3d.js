// grid3d.js - 3D hashed uniform-grid broadphase for ball-ball collisions.
// Units: in, s, in/s. Imports only ./params.js (never balls.js).
//
// Fat margin per ball: m_i = |v|*dt + 0.5*g*dt^2 + speculativeDistance (static: speculativeDistance).
// Candidate pair (lower index first): |pi-pj| < ri + rj + mi + mj, excluding static-static pairs
// (sleeping or pinned) and balls with state !== 'free'.
// Grid: exact integer cell key ((ix+1024)*2048+(iy+1024))*2048+(iz+1024) in a Map (no hash collisions;
// cell index clamped to +/-1023, i.e. +/-4092 in at cellSize 4, far outside the field).
// Output order is deterministic: sorted by (index of first, index of second) in the input array.
import { BB_PARAMS } from './params.js';

const KEY_OFF = 1024;
const KEY_DIM = 2048;
const CELL_LIM = 1023;

function isStaticBall(b) {
  return b.sleeping === true || b.pinned === true;
}

/** Fat margins (in). Static balls get specDist. Returns Float64Array (length >= balls.length). */
export function computeMargins(balls, dt, g, specDist, out) {
  const n = balls.length;
  if (!out || out.length < n) out = new Float64Array(Math.max(n, 16));
  const gt = 0.5 * g * dt * dt;
  for (let i = 0; i < n; i++) {
    const b = balls[i];
    if (isStaticBall(b)) { out[i] = specDist; continue; }
    const vx = b.vx || 0, vy = b.vy || 0, vz = b.vz || 0;
    out[i] = Math.sqrt(vx * vx + vy * vy + vz * vz) * dt + gt + specDist;
  }
  return out;
}

export class BallGrid3D {
  constructor(opts = {}) {
    this.cellSize = opts.cellSize !== undefined ? opts.cellSize : BB_PARAMS.cellSize;
    this.fastMargin = opts.fastMargin !== undefined ? opts.fastMargin : BB_PARAMS.fastMargin;
    this.pairs = [];
    this.pairCount = 0;
    this._cells = new Map();
    this._balls = null;
    this._n = 0;
    this._rMax = 0;
    this._mMaxGrid = 0;
    this._fast = [];
    this._keys = new Float64Array(256);
    this._keyCount = 0;
    this._alloc(64);
  }

  _alloc(cap) {
    this._cap = cap;
    this._px = new Float64Array(cap);
    this._py = new Float64Array(cap);
    this._pz = new Float64Array(cap);
    this._rad = new Float64Array(cap);
    this._mar = new Float64Array(cap);
    this._live = new Uint8Array(cap);
    this._stat = new Uint8Array(cap);
    this._next = new Int32Array(cap);
  }

  _cellKey(ix, iy, iz) {
    return ((ix + KEY_OFF) * KEY_DIM + (iy + KEY_OFF)) * KEY_DIM + (iz + KEY_OFF);
  }

  _ci(v) {
    let c = Math.floor(v / this.cellSize);
    if (c > CELL_LIM) c = CELL_LIM; else if (c < -CELL_LIM) c = -CELL_LIM;
    return c;
  }

  get cellCount() { return this._cells.size; }

  clear() {
    this._cells.clear();
    this._fast.length = 0;
    this.pairs.length = 0;
    this.pairCount = 0;
    this._keyCount = 0;
    this._balls = null;
    this._n = 0;
    this._rMax = 0;
    this._mMaxGrid = 0;
  }

  _pushKey(k) {
    if (this._keyCount >= this._keys.length) {
      const nk = new Float64Array(this._keys.length * 2);
      nk.set(this._keys);
      this._keys = nk;
    }
    this._keys[this._keyCount++] = k;
  }

  build(balls, margins) {
    const n = balls.length;
    if (n > this._cap) this._alloc(Math.max(n, this._cap * 2));
    const px = this._px, py = this._py, pz = this._pz, rad = this._rad, mar = this._mar;
    const live = this._live, stat = this._stat, next = this._next;
    const cells = this._cells;
    const fm = this.fastMargin;
    cells.clear();
    const fast = this._fast;
    fast.length = 0;
    this._balls = balls;
    this._n = n;
    this._keyCount = 0;

    let rMax = 0, mMaxGrid = 0;
    for (let i = 0; i < n; i++) {
      const b = balls[i];
      if (b.state !== 'free') { live[i] = 0; continue; }
      live[i] = 1;
      px[i] = b.x; py[i] = b.y; pz[i] = b.z;
      rad[i] = b.r;
      const m = margins[i];
      mar[i] = m;
      stat[i] = isStaticBall(b) ? 1 : 0;
      if (b.r > rMax) rMax = b.r;
      if (m > fm) { fast.push(i); continue; }
      if (m > mMaxGrid) mMaxGrid = m;
    }
    this._rMax = rMax;
    this._mMaxGrid = mMaxGrid;

    // Insert non-fast live balls (descending so each cell chain is ascending in index).
    for (let i = n - 1; i >= 0; i--) {
      if (!live[i] || mar[i] > fm) continue;
      const key = this._cellKey(this._ci(px[i]), this._ci(py[i]), this._ci(pz[i]));
      const head = cells.get(key);
      next[i] = head === undefined ? -1 : head;
      cells.set(key, i);
    }

    // Grid balls vs grid balls (j > i only, so each pair is found once).
    const cs = this.cellSize;
    for (let i = 0; i < n; i++) {
      if (!live[i] || mar[i] > fm) continue;
      const reach = rad[i] + mar[i] + rMax + mMaxGrid;
      const range = Math.ceil(reach / cs);
      const cx = this._ci(px[i]), cy = this._ci(py[i]), cz = this._ci(pz[i]);
      const si = stat[i];
      for (let ax = cx - range; ax <= cx + range; ax++) {
        if (ax < -CELL_LIM || ax > CELL_LIM) continue;
        for (let ay = cy - range; ay <= cy + range; ay++) {
          if (ay < -CELL_LIM || ay > CELL_LIM) continue;
          for (let az = cz - range; az <= cz + range; az++) {
            if (az < -CELL_LIM || az > CELL_LIM) continue;
            let j = cells.get(this._cellKey(ax, ay, az));
            if (j === undefined) continue;
            for (; j !== -1; j = next[j]) {
              if (j <= i) continue;
              if (si === 1 && stat[j] === 1) continue;
              const dx = px[i] - px[j], dy = py[i] - py[j], dz = pz[i] - pz[j];
              const lim = rad[i] + rad[j] + mar[i] + mar[j];
              if (dx * dx + dy * dy + dz * dz < lim * lim) this._pushKey(i * n + j);
            }
          }
        }
      }
    }

    // Fast balls vs everything (brute force). Fast-fast pairs only from the lower index.
    for (let f = 0; f < fast.length; f++) {
      const i = fast[f];
      for (let j = 0; j < n; j++) {
        if (j === i || !live[j]) continue;
        if (mar[j] > fm && j < i) continue;
        if (stat[i] === 1 && stat[j] === 1) continue;
        const dx = px[i] - px[j], dy = py[i] - py[j], dz = pz[i] - pz[j];
        const lim = rad[i] + rad[j] + mar[i] + mar[j];
        if (dx * dx + dy * dy + dz * dz < lim * lim) {
          this._pushKey(i < j ? i * n + j : j * n + i);
        }
      }
    }

    // Sort keys (numeric typed-array sort), decode; drop duplicates (only possible via clamped cells).
    const kc = this._keyCount;
    const ks = this._keys.subarray(0, kc);
    ks.sort();
    const pairs = this.pairs;
    let w = 0, prev = -1;
    for (let k = 0; k < kc; k++) {
      const key = ks[k];
      if (key === prev) continue;
      prev = key;
      const a = Math.floor(key / n);
      const b = key - a * n;
      pairs[w++] = balls[a];
      pairs[w++] = balls[b];
    }
    pairs.length = w;
    this.pairCount = w >> 1;
    return this;
  }

  /** Free balls from the last build whose centre is within radius + r + margin of (x,y,z), in input order. */
  query(x, y, z, radius, out = []) {
    const balls = this._balls;
    if (!balls) return out;
    const px = this._px, py = this._py, pz = this._pz, rad = this._rad, mar = this._mar;
    const next = this._next, cells = this._cells;
    const reach = radius + this._rMax + this._mMaxGrid;
    const range = Math.ceil(reach / this.cellSize);
    const cx = this._ci(x), cy = this._ci(y), cz = this._ci(z);
    const found = [];
    for (let ax = cx - range; ax <= cx + range; ax++) {
      if (ax < -CELL_LIM || ax > CELL_LIM) continue;
      for (let ay = cy - range; ay <= cy + range; ay++) {
        if (ay < -CELL_LIM || ay > CELL_LIM) continue;
        for (let az = cz - range; az <= cz + range; az++) {
          if (az < -CELL_LIM || az > CELL_LIM) continue;
          let j = cells.get(this._cellKey(ax, ay, az));
          if (j === undefined) continue;
          for (; j !== -1; j = next[j]) {
            const dx = px[j] - x, dy = py[j] - y, dz = pz[j] - z;
            const lim = radius + rad[j] + mar[j];
            if (dx * dx + dy * dy + dz * dz < lim * lim) found.push(j);
          }
        }
      }
    }
    const fast = this._fast;
    for (let f = 0; f < fast.length; f++) {
      const j = fast[f];
      const dx = px[j] - x, dy = py[j] - y, dz = pz[j] - z;
      const lim = radius + rad[j] + mar[j];
      if (dx * dx + dy * dy + dz * dz < lim * lim) found.push(j);
    }
    found.sort((a, b) => a - b);
    let prev = -1;
    for (let k = 0; k < found.length; k++) {
      if (found[k] === prev) continue;
      prev = found[k];
      out.push(balls[prev]);
    }
    return out;
  }
}
