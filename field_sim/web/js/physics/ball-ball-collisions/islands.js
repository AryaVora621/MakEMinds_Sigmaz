// islands.js - Box2D-style island sleeping for ball-ball collisions v2.
// Units: time s, length in, speed in/s. Imports only ./params.js and ./bodies.js.
// Deterministic: all iteration is in list order, no Math.random.

import { BB_PARAMS } from './params.js';
import { pointSpeed } from './bodies.js';

export class IslandManager {
  constructor(params = BB_PARAMS) {
    this.params = params;
    this._islands = new Map();   // id -> Array of balls
    this._nextId = 0;            // monotonic island id
    this._parent = new Int32Array(0);
    this._rank = new Uint8Array(0);
    this._flags = new Uint8Array(0); // bit0 = supported, bit1 = not ready to sleep
    this.lastAwakeComponents = 0;
  }

  get sleepingIslandCount() { return this._islands.size; }

  members(id) { return this._islands.get(id); }

  islandOf(b) { return b._bbIsl === undefined ? -1 : b._bbIsl; }

  reset() {
    this._islands.clear();
    this._nextId = 0;
    this.lastAwakeComponents = 0;
  }

  _ensureScratch(n) {
    if (this._parent.length < n) {
      const cap = Math.max(n, this._parent.length * 2, 64);
      this._parent = new Int32Array(cap);
      this._rank = new Uint8Array(cap);
      this._flags = new Uint8Array(cap);
    }
  }

  _find(i) {
    const p = this._parent;
    while (p[i] !== i) { p[i] = p[p[i]]; i = p[i]; } // path halving
    return i;
  }

  _union(a, b) {
    let ra = this._find(a), rb = this._find(b);
    if (ra === rb) return;
    const rk = this._rank;
    if (rk[ra] < rk[rb]) { const t = ra; ra = rb; rb = t; }
    this._parent[rb] = ra;
    if (rk[ra] === rk[rb]) rk[ra]++;
  }

  _sleepGroup(members) {
    const id = this._nextId++;
    for (let k = 0; k < members.length; k++) {
      const m = members[k];
      m.sleeping = true;
      m._bbIsl = id;
      m.vx = 0; m.vy = 0; m.vz = 0;
      m.wx = 0; m.wy = 0; m.wz = 0;
    }
    this._islands.set(id, members);
    return id;
  }

  // Sleep-timer update + island formation. Call after statics so b._c is valid.
  // Returns number of islands put to sleep.
  update(list, contacts, n, h) {
    const P = this.params;
    const N = list.length;
    this._ensureScratch(N);
    const parent = this._parent, rank = this._rank, flags = this._flags;
    const sleepLinear = P.sleepLinear, tts = P.timeToSleep, touch = P.touchDistance;

    for (let i = 0; i < N; i++) {
      const b = list[i];
      b._bbIdx = i;
      parent[i] = i; rank[i] = 0; flags[i] = 0;
      if (b.state !== 'free' || b.sleeping === true || b.pinned === true) continue;
      if (pointSpeed(b) < sleepLinear) b._bbT += h; else b._bbT = 0;
      b.sleepT = b._bbT;
      if (b._c === true) flags[i] |= 1;
    }

    // link through touching contacts between awake balls
    for (let k = 0; k < n; k++) {
      const c = contacts[k];
      if (c.s >= touch) continue;
      const a = c.a, bb = c.b;
      if (c.plane !== null && c.plane !== undefined) { // ball touching a static plane: supported, no link
        const ip = a._bbIdx;
        if (ip !== undefined && list[ip] === a) flags[ip] |= 1;
        continue;
      }
      const ia = a._bbIdx, ib = bb._bbIdx;
      if (ia === undefined || ib === undefined || list[ia] !== a || list[ib] !== bb) continue;
      const aLive = a.state === 'free' && a.sleeping !== true;
      const bLive = bb.state === 'free' && bb.sleeping !== true;
      if (!aLive || !bLive) continue;
      const aPin = a.pinned === true, bPin = bb.pinned === true;
      if (aPin && bPin) continue;
      if (aPin) { flags[ib] |= 1; continue; }   // supported by a pinned ball, no link
      if (bPin) { flags[ia] |= 1; continue; }
      this._union(ia, ib);
    }

    // aggregate per root
    for (let i = 0; i < N; i++) {
      const b = list[i];
      if (b.state !== 'free' || b.sleeping === true || b.pinned === true) continue;
      const r = this._find(i);
      if (b._bbT < tts) flags[r] |= 2;
      if ((flags[i] & 1) !== 0) flags[r] |= 1;
    }

    // collect sleeping components in list order
    let groups = null; // Map root -> members
    let awake = 0;
    for (let i = 0; i < N; i++) {
      const b = list[i];
      if (b.state !== 'free' || b.sleeping === true || b.pinned === true) continue;
      const r = this._find(i);
      if (r === i) awake++;
      if ((flags[r] & 3) !== 1) continue;
      if (groups === null) groups = new Map();
      let g = groups.get(r);
      if (g === undefined) { g = []; groups.set(r, g); }
      g.push(b);
    }
    this.lastAwakeComponents = awake;
    if (groups === null) return 0;
    let slept = 0;
    for (const g of groups.values()) { this._sleepGroup(g); slept++; }
    this.lastAwakeComponents = awake - slept;
    return slept;
  }

  // Wake the island containing b (or just b if legacy-sleeping). Returns true if anything woke.
  wakeIsland(b) {
    if (b._bbIsl !== undefined && b._bbIsl >= 0) {
      const arr = this._islands.get(b._bbIsl);
      if (arr !== undefined) {
        for (let k = 0; k < arr.length; k++) {
          const m = arr[k];
          m.sleeping = false; m.sleepT = 0; m._bbT = 0; m._bbIsl = -1;
        }
        this._islands.delete(b._bbIsl);
      }
      b.sleeping = false; b.sleepT = 0; b._bbT = 0; b._bbIsl = -1;
      return true;
    }
    if (b.sleeping === true) {
      b.sleeping = false; b.sleepT = 0; b._bbT = 0; b._bbIsl = -1;
      return true;
    }
    return false;
  }

  // Group already-sleeping free balls into islands by brute-force touching. Returns islands created.
  adoptSleeping(list) {
    const N = list.length;
    this._ensureScratch(N);
    const parent = this._parent, rank = this._rank;
    const touch = this.params.touchDistance;
    const cand = [];
    for (let i = 0; i < N; i++) {
      const b = list[i];
      parent[i] = i; rank[i] = 0;
      if (b.state === 'free' && b.sleeping === true && b.pinned !== true && (b._bbIsl === undefined || b._bbIsl < 0)) cand.push(i);
    }
    for (let p = 0; p < cand.length; p++) {
      const a = list[cand[p]];
      for (let q = p + 1; q < cand.length; q++) {
        const b = list[cand[q]];
        const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        const lim = a.r + b.r + touch;
        if (dx * dx + dy * dy + dz * dz < lim * lim) this._union(cand[p], cand[q]);
      }
    }
    const groups = new Map();
    for (let p = 0; p < cand.length; p++) {
      const r = this._find(cand[p]);
      let g = groups.get(r);
      if (g === undefined) { g = []; groups.set(r, g); }
      g.push(list[cand[p]]);
    }
    let created = 0;
    for (const g of groups.values()) { this._sleepGroup(g); created++; }
    return created;
  }

  // b is leaving the world: drop it from its island, wake the rest and any sleeper near it.
  // Returns number of islands woken.
  remove(b, list) {
    let woken = 0;
    if (b._bbIsl !== undefined && b._bbIsl >= 0) {
      const id = b._bbIsl;
      const arr = this._islands.get(id);
      if (arr !== undefined) {
        const k = arr.indexOf(b);
        if (k >= 0) arr.splice(k, 1);
        if (arr.length > 0) {
          for (let j = 0; j < arr.length; j++) {
            const m = arr[j];
            m.sleeping = false; m.sleepT = 0; m._bbT = 0; m._bbIsl = -1;
          }
          woken++;
        }
        this._islands.delete(id);
      }
      b._bbIsl = -1;
    }
    const touch = this.params.touchDistance;
    for (let i = 0; i < list.length; i++) {
      const o = list[i];
      if (o === b || o.sleeping !== true || o.pinned === true) continue;
      const dx = o.x - b.x, dy = o.y - b.y, dz = o.z - b.z;
      const lim = o.r + b.r + touch;
      if (dx * dx + dy * dy + dz * dz < lim * lim) {
        if (this.wakeIsland(o)) woken++;
      }
    }
    return woken;
  }
}
