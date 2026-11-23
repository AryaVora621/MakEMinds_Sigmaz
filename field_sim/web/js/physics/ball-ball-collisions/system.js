// system.js - BallBallSystem facade for ball-ball collisions v2.
//
// INTEGRATION RECIPE for balls.js (applied later by the integrator; this package does not edit balls.js).
// This module replaces BallWorld._ballBall, _pair, the per-ball sleep block at the end of _substep, _wake, _wakeAbove.
// It does not own floor/wall/box/hive/robot contact models, but static HALF-SPACES (the floor z = 0 and the
// straight field walls) SHOULD be handed to it with setPlanes([...]) so they are solved in the same
// sequential-impulse loop as ball-ball contacts. Otherwise a pile's weight cannot reach the floor inside one
// solve and the after-solve floor/wall clamp re-injects ball-ball overlap each substep (see buildPlaneContacts).
// balls.js may keep its own floor/wall clamp afterwards as a safety net; it then rarely fires.
//   planes:           this._bb.setPlanes([{nx:0,ny:0,nz:1,d:0}, {nx:1,ny:0,nz:0,d:-72}, ...]) (in; mu/e optional, default 0)
// balls.js must import this module; this module must NOT import balls.js (avoids a circular ESM import).
//   constructor:      this._bb = new BallBallSystem();
//   reset()/setup:    this._bb.reset(); after start balls are created with sleeping=true: this._bb.adoptSleeping(freeList)
//   step(dt):         after pending adds/releases and before the substep loop: this._bb.beginStep(freeList, dt)
//   _substep(h):      replace `this._ballBall(list, h)` with `this._bb.substep(list, h)`;
//                     after the statics loop replace the per-ball sleep block with `this._bb.endSubstep(list, h)`;
//                     for each free awake ball with a floor contact (z <= r + 0.01) call
//                     syncRollingOnSurface(b, 0, 0, 1, h) and dampSpin(b, h)
//   _wake(b)          -> this._bb.wake(b)
//   _wakeAbove(b), removeBall, hold, score, intake -> this._bb.onRemoved(b, freeList)
// UNITS: length in, time s, mass kg, velocity in/s, angular velocity rad/s.

import { BB_PARAMS, G_IN } from './params.js';
import { ensureBody } from './bodies.js';
import { computeMargins, BallGrid3D } from './grid3d.js';
import { ContactCache, ContactSolver, buildContacts, buildPlaneContacts } from './contact_solver.js';
import { IslandManager } from './islands.js';

const now = (typeof performance !== 'undefined' && performance.now)
  ? () => performance.now()
  : () => Date.now();

export class BallBallSystem {
  constructor(params = BB_PARAMS, opts = {}) {
    this.params = params;
    this.gravity = opts.gravity === undefined ? G_IN : opts.gravity; // in/s^2
    this.grid = new BallGrid3D({ cellSize: params.cellSize, fastMargin: params.fastMargin });
    this.cache = new ContactCache();
    this.solver = new ContactSolver(params);
    this.islands = new IslandManager(params);
    this._pool = [];
    this._n = 0;            // valid contacts in pool
    this._margins = undefined;
    this._dt = 1 / 60;      // frame dt (s)
    this._stamp = 0;
    this._dirty = true;
    this._sigLen = -1; this._sigFirst = -1; this._sigLast = -1;
    this.lastSubstepMs = 0;
    this.planes = [];
    if (opts.planes) this.setPlanes(opts.planes);
  }

  // planes: array (max 64) of {nx, ny, nz, d, mu?, e?}; n is normalised here. Units: d in in.
  setPlanes(planes) {
    if (planes.length > 64) throw new Error('BallBallSystem: at most 64 planes');
    this.planes = planes.map((p) => {
      const l = Math.hypot(p.nx, p.ny, p.nz);
      return Object.freeze({ nx: p.nx / l, ny: p.ny / l, nz: p.nz / l, d: p.d / l, mu: p.mu || 0, e: p.e || 0 });
    });
    this.cache.clear();
    this._dirty = true;
  }

  get contacts() { return this._pool; }
  get contactCount() { return this._n; }

  _signature(list) {
    const n = list.length;
    this._sigLen = n;
    this._sigFirst = n > 0 ? list[0]._bbUid : -1;
    this._sigLast = n > 0 ? list[n - 1]._bbUid : -1;
  }

  _membershipChanged(list) {
    const n = list.length;
    if (n !== this._sigLen) return true;
    if (n === 0) return false;
    return list[0]._bbUid !== this._sigFirst || list[n - 1]._bbUid !== this._sigLast;
  }

  _rebuild(list) {
    const P = this.params;
    for (let i = 0; i < list.length; i++) ensureBody(list[i], P);
    this._margins = computeMargins(list, this._dt, this.gravity, P.speculativeDistance, this._margins);
    this.grid.build(list, this._margins);
    this._signature(list);
    this._dirty = false;
  }

  beginStep(list, dt) {
    this._dt = dt;
    this._rebuild(list);
  }

  substep(list, h) {
    const t0 = now();
    const P = this.params;
    if (this._dirty || this._membershipChanged(list)) this._rebuild(list);
    const grid = this.grid;
    let n = buildContacts(grid.pairs, grid.pairCount, h, P, this._pool);
    n = buildPlaneContacts(list, this.planes, h, P, this._pool, n);

    // wake check: exactly one sleeping body, touching
    let woke = false;
    const touch = P.touchDistance;
    for (let k = 0; k < n; k++) {
      const c = this._pool[k];
      if (c.s >= touch) continue;
      const a = c.a, b = c.b;
      const as = a.sleeping === true, bs = b.sleeping === true;
      if (as === bs) continue;
      if (as) { if (b.pinned !== true && this.islands.wakeIsland(a)) woke = true; }
      else if (a.pinned !== true && this.islands.wakeIsland(b)) woke = true;
    }
    if (woke) {
      this._dirty = true;
      this._rebuild(list);
      n = buildContacts(grid.pairs, grid.pairCount, h, P, this._pool);
      n = buildPlaneContacts(list, this.planes, h, P, this._pool, n);
    }
    this._n = n;

    this._stamp++;
    this.solver.solve(this._pool, n, h, this.cache, this._stamp);
    this.cache.sweep(this._stamp - 1);
    this.lastSubstepMs = now() - t0;
    return n;
  }

  endSubstep(list, h) {
    return this.islands.update(list, this._pool, this._n, h);
  }

  wake(b) {
    const r = this.islands.wakeIsland(b);
    if (r) this._dirty = true;
    else { b._bbT = 0; b.sleepT = 0; }
    return r;
  }

  onRemoved(b, list) {
    this.islands.remove(b, list);
    this._dirty = true;
  }

  adoptSleeping(list) {
    for (let i = 0; i < list.length; i++) ensureBody(list[i], this.params);
    const c = this.islands.adoptSleeping(list);
    this._dirty = true;
    return c;
  }

  reset() {
    this.islands.reset();
    this.cache.clear();
    this.grid.clear();
    this._n = 0;
    this._stamp = 0;
    this._dirty = true;
    this._sigLen = -1; this._sigFirst = -1; this._sigLast = -1;
    this.lastSubstepMs = 0;
  }

  stats() {
    return {
      pairs: this.grid.pairCount,
      contacts: this._n,
      islands: this.islands.lastAwakeComponents,
      sleepingIslands: this.islands.sleepingIslandCount,
      cacheSize: this.cache.size,
      lastSubstepMs: this.lastSubstepMs,
    };
  }
}
