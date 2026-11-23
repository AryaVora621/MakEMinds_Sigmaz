// overlay.js - pure debug geometry for the ball physics (no DOM, no THREE, never writes to world or ball).
// Units follow the sim: PEDRO inches, x right, y far wall, z up; time s; mass kg (BALL_KINDS[kind].massKg).
// Contacts are INFERRED geometrically because contact() in balls.js is module-private. If a physics owner adds
// world.debugContacts = [{p,n,depth,kind}], that array is used first. Hive mesh contacts are NOT inferred
// (documented omission).
import { BALL } from '../../balls.js';
import { LAYERS, OVERLAY, M_PER_IN } from './params.js';
import { energyOf } from './invariants.js';

const WALL_TOP_IN = 12; // in, height of the drawn bounds box. ASSUMED (cosmetic only)
const CROSS_IN = 0.5; // in, half size of a contact cross. Spec value.
const AXIS_IN = 2; // in, body axis length. Spec value.

function isFree(b) { return b.state === 'free'; }

// ---------------------------------------------------------------- contacts
export function probeContacts(world, opts = {}) {
  if (Array.isArray(world.debugContacts)) {
    return world.debugContacts.map((c) => ({
      p: c.p.slice(), n: c.n.slice(), depth: c.depth, kind: c.kind || 'ball', a: c.a != null ? String(c.a) : '', b: c.b != null ? String(c.b) : null,
    }));
  }
  const slop = opts.slopIn != null ? opts.slopIn : OVERLAY.contactSlopIn;
  const out = [];
  const balls = (world.balls || []).filter(isFree);
  const bd = world.bounds;
  for (const b of balls) {
    const id = String(b.id);
    if (b.z - b.r < slop) out.push({ p: [b.x, b.y, 0], n: [0, 0, 1], depth: b.r - b.z, kind: 'floor', a: id, b: null });
    if (bd) {
      const w = [
        [b.x - bd.minX, 1, 0, bd.minX, b.y], [bd.maxX - b.x, -1, 0, bd.maxX, b.y],
        [b.y - bd.minY, 0, 1, b.x, bd.minY], [bd.maxY - b.y, 0, -1, b.x, bd.maxY],
      ];
      for (const [dist, nx, ny, px, py] of w) {
        if (dist - b.r < slop) out.push({ p: [px, py, b.z], n: [nx, ny, 0], depth: b.r - dist, kind: 'wall', a: id, b: null });
      }
    }
    for (const bx of world.boxes || []) boxContact(out, b, bx, slop, id);
    for (const ci of world.circles || []) circleContact(out, b, ci, slop, id);
    if (world.robot && world.robotDims) robotContact(out, b, world.robot, world.robotDims, slop, id);
  }
  // ball pairs: sweep over x-sorted order
  const idx = balls.map((b, i) => i).sort((i, j) => balls[i].x - balls[j].x);
  let rmax = 0;
  for (const b of balls) if (b.r > rmax) rmax = b.r;
  for (let ii = 0; ii < idx.length; ii++) {
    const a = balls[idx[ii]];
    for (let jj = ii + 1; jj < idx.length; jj++) {
      const b = balls[idx[jj]];
      if (b.x - a.x > a.r + rmax + slop) break;
      const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
      const d2 = dx * dx + dy * dy + dz * dz, rs = a.r + b.r + slop;
      if (d2 >= rs * rs) continue;
      const d = Math.sqrt(d2);
      const n = d > 1e-9 ? [dx / d, dy / d, dz / d] : [0, 0, 1]; // points from b to a
      // midpoint of the overlap region along the normal
      const ta = a.r - (a.r + b.r - d) * 0.5;
      out.push({ p: [a.x - n[0] * ta, a.y - n[1] * ta, a.z - n[2] * ta], n, depth: a.r + b.r - d, kind: 'ball', a: String(a.id), b: String(b.id) });
    }
  }
  return out;
}

function boxContact(out, b, bx, slop, id) {
  if (b.z + b.r < bx.zMin - slop || b.z - b.r > bx.zMax + slop) return;
  const qx = clamp(b.x, bx.minX, bx.maxX), qy = clamp(b.y, bx.minY, bx.maxY), qz = clamp(b.z, bx.zMin, bx.zMax);
  const dx = b.x - qx, dy = b.y - qy, dz = b.z - qz, d2 = dx * dx + dy * dy + dz * dz;
  const lim = b.r + slop;
  if (d2 > 1e-12) {
    if (d2 > lim * lim) return;
    const d = Math.sqrt(d2);
    out.push({ p: [qx, qy, qz], n: [dx / d, dy / d, dz / d], depth: b.r - d, kind: 'box', a: id, b: null });
    return;
  }
  // centre inside the box: push out along the nearest face, mirroring _boxCollide
  const c = [[b.x - bx.minX, -1, 0, 0], [bx.maxX - b.x, 1, 0, 0], [b.y - bx.minY, 0, -1, 0], [bx.maxY - b.y, 0, 1, 0], [bx.zMax - b.z, 0, 0, 1]];
  let m = c[0];
  for (const q of c) if (q[0] < m[0]) m = q;
  out.push({ p: [b.x, b.y, b.z], n: [m[1], m[2], m[3]], depth: m[0] + b.r, kind: 'box', a: id, b: null });
}

function circleContact(out, b, ci, slop, id) {
  if (b.z + b.r < ci.zMin - slop || b.z - b.r > ci.zMax + slop) return;
  const dx = b.x - ci.x, dy = b.y - ci.y, dxy = Math.hypot(dx, dy), rad = Math.min(dxy, ci.r);
  const ux = dxy > 1e-9 ? dx / dxy : 1, uy = dxy > 1e-9 ? dy / dxy : 0;
  const qx = ci.x + ux * rad, qy = ci.y + uy * rad, qz = clamp(b.z, ci.zMin, ci.zMax);
  const ex = b.x - qx, ey = b.y - qy, ez = b.z - qz, d2 = ex * ex + ey * ey + ez * ez;
  const lim = b.r + slop;
  if (d2 > 1e-12) {
    if (d2 > lim * lim) return;
    const d = Math.sqrt(d2);
    out.push({ p: [qx, qy, qz], n: [ex / d, ey / d, ez / d], depth: b.r - d, kind: 'circle', a: id, b: null });
    return;
  }
  const radial = ci.r - dxy, top = ci.zMax - b.z;
  if (radial < top) out.push({ p: [b.x, b.y, b.z], n: [ux, uy, 0], depth: radial + b.r, kind: 'circle', a: id, b: null });
  else out.push({ p: [b.x, b.y, b.z], n: [0, 0, 1], depth: top + b.r, kind: 'circle', a: id, b: null });
}

function robotContact(out, b, R, D, slop, id) {
  const c = Math.cos(R.h), s = Math.sin(R.h);
  const dx = b.x - R.x, dy = b.y - R.y;
  const lx = c * dx + s * dy, ly = -s * dx + c * dy; // robot-local
  const qx = clamp(lx, -D.halfX, D.halfX), qy = clamp(ly, -D.halfY, D.halfY), qz = clamp(b.z, 0, D.height);
  const ex = lx - qx, ey = ly - qy, ez = b.z - qz, d2 = ex * ex + ey * ey + ez * ez;
  const lim = b.r + slop;
  let nlx, nly, nlz, depth, px, py, pz;
  if (d2 > 1e-12) {
    if (d2 > lim * lim) return;
    const d = Math.sqrt(d2);
    nlx = ex / d; nly = ey / d; nlz = ez / d; depth = b.r - d; px = qx; py = qy; pz = qz;
  } else {
    const f = [[lx + D.halfX, -1, 0, 0], [D.halfX - lx, 1, 0, 0], [ly + D.halfY, 0, -1, 0], [D.halfY - ly, 0, 1, 0], [D.height - b.z, 0, 0, 1]];
    let m = f[0];
    for (const q of f) if (q[0] < m[0]) m = q;
    nlx = m[1]; nly = m[2]; nlz = m[3]; depth = m[0] + b.r; px = lx; py = ly; pz = b.z;
  }
  // local -> world rotation by +h
  out.push({
    p: [R.x + c * px - s * py, R.y + s * px + c * py, pz],
    n: [c * nlx - s * nly, s * nlx + c * nly, nlz], depth, kind: 'robot', a: id, b: null,
  });
}

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

// ---------------------------------------------------------------- state and energy
export function createOverlayState() {
  return { prevVel: new Map(), trails: new Map() };
}

export function energyReadout(world) {
  const e = energyOf(world);
  const k = M_PER_IN * M_PER_IN; // kg*in^2/s^2 -> J
  return { kinJ: e.kin * k, potJ: e.pot * k, totalJ: e.total * k };
}

// ---------------------------------------------------------------- segment builder
class Seg {
  constructor() { this.pos = []; this.col = []; this.count = 0; }
  line(a, b, c) {
    this.pos.push(a[0], a[1], a[2], b[0], b[1], b[2]);
    this.col.push(c[0], c[1], c[2], c[0], c[1], c[2]);
    this.count++;
  }
  polyline(pts, c, close) {
    for (let i = 0; i + 1 < pts.length; i++) this.line(pts[i], pts[i + 1], c);
    if (close && pts.length > 2) this.line(pts[pts.length - 1], pts[0], c);
  }
  circle(cx, cy, cz, r, plane, n, c) {
    const pts = [];
    for (let i = 0; i < n; i++) {
      const t = (i / n) * Math.PI * 2, u = Math.cos(t) * r, v = Math.sin(t) * r;
      pts.push(plane === 0 ? [cx + u, cy + v, cz] : plane === 1 ? [cx + u, cy, cz + v] : [cx, cy + u, cz + v]);
    }
    this.polyline(pts, c, true);
  }
  cross(p, s, c) {
    this.line([p[0] - s, p[1], p[2]], [p[0] + s, p[1], p[2]], c);
    this.line([p[0], p[1] - s, p[2]], [p[0], p[1] + s, p[2]], c);
    this.line([p[0], p[1], p[2] - s], [p[0], p[1], p[2] + s], c);
  }
  box(x0, y0, z0, x1, y1, z1, c) {
    const lo = [z0, z1];
    for (const z of lo) this.polyline([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]], c, true);
    for (const [x, y] of [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]) this.line([x, y, z0], [x, y, z1], c);
  }
}

// ---------------------------------------------------------------- geometry
export function buildDebugGeometry(world, opts = {}) {
  const layers = opts.layers != null ? opts.layers : LAYERS.DEFAULT;
  const state = opts.state || createOverlayState();
  const dt = opts.dt != null && Number.isFinite(opts.dt) ? opts.dt : 0;
  const C = { ...OVERLAY.colors, ...((opts.style && opts.style.colors) || {}) };
  const S = { ...OVERLAY, ...(opts.style || {}), colors: C };
  const seg = new Seg();
  const balls = (world.balls || []).filter(isFree);
  const contacts = probeContacts(world, { slopIn: S.contactSlopIn });
  const labels = [];

  if (layers & LAYERS.COLLIDERS) {
    const b = world.bounds;
    if (b) {
      seg.box(b.minX, b.minY, 0, b.maxX, b.maxY, WALL_TOP_IN, C.collider);
    }
    for (const bx of world.boxes || []) seg.box(bx.minX, bx.minY, bx.zMin, bx.maxX, bx.maxY, bx.zMax, C.collider);
    for (const ci of world.circles || []) {
      seg.circle(ci.x, ci.y, ci.zMin, ci.r, 0, S.circleSubdiv, C.collider);
      seg.circle(ci.x, ci.y, ci.zMax, ci.r, 0, S.circleSubdiv, C.collider);
      for (const [dx, dy] of [[ci.r, 0], [-ci.r, 0], [0, ci.r], [0, -ci.r]]) seg.line([ci.x + dx, ci.y + dy, ci.zMin], [ci.x + dx, ci.y + dy, ci.zMax], C.collider);
    }
    for (const q of balls) {
      const col = q.sleeping ? C.sleeping : C.active;
      for (let pl = 0; pl < 3; pl++) seg.circle(q.x, q.y, q.z, q.r, pl, S.circleSubdiv, col);
    }
  }
  if (layers & LAYERS.AABBS) {
    for (const q of balls) seg.box(q.x - q.r, q.y - q.r, q.z - q.r, q.x + q.r, q.y + q.r, q.z + q.r, C.aabb);
  }
  if (layers & LAYERS.CONTACTS) {
    for (const c of contacts) seg.cross(c.p, CROSS_IN, C.contact);
  }
  if (layers & LAYERS.CONTACT_NORMALS) {
    for (const c of contacts) {
      const L = S.contactNormalLenIn;
      seg.line(c.p, [c.p[0] + c.n[0] * L, c.p[1] + c.n[1] * L, c.p[2] + c.n[2] * L], C.normal);
    }
  }
  if (layers & LAYERS.VELOCITY) {
    for (const q of balls) {
      const k = S.velScaleS;
      seg.line([q.x, q.y, q.z], [q.x + q.vx * k, q.y + q.vy * k, q.z + q.vz * k], C.velocity);
    }
  }
  if (layers & LAYERS.IMPULSES) {
    const touched = new Set();
    for (const c of contacts) { touched.add(c.a); if (c.b != null) touched.add(c.b); }
    for (const q of balls) {
      const id = String(q.id);
      const pv = state.prevVel.get(id);
      if (!pv || !touched.has(id)) continue;
      // contact delta-v: remove the gravity contribution (g_vec = (0,0,-g)), mass-free
      const dvx = q.vx - pv[0], dvy = q.vy - pv[1], dvz = q.vz - pv[2] + BALL.g * dt;
      const k = S.impulseScaleS;
      seg.line([q.x, q.y, q.z], [q.x + dvx * k, q.y + dvy * k, q.z + dvz * k], C.impulse);
    }
  }
  if (layers & LAYERS.BODY_AXES) {
    for (const q of balls) {
      seg.line([q.x, q.y, q.z], [q.x + AXIS_IN, q.y, q.z], [1, 0, 0]);
      seg.line([q.x, q.y, q.z], [q.x, q.y + AXIS_IN, q.z], [0, 1, 0]);
      seg.line([q.x, q.y, q.z], [q.x, q.y, q.z + AXIS_IN], [0, 0, 1]);
    }
  }
  if (layers & LAYERS.TRAILS) {
    const seen = new Set();
    for (const q of balls) {
      const id = String(q.id);
      seen.add(id);
      let tr = state.trails.get(id);
      if (!tr) { tr = []; state.trails.set(id, tr); }
      tr.push([q.x, q.y, q.z]);
      while (tr.length > S.trailLen) tr.shift();
      seg.polyline(tr, q.sleeping ? C.sleeping : C.velocity, false);
    }
    for (const id of [...state.trails.keys()]) if (!seen.has(id)) state.trails.delete(id);
  }
  if ((layers & LAYERS.ROBOT) && world.robot && world.robotDims) {
    const R = world.robot, D = world.robotDims, c = Math.cos(R.h), s = Math.sin(R.h);
    const w = (lx, ly, z) => [R.x + c * lx - s * ly, R.y + s * lx + c * ly, z];
    for (const z of [0, D.height]) {
      seg.polyline([w(-D.halfX, -D.halfY, z), w(D.halfX, -D.halfY, z), w(D.halfX, D.halfY, z), w(-D.halfX, D.halfY, z)], C.robot, true);
    }
    for (const [lx, ly] of [[-D.halfX, -D.halfY], [D.halfX, -D.halfY], [D.halfX, D.halfY], [-D.halfX, D.halfY]]) seg.line(w(lx, ly, 0), w(lx, ly, D.height), C.robot);
    seg.line(w(0, 0, D.height), w(D.halfX, 0, D.height), C.robot); // heading axis (+x local)
    const k = S.velScaleS;
    seg.line([R.x, R.y, D.height], [R.x + (R.vx || 0) * k, R.y + (R.vy || 0) * k, D.height], C.velocity);
  }

  // prev velocities for the next call's impulse estimate (overlay state only; never the world)
  const live = new Set();
  for (const q of balls) { live.add(String(q.id)); state.prevVel.set(String(q.id), [q.vx, q.vy, q.vz]); }
  for (const id of [...state.prevVel.keys()]) if (!live.has(id)) state.prevVel.delete(id);

  const energy = energyReadout(world);
  if (layers & LAYERS.ENERGY) {
    const b = world.bounds;
    labels.push({ p: b ? [b.minX, b.minY, WALL_TOP_IN] : [0, 0, WALL_TOP_IN], text: `E ${energy.totalJ.toFixed(3)} J` });
  }

  return {
    positions: new Float32Array(seg.pos), colors: new Float32Array(seg.col), segCount: seg.count,
    contacts, energy, labels,
  };
}
