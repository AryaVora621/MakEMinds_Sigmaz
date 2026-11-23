// Physics invariants, checked on world state only (never mutates the world).
// Units: PEDRO inches, s, kg. Energy is in kg*in^2/s^2 (multiply by M_PER_IN^2 for joules).
// All limits come from params.js INVARIANT (ASSUMED except overlapBallIn); they are never loosened here.
import { BALL, BALL_KINDS } from '../../balls.js';
import { INVARIANT } from './params.js';

const MAX_VIOLATIONS = 200;   // cap on stored violations per monitor (memory guard), ASSUMED

function massOf(b) { const k = BALL_KINDS[b.kind]; return k ? k.massKg : BALL_KINDS.pollen.massKg; }

/** E = sum m*(0.5|v|^2 + g*(z - r)) over free balls, in kg*in^2/s^2. */
export function energyOf(world) {
  let kin = 0, pot = 0;
  for (const b of world.balls) {
    if (b.state !== 'free') continue;
    const m = massOf(b);
    kin += 0.5 * m * (b.vx * b.vx + b.vy * b.vy + b.vz * b.vz);
    pot += m * BALL.g * (b.z - b.r);
  }
  return { kin, pot, total: kin + pot };
}

/** Map ball id -> [x,y,z] for the free balls. Used by the tunnelling rules. */
export function snapshotPositions(world) {
  const m = new Map();
  for (const b of world.balls) if (b.state === 'free') m.set(b.id, [b.x, b.y, b.z]);
  return m;
}

function robotLocal(R, x, y) {
  const c = Math.cos(R.h), s = Math.sin(R.h), dx = x - R.x, dy = y - R.y;
  return [c * dx + s * dy, -s * dx + c * dy];
}

// Does the 2D segment p0 -> p1 pass through the open box |lx|<hx, |ly|<hy? (slab method)
function segmentHitsBox(p0, p1, hx, hy) {
  let t0 = 0, t1 = 1;
  for (let a = 0; a < 2; a++) {
    const h = a === 0 ? hx : hy, d = p1[a] - p0[a];
    if (Math.abs(d) < 1e-12) { if (Math.abs(p0[a]) >= h) return false; continue; }
    let ta = (-h - p0[a]) / d, tb = (h - p0[a]) / d;
    if (ta > tb) { const t = ta; ta = tb; tb = t; }
    t0 = Math.max(t0, ta); t1 = Math.min(t1, tb);
    if (t0 >= t1) return false;
  }
  return t0 < t1;
}

/** Check every per-state invariant. prev (from snapshotPositions) enables the tunnelling rules.
 *  opts.limits overrides INVARIANT fields (for tests only). Returns {ok, violations[{kind,ballId,value,limit}], maxPen, maxOverlap}. */
export function checkWorld(world, prev = null, opts = {}) {
  const L = { ...INVARIANT, ...(opts.limits || {}) };
  const out = [];
  const add = (kind, ballId, value, limit) => { if (out.length < MAX_VIOLATIONS) out.push({ kind, ballId, value, limit }); };
  let maxPen = 0, maxOverlap = 0;
  const bd = world.bounds;
  const free = [];
  for (const b of world.balls) {
    if (b.state !== 'free') continue;
    free.push(b);
    if (![b.x, b.y, b.z, b.vx, b.vy, b.vz].every(Number.isFinite)) { add('finite', b.id, NaN, 0); continue; }
    const sp = Math.hypot(b.vx, b.vy, b.vz);
    if (sp > BALL.maxSpeed + L.speedSlackInS) add('speed', b.id, sp, BALL.maxSpeed + L.speedSlackInS);
    const penF = b.r - b.z;
    if (penF > 0) maxPen = Math.max(maxPen, penF);
    if (penF > L.penFloorIn) add('floor', b.id, penF, L.penFloorIn);
    if (bd) {
      const pw = Math.max(bd.minX - (b.x - b.r), (b.x + b.r) - bd.maxX, bd.minY - (b.y - b.r), (b.y + b.r) - bd.maxY);
      if (pw > 0) maxPen = Math.max(maxPen, pw);
      if (pw > L.penWallIn) add('walls', b.id, pw, L.penWallIn);
    }
    if (prev && bd) {
      const p = prev.get(b.id);
      if (p && p[0] >= bd.minX && p[0] <= bd.maxX && p[1] >= bd.minY && p[1] <= bd.maxY) {
        const beyond = Math.max(bd.minX - b.x, b.x - bd.maxX, bd.minY - b.y, b.y - bd.maxY);
        if (beyond > L.penWallIn) add('tunnel-wall', b.id, beyond, L.penWallIn);
      }
    }
  }
  const nf = free.length;
  for (let i = 0; i < nf; i++) {
    const a = free[i];
    for (let j = i + 1; j < nf; j++) {
      const c = free[j];
      const dx = a.x - c.x; if (dx > a.r + c.r + 1 || dx < -(a.r + c.r + 1)) continue;
      const d = Math.hypot(dx, a.y - c.y, a.z - c.z), ov = a.r + c.r - d;
      if (ov > 0) maxOverlap = Math.max(maxOverlap, ov);
      if (ov > L.overlapBallIn) add('overlap', a.id + '|' + c.id, ov, L.overlapBallIn);
    }
  }
  const R = world.robot, D = world.robotDims;
  if (R && D) {
    const hx = D.halfX - L.robotInsideSlackIn, hy = D.halfY - L.robotInsideSlackIn;
    for (const b of free) {
      if (b._ghost) continue;   // freshly fired from inside the robot box: exempt by design (balls.js addBall)
      const l1 = robotLocal(R, b.x, b.y);
      const in1 = Math.abs(l1[0]) < hx && Math.abs(l1[1]) < hy && b.z - b.r < D.height;
      if (in1) { add('robot', b.id, Math.min(hx - Math.abs(l1[0]), hy - Math.abs(l1[1])), 0); continue; }
      if (prev) {
        const p = prev.get(b.id);
        if (p && Math.min(p[2], b.z) - b.r < D.height) {
          const l0 = robotLocal(R, p[0], p[1]);
          const in0 = Math.abs(l0[0]) < hx && Math.abs(l0[1]) < hy;
          if (!in0 && segmentHitsBox(l0, l1, hx, hy)) add('tunnel-robot', b.id, 1, 0);
        }
      }
    }
  }
  return { ok: out.length === 0, violations: out, maxPen, maxOverlap };
}

/** Per-step monitor: call step() after every world.step. closed=true also enforces the energy rules. */
export function createMonitor(world, opts = {}) {
  const closed = !!opts.closed;
  const limits = { ...INVARIANT, ...(opts.limits || {}) };
  const E0 = energyOf(world).total;
  let Eprev = E0, Elast = E0, steps = 0, maxPen = 0, maxOverlap = 0, prev = snapshotPositions(world);
  const violations = [];
  const push = v => { if (violations.length < MAX_VIOLATIONS) violations.push({ ...v, step: steps }); };
  return {
    step() {
      steps++;
      const r = checkWorld(world, prev, { limits });
      maxPen = Math.max(maxPen, r.maxPen); maxOverlap = Math.max(maxOverlap, r.maxOverlap);
      for (const v of r.violations) push(v);
      if (closed) {
        const E = energyOf(world).total;
        const stepLim = Math.max(limits.energyAbs, Math.abs(limits.energyRelStep * E0));
        if (E > Eprev + stepLim) push({ kind: 'energy-step', ballId: null, value: E - Eprev, limit: stepLim });
        const totLim = Math.abs(E0) * (1 + limits.energyRelTotal) + limits.energyAbs;
        if (E > totLim) push({ kind: 'energy-total', ballId: null, value: E, limit: totLim });
        Eprev = E; Elast = E;
      }
      prev = snapshotPositions(world);
    },
    report() {
      const drift = E0 !== 0 ? (Elast - E0) / Math.abs(E0) : 0;
      return { ok: violations.length === 0, violations: violations.slice(), maxPen, maxOverlap, energyDriftRel: drift, steps };
    },
  };
}
