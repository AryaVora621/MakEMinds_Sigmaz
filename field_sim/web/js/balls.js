// balls.js - pure-JS ball physics world for the BIOBUZZ field simulator.
// No DOM, no three.js. Units: inches, seconds, Pedro frame (x right, y toward far wall, z up).
//
// HIVE ANGLE CONVENTION
//   env.hives.<alliance> = { pivot:[x,y,z], cadAngle:-30 (default), cells:{audience:cell, far:cell} }
//   The cell geometry in env MUST be the one at the CAD pose (scene.json constants.hives.<a>.states[cad_pose_state].cells,
//   cadAngle = groups.hive_<a>.cad_pose_angle_deg, i.e. -30 for audience_up).
//   setHiveAngles({red, blue}) takes the ABSOLUTE tip angle in degrees, the same number as app.js `hive[a].shown`
//   (-30 = audience_up, +30 = far_up). The geometry is rotated by (angle - cadAngle) degrees about the +x axis through
//   the pivot, right-handed (y' = y cos - z sin, z' = y sin + z cos), exactly like three.js group.rotation.x in app.js.
//   (If you would rather pass the delta directly, set cadAngle:0 in env.)
//
// HIVE COLLISION: the hive is a real triangle mesh (data/hive_collider.bin from tools_js/extract_hive_mesh.mjs, parsed by
//   hivemesh.js parseHiveCollider). Pass it as env.hives.<a>.mesh = {verts,tris} (or a TriMesh); without it an idealised
//   mesh built from the cell corners is used. The mesh lives in the hive local frame (world - pivot, CAD pose) and the ball is
//   transformed into it with the inverse of the current tip rotation, so walls move correctly when the hive tips.
//   CAPTURE: a ball is scored only if its centre is inside a cell's sensor volume AND (it crossed the opening polygon plane
//   against the opening normal from outside, or it has been slow/at rest inside for >= BALL.captureRest s). A ball that
//   hits the hive from behind / the side / above simply bounces off the solid mesh.
//
// Ball fields: id,type,x,y,z,vx,vy,vz,r,state('free'|'held'|'scored'),hive,cell,sleeping (+ private _ fields).
// stats().free counts all state==='free' balls INCLUDING sleeping ones; sleeping is the subset at rest.

// Ball kinds. Sizes come from the game manual (research r1/r3, see docs/overnight/research_manual.md):
//  - pollen: 2.8 in diameter (manual TU06 Sec 9.8, via ftc.uprobotics.tech/questions/78). Mass 0.055 lb (~25 g) from
//    Plex / AndyMark search snippets (medium confidence, not fetched).
//  - nectar: 3.6 in diameter (same manual section). Mass 0.091 lb (~41 g) is an INFERRED pairing from an ambiguous
//    AndyMark snippet (low-medium confidence), so it is marked assumed. Mass is informational only: the physics is not
//    mass-dependent (BALL.mass is a unitless 1.0).
// radiusIn is the physics radius (inches); radiusM is the same radius in metres (1 in = 0.0254 m).
export const BALL_KINDS = {
  // radiusIn keeps the repo's existing values (2.782 in / 3.599 in, which round to the manual's 2.8 / 3.6 in) so existing
  // physics and tests are unchanged; the exact 2.8/3.6 in halves (1.4 / 1.8) would shift every ball by ~0.01 in.
  pollen: { radiusIn: 1.391, radiusM: 1.391 * 0.0254, massKg: 0.025, source: 'FTC BIOBUZZ manual TU06 Sec 9.8 (2.8 in dia, via ftc.uprobotics.tech/questions/78); repo value 2.782 in; mass 0.055 lb from Plex/AndyMark snippets', assumed: false },
  nectar: { radiusIn: 1.7995, radiusM: 1.7995 * 0.0254, massKg: 0.041, source: 'FTC BIOBUZZ manual TU06 Sec 9.8 (3.6 in dia, via ftc.uprobotics.tech/questions/78); mass 0.091 lb is an ASSUMPTION paired from an ambiguous AndyMark snippet', assumed: true },   // ASSUMPTION: nectar mass 0.091 lb (~41 g) is inferred, not from the manual
};
// Launch speed by kind. The flywheel rpm -> wheel-surface speed model (shot_core speedFromRpm, unchanged) is the input.
// ASSUMPTION (mass scaling, not validated on hardware): the wheel hands every ball the same kinetic energy, so the exit
// speed scales as sqrt(m_ref / m). Reference = pollen, so pollen keeps the existing speed (scale 1.0). For nectar at
// 0.041 kg (assumed mass) the scale is sqrt(0.025/0.041) ~ 0.78: the same rpm gives a ~22% slower nectar ball, and the
// auto solver raises rpm by 1/scale ~ 1.28x to hit the same opening. Only shotrules/aim use this.
export const LAUNCH_REF_KIND = 'pollen';
export function launchScaleFor(kind) {
  const k = BALL_KINDS[kind] ? kind : LAUNCH_REF_KIND;
  return Math.sqrt(BALL_KINDS[LAUNCH_REF_KIND].massKg / BALL_KINDS[k].massKg);
}
// Physics constants. Tags: SOURCED (URL given) or ASSUMED (no source; project placeholder). Research r3 (docs/overnight/physics3.md)
// found no sourced range that excludes any current value, so no value was changed in this pass.
export const BALL = {
  r: BALL_KINDS.pollen.radiusIn, nectarR: BALL_KINDS.nectar.radiusIn, mass: 1.0,
  g: 386.0886,   // SOURCED (unit conversion): standard gravity 9.80665 m/s^2 = 386.0886 in/s^2
  // Coefficients of restitution. The hard-plastic-on-concrete drop test (~0.69, https://hypertextbook.com/facts/2006/restitution.shtml,
  // search snippet only, not fetched) is weak evidence for the wall value. No foam-floor or ball-on-ball source was found.
  restitutionFloor: 0.45,  // ASSUMED: foam-on-floor COR, no source. TODO: measure with a dropped pollen ball.
  restitutionWall: 0.55,   // SOURCED (weak: hard plastic on concrete, not foam), kept as-is.
  restitutionBall: 0.7,    // ASSUMED: ball-ball COR, no source.
  restitutionRobot: 0.35,  // ASSUMED: ball-robot COR, no source. A fast push leaves at (1+e)*v (see sim_physics tests).
  restitutionHive: 0.3,    // ASSUMED: ball-hive COR, no source.
  // Quadratic air drag on free airborne balls: a = -dragK*|v|*v (units 1/in). ASSUMED value, not sourced.
  // Physical estimate from k = 0.5*rho*Cd*A/m with rho 1.2 kg/m3, Cd 0.47, 2.8 in ball, 25 g gives ~0.0011 1/in,
  // which would bleed a 500 in/s shot ~12% in 0.25 s. That is far beyond the envelope the shot solver was tuned to,
  // so the value is set by project tests (sim_physics: 0.5-6% loss over 0.25 s). TODO: measure with a real ball.
  dragK: 0.0004,
  rollFriction: 0.04,      // ASSUMED coefficient (r3 cites a carpet reel value, not fetched); rolling deceleration = rollFriction*g = ~15.4 in/s^2 (40 in/s stops in ~2.6 s)
  slideFriction: 0.5,      // ASSUMED Coulomb coefficient applied on impacts, no source
  restSpeed: 5,            // normal speed (in/s) below which a contact is "resting" (no bounce)
  sleepSpeed: 1.5, sleepTime: 0.25, maxSubstep: 0.004,
  captureRest: 0.15, captureSlow: 30,   // s and in/s: slow-inside-the-cell capture rule
  maxStepTravel: 0.8,                    // max distance a ball (or the robot) may move in one substep (anti-tunnelling)
  maxFired: 90,                          // launched free balls kept alive (oldest recycled)
  maxSpeed: 1800,                        // in/s hard clamp (a runaway ball can never skip through geometry)
};
// Intake: dual full-width rollers (front = robot +x, rear = robot -x). A ball is taken only when it touches the roller
// (its surface within captureGap of the face) with its centre inside the mouth width, or is pulled the last few inches by
// the roller when its surface is within pullRadius of the face and its centre is inside the mouth width. Nothing is taken
// from beside the robot, from beyond the pull radius, or from under/behind the body (centre past the face by > maxOverlap).
export const INTAKE = {
  pullRadius: 2.5,      // in: ball surface gap to the roller face inside which the roller pulls (about one ball diameter)
  mouthInset: 0.5,      // in: mouth half-width for the ball CENTRE = halfY - mouthInset (rollers sit inside the frame)
  pull: 500,            // in/s^2 roller pull toward the robot (only inside pullRadius, only inside the mouth width)
  captureGap: 0.6,      // in: ball surface within this of the face = touching the roller = loaded
  maxOverlap: 2.0,      // in: swept-robot slack; a centre deeper than this inside the face is under the body: never loaded
  maxLift: 3.2,         // in: ball bottom must be below this height to be pulled in (rollers are low)
  maxSpeed: 220,        // in/s: faster balls (shots in flight) are not intaken
  sides: ['front', 'rear'],
};
// Hold limit: the robot holds at most MAX_HELD balls TOTAL (pollen + nectar combined). Matches matchrules RULES.holdLimit
// (G407, 4). Intake of a ball beyond this is refused; the ball is not pulled in. Held balls leave the world in FIFO order
// through BallWorld.dequeueHeld() / fireHeld().
export const MAX_HELD = 4;
import { TriMesh, cellSensors, insideCell, crossesOpening, buildFallbackMesh, pointInPoly } from './hivemesh.js';
import { shouldTip } from './hivetip_model.js';
const G = BALL.g, RAD = Math.PI / 180;
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
// How far (in) behind the opening plane a ball centre may be and still count as entering the hive opening (see _entering).
const ENTRY_DEPTH = 3;
// Segment p0->p1 (hive local) crosses the opening plane against its normal at a point inside the opening polygon grown by r
// (the rim shoulders count as open for a ball of radius r). Used only for a ball that is already entering (see _entering).
function crossesGrownOpening(p0, p1, s, r) {
  const n = s.n, P = s.P;
  const d0 = n[0] * (p0[0] - P[0]) + n[1] * (p0[1] - P[1]) + n[2] * (p0[2] - P[2]);
  const d1 = n[0] * (p1[0] - P[0]) + n[1] * (p1[1] - P[1]) + n[2] * (p1[2] - P[2]);
  if (!(d0 > 0 && d1 <= 0)) return false;
  const f = d0 / (d0 - d1);
  const hx = p0[0] + (p1[0] - p0[0]) * f, hy = p0[1] + (p1[1] - p0[1]) * f, hz = p0[2] + (p1[2] - p0[2]) * f;
  const sl = (hy - s.A[1]) * s.u[0] + (hz - s.A[2]) * s.u[1];
  return pointInPoly(hx, sl, s.op) || distToPolyEdge(hx, sl, s.op) < r;
}
// Distance from (x, y) to the closest edge of a closed polygon [[x, y], ...].
function distToPolyEdge(x, y, poly) {
  let best = Infinity;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const ax = poly[j][0], ay = poly[j][1], bx = poly[i][0] - ax, by = poly[i][1] - ay;
    const L2 = bx * bx + by * by;
    const t = L2 > 0 ? clamp(((x - ax) * bx + (y - ay) * by) / L2, 0, 1) : 0;
    best = Math.min(best, Math.hypot(x - ax - t * bx, y - ay - t * by));
  }
  return best;
}
// Kind of a ball: an explicit 'pollen'|'nectar' wins; otherwise a type string containing 'nectar' means nectar; default pollen.
const kindOf = (type, kind) => (kind && BALL_KINDS[kind] ? kind : /nectar/.test(type || '') ? 'nectar' : 'pollen');
const radiusFor = (kind, d) => (d ? d / 2 : BALL_KINDS[kind].radiusIn);
/** Count balls by kind: {pollen, nectar}. Balls without a kind field are classified from their type. */
export function ballKindCounts(balls) {
  const out = { pollen: 0, nectar: 0 };
  for (const b of balls || []) out[kindOf(b.type, b.kind)]++;
  return out;
}
// Scored balls REPORTED to each hive (onCapture path), by kind. Keyed by hive id ('red' | 'blue'). Monotone within a
// field: a ball recycled out of a full cell still counted when it was reported. Cleared by BallWorld.reset().
const HIVE_BALL_COUNTS = new Map();
/** Record one scored ball of `kind` ('pollen' | 'nectar') reported to hive `hiveId`. */
export function recordHiveBall(hiveId, kind) {
  const c = HIVE_BALL_COUNTS.get(hiveId) || { pollen: 0, nectar: 0 };
  c[kind === 'nectar' ? 'nectar' : 'pollen']++;
  HIVE_BALL_COUNTS.set(hiveId, c);
}
/** Scored balls reported to hive `hiveId`: {pollen, nectar}. Zeros for a hive with no reports. */
export function hiveBallCounts(hiveId) {
  const c = HIVE_BALL_COUNTS.get(hiveId);
  return { pollen: c ? c.pollen : 0, nectar: c ? c.nectar : 0 };
}
/** Clear every hive's reported counts (called from BallWorld.reset). */
export function clearHiveBallCounts() { HIVE_BALL_COUNTS.clear(); }
/** Pure tip test on {pollen, nectar} counts. Delegates to hivetip_model.shouldTip (8 pollen tip; see that file's ledger). */
export function tipReady(counts) { return shouldTip(counts || {}); }

// Resolve a sphere contact with a surface. n = unit normal pointing toward the ball, pen = penetration depth,
// vs = velocity of the surface. Pushes the ball out, bounces, applies impact (Coulomb) or rolling friction.
function contact(b, nx, ny, nz, pen, e, vsx, vsy, vsz, h) {
  b.x += nx * pen; b.y += ny * pen; b.z += nz * pen; b._c = true;
  let rx = b.vx - vsx, ry = b.vy - vsy, rz = b.vz - vsz;
  const vn = rx * nx + ry * ny + rz * nz;
  let impact = false;
  if (vn < 0) {
    impact = -vn > BALL.restSpeed;
    const k = (1 + (impact ? e : 0)) * vn;
    rx -= k * nx; ry -= k * ny; rz -= k * nz;
    if (impact) {
      const tn = rx * nx + ry * ny + rz * nz;
      let tx = rx - tn * nx, ty = ry - tn * ny, tz = rz - tn * nz;
      const tm = Math.hypot(tx, ty, tz), dec = BALL.slideFriction * (1 + e) * -vn;
      const f = tm <= dec ? 0 : (tm - dec) / tm;
      rx = tn * nx + tx * f; ry = tn * ny + ty * f; rz = tn * nz + tz * f;
    }
  }
  if (!impact && nz > 0.2) {
    const tn = rx * nx + ry * ny + rz * nz;
    const tx = rx - tn * nx, ty = ry - tn * ny, tz = rz - tn * nz;
    const tm = Math.hypot(tx, ty, tz), dec = BALL.rollFriction * G * nz * h;
    const f = tm <= dec ? 0 : (tm - dec) / tm;
    rx = tn * nx + tx * f; ry = tn * ny + ty * f; rz = tn * nz + tz * f;
  }
  b.vx = rx + vsx; b.vy = ry + vsy; b.vz = rz + vsz;
}

function prepCell(c) {
  const A = c.opening_corners[0], B = c.opening_corners[2], C = c.floor_corners[2], D = c.back_wall_corners[2];
  const xs = c.opening_corners.map(p => p[0]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs);
  const poly = [[A[1], A[2]], [B[1], B[2]], [D[1], D[2]], [C[1], C[2]]];
  const cen = [poly.reduce((s, p) => s + p[0], 0) / 4, poly.reduce((s, p) => s + p[1], 0) / 4];
  const mkEdge = (p, q) => {
    const ex = q[0] - p[0], ey = q[1] - p[1], L = Math.hypot(ex, ey);
    let nx = -ey / L, ny = ex / L;
    if (nx * (cen[0] - p[0]) + ny * (cen[1] - p[1]) < 0) { nx = -nx; ny = -ny; }
    return { p, ex, ey, L2: L * L, nx, ny };
  };
  const edges = [mkEdge(poly[0], poly[3]), mkEdge(poly[3], poly[2]), mkEdge(poly[2], poly[1])]; // floor, back, lid
  const uy = B[1] - A[1], uz = B[2] - A[2], ul = Math.hypot(uy, uz);
  const u = [uy / ul, uz / ul];
  const op = (c.opening_polygon || c.opening_corners).map(p => [p[0], (p[1] - A[1]) * u[0] + (p[2] - A[2]) * u[1]]);
  const fl = edges[0], fL = Math.sqrt(fl.L2);
  return {
    x0, x1, poly, edges, n: c.opening_normal, aim: c.aim_point, A, op,
    floorU: [fl.ex / fL, fl.ey / fL], floorN: [fl.nx, fl.ny], floorLen: fL, C,
    u
  };
}
// stepFixed defaults: 240 Hz fixed step, at most 8 fixed steps per call (1/30 s of simulated time per frame at most).
const DROP_SLOPE = 0.17;   // sin(~10 deg): a cell drains once its floor falls toward the opening by more than this
const FIXED_HZ = 240, FIXED_MAX_FRAMES = 8;
export class BallWorld {
  constructor(env, rng) {
    env = env || {};
    this.env = env;
    this.rng = rng || null;
    this.bounds = env.bounds;
    this.boxes = env.boxes || [];
    this.circles = env.circles || [];
    this.balls = [];
    this.onCapture = null;
    this.onRelease = null; this.onRecycle = null; this.dropOnTip = env.dropOnTip !== false; this.released = 0;
    this.robot = null; this.robotDims = null;
    this._seq = 0; this._capSeq = 0;
    this._angles = {};
    this.hv = [];
    for (const name of ['red', 'blue']) {
      const hdef = env.hives && env.hives[name];
      if (!hdef) continue;
      const cad = hdef.cadAngle ?? -30, cells = [];
      let R = 0;
      for (const cn of ['audience', 'far']) {
        const cd = hdef.cells && hdef.cells[cn]; if (!cd) continue;
        const pc = prepCell(cd);
        for (const p of [...cd.opening_corners, ...cd.floor_corners, ...cd.back_wall_corners])
          R = Math.max(R, Math.hypot(p[1] - hdef.pivot[1], p[2] - hdef.pivot[2]));
        cells.push({ name: cn, pc });
      }
      let mesh = hdef.mesh;
      if (!mesh) mesh = buildFallbackMesh({ pivot: hdef.pivot, cells: hdef.cells });
      if (!(mesh instanceof TriMesh)) mesh = new TriMesh(mesh.verts, mesh.tris);
      for (let i = 1; i < mesh.v.length; i += 3) R = Math.max(R, Math.hypot(mesh.v[i], mesh.v[i + 1]));
      const sensors = cellSensors({ pivot: hdef.pivot, cells: hdef.cells });
      for (const co of cells) co.sensor = sensors[co.name];
      this.hv.push({ name, pivot: hdef.pivot, cad, cells, mesh, R: R + 4, delta: 0, dPrev: 0, d0: 0, d1: 0, omega: 0 });
      this._angles[name] = cad;
    }
    const b = this.bounds;
    this._cs = 4; this._gx = Math.max(1, Math.ceil((b.maxX - b.minX) / 4)); this._gy = Math.max(1, Math.ceil((b.maxY - b.minY) / 4));
    this._head = new Int32Array(this._gx * this._gy); this._next = new Int32Array(64);
    this._pairs = [];
    this.captureEnabled = true;
    this._ct = new Float64Array(28); this._lp = [0, 0, 0]; this._lq = [0, 0, 0]; this._lr = [0, 0, 0];
    this._startBalls = null;
    this._angles0 = Object.assign({}, this._angles);
    this.intake = { enabled: false, space: 0, taken: 0, params: Object.assign({}, INTAKE) };
    this.onIntake = null;
    this._rob0 = null; this._rob1 = null; this._robI = null; this._intaked = [];
    this._acc = 0;    // fixed-step accumulator (stepFixed)
    this.held = [];   // FIFO of held balls: {id, type, kind}, oldest first (at most MAX_HELD)
  }

  reset(startBalls) {
    this.balls.length = 0;
    clearHiveBallCounts();
    this.held = [];
    if (startBalls) this._startBalls = startBalls.map(s => ({ ...s, position: s.position.slice() }));
    for (const s of startBalls || []) {
      const b = this._mk(s.type, s.position, [0, 0, 0], s.diameter, s.id, s.kind);
      b.sleeping = true;   // every start ball is at rest (incl. stacks inside flowers and hive preloads) until disturbed
      b.start = true;      // start balls (hive preloads) may be captured by resting in a cell; shot balls must cross the opening
      this.balls.push(b);
    }
  }
  /** Restore the whole field: every ball back at its start position (or opts.balls; [] clears the field), hives back to
   *  their initial angles (or opts.hiveAngles), intake counters and robot cleared. opts.hiveDyn = {red,blue}: their
   *  .reset() is called too. Returns the world. */
  resetField(opts) {
    opts = opts || {};
    this.reset(opts.balls !== undefined ? opts.balls : (this._startBalls || []));
    this._seq = 0; this._pending = []; this._intaked = []; this._acc = 0;
    const ang = Object.assign({}, this._angles0, opts.hiveAngles || {});
    for (const hv of this.hv) {
      const a = ang[hv.name]; this._angles[hv.name] = a;
      hv.delta = hv.d0 = hv.d1 = hv.dPrev = (a - hv.cad) * RAD; hv.omega = 0;
    }
    if (opts.hiveDyn) for (const k of Object.keys(opts.hiveDyn)) if (opts.hiveDyn[k] && opts.hiveDyn[k].reset) opts.hiveDyn[k].reset();
    this.intake.taken = 0; this._rob0 = this._rob1 = null; this.robot = null; this.robotDims = null;
    return this;
  }
  /** Configure the intake: {enabled, space (free hopper slots; Infinity = unlimited), params:{INTAKE overrides}}. */
  setIntake(o) {
    o = o || {};
    if (o.enabled !== undefined) this.intake.enabled = !!o.enabled;
    if (o.space !== undefined) this.intake.space = o.space;
    if (o.params) Object.assign(this.intake.params, o.params);
    return this.intake;
  }
  _mk(type, p, v, diameter, id, kind) {
    const k = kindOf(type, kind);
    return {
      id: id || ('shot' + (++this._seq)), type, kind: k, x: p[0], y: p[1], z: p[2], vx: v[0], vy: v[1], vz: v[2],
      r: radiusFor(k, diameter), state: 'free', hive: null, cell: null, sleeping: false,
      sleepT: 0, _enter: null, _rest: 0, _c: false, _px: p[0], _py: p[1], _pz: p[2], _slot: -1, _lx: 0, _ly: 0, _lz: 0,
    };
  }
  /** Add a launched ball. Launched balls are recycled: above BALL.maxFired free launched balls the oldest (sleeping first)
   *  is removed, so unlimited shooting never grows the world without bound. A ball spawned INSIDE the robot box (the shooter
   *  exit is within the robot footprint) is exempt from robot collision until it has left the box, so it is not nudged
   *  or killed by the very robot that fired it. */
  addBall(type, pos, vel, kind) {
    const b = this._mk(type, pos, vel || [0, 0, 0], undefined, undefined, kind);
    b.fired = true;
    if (this.robot && this.robotDims) {
      const R = this.robot, D = this.robotDims, c = Math.cos(R.h), s = Math.sin(R.h), dx = b.x - R.x, dy = b.y - R.y;
      const lx = c * dx + s * dy, ly = -s * dx + c * dy;
      if (Math.abs(lx) < D.halfX + b.r && Math.abs(ly) < D.halfY + b.r && b.z - b.r < D.height) b._ghost = true;
    }
    this.balls.push(b);
    this._recycleFired();
    return b;
  }
  _recycleFired() {
    const cap = BALL.maxFired;
    let n = 0; for (const o of this.balls) if (o.fired && o.state === 'free') n++;
    while (n > cap) {
      let victim = null;
      for (const o of this.balls) if (o.fired && o.state === 'free' && o.sleeping) { victim = o; break; }
      if (!victim) for (const o of this.balls) if (o.fired && o.state === 'free') { victim = o; break; }
      if (!victim) break;
      this.removeBall(victim); n--;
    }
  }
  /** Held balls by kind: {pollen, nectar}. Counts the hold FIFO, so pollen + nectar together never exceed MAX_HELD. */
  heldKindCounts() { return ballKindCounts(this.held); }
  /** Remove and return the OLDEST held ball {id, type, kind}, or null when nothing is held (FIFO). */
  dequeueHeld() { return this.held.shift() || null; }
  /** Launch the OLDEST held ball at pos/vel. It keeps its own kind and type (pollen stays pollen, nectar stays nectar).
   *  Returns the new free ball, or null when nothing is held. */
  fireHeld(pos, vel) {
    const h = this.dequeueHeld();
    if (!h) return null;
    const b = this.addBall(h.type, pos, vel, h.kind);
    b.shot = true; b.shotAge = 0;   // a shot ball is never re-loaded by the roller (same flag app.js shoot() sets)
    return b;
  }
  removeBall(b) { const i = this.balls.indexOf(b); if (i >= 0) this.balls.splice(i, 1); this._wakeAbove(b); }
  hold(b) { this._wakeAbove(b); b._enter = null; b._rest = 0; b.state = 'held'; b.hive = null; b.cell = null; b._slot = -1; b.vx = b.vy = b.vz = 0; b.sleeping = false; }
  release(b, vel) {
    b.state = 'free'; b.hive = null; b.cell = null; b._slot = -1; b.sleeping = false; b.sleepT = 0; b._enter = null; b._rest = 0;
    const v = vel || [0, 0, 0]; b.vx = v[0]; b.vy = v[1]; b.vz = v[2]; b._px = b.x; b._py = b.y; b._pz = b.z;
  }
  setRobot(pose, dims) {
    if (!pose || !dims) { this.robot = null; this.robotDims = null; this._rob0 = this._rob1 = null; return; }
    this._rob0 = this._rob1;
    this.robot = { x: pose.x, y: pose.y, h: pose.h || 0, vx: pose.vx || 0, vy: pose.vy || 0, om: pose.om || 0 };
    this._rob1 = this.robot;
    this.robotDims = { halfX: dims.halfX, halfY: dims.halfY, height: dims.height };
  }
  setHiveAngles(a) { for (const k of Object.keys(a || {})) if (k in this._angles) this._angles[k] = a[k]; }

  stats() {
    const s = { free: 0, sleeping: 0, held: 0, scoredRed: 0, scoredBlue: 0 };
    for (const b of this.balls) {
      if (b.state === 'free') { s.free++; if (b.sleeping) s.sleeping++; }
      else if (b.state === 'held') s.held++;
      else if (b.hive === 'red') s.scoredRed++; else if (b.hive === 'blue') s.scoredBlue++;
    }
    return s;
  }

  _wake(b) {
    if (!b.sleeping) return;
    b.sleeping = false; b.sleepT = 0;
    for (const o of this.balls) {
      if (!o.sleeping || o.state !== 'free' || o.z <= b.z + 0.3) continue;
      const dx = o.x - b.x, dy = o.y - b.y, dz = o.z - b.z, rr = o.r + b.r + 0.5;
      if (dx * dx + dy * dy + dz * dz < rr * rr) this._wake(o);
    }
  }

  // a ball leaves: balls that were resting on/against it must not float
  _wakeAbove(b) {
    for (const o of this.balls) {
      if (!o.sleeping || o.state !== 'free' || o === b || o.z < b.z - 0.3) continue;
      const dx = o.x - b.x, dy = o.y - b.y, dz = o.z - b.z, rr = o.r + b.r + 0.6;
      if (dx * dx + dy * dy + dz * dz < rr * rr) this._wake(o);
    }
  }

  _slotPos(hv, cellObj, k) {
    const pc = cellObj.pc, rS = 1.81, d = 2 * rS;
    const nx = Math.max(1, Math.floor((pc.x1 - pc.x0) / d)), ny = Math.max(1, Math.floor(pc.floorLen / d));
    const col = k % nx, row = Math.floor(k / nx) % ny, layer = Math.floor(k / (nx * ny));
    const x = pc.x0 + ((pc.x1 - pc.x0) / nx) * (col + 0.5);
    const s = rS + row * d, hgt = rS + layer * d;
    const y = pc.C[1] - pc.floorU[0] * s + pc.floorN[0] * hgt;
    const z = pc.C[2] - pc.floorU[1] * s + pc.floorN[1] * hgt;
    return [x, y, z];
  }
  /** Drop-out: a stored ball in a cell whose floor now slopes toward its own opening (the cell has become the DOWN cell,
   *  hive tilted past DROP_SLOPE) is handed back to the free-ball physics at its current spot with the tangential velocity of
   *  the rotating hive; gravity and the hive collider then roll it out through the opening. No teleport: the position is
   *  unchanged. Up-cell balls (floor slopes away from the opening) never drop. ASSUMED rule, the manual is silent
   *  (docs/overnight/hive-physics.md). onRelease(ball, hiveName, cellName) lets the score model forget the ball. */
  _releaseDrained() {
    for (const hv of this.hv) {
      const c = Math.cos(hv.delta), s = Math.sin(hv.delta);
      for (const co of hv.cells) {
        const uz = s * co.pc.floorU[0] + c * co.pc.floorU[1];   // floorU points opening -> back wall
        if (uz < DROP_SLOPE) continue;                           // floor still falls inward: the cell holds
        for (const b of this.balls) {
          if (b.state !== 'scored' || b.hive !== hv.name || b.cell !== co.name) continue;
          const dy = b.y - hv.pivot[1], dz = b.z - hv.pivot[2];
          b.state = 'free'; b.sleeping = false; b.start = false; b._enter = null; b._rest = 0; b.sleepT = 0; b._slot = -1;
          b.vx = 0; b.vy = -hv.omega * dz; b.vz = hv.omega * dy;
          b._px = b.x; b._py = b.y; b._pz = b.z;
          this.released = (this.released || 0) + 1;
          if (this.onRelease) this.onRelease(b, hv.name, co.name);
        }
      }
    }
  }
  _placeScored(b, hv) {
    const c = Math.cos(hv.delta), s = Math.sin(hv.delta), dy = b._ly - hv.pivot[1], dz = b._lz - hv.pivot[2];
    b.x = b._lx; b.y = hv.pivot[1] + dy * c - dz * s; b.z = hv.pivot[2] + dy * s + dz * c;
  }
  _cellCapacity(cellObj) {
    const pc = cellObj.pc, d = 3.62;
    return Math.max(1, Math.floor((pc.x1 - pc.x0) / d)) * Math.max(1, Math.floor(pc.floorLen / d)) * 3;
  }
  // Extra argument for the onCapture hook: the ball's kind and the hive's scored totals by kind ({pollen, nectar}).
  // Existing hooks that take (ball, hive, cell) ignore it.
  _captureInfo(b, hvName) {
    const totals = { pollen: 0, nectar: 0 };
    for (const o of this.balls) if (o.state === 'scored' && o.hive === hvName) totals[kindOf(o.type, o.kind)]++;
    return { kind: kindOf(b.type, b.kind), totals };
  }
  _capture(b, hv, cellObj) {
    const used = new Set();
    for (const o of this.balls) if (o.state === 'scored' && o.hive === hv.name && o.cell === cellObj.name) used.add(o._slot);
    let k = 0; while (used.has(k)) k++;
    const pcap = this._cellCapacity(cellObj);
    if (k >= pcap) {   // cell full: the oldest stored ball is recycled (the hive's own bookkeeping already counted it)
      let old = null;
      for (const o of this.balls) if (o.state === 'scored' && o.hive === hv.name && o.cell === cellObj.name && (!old || o._seq < old._seq)) old = o;
      if (old) { k = old._slot; this._wakeAbove(old); this.balls.splice(this.balls.indexOf(old), 1); if (this.onRecycle) this.onRecycle(old, hv.name, cellObj.name); } else k = 0;
    }
    b._seq = ++this._capSeq;
    const p = this._slotPos(hv, cellObj, k);
    b.state = 'scored'; b.hive = hv.name; b.cell = cellObj.name; b._slot = k; b.sleeping = false;
    b._lx = p[0]; b._ly = p[1]; b._lz = p[2]; b.vx = b.vy = b.vz = 0; b._enter = null; b._rest = 0;
    this._placeScored(b, hv);
    this._wakeAbove(b);
    this._pending.push([b, hv.name, cellObj.name]);
  }

  step(dt) {
    if (!(dt > 0)) return;
    dt = Math.min(dt, 0.1);
    let vmax = 0;
    for (const b of this.balls) if (b.state === 'free' && !b.sleeping) vmax = Math.max(vmax, Math.hypot(b.vx, b.vy, b.vz) + G * dt);
    this._pending = [];
    for (const hv of this.hv) {
      const nd = (this._angles[hv.name] - hv.cad) * RAD;
      hv.d0 = hv.delta; hv.d1 = nd;
      const dd = nd - hv.d0;
      if (Math.abs(dd) > 20 * RAD) { hv.d0 = nd; hv.delta = nd; hv.omega = 0; this._wakeNear(hv); }
      else { hv.omega = dd / dt; if (Math.abs(dd) > 0.02 * RAD) this._wakeNear(hv); }
      vmax = Math.max(vmax, Math.abs(hv.omega) * hv.R);
    }
    // robot sweep: the robot pose is interpolated across the substeps so a fast robot cannot jump over a ball
    const r1 = this._rob1, r0 = this._rob0;
    let interp = false, dh = 0;
    if (r1 && r0 && r1 !== r0) {
      const dx = r1.x - r0.x, dy = r1.y - r0.y; dh = Math.atan2(Math.sin(r1.h - r0.h), Math.cos(r1.h - r0.h));
      const reach = this.robotDims ? Math.hypot(this.robotDims.halfX, this.robotDims.halfY) : 12;
      const mv = Math.hypot(dx, dy) + Math.abs(dh) * reach;
      if (mv < 40 && mv > 1e-9) { interp = true; vmax = Math.max(vmax, mv / dt); }
    }
    const n = Math.min(120, Math.max(1, Math.ceil(dt / BALL.maxSubstep), Math.ceil(dt * vmax / BALL.maxStepTravel))), h = dt / n;
    if (interp && !this._robI) this._robI = {};
    for (let k = 0; k < n; k++) {
      for (const hv of this.hv) { hv.dPrev = hv.delta; hv.delta = hv.d0 + (hv.d1 - hv.d0) * ((k + 1) / n); }
      if (interp) {
        const f = (k + 1) / n, I = this._robI;
        I.x = r0.x + (r1.x - r0.x) * f; I.y = r0.y + (r1.y - r0.y) * f; I.h = r0.h + dh * f;
        I.vx = r1.vx; I.vy = r1.vy; I.om = r1.om; this.robot = I;
      }
      this._substep(h);
    }
    this.robot = r1 || null; this._rob0 = r1 || null;
    if (this._intaked.length) {
      const taken = this._intaked; this._intaked = [];
      this.balls = this.balls.filter(b => b.state !== 'intaken');
      if (this.onIntake) for (const [b, side] of taken) this.onIntake(b, side);
    }
    for (const hv of this.hv) { hv.delta = hv.d1; }
    for (const b of this.balls) if (b.state === 'scored') { const hv = this.hv.find(q => q.name === b.hive); if (hv) this._placeScored(b, hv); }
    if (this.dropOnTip) this._releaseDrained();
    for (const [b, a] of this._pending) recordHiveBall(a, kindOf(b.type, b.kind));
    if (this.onCapture) for (const [b, a, c] of this._pending) this.onCapture(b, a, c, this._captureInfo(b, a));
    this._pending = [];
  }
  /** Fixed-step wrapper for frame-rate independence (see docs/overnight/physics3.md). Accumulates frameDt and calls
   *  this.step(1/hz) a whole number of times. At most FIXED_MAX_FRAMES steps run per call; if more are due, the whole
   *  backlog is dropped (no spiral of death). step(dt) itself is unchanged. Returns the interpolation alpha in [0,1]:
   *  the fraction of one fixed step left in the accumulator (for rendering between steps). */
  stepFixed(frameDt, hz = FIXED_HZ) {
    const h = 1 / hz;
    if (frameDt > 0) this._acc += frameDt;
    let n = Math.floor(this._acc / h + 1e-9);
    if (n > FIXED_MAX_FRAMES) { n = FIXED_MAX_FRAMES; this._acc = 0; }
    else this._acc -= n * h;
    for (let k = 0; k < n; k++) this.step(h);
    if (this._acc < 0) this._acc = 0;
    return Math.min(1, this._acc / h);
  }
  _wakeNear(hv) {
    for (const b of this.balls) {
      if (!b.sleeping || b.state !== 'free') continue;
      const dy = b.y - hv.pivot[1], dz = b.z - hv.pivot[2];
      if (dy * dy + dz * dz < hv.R * hv.R) this._wake(b);
    }
  }

  _substep(h) {
    const balls = this.balls, bd = this.bounds, e = BALL;
    const list = [];
    for (let i = 0; i < balls.length; i++) {
      const b = balls[i]; if (b.state !== 'free') continue;
      list.push(b);
      if (b.sleeping) continue;
      b._c = false; b._px = b.x; b._py = b.y; b._pz = b.z;
      if (b.z > b.r + 0.05) {   // airborne: quadratic drag, applied before integration (deterministic, no RNG)
        const sp = Math.sqrt(b.vx * b.vx + b.vy * b.vy + b.vz * b.vz), f = 1 / (1 + e.dragK * sp * h);
        b.vx *= f; b.vy *= f; b.vz *= f;
      }
      b.x += b.vx * h; b.y += b.vy * h; b.z += b.vz * h - 0.5 * G * h * h; b.vz -= G * h;
      const sp2 = b.vx * b.vx + b.vy * b.vy + b.vz * b.vz;
      if (sp2 > BALL.maxSpeed * BALL.maxSpeed) { const k = BALL.maxSpeed / Math.sqrt(sp2); b.vx *= k; b.vy *= k; b.vz *= k; }
    }
    if (this.robot && this.intake.enabled) this._intakeStep(list, h);
    // hive: opening-crossing bookkeeping and swept test against the mesh (before the discrete collisions)
    for (const hv of this.hv) for (const b of list) if (!b.sleeping && b.state === 'free') this._hiveSweep(b, hv, h);
    // ball-ball
    this._ballBall(list, h);
    // statics
    const rob = this.robot;
    for (const b of list) {
      if (b.state !== 'free') continue;
      if (rob) this._robotCollide(b, h);
      if (b.sleeping) continue;
      for (let it = 0; it < 2; it++) {
        if (b.z < b.r) contact(b, 0, 0, 1, b.r - b.z, e.restitutionFloor, 0, 0, 0, h);
        const wh = bd.wallHeight === undefined || b.z - b.r < bd.wallHeight;
        if (wh) {
          if (b.x < bd.minX + b.r) contact(b, 1, 0, 0, bd.minX + b.r - b.x, e.restitutionWall, 0, 0, 0, h);
          else if (b.x > bd.maxX - b.r) contact(b, -1, 0, 0, b.x - (bd.maxX - b.r), e.restitutionWall, 0, 0, 0, h);
          if (b.y < bd.minY + b.r) contact(b, 0, 1, 0, bd.minY + b.r - b.y, e.restitutionWall, 0, 0, 0, h);
          else if (b.y > bd.maxY - b.r) contact(b, 0, -1, 0, b.y - (bd.maxY - b.r), e.restitutionWall, 0, 0, 0, h);
        }
        for (const bx of this.boxes) this._boxCollide(b, bx, h);
        for (const ci of this.circles) this._circleCollide(b, ci, h);
        for (const hv of this.hv) this._hiveCollide(b, hv, h);
      }
      if (this.captureEnabled && b.state === 'free') for (const hv of this.hv) { this._hiveCapture(b, hv, h); if (b.state !== 'free') break; }
      if (b.state !== 'free') continue;
      // sleep
      if (b._c && b.vx * b.vx + b.vy * b.vy + b.vz * b.vz < BALL.sleepSpeed * BALL.sleepSpeed) {
        b.sleepT += h;
        if (b.sleepT >= BALL.sleepTime) { b.sleeping = true; b.vx = b.vy = b.vz = 0; }
      } else b.sleepT = 0;
    }
  }

  _ballBall(list, h) {
    const n = list.length; if (n < 2) return;
    if (this._next.length < n) this._next = new Int32Array(n * 2);
    const head = this._head, next = this._next, bd = this.bounds, gx = this._gx, gy = this._gy, cs = this._cs;
    head.fill(-1);
    for (let i = 0; i < n; i++) {
      const b = list[i];
      const ix = clamp(Math.floor((b.x - bd.minX) / cs), 0, gx - 1), iy = clamp(Math.floor((b.y - bd.minY) / cs), 0, gy - 1);
      b._cell = iy * gx + ix; next[i] = head[b._cell]; head[b._cell] = i;
    }
    const pairs = this._pairs; pairs.length = 0;
    for (let i = 0; i < n; i++) {
      const b = list[i], ix = b._cell % gx, iy = (b._cell / gx) | 0;
      for (let j = next[i]; j >= 0; j = next[j]) pairs.push(i, j);
      for (let k = 0; k < 4; k++) {
        const jx = ix + (k === 3 ? -1 : 1 - (k === 2 ? 1 : 0)), jy = iy + (k === 0 ? 0 : 1);
        if (jx < 0 || jx >= gx || jy >= gy) continue;
        for (let j = head[jy * gx + jx]; j >= 0; j = next[j]) pairs.push(i, j);
      }
    }
    for (let it = 0; it < 3; it++)
      for (let p = 0; p < pairs.length; p += 2) {
        const a = list[pairs[p]], c = list[pairs[p + 1]];
        if (a.state === 'free' && c.state === 'free') this._pair(a, c, h);
      }
  }

  // Roller: a low ball whose centre is inside the mouth width and whose surface touches (or is within pullRadius of) the
  // roller face is pulled in and loaded. No funnel, no sideways guide, no capture from beside or through the body.
  _intakeStep(list, h) {
    const I = this.intake, P = I.params, R = this.robot, D = this.robotDims;
    if (!(I.space > 0) || !D) return;
    const c = Math.cos(R.h), s = Math.sin(R.h);
    const reachSq = (D.halfX + P.pullRadius + 2 * D.halfY + 6) ** 2;
    const mouth = D.halfY - P.mouthInset;
    const sides = [];
    if (P.sides.indexOf('front') >= 0) sides.push(1);
    if (P.sides.indexOf('rear') >= 0) sides.push(-1);
    for (const b of list) {
      if (b.state !== 'free' || b.shot) continue;
      if (I.space <= 0 || this.held.length >= MAX_HELD) break;   // hold full: refuse (no pull, no load)
      if (b.z - b.r > P.maxLift) continue;
      const dx = b.x - R.x, dy = b.y - R.y;
      if (dx * dx + dy * dy > reachSq) continue;
      const lx = c * dx + s * dy, ly = -s * dx + c * dy;
      if (Math.abs(ly) > mouth) continue;              // beside the mouth: ordinary robot collision only
      if (b.vx * b.vx + b.vy * b.vy > P.maxSpeed * P.maxSpeed || Math.abs(b.vz) > 150) continue;
      let best = null;
      for (const sd of sides) {
        const u = sd * lx - D.halfX;                   // centre distance beyond this intake's face (negative = inside the footprint)
        if (u < -P.maxOverlap || u > b.r + P.pullRadius) continue;
        if (best === null || u < best.u) best = { sd, u };
      }
      if (!best) continue;
      if (best.u <= b.r + P.captureGap) {              // touching the roller: load it
        b.state = 'intaken'; I.space--; I.taken++;
        this.held.push({ id: b.id, type: b.type, kind: b.kind });
        this._intaked.push([b, best.sd > 0 ? 'front' : 'rear']);
        this._wakeAbove(b);
        continue;
      }
      if (b.sleeping) this._wake(b);
      const ax = -best.sd * P.pull;                    // robot-local pull straight at the roller face
      b.vx += c * ax * h; b.vy += s * ax * h;
      b.sleepT = 0;
    }
  }
  _pair(a, b, h) {
    if (a.sleeping && b.sleeping) return;
    const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z, rr = a.r + b.r, d2 = dx * dx + dy * dy + dz * dz;
    if (d2 >= rr * rr) return;
    const d = Math.sqrt(d2);
    let nx = 0, ny = 0, nz = 1;
    if (d > 1e-6) { nx = dx / d; ny = dy / d; nz = dz / d; }
    const pen = rr - d, e = BALL.restitutionBall;
    let vn = (a.vx - b.vx) * nx + (a.vy - b.vy) * ny + (a.vz - b.vz) * nz;
    if (a.sleeping && -vn >= 0) { /* a asleep: b is the mover */ }
    if (b.sleeping && b.state === 'free' && vn < -3) this._wake(b);
    if (a.sleeping && vn < -3) this._wake(a);
    if (a.sleeping) { contact(b, -nx, -ny, -nz, pen, e, 0, 0, 0, h); return; }
    if (b.sleeping) { contact(a, nx, ny, nz, pen, e, 0, 0, 0, h); return; }
    a._c = b._c = true;
    a.x += nx * pen * 0.5; a.y += ny * pen * 0.5; a.z += nz * pen * 0.5;
    b.x -= nx * pen * 0.5; b.y -= ny * pen * 0.5; b.z -= nz * pen * 0.5;
    const rvx = a.vx - b.vx, rvy = a.vy - b.vy, rvz = a.vz - b.vz;
    vn = rvx * nx + rvy * ny + rvz * nz;
    let J = 0, impact = false;
    if (vn < 0) {
      impact = -vn > BALL.restSpeed;
      J = -(1 + (impact ? e : 0)) * vn * 0.5;
      a.vx += J * nx; a.vy += J * ny; a.vz += J * nz; b.vx -= J * nx; b.vy -= J * ny; b.vz -= J * nz;
    }
    // tangential friction
    const tx = rvx - vn * nx, ty = rvy - vn * ny, tz = rvz - vn * nz, tm = Math.hypot(tx, ty, tz);
    if (tm > 1e-9) {
      const dec = impact ? BALL.slideFriction * J : BALL.rollFriction * G * Math.abs(nz) * h * 0.5;
      const m = Math.min(dec, tm * 0.5) / tm;
      a.vx -= tx * m; a.vy -= ty * m; a.vz -= tz * m; b.vx += tx * m; b.vy += ty * m; b.vz += tz * m;
    }
  }

  _boxCollide(b, bx, h) {
    if (b.z + b.r < bx.zMin || b.z - b.r > bx.zMax) return;
    const qx = clamp(b.x, bx.minX, bx.maxX), qy = clamp(b.y, bx.minY, bx.maxY), qz = clamp(b.z, bx.zMin, bx.zMax);
    const dx = b.x - qx, dy = b.y - qy, dz = b.z - qz, d2 = dx * dx + dy * dy + dz * dz;
    if (d2 > b.r * b.r) return;
    if (d2 > 1e-12) { const d = Math.sqrt(d2); contact(b, dx / d, dy / d, dz / d, b.r - d, BALL.restitutionWall, 0, 0, 0, h); return; }
    const c = [[b.x - bx.minX, -1, 0, 0], [bx.maxX - b.x, 1, 0, 0], [b.y - bx.minY, 0, -1, 0], [bx.maxY - b.y, 0, 1, 0], [bx.zMax - b.z, 0, 0, 1]];
    let m = c[0]; for (const q of c) if (q[0] < m[0]) m = q;
    contact(b, m[1], m[2], m[3], m[0] + b.r, BALL.restitutionWall, 0, 0, 0, h);
  }
  _circleCollide(b, ci, h) {
    if (b.z + b.r < ci.zMin || b.z - b.r > ci.zMax) return;
    const dx = b.x - ci.x, dy = b.y - ci.y, dxy = Math.hypot(dx, dy), rad = Math.min(dxy, ci.r);
    const ux = dxy > 1e-9 ? dx / dxy : 1, uy = dxy > 1e-9 ? dy / dxy : 0;
    const qx = ci.x + ux * rad, qy = ci.y + uy * rad, qz = clamp(b.z, ci.zMin, ci.zMax);
    const ex = b.x - qx, ey = b.y - qy, ez = b.z - qz, d2 = ex * ex + ey * ey + ez * ez;
    if (d2 > b.r * b.r) return;
    if (d2 > 1e-12) { const d = Math.sqrt(d2); contact(b, ex / d, ey / d, ez / d, b.r - d, BALL.restitutionWall, 0, 0, 0, h); return; }
    const radial = ci.r - dxy, top = ci.zMax - b.z;
    if (radial < top) contact(b, ux, uy, 0, radial + b.r, BALL.restitutionWall, 0, 0, 0, h);
    else contact(b, 0, 0, 1, top + b.r, BALL.restitutionWall, 0, 0, 0, h);
  }
  _robotCollide(b, h) {
    const R = this.robot, D = this.robotDims, c = Math.cos(R.h), s = Math.sin(R.h);
    const dx = b.x - R.x, dy = b.y - R.y;
    const reach = Math.hypot(D.halfX, D.halfY) + b.r;
    if (b._ghost) {   // freshly fired from inside the robot: no robot contact until it has cleared the box
      const lx0 = c * dx + s * dy, ly0 = -s * dx + c * dy;
      if (Math.abs(lx0) < D.halfX + b.r && Math.abs(ly0) < D.halfY + b.r && b.z - b.r < D.height) return;
      b._ghost = false;
    }
    if (b.sleeping && b.z > b.r + 0.3 && dx * dx + dy * dy < (reach + 1.5) * (reach + 1.5) && b.z - b.r < D.height + 1) this._wake(b);
    if (dx * dx + dy * dy > reach * reach || b.z - b.r > D.height) return;
    const lx = c * dx + s * dy, ly = -s * dx + c * dy;
    const qx = clamp(lx, -D.halfX, D.halfX), qy = clamp(ly, -D.halfY, D.halfY), qz = clamp(b.z, 0, D.height);
    const ex = lx - qx, ey = ly - qy, ez = b.z - qz, d2 = ex * ex + ey * ey + ez * ez;
    if (d2 > b.r * b.r) return;
    let nlx, nly, nz, pen;
    if (d2 > 1e-12) { const d = Math.sqrt(d2); nlx = ex / d; nly = ey / d; nz = ez / d; pen = b.r - d; }
    else {
      const cands = [[lx + D.halfX, -1, 0, 0], [D.halfX - lx, 1, 0, 0], [ly + D.halfY, 0, -1, 0], [D.halfY - ly, 0, 1, 0], [D.height - b.z, 0, 0, 1]];
      let m = cands[0]; for (const q of cands) if (q[0] < m[0]) m = q;
      nlx = m[1]; nly = m[2]; nz = m[3]; pen = m[0] + b.r;
    }
    if (b.sleeping) this._wake(b);
    const nx = c * nlx - s * nly, ny = s * nlx + c * nly;
    contact(b, nx, ny, nz, pen, BALL.restitutionRobot, R.vx - R.om * dy, R.vy + R.om * dx, 0, h);
  }
  // world -> hive local (relative to pivot, CAD pose) for tip rotation delta
  _loc(hv, delta, x, y, z, o) {
    const c = Math.cos(delta), s = Math.sin(delta), dy = y - hv.pivot[1], dz = z - hv.pivot[2];
    o[0] = x - hv.pivot[0]; o[1] = dy * c + dz * s; o[2] = -dy * s + dz * c;
  }
  // Entry zone (hive local frame). A ball is ENTERING a cell when its centre moves INTO the cell (plane distance falls), lies
  // from ENTRY_DEPTH behind the opening plane to 2r+1 in front of it, and its centre is inside the opening polygon grown by its
  // radius. An entering ball is not bounced off the lip or rim of that opening (the rim is open to it); it is captured as soon
  // as its centre is inside the cell (_hiveCapture). Balls that only graze the outside of the lip still bounce.
  _entering(hv, p, q, r) {
    for (const co of hv.cells) {
      const s = co.sensor, n = s.n, P = s.P;
      const dP = n[0] * (p[0] - P[0]) + n[1] * (p[1] - P[1]) + n[2] * (p[2] - P[2]);
      const dQ = n[0] * (q[0] - P[0]) + n[1] * (q[1] - P[1]) + n[2] * (q[2] - P[2]);
      if (!(dQ < dP) || dQ < -ENTRY_DEPTH || dQ > 2 * r + 1) continue;
      const sl = (q[1] - s.A[1]) * s.u[0] + (q[2] - s.A[2]) * s.u[1];
      if (pointInPoly(q[0], sl, s.op) || distToPolyEdge(q[0], sl, s.op) < r) return true;
    }
    return false;
  }
  _hiveSweep(b, hv, h) {
    const py = hv.pivot[1], pz = hv.pivot[2];
    const dy = b.y - py, dz = b.z - pz;
    if (dy * dy + dz * dz > hv.R * hv.R) return;
    const p = this._lp, q = this._lq;
    this._loc(hv, hv.dPrev, b._px, b._py, b._pz, p);
    this._loc(hv, hv.delta, b.x, b.y, b.z, q);
    for (const co of hv.cells) {
      if (b._enter === hv.name + ':' + co.name) {   // left through the mouth again -> forget
        const s = co.sensor, d = s.n[0] * (q[0] - s.P[0]) + s.n[1] * (q[1] - s.P[1]) + s.n[2] * (q[2] - s.P[2]);
        if (d > 0) b._enter = null;
      }
    }
    const entering = this._entering(hv, p, q, b.r);
    const hit = hv.mesh.sweepSegment(p, q);
    if (hit.hit && !entering) {   // centre crossed a wall within one substep: put the ball back on the side it came from (not when entering the opening)
      const r = b.r + 0.01, nl = hit.normal;
      q[0] = hit.point[0] + nl[0] * r; q[1] = hit.point[1] + nl[1] * r; q[2] = hit.point[2] + nl[2] * r;
      this._toWorld(hv, q, b);
      b._c = true;
      const c = Math.cos(hv.delta), s = Math.sin(hv.delta);
      const nwy = nl[1] * c - nl[2] * s, nwz = nl[1] * s + nl[2] * c, rdy = b.y - py, rdz = b.z - pz;
      contact(b, nl[0], nwy, nwz, 0, BALL.restitutionHive, 0, -hv.omega * rdz, hv.omega * rdy, h);
      return;
    }
    for (const co of hv.cells) if (crossesOpening(p, q, co.sensor) || (entering && crossesGrownOpening(p, q, co.sensor, b.r))) b._enter = hv.name + ':' + co.name;
  }
  _toWorld(hv, q, b) {
    const c = Math.cos(hv.delta), s = Math.sin(hv.delta);
    b.x = hv.pivot[0] + q[0]; b.y = hv.pivot[1] + q[1] * c - q[2] * s; b.z = hv.pivot[2] + q[1] * s + q[2] * c;
  }
  _hiveCollide(b, hv, h) {
    const py = hv.pivot[1], pz = hv.pivot[2];
    let dy = b.y - py, dz = b.z - pz;
    if (dy * dy + dz * dz > hv.R * hv.R) return;
    const q = this._lq, ct = this._ct;
    this._loc(hv, hv.dPrev, b._px, b._py, b._pz, this._lr); this._loc(hv, hv.delta, b.x, b.y, b.z, q);
    if (this._entering(hv, this._lr, q, b.r)) return;   // entering the opening: the rim does not bounce it
    for (let it = 0; it < 4; it++) {
      this._loc(hv, hv.delta, b.x, b.y, b.z, q);
      if (!hv.mesh.sphereContacts(q[0], q[1], q[2], b.r, ct, 1)) break;
      const c = Math.cos(hv.delta), s = Math.sin(hv.delta);
      dy = b.y - py; dz = b.z - pz;
      contact(b, ct[0], ct[1] * c - ct[2] * s, ct[1] * s + ct[2] * c, ct[3], BALL.restitutionHive, 0, -hv.omega * dz, hv.omega * dy, h);
    }
  }
  _hiveCapture(b, hv, h) {
    if (b.sleeping) return;
    const dy = b.y - hv.pivot[1], dz = b.z - hv.pivot[2];
    if (dy * dy + dz * dz > hv.R * hv.R) return;
    const q = this._lq; this._loc(hv, hv.delta, b.x, b.y, b.z, q);
    const slow = b.vx * b.vx + b.vy * b.vy + b.vz * b.vz < BALL.captureSlow * BALL.captureSlow;
    for (const co of hv.cells) {
      if (!insideCell(q, co.sensor)) continue;
      if (b._enter === hv.name + ':' + co.name) { this._capture(b, hv, co); return; }
      // only start-field balls (hive preloads) may be taken by resting inside; every other ball must have crossed the opening
      if (b.start && slow) { b._rest += h; if (b._rest >= BALL.captureRest) { this._capture(b, hv, co); return; } }
      else b._rest = 0;
      return;
    }
    b._rest = 0;
  }
}

// Free balls whose circle overlaps the rectangle centred on pose {x,y,h} with half sizes halfX/halfY (robot frame).
export function ballsInBox(world, pose, halfX, halfY) {
  const c = Math.cos(pose.h || 0), s = Math.sin(pose.h || 0), out = [];
  for (const b of world.balls) {
    if (b.state !== 'free') continue;
    const dx = b.x - pose.x, dy = b.y - pose.y, lx = c * dx + s * dy, ly = -s * dx + c * dy;
    const ex = Math.max(Math.abs(lx) - halfX, 0), ey = Math.max(Math.abs(ly) - halfY, 0);
    if (ex * ex + ey * ey <= b.r * b.r) out.push(b);
  }
  return out;
}
