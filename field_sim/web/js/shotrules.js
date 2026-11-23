/* Shot realism rules for the BIOBUZZ field sim. Pure ES module (no DOM, no three.js).
 * Units: inches, radians (turret/hood in DEGREES in ctx, as in app.js), seconds. Pedro frame.
 *
 * The game has no official launch zone, so realism comes from PHYSICS (range, rpm, hood, turret travel,
 * entry direction, blocked ball path, field bounds, ammo) plus an optional user-configurable rules zone
 * (ctx.legalZone with contains(x, y, alliance) -> bool and describe() -> string; ctx.enforceZone, default true).
 *
 * MODES (ctx.mode):
 *  'auto'   Full auto-aim: turret/hood/rpm follow the solver. Allowed only when every physics check passes
 *           AND turret/hood/rpm are within tolerance of the solver setpoint. Never allowed when infeasible.
 *  'assist' Driver owns turret/hood/rpm (actual values), but every physics rule is still enforced (solver feasibility
 *           for the aim target, rpm/hood/turret limits, min spin, ball path blocked, field, zone, ammo). The ball is also
 *           forward-simulated from the ACTUAL setpoints and `forward.lands` reports whether it enters the opening
 *           (informational only; the shot is still allowed).
 *  'manual' Ball flies from the actual turret/hood/rpm with zero help. Only hard impossibilities block: no ammo,
 *           flywheel below MIN_SPIN_RPM, robot out of the field, ball path blocked within EXIT_BLOCK_IN of the exit, and the
 *           rules zone (if one is given and enforced).
 *  'free'   Practice sandbox: nothing blocks except no ammo (and only if ammo is limited).
 *
 * PHYSICS-ONLY SHOOTING (default, ctx.strict falsy): the shot is allowed whatever the flywheel/turret/hood state is. Every
 * check that used to block (rpm not ready / too low / too high, range, hood, turret, trajectory, behind opening, out of field)
 * is demoted to severity 'warn' (reason.wouldBlock = true) and tells the driver why the ball will probably miss. Only NO_AMMO,
 * WRONG_MODE_PARAM and a configured ILLEGAL_ZONE still block. The ball then leaves with the velocity of the ACTUAL rpm / hood /
 * turret (launchBall), so a low rpm gives a short weak shot that really falls short. ctx.strict = true restores the old gating.
 *
 * The Core solver (shot_core.js, classic script) is taken from ctx.core, setCore(), or globalThis.Core.
 * rpm <-> speed is the exact inverse of app.js: rpm = Core.rpmEstimate(speed, wheelR, eff) * rpmScale,
 * so speed = rpm / rpmScale * 2*pi*wheelR*eff / 60 (identical to the spec formula when rpmScale = 1). */

import { BALL_KINDS, launchScaleFor, LAUNCH_REF_KIND } from './balls.js';
import { rimClearanceIn } from './aimcontrol.js';
import { PARAMS } from './physics/shooter-exit/params.js';
import { exitState, rpmForExitSpeed } from './physics/shooter-exit/exit_model.js';
import { hoodToElevDeg, elevToHoodDeg, exitPointWorld, launchVelocityWorld, makeExitFn } from './physics/shooter-exit/launch_geometry.js';
import { shotSigmas, perturbLaunch, launchVel, analyticHitProbability, monteCarloHit } from './physics/shooter-exit/dispersion.js';
import { mulberry32, makeGaussian } from './physics/shooter-exit/rng.js';
import { predictPose } from './physics/shooter-exit/lead.js';

export const SHOOT_MODES = ['auto', 'assist', 'manual', 'free'];
/* Ball kind of a shot ctx.kind ('pollen' | 'nectar'); anything else is the reference pollen. The kind sets the ball radius
 * (flight clearance and opening rim) and the launch-speed mass factor (balls.js launchScaleFor). */
const kindName = (ctx) => (ctx && ctx.kind && BALL_KINDS[ctx.kind] ? ctx.kind : LAUNCH_REF_KIND);
export const REASON = {
  NO_SOLUTION: 'NO_SOLUTION', OUT_OF_RANGE_FAR: 'OUT_OF_RANGE_FAR', OUT_OF_RANGE_NEAR: 'OUT_OF_RANGE_NEAR',
  RPM_TOO_HIGH: 'RPM_TOO_HIGH', HOOD_LIMIT: 'HOOD_LIMIT', TURRET_LIMIT: 'TURRET_LIMIT',
  BEHIND_OPENING: 'BEHIND_OPENING', TRAJECTORY_BLOCKED: 'TRAJECTORY_BLOCKED',
  FLYWHEEL_NOT_READY: 'FLYWHEEL_NOT_READY', TURRET_NOT_READY: 'TURRET_NOT_READY', HOOD_NOT_READY: 'HOOD_NOT_READY',
  NO_AMMO: 'NO_AMMO', ILLEGAL_ZONE: 'ILLEGAL_ZONE', OUT_OF_FIELD: 'OUT_OF_FIELD', WRONG_MODE_PARAM: 'WRONG_MODE_PARAM',
  LOW_HIT_PROB: 'LOW_HIT_PROB',
};

export const G = 386.0886;
export const BALL_R = 1.391;
export const MIN_SPIN_RPM = 300;
export const EXIT_BLOCK_IN = 24;
const DEFAULT_BOUNDS = { minX: 1.326, minY: 1.326, maxX: 142.674, maxY: 142.674 };
const DEFAULT_ENTRY_DEG = 50;
const HARD = new Set(['NO_AMMO', 'WRONG_MODE_PARAM', 'ILLEGAL_ZONE']);   // still block in physics-only mode
const DEG = 180 / Math.PI, RAD = Math.PI / 180;

let _core = null;
export function setCore(c) { _core = c; }
function getCore(ctx) {
  const c = (ctx && ctx.core) || _core || (typeof globalThis !== 'undefined' ? globalThis.Core : null);
  if (!c) throw new Error('shotrules: Core solver not available (call setCore or load shot_core.js)');
  return c;
}

// ---------------------------------------------------------------- geometry helpers
function wrapDeg(a) { a = ((a + 180) % 360 + 360) % 360 - 180; return a; }

function allBoxes(obstacles, hiveBoxes) {
  const out = [];
  const add = (b, kind) => out.push([b.minX, b.minY, b.maxX, b.maxY, b.zMin == null ? 0 : b.zMin, b.zMax == null ? 1e9 : b.zMax, b.kind || kind]);
  if (obstacles && obstacles.boxes) for (const b of obstacles.boxes) add(b, 'box');
  if (hiveBoxes) for (const b of hiveBoxes) add(b, 'hive');
  return out;
}

/** Core ballistic flight with obstacle tests. Returns {blocked, at, by, cross:{p,v,t}|null, end:{p,t}, startDist}. */
function flight(start, vel, obstacles, hiveBoxes, opts) {
  opts = opts || {};
  const dt = opts.dt || 0.002, tMax = opts.tMax || 3, R = opts.ballR == null ? BALL_R : opts.ballR;
  const ignore = opts.ignoreStart == null ? 4 : opts.ignoreStart;
  const boxes = opts._boxes || allBoxes(obstacles, hiveBoxes);
  const b = (obstacles && obstacles.bounds) || opts.bounds || DEFAULT_BOUNDS;
  const tg = opts.target || null, n = tg && tg.normal ? tg.normal : null;
  const nb = boxes.length;
  let prevS = null, px = start[0], py = start[1], pz = start[2], pt = 0;
  if (tg && n) prevS = (px - tg.x) * n[0] + (py - tg.y) * n[1] + (pz - tg.z) * n[2];
  const res = { blocked: false, at: null, by: null, cross: null, end: null };
  const steps = Math.ceil(tMax / dt);
  for (let i = 1; i <= steps; i++) {
    const t = i * dt;
    const x = start[0] + vel[0] * t, y = start[1] + vel[1] * t, z = start[2] + vel[2] * t - 0.5 * G * t * t;
    const dx0 = x - start[0], dy0 = y - start[1], dz0 = z - start[2];
    const far = dx0 * dx0 + dy0 * dy0 + dz0 * dz0 > ignore * ignore;
    if (far) {
      if (x - R < b.minX || x + R > b.maxX || y - R < b.minY || y + R > b.maxY) {
        res.blocked = true; res.at = [x, y, z]; res.by = 'wall'; return res;
      }
      for (let k = 0; k < nb; k++) {
        const bx = boxes[k];
        if (z + R < bx[4] || z - R > bx[5]) continue;
        const cx = x < bx[0] ? bx[0] : x > bx[2] ? bx[2] : x;
        const cy = y < bx[1] ? bx[1] : y > bx[3] ? bx[3] : y;
        const cz = z < bx[4] ? bx[4] : z > bx[5] ? bx[5] : z;
        const ddx = x - cx, ddy = y - cy, ddz = z - cz;
        if (ddx * ddx + ddy * ddy + ddz * ddz <= R * R) {
          res.blocked = true; res.at = [x, y, z]; res.by = bx[6]; return res;
        }
      }
    }
    if (prevS !== null) {
      const s = (x - tg.x) * n[0] + (y - tg.y) * n[1] + (z - tg.z) * n[2];
      if (prevS > 0 && s <= 0) {
        const f = prevS / (prevS - s), tc = pt + f * dt;
        res.cross = { p: [px + (x - px) * f, py + (y - py) * f, pz + (z - pz) * f], v: [vel[0], vel[1], vel[2] - G * tc], t: tc };
        res.end = { p: res.cross.p, t: tc };
        return res;
      }
      prevS = s;
    }
    px = x; py = y; pz = z; pt = t;
    if (z <= R && vel[2] - G * t < 0) break;
  }
  res.end = { p: [px, py, pz], t: pt };
  return res;
}

/** Sample the ballistic path (2 ms steps) until the target plane, the floor or 3 s. Sphere vs extruded boxes + field walls.
 *  opts: {target:{x,y,z,normal}, dt, tMax, ballR, ignoreStart (in), bounds}. */
export function trajectoryBlocked(start, vel, obstacles, hiveBoxes, opts) {
  const r = flight(start, vel, obstacles, hiveBoxes, opts);
  return { blocked: r.blocked, at: r.at, by: r.by };
}

// ---------------------------------------------------------------- shooter kinematics
/* Exit model: 'legacy' (default, bit-identical to the original formula) or 'physical' (shooter-exit/exit_model.js, opt-in via
 * shooter.exitModel === 'physical'). */
export function exitModelOf(sh) { return sh && sh.exitModel === 'physical' ? 'physical' : 'legacy'; }
const gapOf = (sh, kind) => (sh && sh.gapIn ? sh.gapIn[kind] : undefined);
const exitOpts = (sh, kind) => ({ model: exitModelOf(sh), wheelR: sh.wheelR, eff: sh.eff, rpmScale: sh.rpmScale || 1, gapIn: gapOf(sh, kind), maxRpm: sh.maxRpm });
const geomOpts = (sh, kind) => ({ model: exitModelOf(sh), gapIn: gapOf(sh, kind), wheelR: sh.wheelR });
// Wheel-surface speed from rpm (the existing model), then the ball's mass factor: exit speed = wheel speed * massScale.
function speedFromRpm(rpm, sh, massScale, kind) {
  if (exitModelOf(sh) === 'physical') return exitState(rpm, kind || LAUNCH_REF_KIND, exitOpts(sh, kind || LAUNCH_REF_KIND)).speed;
  return Math.max(0, rpm) / (sh.rpmScale || 1) * 2 * Math.PI * sh.wheelR * sh.eff / 60 * (massScale == null ? 1 : massScale);
}
// Inverse: rpm needed for a ball of massScale to leave at `speed` (exact inverse of speedFromRpm). Physical: NaN (unreachable) -> Infinity.
function rpmFromSpeed(speed, sh, core, massScale, kind) {
  if (exitModelOf(sh) === 'physical') {
    const k = kind || LAUNCH_REF_KIND, r = rpmForExitSpeed(speed, k, exitOpts(sh, k));
    return Number.isNaN(r) ? Infinity : r;
  }
  return core.rpmEstimate(speed / (massScale == null ? 1 : massScale), sh.wheelR, sh.eff) * (sh.rpmScale || 1);
}
// Hood actuator angle [deg] of a solver result (legacy: elevation in deg, exactly r.elev*DEG).
function hoodOf(sol, sh, kind) {
  if (exitModelOf(sh) !== 'physical') return sol.elev * DEG;
  return sol.exitOffset && sol.hoodDeg != null ? sol.hoodDeg : elevToHoodDeg(sol.elev * DEG, kind, geomOpts(sh, kind));
}

function pivotWorld(pose, sh) {
  const c = Math.cos(pose.h), s = Math.sin(pose.h), px = sh.pivotX || 0, py = sh.pivotY || 0;
  return [pose.x + c * px - s * py, pose.y + s * px + c * py];
}

/** Ball launch from the REAL turret/hood/rpm, for the kind of ctx.kind (default pollen). Returns the kind and its radius too.
 *  Physical exit model adds spinRadS, compression, contactTimeS, sigma (and uses the hood map + exit geometry). */
export function forwardShot(ctx) {
  const { pose, shooter: sh } = ctx;
  const kind = kindName(ctx), ms = launchScaleFor(kind);
  if (exitModelOf(sh) === 'physical') {
    const go = geomOpts(sh, kind), st = exitState(ctx.rpm || 0, kind, exitOpts(sh, kind));
    const az = pose.h + (ctx.turretDeg || 0) * RAD, el = hoodToElevDeg(ctx.hoodDeg || 0, kind, go) * RAD;
    const exit = exitPointWorld(pose, sh, az, el, kind, go);
    const { vel, vBase } = launchVelocityWorld({ pose, robotVel: ctx.robotVel, sh, azRad: az, elevRad: el, speed: st.speed, exit, turretRateDegS: ctx.turretRateDegS || 0 });
    return { speed: st.speed, elevRad: el, azRad: az, vel, exit, kind, ballR: rimClearanceIn(kind), massScale: ms, vBase,
      spinRadS: st.spinRadS, compression: st.compression, contactTimeS: st.contactTimeS, sigma: shotSigmas(kind, { gapIn: gapOf(sh, kind) }) };
  }
  const az = pose.h + (ctx.turretDeg || 0) * RAD, el = (ctx.hoodDeg || 0) * RAD;
  const speed = speedFromRpm(ctx.rpm || 0, sh, ms, kind);
  const [px, py] = pivotWorld(pose, sh);
  const rv = ctx.robotVel || pose;   // optional override {vx,vy,om} (robot velocity at the moment of firing)
  const vx0 = (rv.vx || 0) - (rv.om || 0) * (py - pose.y), vy0 = (rv.vy || 0) + (rv.om || 0) * (px - pose.x);
  const vel = [vx0 + Math.cos(az) * Math.cos(el) * speed, vy0 + Math.sin(az) * Math.cos(el) * speed, Math.sin(el) * speed];
  return { speed, elevRad: el, azRad: az, vel, exit: [px, py, sh.exitHeight], kind, ballR: rimClearanceIn(kind), massScale: ms };
}

/** The ball that leaves the shooter right now: position and velocity from the ACTUAL rpm / hood / turret / robot motion.
 *  Never refuses (rpm 0 gives speed 0, the ball just drops out). Same numbers as forwardShot unless ctx.noise is given:
 *  ctx.noise {seed?, rng?} scatters speed/elevation/azimuth/release time (dispersion.js); rng persists in ctx.noise.rng. */
export function launchBall(ctx) {
  const f = forwardShot(ctx);
  const out = { pos: f.exit, vel: f.vel, speed: f.speed, elevRad: f.elevRad, azRad: f.azRad, kind: f.kind, ballR: f.ballR,
    spinRadS: f.spinRadS, noiseApplied: false };
  if (ctx.noise) {
    const nz = ctx.noise;
    if (!nz.rng) nz.rng = mulberry32(nz.seed == null ? PARAMS.dispersion.SEED : nz.seed);
    if (!nz.gauss) nz.gauss = makeGaussian(nz.rng);
    const sig = f.sigma || shotSigmas(f.kind, { gapIn: gapOf(ctx.shooter, f.kind) });
    const pl = perturbLaunch(launchFrom(f), sig, nz.gauss);
    out.pos = pl.pos; out.speed = pl.speed; out.elevRad = pl.elevRad; out.azRad = pl.azRad; out.vel = launchVel(pl);
    out.noiseApplied = true;
  }
  return out;
}

// Dispersion launch object of a forwardShot: vBase = vel - speed*direction (works for both exit models).
function launchFrom(f) {
  const ce = Math.cos(f.elevRad);
  const vBase = f.vBase || [f.vel[0] - f.speed * ce * Math.cos(f.azRad), f.vel[1] - f.speed * ce * Math.sin(f.azRad), f.vel[2] - f.speed * Math.sin(f.elevRad)];
  return { pos: f.exit.slice(), speed: f.speed, elevRad: f.elevRad, azRad: f.azRad, vBase };
}

function targetOf(ctx) { return ctx.target; }

/** Where would the ball cross the opening plane? lands = inside the opening rectangle (ball radius allowance), arriving from the front. */
function landing(fs, ctx, boxes) {
  const tg = ctx.target, env = ctx.obstacles, R = fs.ballR;
  const fl = flight(fs.exit, fs.vel, env, ctx.hiveBoxes, { target: tg, _boxes: boxes, ballR: R });
  const out = { lands: false, blocked: fl.blocked, by: fl.by, at: fl.at, cross: fl.cross ? fl.cross.p : null, miss: null };
  if (!fl.cross) return out;
  const n = tg.normal, hn = Math.hypot(n[0], n[1]) || 1;
  const lat = [-n[1] / hn, n[0] / hn, 0];                         // horizontal axis in the opening plane
  const up = [n[1] * lat[2] - n[2] * lat[1], n[2] * lat[0] - n[0] * lat[2], n[0] * lat[1] - n[1] * lat[0]];
  const d = [fl.cross.p[0] - tg.x, fl.cross.p[1] - tg.y, fl.cross.p[2] - tg.z];
  const a = d[0] * lat[0] + d[1] * lat[1] + d[2] * lat[2], v = d[0] * up[0] + d[1] * up[1] + d[2] * up[2];
  const hw = (tg.halfW || 10) - R, hh = (tg.halfH || 7) - R;   // rim clearance = this kind's radius
  out.miss = [a, v];
  out.lands = !fl.blocked && Math.abs(a) <= hw && Math.abs(v) <= hh;
  return out;
}

// ---------------------------------------------------------------- auto solver (mirrors app.js solveShot)
function solveAuto(pose, sh, tg, core, kind, robotVel, leadS) {
  let rv = robotVel || pose;   // robot velocity used by the solver (closed form: the pivot velocity enters the ball's ground velocity)
  if (leadS > 0) {   // latency look-ahead: solve for the pose/velocity the robot will have at release
    const pp = predictPose(pose, { vx: rv.vx || 0, vy: rv.vy || 0, om: rv.om || 0 }, leadS);
    pose = { ...pose, x: pp.x, y: pp.y, h: pp.h }; rv = { vx: pp.vx, vy: pp.vy, om: pp.om };
  }
  const p = { x: pose.x, y: pose.y, psi: pose.h, vx: rv.vx || 0, vy: rv.vy || 0, om: rv.om || 0,
    rx: sh.pivotX || 0, ry: sh.pivotY || 0, z0: sh.exitHeight, X: tg.x, Y: tg.y, Z: tg.z };
  const physical = exitModelOf(sh) === 'physical';
  const gam = (sh.entryDeg == null ? DEFAULT_ENTRY_DEG : sh.entryDeg) * RAD;
  const kk = kind || LAUNCH_REF_KIND, go = geomOpts(sh, kk);
  const exitFn = physical ? makeExitFn(pose, sh, kk, go) : null;
  const slv = physical ? (mode, param) => core.solveExit(p, mode, param, 'low', exitFn) : (mode, param) => core.solve(p, mode, param, 'low');
  let r = slv('gamma', gam);
  if (r.ok) {
    const e = physical ? elevToHoodDeg(r.elev * DEG, kk, go) : r.elev * DEG, c = Math.min(sh.hoodMax, Math.max(sh.hoodMin, e));
    if (c !== e) { const r2 = slv('theta', (physical ? hoodToElevDeg(c, kk, go) : c) * RAD); if (r2.ok) r = r2; }
  }
  if (!physical) r.p = p;   // physical: solveExit already set r.p to the offset-exit parameters
  // r.speed is the BALL exit speed the aim needs; the rpm is the wheel rpm that gives that speed for this kind's mass.
  r.rpm = r.ok ? rpmFromSpeed(r.speed, sh, core, launchScaleFor(kk), kk) : 0;
  r.turretDeg = r.ok ? r.turret * DEG : 0;
  r.hoodDeg = physical ? (r.ok ? elevToHoodDeg(r.elev * DEG, kk, go) : NaN) : r.elev * DEG;
  if (r.ok) {
    const n = tg.normal, vg = [r.vs[0] + r.uh[0], r.vs[1] + r.uh[1], r.uz - G * r.T];
    r.fromBehind = n[0] * vg[0] + n[1] * vg[1] + n[2] * vg[2] >= 0;
  }
  return r;
}

const rangeCache = new Map();
let _paramsKey = null;
function paramsKey() { return _paramsKey || (_paramsKey = JSON.stringify([PARAMS.geom, PARAMS.hood, PARAMS.compression, PARAMS.model, PARAMS.wheel, PARAMS.slip, PARAMS.ball])); }
function rangeKey(sh, tg, kind) {
  const base = [sh.exitHeight, sh.wheelR, sh.eff, sh.rpmScale, sh.maxRpm, sh.hoodMin, sh.hoodMax, sh.entryDeg, sh.pivotX, sh.pivotY, tg.z, kind].join('|');
  const phys = exitModelOf(sh) === 'physical';
  return base + '|' + exitModelOf(sh) + '|' + JSON.stringify(sh.gapIn || null) + (phys ? '|' + paramsKey() : '');
}

function stationaryOk(d, sh, tg, core, kind) {
  const r = solveAuto({ x: tg.x - d, y: tg.y, h: 0 }, sh, { x: tg.x, y: tg.y, z: tg.z, normal: [-1, 0, 0] }, core, kind);
  if (!r.ok) return false;
  const e = r.hoodDeg;
  return r.rpm <= sh.maxRpm && e >= sh.hoodMin - 1e-9 && e <= sh.hoodMax + 1e-9 && !r.fromBehind;
}
function ranges(sh, tg, core, kind) {
  const key = rangeKey(sh, tg, kind);
  let r = rangeCache.get(key);
  if (r) return r;
  let lo = null, hi = null;
  const STEP = 1, MAXD = 1500;
  for (let d = 1; d <= MAXD; d += STEP) {
    const ok = stationaryOk(d, sh, tg, core, kind);
    if (ok && lo === null) lo = d;
    if (ok) hi = d;
  }
  if (lo === null) { r = { near: NaN, far: NaN }; }
  else {
    // refine edges by bisection
    let a = Math.max(0.01, lo - STEP), b = lo;
    if (!stationaryOk(a, sh, tg, core, kind)) for (let i = 0; i < 40; i++) { const m = 0.5 * (a + b); if (stationaryOk(m, sh, tg, core, kind)) b = m; else a = m; } else b = a;
    const near = b;
    a = hi; b = hi + STEP;
    for (let i = 0; i < 40; i++) { const m = 0.5 * (a + b); if (stationaryOk(m, sh, tg, core, kind)) a = m; else b = m; }
    r = { near, far: a };
  }
  rangeCache.set(key, r);
  return r;
}
/** Farthest horizontal distance (pivot to aim point, stationary) with an auto solution inside rpm/hood limits. kind: pollen|nectar. */
export function maxRangeEstimate(shooter, target, core, kind) { return ranges(shooter, target, core || getCore(), kindName({ kind })).far; }
/** Nearest such distance. */
export function nearestRangeEstimate(shooter, target, core, kind) { return ranges(shooter, target, core || getCore(), kindName({ kind })).near; }
/** Auto solution for one ball kind from a pose (the same solve the auto mode uses): {ok, turretDeg, elevDeg, rpm, speed, ...}. */
/* robotVel (optional, trailing, default = pose.vx/vy/om): {vx,vy,om} of the robot at the moment of firing. */
export function solveAutoShot(pose, shooter, target, kind, core, robotVel, leadS) { return solveAuto(pose, shooter, target, core || getCore(), kindName({ kind }), robotVel, leadS); }

// ---------------------------------------------------------------- the physics rule set shared by auto/assist/region
function R(code, severity, text) { return { code, severity, text }; }

/** Evaluate the solver feasibility for the aim target. Returns {reasons, sol}. checkPath: also test the solver path vs obstacles. */
function autoPhysics(pose, sh, tg, ctx, core, sol, checkPath, boxes) {
  const reasons = [], kind = kindName(ctx), ballR = rimClearanceIn(kind);
  if (!sol) sol = solveAuto(pose, sh, tg, core, kind, ctx.robotVel, ctx.leadS);
  const [px, py] = pivotWorld(pose, sh);
  const d = Math.hypot(tg.x - px, tg.y - py);
  const rg = ranges(sh, tg, core, kind);
  if (!sol.ok) {
    if (rg.far === rg.far && d > rg.far) reasons.push(R(REASON.OUT_OF_RANGE_FAR, 'block', `Too far: ${d.toFixed(0)} in away, max range is about ${Math.floor(rg.far)} in`));
    else if (rg.near === rg.near && d < rg.near) reasons.push(R(REASON.OUT_OF_RANGE_NEAR, 'block', `Too close: ${d.toFixed(0)} in away, min range is about ${Math.ceil(rg.near)} in`));
    else reasons.push(R(REASON.NO_SOLUTION, 'block', 'No ballistic solution to the opening from here' + (sol.reason ? ` (${sol.reason})` : '')));
    return { reasons, sol, d };
  }
  const elev = hoodOf(sol, sh, kind), rpm = sol.rpm;
  if (elev < sh.hoodMin - 1e-6 || elev > sh.hoodMax + 1e-6) {
    if (rg.far === rg.far && d > rg.far) reasons.push(R(REASON.OUT_OF_RANGE_FAR, 'block', `Too far: ${d.toFixed(0)} in away, max range is about ${Math.floor(rg.far)} in`));
    else if (rg.near === rg.near && d < rg.near) reasons.push(R(REASON.OUT_OF_RANGE_NEAR, 'block', `Too close: ${d.toFixed(0)} in away, min range is about ${Math.ceil(rg.near)} in`));
    reasons.push(R(REASON.HOOD_LIMIT, 'block', `Needs hood ${elev.toFixed(0)} deg, limits are ${sh.hoodMin}..${sh.hoodMax}`));
  }
  if (rpm > sh.maxRpm) {
    reasons.push(R(REASON.RPM_TOO_HIGH, 'block', `Needs ${Math.round(rpm)} rpm, flywheel max is ${sh.maxRpm}`));
    if (rg.far === rg.far && d > rg.far && !reasons.some(x => x.code === REASON.OUT_OF_RANGE_FAR)) reasons.push(R(REASON.OUT_OF_RANGE_FAR, 'block', `Too far: ${d.toFixed(0)} in away, max range is about ${Math.floor(rg.far)} in`));
  }
  if (sol.fromBehind) reasons.push(R(REASON.BEHIND_OPENING, 'block', 'Ball would arrive from behind the hive opening'));
  if (checkPath) {
    const vel = [sol.vs[0] + sol.uh[0], sol.vs[1] + sol.uh[1], sol.uz];
    const fl = flight([sol.P[0], sol.P[1], sol.p && sol.p.z0 != null && exitModelOf(sh) === 'physical' ? sol.p.z0 : sh.exitHeight], vel, ctx.obstacles, ctx.hiveBoxes, { target: tg, _boxes: boxes, ballR });
    if (fl.blocked) reasons.push(R(REASON.TRAJECTORY_BLOCKED, 'block', `Ball path blocked by ${fl.by} at (${fl.at.map(v => v.toFixed(0)).join(', ')})`));
  }
  const pri = c => (c === REASON.OUT_OF_RANGE_FAR || c === REASON.OUT_OF_RANGE_NEAR ? 0 : 1);
  reasons.sort((a, b) => pri(a.code) - pri(b.code));   // stable: range explanations first
  return { reasons, sol, d };
}

function turretEquivalent(sol, sh, actual) {
  let best = null;
  for (let k = -2; k <= 2; k++) {
    const c = sol.turretDeg + 360 * k;
    if (c >= sh.turretMinDeg && c <= sh.turretMaxDeg && (best === null || Math.abs(c - actual) < Math.abs(best - actual))) best = c;
  }
  return best;
}

function inField(pose, b) { return pose.x >= b.minX && pose.x <= b.maxX && pose.y >= b.minY && pose.y <= b.maxY; }

function hintFor(reasons, sh, tg, core, kind) {
  const blocks = reasons.filter(r => r.severity === 'block');
  if (!blocks.length) return null;
  const rg = ranges(sh, tg, core, kind);
  for (const r of blocks) {
    switch (r.code) {
      case REASON.OUT_OF_RANGE_FAR: return `Move closer than ${Math.floor(rg.far)} in to the hive`;
      case REASON.OUT_OF_RANGE_NEAR: return `Back up to at least ${Math.ceil(rg.near)} in from the hive`;
      case REASON.RPM_TOO_HIGH: return 'Move closer or shoot with the robot moving toward the hive';
      case REASON.HOOD_LIMIT: return 'Change distance so the hood stays within its limits';
      case REASON.BEHIND_OPENING: return 'Shoot from the open (front) side of the hive';
      case REASON.TRAJECTORY_BLOCKED: return 'Move to a spot where the hive frame, legs or walls are not in the way';
      case REASON.TURRET_LIMIT: return 'Rotate the robot so the turret can reach the target';
      case REASON.NO_AMMO: return 'Intake more balls first';
      case REASON.OUT_OF_FIELD: return 'Drive back inside the field';
      case REASON.ILLEGAL_ZONE: return 'Drive into the allowed shooting zone';
      case REASON.FLYWHEEL_NOT_READY: return 'Wait for the flywheel to spin up';
      case REASON.TURRET_NOT_READY: return 'Wait for the turret to settle on the target';
      case REASON.LOW_HIT_PROB: return 'Move closer or to a steadier pose to raise the hit probability';
      case REASON.HOOD_NOT_READY: return 'Wait for the hood to settle';
      case REASON.WRONG_MODE_PARAM: return 'Use one of: ' + SHOOT_MODES.join(', ');
      default: return 'Reposition and try again';
    }
  }
  return null;
}

// ---------------------------------------------------------------- dispersion (opt-in: ctx.dispersion / ctx.minHitProb)
// Launch object ({pos, speed, elevRad, azRad, vBase}) of the auto solution (exit point, ground velocity of the exit, ball speed).
function solverLaunch(sol, sh, kind) {
  const z0 = sol.p && sol.p.z0 != null && exitModelOf(sh) === 'physical' ? sol.p.z0 : sh.exitHeight;
  return { pos: [sol.P[0], sol.P[1], z0], speed: sol.speed, elevRad: sol.elev, azRad: sol.az, vBase: [sol.vs[0], sol.vs[1], 0], ballR: rimClearanceIn(kind), sigma: shotSigmas(kind, { gapIn: gapOf(sh, kind) }) };
}
function dispOpts(ctx, opts) {
  const dc = ctx.dispersion && typeof ctx.dispersion === 'object' ? ctx.dispersion : {};
  const o = opts || {};
  return { method: o.method || dc.method || 'analytic', n: o.n || dc.n, seed: o.seed != null ? o.seed : dc.seed };
}
/** Hit probability of a launch {pos, speed, elevRad, azRad, vBase, ballR?, sigma?}: analytic, Monte Carlo (through landing()) or both. */
function dispersionFor(ctx, launch, boxes, kind, opts) {
  const o = dispOpts(ctx, opts), core = getCore(ctx), tg = ctx.target;
  const ballR = launch.ballR != null ? launch.ballR : rimClearanceIn(kind);
  const sig = launch.sigma || shotSigmas(kind, { gapIn: gapOf(ctx.shooter, kind) });
  const res = { method: o.method };
  if (o.method !== 'mc') res.analytic = analyticHitProbability({ launch, sig, target: tg, ballR, core });
  if (o.method === 'mc' || o.method === 'both') {
    const mcOpts = { launch, sig, simulate: (l) => { const r = landing({ exit: l.pos, vel: launchVel(l), ballR }, ctx, boxes); return { lands: r.lands, miss: r.miss }; } };
    if (o.n) mcOpts.n = o.n;
    if (o.seed != null) mcOpts.seed = o.seed;
    res.mc = monteCarloHit(mcOpts);
  }
  res.p = o.method === 'mc' ? res.mc.p : res.analytic.p;
  res.spread = o.method === 'mc' ? res.mc.spread : res.analytic.sigma;
  return res;
}
// Adds forward.hitProb/spread/method (when ctx.dispersion) and LOW_HIT_PROB (when ctx.minHitProb) for assist/manual.
function dispersionOf(ctx, fs, forward, boxes, reasons) {
  if (!ctx.dispersion && ctx.minHitProb == null) return;
  const d = dispersionFor(ctx, { ...launchFrom(fs), ballR: fs.ballR, sigma: fs.sigma }, boxes, fs.kind);
  if (ctx.dispersion) { forward.hitProb = d.p; forward.spread = d.spread; forward.method = d.method; }
  if (ctx.minHitProb != null && d.p < ctx.minHitProb) reasons.push(R(REASON.LOW_HIT_PROB, 'block', `Hit probability ${(d.p * 100).toFixed(0)}% is below ${(ctx.minHitProb * 100).toFixed(0)}%`));
}

/** Hit probability for the shot of ctx (same ctx as evaluateShot). auto: the solver launch; assist/manual: the actual setpoints.
 *  opts {method:'analytic'|'mc'|'both' = 'analytic', n, seed}. Returns {p, method, analytic?, mc?, spread:[sigmaLat, sigmaUp]}. */
export function hitProbability(ctx, opts) {
  const kind = kindName(ctx), boxes = allBoxes(ctx.obstacles, ctx.hiveBoxes);
  let launch;
  if (ctx.mode === 'auto') {
    const sol = solveAuto(ctx.pose, ctx.shooter, ctx.target, getCore(ctx), kind, ctx.robotVel, ctx.leadS);
    if (!sol.ok) return { p: 0, method: dispOpts(ctx, opts).method, spread: [0, 0] };
    launch = solverLaunch(sol, ctx.shooter, kind);
  } else {
    const fs = forwardShot(ctx);
    launch = { ...launchFrom(fs), ballR: fs.ballR, sigma: fs.sigma };
  }
  const d = dispersionFor(ctx, launch, boxes, kind, opts);
  const out = { p: d.p, method: d.method, spread: d.spread };
  if (d.analytic) out.analytic = d.analytic;
  if (d.mc) out.mc = d.mc;
  return out;
}

/** Main entry: should this shot be allowed, and why not. */
export function evaluateShot(ctx) {
  const mode = ctx.mode;
  const reasons = [];
  const finish = () => {
    let hint = null;
    try { hint = ctx.target && ctx.shooter ? hintFor(reasons, ctx.shooter, ctx.target, getCore(ctx), kindName(ctx)) : null; } catch (e) { hint = null; }
    if (!ctx.strict) {
      for (const r of reasons) if (r.severity === 'block' && !HARD.has(r.code)) { r.severity = 'warn'; r.wouldBlock = true; }
    }
    const allowed = !reasons.some(r => r.severity === 'block');
    const blocks = reasons.filter(r => r.severity === 'block');
    const wouldMiss = reasons.filter(r => r.wouldBlock);
    const summary = allowed ? (wouldMiss.length ? 'Will fire, likely to miss: ' + wouldMiss[0].text : mode === 'auto' ? 'READY' : mode === 'free' ? 'FREE: no rules' : 'OK to shoot') : 'Off target: ' + blocks[0].text;
    if (!hint && blocks.length && blocks[0].code === REASON.WRONG_MODE_PARAM) hint = 'Use one of: ' + SHOOT_MODES.join(', ');
    const out = { allowed, reasons, summary, hint };
    if (forward) out.forward = forward;
    if (autoHit != null) out.hitProb = autoHit;
    return out;
  };
  let forward = null, autoHit = null;
  if (!SHOOT_MODES.includes(mode)) { reasons.push(R(REASON.WRONG_MODE_PARAM, 'block', `Unknown shoot mode '${mode}'`)); return finish(); }

  if (ctx.ammo && ctx.ammo.limited && !(ctx.ammo.count > 0)) reasons.push(R(REASON.NO_AMMO, 'block', 'No balls loaded'));
  if (mode === 'free') return finish();

  const core = getCore(ctx), sh = ctx.shooter, tg = targetOf(ctx), pose = ctx.pose;
  const bounds = (ctx.obstacles && ctx.obstacles.bounds) || DEFAULT_BOUNDS;
  const boxes = allBoxes(ctx.obstacles, ctx.hiveBoxes);
  const tolR = ctx.toleranceRpmFrac == null ? 0.04 : ctx.toleranceRpmFrac;
  const tolT = ctx.toleranceTurretDeg == null ? 2 : ctx.toleranceTurretDeg;
  const tolH = ctx.toleranceHoodDeg == null ? 2 : ctx.toleranceHoodDeg;

  if (!inField(pose, bounds)) reasons.push(R(REASON.OUT_OF_FIELD, 'block', 'Robot is outside the field'));
  if (ctx.legalZone && ctx.enforceZone !== false && !ctx.legalZone.contains(pose.x, pose.y, ctx.alliance)) {
    reasons.push(R(REASON.ILLEGAL_ZONE, 'block', 'Outside the allowed shooting zone' + (ctx.legalZone.describe ? ': ' + ctx.legalZone.describe() : '')));
  }

  if (mode === 'manual') {
    if ((ctx.rpm || 0) < MIN_SPIN_RPM) reasons.push(R(REASON.FLYWHEEL_NOT_READY, 'block', `Flywheel below ${MIN_SPIN_RPM} rpm`));
    const fs = forwardShot(ctx);
    forward = landing(fs, ctx, boxes);
    if (forward.blocked) {
      const dd = Math.hypot(forward.at[0] - fs.exit[0], forward.at[1] - fs.exit[1], forward.at[2] - fs.exit[2]);
      reasons.push(R(REASON.TRAJECTORY_BLOCKED, dd <= EXIT_BLOCK_IN ? 'block' : 'warn', `Ball path blocked by ${forward.by} ${dd.toFixed(0)} in from the exit`));
    }
    dispersionOf(ctx, fs, forward, boxes, reasons);
    reasons.push({ code: 'FORWARD_SIM', severity: 'info', text: forward.lands ? 'Ball would enter the opening' : 'Ball would miss the opening' });
    return finish();
  }

  // auto + assist: physics on the aim target
  const sol0 = mode === 'auto' && ctx.sol && ctx.sol.ok && ctx.sol.uh ? { ...ctx.sol } : null;   // copy: do not mutate the caller's solution
  const ms0 = launchScaleFor(kindName(ctx));
  if (sol0) {
    // the caller's solution holds the pollen rpm; a non-reference kind needs its own rpm for the same exit speed
    if (sol0.rpm == null || ms0 !== 1 || exitModelOf(sh) === 'physical') sol0.rpm = rpmFromSpeed(sol0.speed, sh, core, ms0, kindName(ctx));
    if (sol0.turretDeg == null) sol0.turretDeg = sol0.turret * DEG;
    if (sol0.fromBehind == null) { const n = tg.normal; sol0.fromBehind = n[0] * (sol0.vs[0] + sol0.uh[0]) + n[1] * (sol0.vs[1] + sol0.uh[1]) + n[2] * (sol0.uz - G * sol0.T) >= 0; }
  }
  const ap = autoPhysics(pose, sh, tg, ctx, core, sol0, mode === 'auto', boxes);
  const sol = ap.sol;
  for (const r of ap.reasons) reasons.push(r);

  if (mode === 'auto') {
    if (sol.ok) {
      const tc = turretEquivalent(sol, sh, ctx.turretDeg || 0);
      if (tc === null) reasons.push(R(REASON.TURRET_LIMIT, 'block', `Turret would need ${sol.turretDeg.toFixed(0)} deg, travel is ${sh.turretMinDeg}..${sh.turretMaxDeg}`));
      else {
        const tErr = Math.abs(tc - (ctx.turretDeg || 0));
        if (tErr > tolT) reasons.push(R(REASON.TURRET_NOT_READY, 'block', `Turret ${tErr.toFixed(1)} deg off target`));
      }
      const hErr = Math.abs(hoodOf(sol, sh, kindName(ctx)) - (ctx.hoodDeg || 0));
      if (hErr > tolH) reasons.push(R(REASON.HOOD_NOT_READY, 'block', `Hood ${hErr.toFixed(1)} deg off setpoint`));
      const rErr = Math.abs((ctx.rpm || 0) - sol.rpm) / Math.max(sol.rpm, 1);
      if ((ctx.rpm || 0) < MIN_SPIN_RPM) reasons.push(R(REASON.FLYWHEEL_NOT_READY, 'block', 'Flywheel is off'));
      else if (rErr > tolR) reasons.push(R(REASON.FLYWHEEL_NOT_READY, 'block', `Flywheel ${(rErr * 100).toFixed(0)}% off setpoint`));
      if (ctx.dispersion || ctx.minHitProb != null) {
        const d = dispersionFor(ctx, solverLaunch(sol, sh, kindName(ctx)), boxes, kindName(ctx));
        autoHit = d.p;
        if (ctx.minHitProb != null && d.p < ctx.minHitProb) reasons.push(R(REASON.LOW_HIT_PROB, 'block', `Hit probability ${(d.p * 100).toFixed(0)}% is below ${(ctx.minHitProb * 100).toFixed(0)}%`));
      }
    }
    return finish();
  }

  // assist: actual setpoints must be physically legal
  if ((ctx.rpm || 0) > sh.maxRpm) reasons.push(R(REASON.RPM_TOO_HIGH, 'block', `Flywheel set to ${Math.round(ctx.rpm)} rpm, max is ${sh.maxRpm}`));
  if ((ctx.rpm || 0) < MIN_SPIN_RPM) reasons.push(R(REASON.FLYWHEEL_NOT_READY, 'block', `Flywheel below ${MIN_SPIN_RPM} rpm`));
  if (ctx.hoodDeg < sh.hoodMin - 1e-6 || ctx.hoodDeg > sh.hoodMax + 1e-6) reasons.push(R(REASON.HOOD_LIMIT, 'block', `Hood ${ctx.hoodDeg.toFixed(0)} deg outside ${sh.hoodMin}..${sh.hoodMax}`));
  if (ctx.turretDeg < sh.turretMinDeg - 1e-6 || ctx.turretDeg > sh.turretMaxDeg + 1e-6) reasons.push(R(REASON.TURRET_LIMIT, 'block', `Turret ${ctx.turretDeg.toFixed(0)} deg outside travel ${sh.turretMinDeg}..${sh.turretMaxDeg}`));
  const fs = forwardShot(ctx);
  forward = landing(fs, ctx, boxes);
  if (forward.blocked) reasons.push(R(REASON.TRAJECTORY_BLOCKED, 'block', `Ball path blocked by ${forward.by} at (${forward.at.map(v => v.toFixed(0)).join(', ')})`));
  dispersionOf(ctx, fs, forward, boxes, reasons);
  reasons.push({ code: 'FORWARD_SIM', severity: 'info', text: forward.lands ? 'Ball would enter the opening' : 'Ball would miss the opening' });
  return finish();
}

// ---------------------------------------------------------------- feasibility heat map
/** opts: {shooter, target, obstacles, hiveBoxes, legalZone, enforceZone, alliance, step=3, core}. Stationary robot, turret aimed. */
export function feasibleRegion(opts) {
  const step = opts.step || 3, sh = opts.shooter, tg = opts.target, core = opts.core || getCore(opts);
  const env = opts.obstacles || {}, b = env.bounds || DEFAULT_BOUNDS;
  const boxes = allBoxes(env, opts.hiveBoxes);
  const cells = [];
  const zone = opts.legalZone && opts.enforceZone !== false ? opts.legalZone : null;
  for (let y = b.minY + step / 2; y < b.maxY; y += step) {
    for (let x = b.minX + step / 2; x < b.maxX; x += step) {
      let inBox = false;
      for (const bx of boxes) if (x >= bx[0] && x <= bx[2] && y >= bx[1] && y <= bx[3] && bx[4] < 18) { inBox = true; break; }
      if (inBox) continue;
      const h = Math.atan2(tg.y - y, tg.x - x);
      const pose = { x, y, h, vx: 0, vy: 0, om: 0 };
      const ap = autoPhysics(pose, sh, tg, { obstacles: env, hiveBoxes: opts.hiveBoxes, kind: opts.kind }, core, null, true, boxes);
      let code = null;
      const blk = ap.reasons.find(r => r.severity === 'block');
      if (blk) code = blk.code;
      if (!code && ap.sol.ok) {
        const tc = turretEquivalent(ap.sol, sh, 0);
        if (tc === null) code = REASON.TURRET_LIMIT;
      }
      if (!code && zone && !zone.contains(x, y, opts.alliance)) code = REASON.ILLEGAL_ZONE;
      cells.push({ x, y, ok: code === null, reasonCode: code });
    }
  }
  return { cells, step };
}
