// Closed-loop Pedro 2.1.0 follower simulation (pure ES module, no DOM, deterministic).
//
//   simulateBridgeRequest(body, overrides) -> Traj    offline run of a whole bridge request (mirrors tools_java/PedroFollowSim.java tick())
//   PedroDriveService                                 live, host-coupled drive used by the autorunner ('pedro' follower model)
//
// Plant: MecanumPlant (physical model derived from Pedro constants, params.js). Controller: PedroFollower (port of Pedro 2.1.0).
// Localizer: SimLocalizer (truth + optional noise/latency). Time: one SimClock shared by all of them (nanoseconds, like Pedro).
//
// Units: inches, seconds, radians (h, w), field-frame velocities in/s, wheel powers dimensionless [-1,1] in order [LF, LR, RF, RR].
// Trajectory sample: {t, x, y, h (rad, [-PI,PI]), vx, vy, w (rad/s), p[4], seg, phase, stuck, mode}.
//
// Module rule: this file imports only from this folder. It never imports pathing.js, autorunner.js or pedrofollow.js.
//
// Known Pedro 2.1.0 behaviour surfaced here: chain callbacks (TemporalCallback / parametric path-0) are broken in the real
// library (Pedro:paths/callbacks/TemporalCallback.java:58,66). By default this sim fires callbacks with INTENDED semantics, so a
// real 2.1.0 robot will differ unless params.sim.emulate210CallbackBug is true. info.notes records which is in force.
import { resolveParams } from './params.js';
import { SimClock } from './pedromath.js';
import { groupsFromBridgeRequest } from './pedropath.js';
import { MecanumPlant } from './plant.js';
import { SimLocalizer } from './localizer.js';
import { MecanumDrivetrain } from './mecanumdrive.js';
import { PedroFollower } from './follower.js';

export const PHASE = { FOLLOW: 0, WAIT: 1, SETTLE: 2, DONE: 3 };

const TAU = Math.PI * 2;
const wrapSigned = a => { a = ((a % TAU) + TAU) % TAU; return a > Math.PI ? a - TAU : a; };
const OPT_KEYS = ['maxTimeS', 'settleS', 'trajStride'];   // live under params.sim
const RUN_KEYS = ['holdEnd', 'maxPower'];                  // per-run options, not params

/** Splits user overrides into {params overrides (for resolveParams), holdEnd, maxPower}. */
function splitOverrides(overrides = {}) {
  const o = overrides || {}, rest = {}, simOpts = {};
  for (const k of Object.keys(o)) {
    if (OPT_KEYS.includes(k)) { if (o[k] !== undefined) simOpts[k] = o[k]; }
    else if (!RUN_KEYS.includes(k)) rest[k] = o[k];
  }
  if (Object.keys(simOpts).length) rest.sim = { ...(rest.sim || {}), ...simOpts };
  return { paramOverrides: rest, holdEnd: o.holdEnd, maxPower: o.maxPower };
}

/** Builds plant + localizer + drivetrain + follower on one SimClock (spec section 5). */
function makeRig(P, startPose, voltageSource = null) {
  const clock = new SimClock(0);
  const plant = new MecanumPlant(P, { x: startPose.x, y: startPose.y, h: startPose.h || 0 }, { voltageSource });
  const localizer = new SimLocalizer(plant, P.localizer, clock);
  const drivetrain = new MecanumDrivetrain(P, { voltage: () => plant.voltage() });
  const follower = new PedroFollower({ params: P, clock, localizer, drivetrain });
  localizer.update();
  return { clock, plant, localizer, drivetrain, follower };
}

/** Group bookkeeping shared by the offline sim and the live service: callbacks fire through the follower listener. */
class GroupRunner {
  constructor(rig, P, emit) { this.rig = rig; this.P = P; this.emit = emit; this.groups = []; this.gi = 0; this.curSeg = 0; this.segStartT = 0; this.stuckSeen = false; this.fired = []; }
  attach() {
    this.unsub = this.rig.follower.onCallback((name, seg, pathIndex) => this._onCallback(name, seg, pathIndex));
  }
  detach() { try { this.unsub?.(); } catch (_) { /* ignore */ } }
  _onCallback(name, seg, pathIndex) {
    const g = this.groups[this.gi]; if (!g) return;
    let i = g.callbacks.findIndex((c, k) => !this.fired[k] && c.name === name && (pathIndex === undefined || c.pathIndex === pathIndex));
    if (i < 0) i = g.callbacks.findIndex((c, k) => !this.fired[k] && c.name === name);
    if (i < 0) return;
    this.fired[i] = true;
    this.emit('callback', name, g.callbacks[i].seg ?? seg);
  }
  /** Starts group gi (Java startGroup). */
  start(gi, t, opts) {
    this.gi = gi; const g = this.groups[gi];
    this.curSeg = g.firstSeg; this.segStartT = t; this.fired = g.callbacks.map(() => false); this.stuckSeen = false;
    this.emit('pathStart', 'group' + gi, g.firstSeg);
    this.rig.follower.followChain(g.chain, { maxPower: opts.maxPower, holdEnd: opts.holdEnd });
  }
  /** Per-tick bookkeeping while FOLLOW: segment index and the stuck event. */
  track(t) {
    const f = this.rig.follower, g = this.groups[this.gi];
    if (f.getFollowingPathChain()) { const ns = g.firstSeg + f.getChainIndex(); if (ns !== this.curSeg) { this.curSeg = ns; this.segStartT = t; } }
    if (!this.stuckSeen && f.isRobotStuck()) { this.stuckSeen = true; this.emit('stuck', 'stuck', this.curSeg); }
  }
  /** Fires leftover callbacks and emits pathEnd (Java finishGroupOrAdvance). */
  finish() {
    const g = this.groups[this.gi];
    g.callbacks.forEach((c, k) => { if (!this.fired[k]) { this.fired[k] = true; this.emit('callback', c.name, c.seg ?? g.firstSeg + c.pathIndex); } });
    const last = g.firstSeg + g.chain.size() - 1;
    this.emit('pathEnd', 'group' + this.gi, last);
    return g;
  }
}

/**
 * Offline closed-loop run of a bridge request body (the shape of pathing.bridgeBody / pedrofollow.followRequest).
 * overrides: resolveParams overrides plus {maxTimeS, settleS, trajStride, holdEnd=true, maxPower=1}.
 * body.maxTime/settle/stride/holdEnd/maxPower (Java request keys) are honoured when no override is given.
 * Phase machine and tick order mirror PedroFollowSim.java tick(): clock advance -> plant step -> t += dt -> follower.update -> phases.
 */
export function simulateBridgeRequest(body, overrides = {}) {
  const sp = splitOverrides(overrides);
  const pre = { ...sp.paramOverrides };
  const sim0 = { ...(pre.sim || {}) };
  if (sim0.maxTimeS === undefined && Number.isFinite(body?.maxTime)) sim0.maxTimeS = body.maxTime;
  if (sim0.settleS === undefined && Number.isFinite(body?.settle)) sim0.settleS = body.settle;
  if (sim0.trajStride === undefined && Number.isFinite(body?.stride)) sim0.trajStride = body.stride;
  pre.sim = sim0;
  const P = resolveParams(pre);
  const holdEnd = sp.holdEnd ?? body?.holdEnd ?? true, maxPower = sp.maxPower ?? body?.maxPower ?? 1;
  const dt = P.loop.dt, stride = Math.max(1, Math.round(P.sim.trajStride)), maxT = P.sim.maxTimeS, settleS = P.sim.settleS;
  const st = body.start || { x: 0, y: 0, h: 0 };
  const start = { x: st.x, y: st.y, h: (st.h || 0) * Math.PI / 180 };

  const events = []; let t = 0, curSegForEvents = 0;
  const emit = (kind, name, seg) => events.push({ t: +t.toFixed(4), kind, name, seg: seg ?? curSegForEvents });
  const rig = makeRig(P, start);
  const run = new GroupRunner(rig, P, emit);
  run.groups = groupsFromBridgeRequest(body, P.constraints);
  const traj = [];
  const finishTraj = (finished, endReason) => {
    traj.source = 'pedro-js'; traj.approximate = false; traj.events = events; traj.finished = finished; traj.endReason = endReason;
    traj.duration = traj.at(-1)?.t ?? 0;
    traj.info = {
      model: 'pedro-js', dt, stride, groups: run.groups.length, holdEnd, maxPower,
      emulate210CallbackBug: !!P.sim.emulate210CallbackBug,
      notes: P.sim.emulate210CallbackBug
        ? 'Pedro 2.1.0 chain-callback bug emulated (path-0 parametric callbacks never fire, temporal fire immediately).'
        : 'Callbacks fire with INTENDED semantics; real Pedro 2.1.0 chain callbacks are buggy (TemporalCallback.java:58,66), so do not rely on them on the robot.',
      constants: { mass: P.pedro.mass, forwardZeroPowerAcceleration: P.pedro.forwardZeroPowerAcceleration, lateralZeroPowerAcceleration: P.pedro.lateralZeroPowerAcceleration, centripetalScaling: P.pedro.centripetalScaling, xVelocity: P.mecanum.xVelocity, yVelocity: P.mecanum.yVelocity, omegaMax: P.plant.omegaMax },
    };
    traj.error = null;
    return traj;
  };
  const record = (phase) => {
    const pl = rig.plant, pose = pl.pose, v = pl.velocity, f = rig.follower;
    traj.push({ t: +t.toFixed(6), x: pose.x, y: pose.y, h: wrapSigned(pose.h), vx: v.vx, vy: v.vy, w: v.w, p: f.powers().slice(), seg: run.curSeg, phase, stuck: f.isRobotStuck(), mode: f.telemetry().mode });
  };

  if (!run.groups.length) { record(PHASE.DONE); events.push({ t: 0, kind: 'done', name: 'route', seg: 0 }); return finishTraj(true, 'finished'); }
  run.attach();
  const plantPowers = () => rig.plant.setPowers(rig.follower.powers());

  let phase = PHASE.FOLLOW, phaseTicks = 0, waitEnd = 0, settleEnd = 0, done = false, endReason = 'finished', tickNo = 0;
  record(phase);                      // t = 0 sample
  run.start(0, t, { holdEnd, maxPower }); plantPowers();
  while (!done) {
    // tick(): same order as PedroFollowSim.tick
    rig.clock.advance(dt); rig.plant.step(dt); t += dt; tickNo++; phaseTicks++;
    rig.follower.update(); plantPowers();
    curSegForEvents = run.curSeg;
    if (phase === PHASE.FOLLOW) {
      run.track(t);
      if (!rig.follower.isBusy() && phaseTicks > 2) {
        const g = run.finish();
        if (g.waitAfter > 0) { phase = PHASE.WAIT; waitEnd = t + g.waitAfter; emit('waitStart', 'wait' + run.gi, run.curSeg); }
        else { phase = PHASE.SETTLE; settleEnd = t + settleS; }
      }
    } else if (phase === PHASE.WAIT) {
      if (t >= waitEnd - 1e-12) {
        emit('waitEnd', 'wait' + run.gi, run.curSeg);
        if (run.gi < run.groups.length - 1) { phase = PHASE.FOLLOW; phaseTicks = 0; run.start(run.gi + 1, t, { holdEnd, maxPower }); plantPowers(); }
        else { phase = PHASE.SETTLE; settleEnd = t + settleS; }
      }
    } else if (phase === PHASE.SETTLE) {
      if (t >= settleEnd - 1e-12) { phase = PHASE.DONE; done = true; endReason = 'finished'; emit('done', 'route', run.curSeg); }
    }
    if (!done && t >= maxT - 1e-12) { done = true; phase = PHASE.DONE; endReason = 'timeout'; emit('timeout', 'maxTime', run.curSeg); }
    if (tickNo % stride === 0 || done) record(phase);
  }
  run.detach();
  return finishTraj(endReason === 'finished', endReason);
}

/**
 * Velocity after the host moved the robot by (dx, dy, dh) since the service's last write, over one loop of L seconds.
 * The host correction is treated as an inelastic position constraint (like a contact solver's penetration fix): it may
 * REMOVE the velocity component pointing against the correction (the part the obstacle stopped), at most |d|/L of it and
 * never past zero, but it never ADDS velocity along the correction. Rationale (round 2 fix): the old rule v += d/L turned
 * a 3 in push applied in one 10 ms loop into +300 in/s (capped at 122 in/s), which is kinetic energy no collision gave
 * the robot; in HOLD it overshot the hold point by ~25-27 in and took ~1.5 s to come back. A wall that undoes exactly
 * this loop's motion (d = -v*L) still zeroes v (stuck detection keeps working); a pure push of a resting robot moves it
 * without making it fly, and the hold PIDF brings it back. Units: in, in/s, rad, rad/s, s.
 */
export function contactVelocity(v, dx, dy, dh, L) {
  let vx = v.vx, vy = v.vy, w = v.w;
  const d = Math.hypot(dx, dy);
  if (d > 1e-12 && L > 0) {
    const nx = dx / d, ny = dy / d, vn = vx * nx + vy * ny;        // in/s along the correction
    if (vn < 0) { const k = Math.min(-vn, d / L); vx += nx * k; vy += ny * k; }
  }
  if (Math.abs(dh) > 1e-12 && L > 0 && w * dh < 0) w += Math.sign(dh) * Math.min(Math.abs(w), Math.abs(dh) / L);
  return { vx, vy, w };
}

/**
 * Live closed-loop drive for the autorunner. Owns plant, localizer, clock and follower.
 *
 * Host contract (all optional): getPose() -> {x,y,h}, setPose({x,y,h}), setVelocity(vx,vy,w).
 * getPose() is read every control loop; if it differs from what the service last wrote (collision push, teleport, wall
 * that undoes motion) the plant is re-synced to the host pose and the velocity component the correction opposes is
 * removed (contactVelocity: inelastic, never injected), so the follower reacts (hold pushback, stuck detection).
 *
 * constructor({host, params (resolveParams overrides), startPose {x,y,h rad}})
 */
export class PedroDriveService {
  constructor({ host = null, params = {}, startPose = { x: 0, y: 0, h: 0 } } = {}) {
    this.host = host; this.P = resolveParams(params || {});
    this.rig = makeRig(this.P, startPose);
    this.t = 0; this._acc = 0; this._events = []; this._busy = false; this._endReason = null;
    this._committed = { ...this.rig.plant.pose };
    this._run = new GroupRunner(this.rig, this.P, (kind, name, seg) => this._events.push({ t: +this.t.toFixed(4), kind, name, seg: seg ?? 0 }));
    this._run.attach();
    this._phase = PHASE.DONE; this._phaseTicks = 0; this._waitEnd = 0; this._groupStartT = 0; this._opts = { maxPower: 1, holdEnd: true };
    this._doneLen = 0; this._totalLen = 0; this._cum = [];
  }
  get follower() { return this.rig.follower; }
  get plant() { return this.rig.plant; }
  get events() { return this._events; }
  get time() { return this.t; }
  get endReason() { return this._endReason; }

  /** Teleports the robot (zero velocity) and re-anchors the sync reference. Does not touch the follower's path. */
  setPose(pose) {
    this.rig.plant.syncTo({ x: pose.x, y: pose.y, h: pose.h || 0 }, { vx: 0, vy: 0, w: 0 });
    this._committed = { ...this.rig.plant.pose }; this.rig.localizer.update();
  }

  /** Starts following groups (from groupsFromBridgeRequest) one after another, with waitAfter holds between them. */
  followGroups(groups, { maxPower = 1, holdEnd = true } = {}) {
    this._run.groups = groups || []; this._opts = { maxPower, holdEnd }; this._endReason = null;
    this._cum = []; let s = 0;
    for (const g of this._run.groups) { this._cum.push(s); for (let i = 0; i < g.chain.size(); i++) s += g.chain.getPath(i).length; }
    this._totalLen = s;
    if (!this._run.groups.length) { this._busy = false; this._phase = PHASE.DONE; return this; }
    this._busy = true; this._startGroup(0);
    return this;
  }
  _startGroup(i) {
    this._phase = PHASE.FOLLOW; this._phaseTicks = 0; this._groupStartT = this.t;
    this._run.start(i, this.t, this._opts); this.rig.plant.setPowers(this.rig.follower.powers());
  }

  /** Advances by dtS seconds of wall/sim time in whole control loops of params.loop.dt (remainder carries over). */
  tick(dtS) {
    this._acc += Math.max(0, dtS || 0);
    const L = this.P.loop.dt;
    while (this._acc >= L - 1e-12) { this._acc -= L; this._loop(L); }
    if (this._acc < 0) this._acc = 0;
    return this;
  }

  /** i. host sync  ii. clock  iii. plant step  iv. host write  v. follower update + powers (spec 5b). */
  _loop(L) {
    const { plant, clock, follower } = this.rig, H = this.host, P = this.P;
    if (H && typeof H.getPose === 'function') {
      const m = H.getPose();
      if (m && Number.isFinite(m.x) && Number.isFinite(m.y)) {
        const dx = m.x - this._committed.x, dy = m.y - this._committed.y, dh = wrapSigned((m.h || 0) - this._committed.h);
        if (Math.hypot(dx, dy) > P.sim.poseSyncEpsIn || Math.abs(dh) > P.sim.poseSyncEpsRad) {
          const { vx, vy, w } = contactVelocity(plant.velocity, dx, dy, dh, L);
          plant.syncTo({ x: m.x, y: m.y, h: plant.pose.h + dh }, { vx, vy, w });
        }
      }
    }
    clock.advance(L); plant.step(L); this.t += L; this._phaseTicks++;
    const pose = plant.pose, v = plant.velocity;
    this._committed = { x: pose.x, y: pose.y, h: pose.h };
    if (H) { try { H.setPose?.({ x: pose.x, y: pose.y, h: wrapSigned(pose.h) }); H.setVelocity?.(v.vx, v.vy, v.w); } catch (_) { /* host errors never break the loop */ } }
    follower.update(); plant.setPowers(follower.powers());
    if (this._busy) this._phaseLogic();
  }
  _phaseLogic() {
    const run = this._run, f = this.rig.follower, P = this.P;
    if (this._phase === PHASE.FOLLOW) {
      run.track(this.t);
      if (!f.isBusy() && this._phaseTicks > 2) {
        const g = run.finish();
        if (g.waitAfter > 0) { this._phase = PHASE.WAIT; this._waitEnd = this.t + g.waitAfter; this._events.push({ t: +this.t.toFixed(4), kind: 'waitStart', name: 'wait' + run.gi, seg: run.curSeg }); }
        else this._finishAll();
      } else if (this.t - this._groupStartT > P.sim.followWatchdogS) {
        // Sim-only safety net (FTC auto length): not Pedro behaviour. Hold where we are and end the follow.
        this._events.push({ t: +this.t.toFixed(4), kind: 'watchdog', name: 'followWatchdog', seg: run.curSeg });
        const pz = this.rig.plant.pose; f.holdPoint({ x: pz.x, y: pz.y, h: pz.h });
        this._busy = false; this._endReason = 'watchdog'; this._phase = PHASE.DONE;
      }
    } else if (this._phase === PHASE.WAIT) {
      if (this.t >= this._waitEnd - 1e-12) {
        this._events.push({ t: +this.t.toFixed(4), kind: 'waitEnd', name: 'wait' + run.gi, seg: run.curSeg });
        if (run.gi < run.groups.length - 1) this._startGroup(run.gi + 1); else this._finishAll();
      }
    }
  }
  _finishAll() { this._busy = false; this._phase = PHASE.DONE; this._endReason = 'finished'; this._events.push({ t: +this.t.toFixed(4), kind: 'done', name: 'route', seg: this._run.curSeg }); }

  /** True while any group or waitAfter remains. The follower keeps HOLDING afterwards until the next command or stop(). */
  isBusy() { return this._busy; }

  /** Arc-length fraction 0..1 over all groups. */
  progress() {
    if (!this._run.groups.length || this._totalLen <= 1e-9) return this._busy ? 0 : 1;
    if (!this._busy) return this._endReason === 'finished' || this._endReason === 'watchdog' ? 1 : 0;
    const gi = this._run.gi, g = this._run.groups[gi], f = this.rig.follower;
    let s = this._cum[gi] ?? 0;
    if (this._phase === PHASE.FOLLOW) {
      const ci = Math.min(f.getChainIndex(), g.chain.size() - 1);
      for (let i = 0; i < ci; i++) s += g.chain.getPath(i).length;
      const p = g.chain.getPath(ci);
      s += p.distanceTraveled(p.closestT);
    } else for (let i = 0; i < g.chain.size(); i++) s += g.chain.getPath(i).length;
    return Math.max(0, Math.min(1, s / this._totalLen));
  }

  /** breakFollowing (FLOAT) and zero velocity; the service becomes idle. */
  stop() {
    this.rig.follower.breakFollowing(); this.rig.plant.setPowers([0, 0, 0, 0]);
    const pz = this.rig.plant.pose; this.rig.plant.syncTo({ x: pz.x, y: pz.y, h: pz.h }, { vx: 0, vy: 0, w: 0 });
    this._committed = { ...this.rig.plant.pose };
    this._busy = false; this._phase = PHASE.DONE; this._run.groups = [];
    try { this.host?.setVelocity?.(0, 0, 0); } catch (_) { /* ignore */ }
    return this;
  }

  telemetry() {
    const f = this.rig.follower;
    return { ...f.telemetry(), t: this.t, group: this._run.gi, phase: this._phase, busy: this._busy, endReason: this._endReason, slipping: this.rig.plant.slipping, progress: this.progress() };
  }
}
