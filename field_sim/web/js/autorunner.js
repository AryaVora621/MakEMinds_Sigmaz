// AutoRunner: runs an Auto (autos.js) against the simulator through a `host` object. Pure, no DOM, deterministic.
//
// HOST interface (all methods optional unless marked; missing ones are skipped / return safe defaults):
//   getPose() -> {x,y,h}          setPose({x,y,h})            setVelocity(vx,vy,omega)   (Pedro frame, in/s, rad/s)
//   intake(on)                    shoot(args) -> bool         (args {count} or {all:true}; {all:true} must keep firing until empty)
//   ballCount() hopperFull() hopperEmpty() flywheelReady() aimed()      (condition sources)
//   setAim(on) setRpm(r) flower(deploy) gate(open) park(args) transfer(on)
//   autoAimTarget() -> {rpm,turretDeg,hoodDeg,feasible}|null   (solved shot right now; null = no solution)
//   autoAimCurrent() -> {rpm,turretDeg,hoodDeg}                 (flywheel / turret / hood state)
//   autoAimApply(cmd)                                           (drive flywheel / turret / hood toward cmd)
//                                 (autoAim command: needs autoAimTarget; without it the command logs a warning and completes)
//   drive() -> {vmax, aMax}       (fallback speed limits, used only if auto.settings has none)
//   snapshot() -> opaque / restore(opaque)   (optional; used by seek() to rewind sim state)
//   resetHives()                  (optional; called by reset() after the restore/pose step, so Stop/Restart also put every
//                                  hive mechanism back to its start state; a throwing hook is ignored)
//   time()                        (informational; the runner uses its own deterministic clock for timeElapsed)
//   onLog(msg)                    (optional log sink)
//
// pathSampler(route) -> samples [{x,y,h,s?}] (or a Promise of them). `route` = {name, alliance, start, segments}.
// Samples are pre-computed at load() (sync samplers are ready immediately; for async ones `await runner.prepare()`).
// follow(pathId) drives the robot along the samples with a trapezoidal speed profile, setting the pose every step.
//
// FOLLOWER MODEL (constructor option followerModel, overridable per auto by auto.settings.followerModel):
//   'auto' (default)  today's behaviour: replay a trajectorySampler trajectory if one exists, else the trapezoid profile on pathSampler samples.
//   'replay'          same as 'auto'.      'trapezoid'  ignore trajectories, always the trapezoid profile.
//   'pedro'           closed-loop Pedro 2.1.0 follower in JS (physics/pedro-fidelity): one PedroDriveService per run, created at the first
//                     follow and ticked on EVERY scheduler substep (also when no follow is active, so the robot keeps HOLDING its end point).
//                     A follow command is done when the service has finished all its groups (incl. waitAfter). pedroParams = resolveParams overrides.
//   Extra host expectations in 'pedro' mode: getPose() is read every control loop (10 ms by default). If it differs from the pose the
//   service last wrote (collision push, teleport, a wall undoing the motion) the plant is re-synced and its velocity corrected, so host
//   pushes feed back into the follower (hold pushback, stuck detection). setPose/setVelocity are written every loop (rad, in/s).
//   stop()/reset()/seek()/load()/start() discard the service (next follow builds a fresh one at the host's current pose).
//
// Time: step(dt) multiplies dt by `speed` and advances the scheduler in fixed substeps of at most 1/60 s.
// Errors: any command exception aborts the whole auto (all commands interrupted, velocity zeroed), state -> 'error'.
// seek(t): rewinds with host.restore(snapshot taken at start()) if available (else just re-runs from the current host
// state, so seek is only exact with a host snapshot), rebuilds the command tree and re-simulates at 1/60 s up to t.

import { Scheduler } from './scheduler.js';
import { buildCommandTree, getCommandDef, walkNodes, parseCondition } from './autos.js';
import { trajectoryPoseAt } from './pathing.js';
import { makeAutoAim } from './autoaim_cmd.js';
import { bridgeBody } from './pathing.js';
import { PedroDriveService } from './physics/pedro-fidelity/closedloop.js';
import { groupsFromBridgeRequest } from './physics/pedro-fidelity/pedropath.js';

const FOLLOWER_MODELS = ['auto', 'pedro', 'replay', 'trapezoid'];

const FIXED_DT = 1 / 60;
const TAU = Math.PI * 2;
const wrapAngle = a => { a = ((a % TAU) + TAU) % TAU; return a > Math.PI ? a - TAU : a; };

function normalizeSamples(pts) {
  if (!Array.isArray(pts) || !pts.length) throw new Error('path sampler returned no samples');
  const out = []; let s = 0;
  pts.forEach((p, i) => {
    if (i > 0 && p.s === undefined) s += Math.hypot(p.x - pts[i - 1].x, p.y - pts[i - 1].y);
    out.push({ x: p.x, y: p.y, h: p.h ?? 0, s: p.s !== undefined ? p.s : s });
    if (p.s !== undefined) s = p.s;
  });
  return out;
}

function poseAt(pts, s) {
  let lo = 0, hi = pts.length - 1;
  if (hi === 0) return { x: pts[0].x, y: pts[0].y, h: pts[0].h };
  while (lo < hi - 1) { const m = (lo + hi) >> 1; if (pts[m].s <= s) lo = m; else hi = m; }
  const a = pts[lo], b = pts[hi], f = b.s > a.s ? Math.min(1, Math.max(0, (s - a.s) / (b.s - a.s))) : 0;
  return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, h: wrapAngle(a.h + wrapAngle(b.h - a.h) * f) };
}

export class AutoRunner {
  /** trajectorySampler(route) -> traj [{t,x,y,h}] | {traj} | null (sync or Promise): when it returns a trajectory (e.g. from the real Pedro follower)
   *  the robot replays it by time instead of using the built-in trapezoid profile. */
  constructor({ host, pathSampler, trajectorySampler, followerModel = 'auto', pedroParams = {} } = {}) {
    this.followerModel = FOLLOWER_MODELS.includes(followerModel) ? followerModel : 'auto'; this.pedroParams = pedroParams || {}; this._svc = null;
    this.host = host || {}; this.pathSampler = pathSampler || null; this.trajSampler = trajectorySampler || null; this.trajs = {};
    this.auto = null; this.samples = {}; this._prepared = false; this._readyP = Promise.resolve();
    this.state = 'idle'; this.speed = 1; this.errors = []; this.sched = null; this.time = 0;
    this._snap = null; this._scan = 0; this._active = new Set(); this._abort = false;
    this.warnings = []; this._aimCmd = null; this._trackAim = null;
  }
  /** Effective follower model: auto.settings.followerModel overrides the constructor option. */
  _model() { const m = this.auto?.settings?.followerModel; return FOLLOWER_MODELS.includes(m) ? m : this.followerModel; }
  /** The run's PedroDriveService (created lazily at the host's current pose; discarded by stop/reset/seek/start/load). */
  _pedroService() {
    if (!this._svc) {
      const hp = typeof this.host.getPose === 'function' ? this.host.getPose() : null, st = this.auto?.start || {};
      const sp = hp && Number.isFinite(hp.x) ? { x: hp.x, y: hp.y, h: hp.h || 0 } : { x: st.x || 0, y: st.y || 0, h: st.h || 0 };
      this._svc = new PedroDriveService({ host: this.host, params: this.pedroParams, startPose: sp });
    }
    return this._svc;
  }
  _dropService() { const s = this._svc; this._svc = null; try { s?.stop(); } catch (_) { /* ignore */ } }
  /** Soft failure: recorded in this.warnings and logged; never aborts the auto. */
  _warn(msg) { this.warnings.push(msg); this._log('warn: ' + msg); }
  get trace() { return this.sched ? this.sched.trace : []; }
  /** Node ids whose command is currently running (derived from the trace). */
  get activeNodes() { this._scanTrace(); return this._active; }

  _log(m) { try { this.host.onLog?.(m); } catch (_) { /* ignore */ } }
  _scanTrace() {
    const tr = this.trace;
    for (; this._scan < tr.length; this._scan++) {
      const e = tr[this._scan]; if (!e.node) continue;
      if (e.event === 'start') this._active.add(e.node); else this._active.delete(e.node);
    }
  }

  load(auto) {
    this.stop(); this._dropService(); this.auto = auto; this.samples = {}; this.trajs = {}; this.errors = []; this._prepared = false; this._snap = null;
    this.time = 0; this.sched = null; this._scan = 0; this._active = new Set(); this.state = 'idle';
    const r = this._sampleAll();
    if (r && typeof r.then === 'function') this._readyP = r; else { this._prepared = true; this._readyP = Promise.resolve(); }
    return this;
  }
  _routeFor(id) {
    const p = this.auto.paths[id], st = p.segments?.[0]?.points?.[0] || this.auto.start;
    return { name: id, alliance: this.auto.alliance, start: { x: st.x, y: st.y, h: this.auto.start?.h || 0 }, segments: p.segments || [], markers: p.markers || [] };
  }
  /** Samples one path. Returns a promise when the sampler is async, else undefined (sync result stored at once). */
  _samplePath(id) {
    if (!this.pathSampler && this._model() === 'pedro') return undefined;   // the closed-loop follower builds its own path chains
    let r;
    try { r = this.pathSampler ? this.pathSampler(this._routeFor(id)) : null; } catch (e) { this.errors.push(`path "${id}": ${e.message}`); return undefined; }
    if (r && typeof r.then === 'function') return r.then(v => { this.samples[id] = normalizeSamples(v); }, e => { this.errors.push(`path "${id}": ${e.message}`); });
    try { this.samples[id] = normalizeSamples(r); } catch (e) { this.errors.push(`path "${id}": ${e.message}`); }
    return undefined;
  }
  _sampleAll() {
    const ids = Object.keys(this.auto?.paths || {}); const pend = [];
    // The Pedro sampler is shared and a newer request aborts the older one, so once a sampler is async the
    // remaining paths are sampled one after another (sync samplers still finish inside this call).
    let chain = null;
    for (const id of ids) {
      if (chain) chain = chain.then(() => this._samplePath(id));
      else { const p = this._samplePath(id); if (p) chain = p; }
    }
    if (chain) pend.push(chain);
    if (this.trajSampler) for (const id of ids) {
      try {
        const r = this.trajSampler(this._routeFor(id)), set = v => this._setTraj(id, v);
        if (r && typeof r.then === 'function') pend.push(r.then(set, () => {})); else set(r);
      } catch (_) { /* no trajectory: trapezoid fallback */ }
    }
    return pend.length ? Promise.all(pend).then(() => { this._prepared = true; }) : undefined;
  }
  _setTraj(id, v) {
    const raw = Array.isArray(v) ? v : v && Array.isArray(v.traj) ? v.traj : null;
    if (raw && (raw.approximate === true || raw.source === "js-fallback")) return;   // crude kinematic stand-in from a down bridge: keep the trapezoid model instead of replaying it as Pedro motion
    const tr = raw && raw.some((p, i) => i && p.t < raw[i - 1].t) ? raw.slice().sort((p, q) => p.t - q.t) : raw;   // time-sorted so the pose lookup and T are right
    if (!tr || tr.length < 2 || !tr.every(p => Number.isFinite(p.t) && Number.isFinite(p.x) && Number.isFinite(p.y))) return;
    const cum = [0]; for (let i = 1; i < tr.length; i++) cum.push(cum[i - 1] + Math.hypot(tr[i].x - tr[i - 1].x, tr[i].y - tr[i - 1].y));
    this.trajs[id] = { tr, cum, T: Math.max(...tr.map(p => p.t)) };
  }
  /** Resolves when all paths are sampled (immediately for sync samplers). */
  async prepare() { await this._readyP; this._prepared = true; return this; }

  // ----- adapter -----
  _adapter() {
    const R = this, H = this.host, call = (m, ...a) => (!R._abort && typeof H[m] === 'function' ? H[m](...a) : undefined);
    const settings = this.auto.settings || {}, drv = call('drive') || {};
    const vmax = settings.vmax ?? drv.vmax ?? 60, aMax = settings.aMax ?? drv.aMax ?? 90;
    return {
      // Wraps the raw follower: when it finishes, any tracking aim from autoAim{track} ends (the next drive wins).
      follow(pathId) {
        const f = this.followRaw(pathId);
        return {
          start: () => f.start(), update: dt => f.update(dt), progress: () => f.progress(), stop: () => f.stop(),
          done: () => { const d = f.done(); if (d) R._trackAim = null; return d; },
        };
      },
      followRaw(pathId) {
        if (R._model() === 'pedro') {
          if (!R.auto.paths[pathId]) throw new Error(`No samples for path "${pathId}"`);
          return {
            start() {
              const svc = R._pedroService();
              svc.followGroups(groupsFromBridgeRequest(bridgeBody(R._routeFor(pathId)), svc.P.constraints));
            },
            update() { /* the service is ticked by the runner on every scheduler substep */ },
            done: () => !R._svc || !R._svc.isBusy(),
            progress: () => (R._svc ? R._svc.progress() : 1),
            stop() { R._svc?.stop(); },
          };
        }
        const pts = R.samples[pathId], tj = R._model() === 'trapezoid' ? null : R.trajs[pathId];
        if (!pts && !tj) throw new Error(`No samples for path "${pathId}"`);
        if (tj) { // replay the time-parameterised trajectory (real follower motion)
          let t = 0, fin = false, last = null;
          const apply = dt => {
            const p = trajectoryPoseAt(tj.tr, t); call('setPose', { x: p.x, y: p.y, h: p.h });
            if (dt > 0 && last) call('setVelocity', (p.x - last.x) / dt, (p.y - last.y) / dt, wrapAngle(p.h - last.h) / dt);
            last = p;
          };
          const frac = () => { if (tj.T <= 0) return 1; const q = trajectoryPoseAt(tj.tr, t); let lo = 0, hi = tj.tr.length - 1; while (lo < hi - 1) { const m = (lo + hi) >> 1; if (tj.tr[m].t <= t) lo = m; else hi = m; } const tot = tj.cum.at(-1) || 1; return Math.min(1, (tj.cum[lo] + Math.hypot(q.x - tj.tr[lo].x, q.y - tj.tr[lo].y)) / tot); };
          return {
            start() { t = 0; fin = tj.T <= 1e-9; last = null; apply(0); if (fin) call('setVelocity', 0, 0, 0); },
            update(dt) { if (fin) return; t = Math.min(tj.T, t + dt); apply(dt); if (t >= tj.T - 1e-9) { fin = true; call('setVelocity', 0, 0, 0); } },
            done: () => fin, progress: () => (fin ? 1 : frac()), stop() { call('setVelocity', 0, 0, 0); },
          };
        }
        const L = Math.max(...pts.map(p => p.s)); let s = 0, v = 0, fin = false, last = null;
        const apply = dt => {
          const p = poseAt(pts, s);
          call('setPose', p);
          if (dt > 0 && last) call('setVelocity', (p.x - last.x) / dt, (p.y - last.y) / dt, wrapAngle(p.h - last.h) / dt);
          last = p;
        };
        return {
          start() { s = 0; v = 0; fin = L < 1e-9; last = null; apply(0); if (fin) call('setVelocity', 0, 0, 0); },
          update(dt) {
            if (fin) return;
            const remaining = Math.max(0, L - s);
            v = Math.min(v + aMax * dt, vmax, Math.sqrt(2 * aMax * remaining));
            if (remaining > 1e-6) v = Math.max(v, Math.min(vmax, 2));
            s = Math.min(L, s + v * dt); apply(dt);
            if (L - s < 1e-6) { s = L; fin = true; call('setVelocity', 0, 0, 0); }
          },
          done: () => fin, progress: () => (L > 1e-9 ? s / L : 1),
          stop() { call('setVelocity', 0, 0, 0); },
        };
      },
      intake: on => call('intake', on),
      shoot: args => call('shoot', args),
      shootAll() {
        const t0 = R.time; call('shoot', { all: true });
        return { done: () => (typeof H.hopperEmpty === 'function' ? !!H.hopperEmpty() : false) || R.time - t0 >= 6 };
      },
      aim: on => call('setAim', on), setRpm: r => call('setRpm', r),
      flower: d => call('flower', d), gate: o => call('gate', o), park: a => call('park', a),
      /** autoAim{shootAfter, timeout, track}: aligns flywheel/turret/hood and turns transfer on. Failure never throws. */
      autoAim(args) {
        if (typeof H.autoAimTarget !== 'function') { R._warn('autoAim: host has no autoAimTarget; command skipped'); return { done: () => true }; }
        const a = R._makeAim(args, !!(args && args.track));
        a.start();
        R._aimCmd = a;
        if (args && args.track) R._trackAim = a;
        return { done: () => a.done() };
      },
      custom: (name, args) => R._log(`custom command "${name}" ${JSON.stringify(args)}`),
      ballCount: () => (typeof H.ballCount === 'function' ? H.ballCount() : 0),
      flywheelReady: () => !!call('flywheelReady'), aimed: () => !!call('aimed'),
      hopperFull: () => !!call('hopperFull'), hopperEmpty: () => !!call('hopperEmpty'),
      time: () => R.time,
    };
  }

  /** Builds an autoAim command object wired to the host (shared by the autoAim command and keepAim). */
  _makeAim(args, track) {
    const R = this, H = this.host, call = (m, ...a) => (!R._abort && typeof H[m] === 'function' ? H[m](...a) : undefined);
    return makeAutoAim({
      host: {
        target: () => call('autoAimTarget'), current: () => call('autoAimCurrent'), apply: c => call('autoAimApply', c),
        transfer: on => call('transfer', on), shoot: a => call('shoot', a), hopperEmpty: () => !!call('hopperEmpty'),
      },
      shootAfter: !!(args && args.shootAfter), timeoutSec: args && args.timeout, track,
      onLog: m => R._log(/^autoAim/.test(m) ? m : 'autoAim: ' + m),
      onFail: m => R._warn('autoAim: ' + m),
    });
  }
  /** Public: keeps the robot aimed (flywheel, turret, hood) with transfer on until the next follow finishes, stop() or reset(). */
  keepAim(args) {
    if (typeof this.host.autoAimTarget !== 'function') { this._warn('autoAim: host has no autoAimTarget; keepAim skipped'); return this; }
    this._stopAim();
    const a = this._makeAim(args || {}, true);
    a.start(); this._trackAim = a;
    return this;
  }
  _stopAim() {
    const a = this._aimCmd; this._aimCmd = null; this._trackAim = null;
    try { a?.stop(); } catch (_) { /* ignore */ }
  }

  _begin() {
    this._dropService(); this._abort = false; this.errors = this.errors.filter(e => e.startsWith('path ')); this.time = 0; this.warnings = [];
    this._scan = 0; this._active = new Set(); this._stopAim();
    this.sched = new Scheduler();
    this.sched.schedule(buildCommandTree(this.auto, this._adapter()));
    this._ctx = { onError: (err, cmd) => { this.errors.push(`${cmd?.name || 'command'}: ${err.message}`); this._abort = true; this._log(`error: ${err.message}`); } };
    const st = this.auto.start; if (st && typeof this.host.setPose === 'function') this.host.setPose({ x: st.x, y: st.y, h: st.h || 0 });
  }
  _tick(dt) {
    if (!this.sched) return;
    // Aim: the running autoAim is driven here (the scheduler only polls done()); a tracking aim keeps going after it finishes.
    const running = this._aimCmd;
    if (running && !running.done()) running.update(dt);
    else if (this._trackAim) this._trackAim.update(dt);
    this.sched.tick(this._ctx, dt); this.time = this.sched.time;
    if (this._svc && !this._abort) this._svc.tick(dt);
    if (this._abort) {
      this.sched.cancelAll(this._ctx); this._stopAim(); this._zero(); this.state = 'error'; return;
    }
    if (!this.sched.isRunning()) { this._stopAim(); this._zero(); this.state = 'done'; }
  }
  _zero() { try { this.host.setVelocity?.(0, 0, 0); } catch (_) { /* ignore */ } }

  start() {
    if (!this.auto) { this.errors.push('no auto loaded'); this.state = 'error'; return this; }
    if (!this._prepared) { this.errors.push('paths not prepared: await runner.prepare() first'); this.state = 'error'; return this; }
    if (this.sched && this.sched.isRunning()) this.sched.cancelAll(this._ctx);
    try { this._snap = this.host.snapshot ? this.host.snapshot() : null; } catch (_) { this._snap = null; }
    this._begin(); this.state = 'running';
    this._tick(0); // schedules + initializes the root (zero-length leading commands finish here)
    return this;
  }
  pause() { if (this.state === 'running') this.state = 'paused'; return this; }
  resume() { if (this.state === 'paused') this.state = 'running'; return this; }
  stop() {
    if (this.sched && this.sched.isRunning()) this.sched.cancelAll(this._ctx || {});
    this._scanTrace(); this._stopAim(); this._dropService(); this._zero();
    if (this.state === 'running' || this.state === 'paused') this.state = 'idle';
    return this;
  }
  reset() {
    this.stop(); this._dropService(); this.sched = null; this.time = 0; this.errors = []; this.warnings = []; this._scan = 0; this._active = new Set(); this.state = 'idle';
    if (this._snap && this.host.restore) { try { this.host.restore(this._snap); } catch (_) { /* ignore */ } }
    else if (this.auto?.start && this.host.setPose) this.host.setPose({ x: this.auto.start.x, y: this.auto.start.y, h: this.auto.start.h || 0 });
    try { this.host.resetHives?.(); } catch (_) { /* ignore */ }
    return this;
  }
  /** Advances by dt seconds of real time (scaled by `speed`). No-op unless running. */
  step(dt) {
    if (this.state !== 'running') return this;
    let rem = Math.max(0, dt) * this.speed;
    const n = Math.max(1, Math.ceil(rem / FIXED_DT - 1e-9)), h = rem / n;
    for (let i = 0; i < n && this.state === 'running'; i++) this._tick(h);
    return this;
  }
  /** Deterministic re-simulation from the start to time t. Leaves the runner paused (or done/error). */
  seek(t) {
    if (!this.auto) return this;
    if (!this._prepared) { this.errors.push('paths not prepared'); return this; }
    if (this.sched && this.sched.isRunning()) this.sched.cancelAll(this._ctx);
    if (this._snap && this.host.restore) { try { this.host.restore(this._snap); } catch (_) { /* ignore */ } }
    else if (!this._snap && this.host.snapshot) { try { this._snap = this.host.snapshot(); } catch (_) { /* ignore */ } }
    this._begin(); this.state = 'running'; this._tick(0);
    const target = Math.max(0, t), n = Math.floor(target / FIXED_DT + 1e-9);
    for (let i = 0; i < n && this.state === 'running'; i++) this._tick(FIXED_DT);
    const rest = target - n * FIXED_DT;
    if (rest > 1e-9 && this.state === 'running') this._tick(rest);
    if (this.state === 'running') this.state = 'paused';
    return this;
  }
}

// ---------- dry-run timeline ----------
/**
 * Flat Gantt bars from a dry run of the scheduler with a fake host.
 * ctx: { pathTime(id) -> seconds, assumedWait = 1 (waitUntil without timeout), maxSeconds = 120 }.
 * Durations: follow = pathTime, wait = seconds, waitUntil = timeout or assumedWait, commands = simDuration.
 * Returns bars [{id,label,kind,start,end,lane,depth}] (array has .total seconds and .errors).
 * Groups have kind 'group' (and .group = type); lanes are packed greedily over leaf bars so parallel work gets
 * separate lanes; a group takes the lane of its first leaf.
 */
export function buildTimeline(auto, ctx = {}) {
  const pathTime = ctx.pathTime || (() => 0), assumed = ctx.assumedWait ?? 1, maxS = ctx.maxSeconds ?? 120;
  const info = new Map();
  const dry = JSON.parse(JSON.stringify(auto));
  const fix = (n, depth) => {
    info.set(n.id, { node: n, depth });
    if (n.type === 'waitUntil') {
      const c = parseCondition(n.condition);
      if (!c || c.kind !== 'timeElapsed') { n.type = 'wait'; n.seconds = n.timeout > 0 ? n.timeout : assumed; n._wasWaitUntil = true; }
    }
    (n.children || []).forEach(c => fix(c, depth + 1));
  };
  fix(dry.root, 0);
  for (const p of Object.values(dry.paths || {})) for (const m of p.markers || []) fix(m.command, 1);
  const errors = []; let clock = 0;
  const adapter = {
    follow(id) {
      const dur = pathTime(id); let el = 0;
      return { start() { el = 0; }, update(dt) { el += dt; }, done: () => el >= dur - 1e-9, progress: () => (dur > 0 ? Math.min(1, el / dur) : 1) };
    },
    ballCount: () => 0, flywheelReady: () => true, aimed: () => true, hopperFull: () => true, hopperEmpty: () => true, time: () => clock,
  };
  const sch = new Scheduler(), cx = { onError: (e, c) => errors.push(`${c?.name || 'command'}: ${e.message}`) };
  let total = 0;
  try {
    sch.schedule(buildCommandTree(dry, adapter));
    sch.tick(cx, 0);
    while (sch.isRunning() && sch.time < maxS && !errors.length) { clock = sch.time; sch.tick(cx, FIXED_DT); }
    total = sch.time;
  } catch (e) { errors.push(e.message); }
  const open = new Map(), bars = [];
  for (const e of sch.trace) {
    if (!e.node) continue;
    if (e.event === 'start') open.set(e.node, e.t);
    else if (open.has(e.node)) {
      const inf = info.get(e.node); if (!inf) continue;
      const n = auto.root && findOrig(auto, e.node) || inf.node, st = open.get(e.node); open.delete(e.node);
      bars.push(mkBar(n, inf.depth, st, e.t));
    }
  }
  for (const [id, st] of open) { const inf = info.get(id); if (inf) bars.push(mkBar(findOrig(auto, id) || inf.node, inf.depth, st, total)); }
  bars.sort((a, b) => a.start - b.start || a.depth - b.depth);
  const laneEnd = [];
  for (const b of bars.filter(x => x.kind !== 'group')) {
    let l = laneEnd.findIndex(end => end <= b.start + 1e-9); if (l < 0) { l = laneEnd.length; laneEnd.push(0); }
    b.lane = l; laneEnd[l] = Math.max(b.end, b.start + 1e-6);
  }
  for (const g of bars.filter(x => x.kind === 'group')) {
    const kids = [];
    walkNodes(info.get(g.id).node, n => { if (n.id !== g.id) kids.push(n.id); });
    const ls = bars.filter(b => b.kind !== 'group' && kids.includes(b.id)).map(b => b.lane);
    g.lane = ls.length ? Math.min(...ls) : 0;
  }
  bars.total = total; bars.errors = errors; bars.lanes = laneEnd.length;
  return bars;
}
function findOrig(auto, id) { let r = null; walkNodes(auto, n => { if (n.id === id) { r = n; return false; } }, { markers: true }); return r; }
const GROUP_TYPES = ['sequential', 'parallel', 'race', 'deadline'];
function mkBar(n, depth, start, end) {
  let kind, label;
  if (GROUP_TYPES.includes(n.type)) { kind = 'group'; label = n.type; }
  else if (n.type === 'command') { const d = getCommandDef(n.name); kind = d ? d.category : 'custom'; label = d ? d.label : n.name; }
  else if (n.type === 'follow') { kind = 'follow'; label = `follow ${n.path}`; }
  else if (n.type === 'wait') { kind = 'wait'; label = `wait ${n.seconds}s`; }
  else { kind = 'waitUntil'; label = `until ${n.condition}`; }
  const b = { id: n.id, label, kind, start, end, lane: 0, depth };
  if (kind === 'group') b.group = n.type;
  return b;
}
