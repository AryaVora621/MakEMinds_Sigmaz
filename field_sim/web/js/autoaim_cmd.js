/**
 * autoaim_cmd.js - pure "autoAim" auto command (no DOM, no timers, no imports).
 *
 * Aligns flywheel rpm, turret angle and hood angle to a solved shot, then (optionally)
 * fires a full hopper. It is a scheduler-style command object:
 *   start()        begin aiming; turns transfer on via host.transfer(true)
 *   update(dt)     re-reads host.target(), drives plants via host.apply(target), checks readiness
 *   done()         true when the command has finished (ready and tracking not needed, timeout, or fired)
 *   stop()         ends the command; host.transfer(false) unless opts.keepTransfer
 *   status()       plain snapshot for the UI / logs
 *
 * Host contract (all methods may be called every update; exceptions are caught, never thrown):
 *   target()   -> {rpm, turretDeg, hoodDeg, feasible} | null   (null = no solution right now)
 *   current()  -> {rpm, turretDeg, hoodDeg}
 *   apply(t)   drive the flywheel / turret / hood plants toward target t
 *   transfer(on)
 *   shoot(args)            called once when shootAfter fires, with {all: true}
 *   hopperEmpty?()         optional; firing ends early when it returns true
 *
 * Tolerance sources:
 *   rpmTolFrac 0.04 and turretTolDeg 2 mirror the existing sim thresholds in app.js autoHost
 *   (flywheelReady uses 0.04 and the turret check uses 2 deg). These are SIM-ONLY values.
 *   Robot-side constants live in TeamCode (Flywheel.AT_TARGET_TOLERANCE = 50 ticks/s,
 *   Turret.AT_TARGET_TOLERANCE_DEG = 1.5, Hood.AT_TARGET_TOL_DEG = 1.5 PLACEHOLDER).
 *   hoodTolDeg and settleSec are sim-only defaults, not robot constants.
 */

/** Default tolerances and timing for autoAim. Sim-only defaults except where noted above. */
export const AUTOAIM_DEFAULTS = Object.freeze({
  timeoutSec: 4,      // sim default for how long to wait for ready before giving up
  rpmTolFrac: 0.04,   // mirrors app.js autoHost flywheelReady (0.04)
  minRpm: 300,        // mirrors app.js autoHost flywheelReady (rpm > 300)
  turretTolDeg: 2,    // mirrors app.js autoHost turret check (2 deg)
  hoodTolDeg: 1.5,    // sim-only default (TeamCode Hood.AT_TARGET_TOL_DEG is 1.5 but marked PLACEHOLDER)
  settleSec: 0.1,     // sim-only default: ready must hold this long (not a robot constant)
  transferOnStart: false, // mirrors Java RobotSettings.AutoAim.TRANSFER_ON_START (false: do not feed while still aiming)
});

/** Seconds allowed for the post-ready firing phase before the command gives up on an empty-hopper signal. */
const FIRE_MAX_SEC = 6;

/**
 * True when the current state is aimed at the target within tolerance.
 * @param {{rpm:number, turretDeg:number, hoodDeg:number}} cur
 * @param {{rpm:number, turretDeg:number, hoodDeg:number, feasible:boolean}|null} target
 * @param {typeof AUTOAIM_DEFAULTS} tol
 * @returns {boolean}
 */
export function aimReady(cur, target, tol = AUTOAIM_DEFAULTS) {
  if (!cur || !target || !target.feasible) return false;
  const t = { ...AUTOAIM_DEFAULTS, ...(tol || {}) };
  if (!(target.rpm >= t.minRpm) || !(target.rpm > 0)) return false;
  if (!(Math.abs(cur.rpm - target.rpm) / target.rpm < t.rpmTolFrac)) return false;
  if (!(Math.abs(cur.turretDeg - target.turretDeg) < t.turretTolDeg)) return false;
  if (!(Math.abs(cur.hoodDeg - target.hoodDeg) < t.hoodTolDeg)) return false;
  return true;
}

/**
 * Creates an autoAim command.
 * @param {object} opts
 * @param {object} opts.host                   see host contract above
 * @param {boolean} [opts.shootAfter=false]    fire a full hopper once ready
 * @param {boolean} [opts.fireOnTimeout=false] also fire if the timeout expires before ready
 * @param {boolean} [opts.track=false]         keep applying the target after ready (for a following drive)
 * @param {boolean} [opts.keepTransfer=false]  do not call host.transfer(false) on stop
 * @param {number}  [opts.timeoutSec]          overrides AUTOAIM_DEFAULTS.timeoutSec
 * @param {object}  [opts.tolerances]          partial override of AUTOAIM_DEFAULTS tolerances
 * @param {(msg:string)=>void} [opts.onLog]
 * @param {(reason:string)=>void} [opts.onFail]
 */
export function makeAutoAim(opts = {}) {
  const host = opts.host || {};
  const shootAfter = !!opts.shootAfter;
  const fireOnTimeout = opts.fireOnTimeout === true;
  const track = opts.track === true;
  const keepTransfer = opts.keepTransfer === true;
  const tol = { ...AUTOAIM_DEFAULTS, ...(opts.tolerances || {}) };
  const timeoutSec = Number.isFinite(opts.timeoutSec) ? opts.timeoutSec : tol.timeoutSec;
  const log = msg => { try { opts.onLog?.(String(msg)); } catch (_) { /* never throw */ } };
  const fail = reason => { try { opts.onFail?.(String(reason)); } catch (_) { /* never throw */ } };

  // Calls a host method safely. Returns {ok, value}.
  function call(name, ...args) {
    const fn = host[name];
    if (typeof fn !== 'function') return { ok: false, value: undefined };
    try { return { ok: true, value: fn(...args) }; }
    catch (e) { log(`autoAim: host.${name} threw: ${e?.message ?? e}`); return { ok: false, value: undefined }; }
  }

  // phase: 'idle' | 'aiming' | 'tracking' | 'firing' | 'finished'
  const S = {
    phase: 'idle',
    result: null,       // null | 'ready' | 'timeout' | 'stopped' | 'failed'
    reason: null,
    elapsed: 0,
    readySec: 0,
    ready: false,
    tracking: false,
    fired: false,
    firedOnTimeout: false,
    fireElapsed: 0,
    finished: false,
    stopped: false,
    cur: null,
    target: null,
  };

  function finish(result, reason) {
    if (S.finished) return;
    S.finished = true;
    if (result && !S.result) S.result = result;
    if (reason) S.reason = reason;
    S.phase = 'finished';
  }

  function fire(why) {
    if (S.fired) return;
    S.fired = true;
    S.phase = 'firing';
    S.fireElapsed = 0;
    const r = call('shoot', { all: true });
    if (!r.ok) {
      fail(`shoot failed (${why})`);
      finish('failed', 'shoot failed');
      return;
    }
    log(`autoAim: firing (${why})`);
  }

  function applyTarget(t) {
    if (t) call('apply', t);
  }

  function readState() {
    const c = call('current');
    S.cur = c.ok ? c.value : null;
    const t = call('target');
    S.target = t.ok && t.value ? t.value : null;
  }

  return {
    name: 'autoAim',
    /** Begins aiming. Turns transfer on only if transferOnStart (default false, like Java TRANSFER_ON_START). */
    start() {
      if (S.phase !== 'idle') return;
      S.phase = 'aiming';
      S.elapsed = 0;
      if ((opts.transferOnStart ?? AUTOAIM_DEFAULTS.transferOnStart) === true) call('transfer', true);
      log(`autoAim: start (timeout ${timeoutSec}s${shootAfter ? ', shoot after' : ''}${track ? ', track' : ''})`);
    },

    /** Advances the command by dt seconds. Safe to call after done(). */
    update(dt = 0) {
      if (S.phase === 'idle' || S.stopped) return;
      const step = Number.isFinite(dt) && dt > 0 ? dt : 0;

      if (S.phase === 'firing') {
        readState();
        applyTarget(S.target);
        S.fireElapsed += step;
        const empty = call('hopperEmpty');
        if ((empty.ok && empty.value === true) || S.fireElapsed >= FIRE_MAX_SEC) {
          finish(null, empty.ok && empty.value === true ? 'hopper empty' : 'fire timeout');
          log(`autoAim: done (${S.reason})`);
        }
        return;
      }

      if (S.finished) {
        // Only tracking keeps running after finish (track mode).
        if (S.tracking && !S.stopped) { readState(); applyTarget(S.target); }
        return;
      }

      S.elapsed += step;
      readState();
      applyTarget(S.target);
      S.ready = aimReady(S.cur, S.target, tol);
      S.readySec = S.ready ? S.readySec + step : 0;

      if (S.readySec >= tol.settleSec - 1e-9) {
        S.result = 'ready';
        log('autoAim: ready');
        if (shootAfter) { fire('ready'); return; }
        if (track) { S.phase = 'tracking'; S.tracking = true; finish(null, 'ready (tracking)'); return; }
        finish(null, 'ready');
        return;
      }

      if (S.elapsed >= timeoutSec - 1e-9) {
        const why = !S.target ? 'no solution' : !S.target.feasible ? 'infeasible shot' : 'not on target';
        const reason = `timeout after ${timeoutSec}s (${why})`;
        S.result = 'timeout';
        log(`autoAim: ${reason}`);
        fail(reason);
        if (shootAfter && fireOnTimeout) { S.firedOnTimeout = true; fire('timeout'); return; }
        if (track) { S.tracking = true; finish('timeout', reason); return; }
        finish('timeout', reason);
      }
    },

    /** True once the command has finished (ready without shoot, timeout, firing complete, or stopped). */
    done() { return S.finished; },

    /** Ends the command. Calls host.transfer(false) unless opts.keepTransfer. Idempotent. */
    stop() {
      if (S.stopped) return;
      S.stopped = true;
      if (!S.finished) { S.result = S.result || 'stopped'; S.reason = S.reason || 'stopped'; }
      S.finished = true;
      S.phase = 'finished';
      S.tracking = false;
      if (!keepTransfer) call('transfer', false);
    },

    /** Plain snapshot for UI and logs. */
    status() {
      return {
        phase: S.phase,
        result: S.result,
        reason: S.reason,
        elapsed: S.elapsed,
        readySec: S.readySec,
        ready: S.ready,
        tracking: S.tracking,
        fired: S.fired,
        firedOnTimeout: S.firedOnTimeout,
        done: S.finished,
        cur: S.cur,
        target: S.target,
      };
    },
  };
}
