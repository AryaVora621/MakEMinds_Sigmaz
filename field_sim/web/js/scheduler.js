// Pure deterministic command scheduler (no DOM, no timers). Semantics follow WPILib / Ivy command libraries.
//
// Command = { name, initialize(ctx), execute(ctx, dt), isFinished(ctx), end(ctx, interrupted), node? }
//   All methods are optional except that a command without isFinished never finishes on its own.
// Lifecycle per tick: execute(ctx, dt) then isFinished(ctx). When a command (or a group child) is started,
// isFinished is also polled immediately after initialize, so zero-length commands finish in the same tick
// and a sequential group runs through any number of zero-length children in one tick.
// end(ctx, interrupted) is called exactly once for every command that was initialized.
// An exception in any command method is reported through ctx.onError(err, cmd) (if present), and that
// command is ended (interrupted = true) and treated as finished: nothing hangs.
// Trace entries: { t, name, event: 'start'|'end'|'interrupt', node } where node is command.node (a node id) or null.

let active = null; // scheduler currently ticking/scheduling (for trace logging from groups)
const MAX_LOOPS = 10000;

function log(c, event) {
  if (active) active.trace.push({ t: active.time, name: c.name || 'command', event, node: c.node ?? null });
}
function fail(ctx, c, err) {
  c._failed = true;
  try { ctx?.onError?.(err, c); } catch (_) { /* never propagate */ }
}
export function startCmd(c, ctx) {
  c._running = true; c._failed = false; log(c, 'start');
  try { c.initialize?.(ctx); } catch (e) { fail(ctx, c, e); endCmd(c, ctx, true); }
}
/** Executes one step; returns true when the command is finished (or failed). */
export function stepCmd(c, ctx, dt) {
  if (!c._running || c._failed) return true;
  try { c.execute?.(ctx, dt); } catch (e) { fail(ctx, c, e); endCmd(c, ctx, true); return true; }
  return pollDone(c, ctx);
}
/** isFinished only (used right after initialize). */
export function pollDone(c, ctx) {
  if (!c._running || c._failed) return true;
  try { return !!c.isFinished?.(ctx); } catch (e) { fail(ctx, c, e); endCmd(c, ctx, true); return true; }
}
export function endCmd(c, ctx, interrupted) {
  if (!c._running) return;
  c._running = false; // children are ended inside c.end, so the group's trace event comes after theirs
  try { c.end?.(ctx, !!interrupted); } catch (e) { fail(ctx, c, e); }
  log(c, interrupted ? 'interrupt' : 'end');
}

// ---------- leaf factories ----------
export function waitCmd(seconds) {
  let el = 0;
  return { name: `wait(${seconds})`, initialize() { el = 0; }, execute(_c, dt) { el += dt; }, isFinished() { return el >= seconds - 1e-9; } };
}
export function instantCmd(fn, name = 'instant') {
  return { name, initialize(ctx) { fn?.(ctx); }, isFinished: () => true };
}
export function runCmd(fn, isFinishedFn, name = 'run') {
  return { name, execute(ctx, dt) { fn?.(ctx, dt); }, isFinished: ctx => (isFinishedFn ? !!isFinishedFn(ctx) : false) };
}
/** Finishes when predicate(ctx) is true or `timeout` seconds elapse (timeout undefined/<=0/Infinity = none). */
export function waitUntilCmd(predicate, timeout, name = 'waitUntil') {
  let el = 0; const c = { name, timedOut: false };
  c.initialize = () => { el = 0; c.timedOut = false; };
  c.execute = (_x, dt) => { el += dt; };
  c.isFinished = ctx => {
    if (predicate(ctx)) return true;
    if (timeout > 0 && Number.isFinite(timeout) && el >= timeout - 1e-9) { c.timedOut = true; return true; }
    return false;
  };
  return c;
}

// ---------- autoAim (aim turret + hood + flywheel, optionally feed after aiming) ----------
/** Fallback numbers for the aim-time estimate. Used only when the caller's spec lacks the value. */
export const AUTOAIM_ASSUMPTIONS = Object.freeze({
  turretRateDegS: 180, // ASSUMPTION - not measured; replace with robot measurements
  flywheelTauS: 0.35, // ASSUMPTION - not measured; replace with robot measurements
  hoodRateDegS: 120, // ASSUMPTION - not measured; replace with robot measurements
  settleFrac: 0.03, // ASSUMPTION - not measured; replace with robot measurements
});
const positiveOr = (v, fallback) => (Number.isFinite(v) && v > 0 ? v : fallback);
const magOr0 = v => (Number.isFinite(v) ? Math.abs(v) : 0);

/**
 * Estimated seconds for turret, hood and flywheel to reach setpoint, clamped to [0.15, 3.0].
 * Turret and hood are rate-limited slews; flywheel spin-up is first-order: t = tau * ln(1/settleFrac) * flywheelFrac.
 * Spec keys read: turretRateDegS, flywheelTauS, hoodRateDegS (any missing key uses AUTOAIM_ASSUMPTIONS).
 */
export function estimateAutoAimSeconds({ turretErrDeg = 0, flywheelFrac = 1, hoodErrDeg = 0, spec = {} } = {}) {
  const A = AUTOAIM_ASSUMPTIONS;
  const s = spec ?? {};
  const turretRate = positiveOr(s.turretRateDegS, A.turretRateDegS);
  const tau = positiveOr(s.flywheelTauS, A.flywheelTauS);
  const hoodRate = positiveOr(s.hoodRateDegS, A.hoodRateDegS);
  const turretT = magOr0(turretErrDeg) / turretRate;
  const spinUpT = tau * Math.log(1 / A.settleFrac) * magOr0(flywheelFrac);
  const hoodT = magOr0(hoodErrDeg) / hoodRate;
  const raw = Math.max(turretT, spinUpT, hoodT);
  return Math.min(3.0, Math.max(0.15, raw));
}

/**
 * Leaf command: sets auto-aim on, then finishes once aimed. Aimed = ready(ctx) is true, or elapsed >= seconds
 * (seconds defaults to estimateAutoAimSeconds()). With shootAfter, feeds (ctx.feed(true)) for shootSeconds after aiming.
 * Hooks used if present: ctx.setAutoAim(bool), ctx.feed(bool). Missing hooks are ignored.
 */
export function autoAimCmd({ seconds, shootAfter = false, shootSeconds = 1.0, ready, name = 'autoAim' } = {}) {
  const shoot = Number.isFinite(shootSeconds) ? Math.max(0, shootSeconds) : 0;
  let el = 0, target = 0, aimed = false, aimedAt = 0, armed = false;
  const c = { name };
  c.initialize = () => {
    el = 0; aimed = false; aimedAt = 0; armed = false;
    target = Number.isFinite(seconds) ? seconds : estimateAutoAimSeconds();
  };
  c.execute = (ctx, dt) => {
    el += Number.isFinite(dt) ? dt : 0;
    if (!armed) { armed = true; ctx?.setAutoAim?.(true); }
    if (aimed && shootAfter) ctx?.feed?.(true);
  };
  c.isFinished = ctx => {
    if (!aimed) {
      const readyNow = typeof ready === 'function' && !!ready(ctx);
      if (readyNow || el >= target - 1e-9) { aimed = true; aimedAt = el; }
      else return false;
    }
    return !shootAfter || el >= aimedAt + shoot - 1e-9;
  };
  c.end = ctx => {
    if (shootAfter) ctx?.setAutoAim?.(false);
    ctx?.feed?.(false);
  };
  return c;
}

// ---------- group base ----------
function group(name, cmds, { onChildDone, finishedWhen }) {
  const g = { name, children: cmds, act: [], done: false };
  const settle = ctx => { // end children that are already finished, in order
    for (const c of [...g.act]) {
      if (g.done) break;
      if (pollDone(c, ctx)) childFinished(ctx, c);
    }
  };
  const childFinished = (ctx, c) => {
    const i = g.act.indexOf(c); if (i >= 0) g.act.splice(i, 1);
    endCmd(c, ctx, false);
    onChildDone(g, c, ctx);
    if (finishedWhen(g)) g.done = true;
  };
  g.initialize = ctx => {
    g.act = []; g.done = false;
    for (const c of cmds) g.act.push(c);
    for (const c of [...g.act]) startCmd(c, ctx);
    if (finishedWhen(g)) g.done = true;
    settle(ctx);
  };
  g.execute = (ctx, dt) => {
    for (const c of [...g.act]) {
      if (g.done) break;
      if (!g.act.includes(c)) continue;
      if (stepCmd(c, ctx, dt)) childFinished(ctx, c);
    }
  };
  g.isFinished = () => g.done;
  g.end = (ctx, interrupted) => { for (const c of [...g.act]) endCmd(c, ctx, true); g.act = []; };
  return g;
}

export function parallel(...cmds) {
  return group('parallel', cmds, { onChildDone() {}, finishedWhen: g => g.act.length === 0 });
}
export function race(...cmds) {
  return group('race', cmds, { onChildDone() {}, finishedWhen: g => g.children.length === 0 || g.act.length < g.children.length });
}
export function deadline(deadlineCmd, ...others) {
  const g = group('deadline', [deadlineCmd, ...others], {
    onChildDone(gr, c) { if (c === deadlineCmd) gr.deadlineDone = true; },
    finishedWhen: gr => !!gr.deadlineDone,
  });
  const init = g.initialize; g.initialize = ctx => { g.deadlineDone = false; init(ctx); };
  return g;
}

export function sequential(...cmds) {
  const g = { name: 'sequential', children: cmds, idx: 0, cur: null, done: false };
  const advance = ctx => { // start the next child(ren); loops through zero-length ones
    for (let n = 0; n < MAX_LOOPS; n++) {
      if (g.idx >= cmds.length) { g.cur = null; g.done = true; return; }
      g.cur = cmds[g.idx++]; startCmd(g.cur, ctx);
      if (!pollDone(g.cur, ctx)) return;
      endCmd(g.cur, ctx, false);
    }
    g.cur = null; g.done = true;
  };
  g.initialize = ctx => { g.idx = 0; g.cur = null; g.done = false; advance(ctx); };
  g.execute = (ctx, dt) => {
    if (g.done || !g.cur) return;
    if (stepCmd(g.cur, ctx, dt)) { endCmd(g.cur, ctx, false); advance(ctx); }
  };
  g.isFinished = () => g.done;
  g.end = ctx => { if (g.cur) endCmd(g.cur, ctx, true); g.cur = null; };
  return g;
}

/** repeat(cmd, n): runs cmd n times in a row (n = Infinity loops forever, one restart cap per tick for zero-length cmds). */
export function repeat(cmd, n = Infinity) {
  const g = { name: `repeat(${n})`, children: [cmd], i: 0, done: false };
  const next = ctx => {
    for (let k = 0; k < MAX_LOOPS; k++) {
      if (g.i >= n) { g.done = true; return; }
      g.i++; startCmd(cmd, ctx);
      if (!pollDone(cmd, ctx)) return;
      endCmd(cmd, ctx, false);
    }
  };
  g.initialize = ctx => { g.i = 0; g.done = false; next(ctx); };
  g.execute = (ctx, dt) => { if (!g.done && stepCmd(cmd, ctx, dt)) { endCmd(cmd, ctx, false); next(ctx); } };
  g.isFinished = () => g.done;
  g.end = ctx => endCmd(cmd, ctx, true);
  return g;
}

/** conditional(pred, a, b): picks a or b when started (b optional: nothing). */
export function conditional(pred, a, b) {
  const g = { name: 'conditional', cur: null, done: false, children: [a, b].filter(Boolean) };
  g.initialize = ctx => {
    g.done = false; g.cur = null;
    let pick = null; try { pick = pred(ctx) ? a : b; } catch (e) { fail(ctx, g, e); g.done = true; return; }
    if (!pick) { g.done = true; return; }
    g.cur = pick; startCmd(pick, ctx);
    if (pollDone(pick, ctx)) { endCmd(pick, ctx, false); g.done = true; }
  };
  g.execute = (ctx, dt) => { if (g.cur && g.cur._running && stepCmd(g.cur, ctx, dt)) { endCmd(g.cur, ctx, false); g.done = true; } else if (g.cur && !g.cur._running) g.done = true; };
  g.isFinished = () => g.done;
  g.end = ctx => { if (g.cur) endCmd(g.cur, ctx, true); };
  return g;
}

// ---------- scheduler ----------
export class Scheduler {
  constructor() { this.time = 0; this.trace = []; this.running = []; this.pending = []; }
  schedule(cmd) { this.pending.push(cmd); return cmd; }
  isRunning() { return this.running.length > 0 || this.pending.length > 0; }
  tick(ctx, dt) {
    const prev = active; active = this;
    try {
      this.time += dt;
      while (this.pending.length) {
        const c = this.pending.shift(); this.running.push(c);
        startCmd(c, ctx);
        if (pollDone(c, ctx)) this._finish(c, ctx, false);
      }
      for (const c of [...this.running]) {
        if (!this.running.includes(c)) continue;
        if (stepCmd(c, ctx, dt)) this._finish(c, ctx, false);
      }
    } finally { active = prev; }
  }
  _finish(c, ctx, interrupted) {
    const i = this.running.indexOf(c); if (i >= 0) this.running.splice(i, 1);
    endCmd(c, ctx, interrupted);
  }
  cancelAll(ctx) {
    const prev = active; active = this;
    try {
      const all = [...this.running]; this.running = []; this.pending = [];
      for (const c of all) endCmd(c, ctx, true);
    } finally { active = prev; }
  }
}
