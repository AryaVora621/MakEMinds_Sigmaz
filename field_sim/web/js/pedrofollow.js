// Client for the REAL Pedro 2.1.0 Follower simulation (tools_java/PedroFollowSim.java, served by run_sim.py).
// Pure ES module (no DOM); fetch is injectable for tests.
//
//   followRoute(route, startPose?, opts?) -> Promise<Traj>      Traj = [{t,x,y,h,vx,vy,w,p,seg,phase}] (+ .events .finished .source ...)
//   PedroLiveFollower                                              live stepping (start / step / stop)
//   sampleTrajectory(traj, t) -> {t,x,y,h,vx,vy}                   interpolated pose at time t (for playback)
//
// Units: inches, seconds, radians (h, w); vx/vy are field-frame in/s; p = wheel powers [LF, LR, RF, RR];
// seg = segment index being followed; phase 0 follow, 1 wait/hold, 2 settle, 3 done.
// source === 'pedro-core'  -> the real Pedro Follower ran (PIDFs, Kalman drive error, centripetal, braking, hold).
// source === 'js-fallback' -> the server/JDK was unavailable: a crude kinematic JS approximation (approximate:true).
import { routeToBridgeJson, followSim } from './pathing.js';

export const FOLLOW_URL = '/api/pedro/follow';
export const LIVE_START_URL = '/api/pedro/live/start';
export const LIVE_STEP_URL = '/api/pedro/live/step';
export const LIVE_STOP_URL = '/api/pedro/live/stop';

const rad = d => d * Math.PI / 180;
const wrap = a => { a = ((a % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI); return a > Math.PI ? a - 2 * Math.PI : a; };
const defFetch = () => (typeof globalThis.fetch === 'function' ? globalThis.fetch.bind(globalThis) : null);

/** Request body for /api/pedro/follow. startPose = {x,y,h(rad)}; defaults to route.start. A segment may carry
 *  waitAfter (seconds, robot holds the end point before the next segment) and callbacks [{t:0..1, name}] / [{ms, name}]. */
export function followRequest(route, startPose, opts = {}) {
  const base = routeToBridgeJson(route, 1);
  const sp = startPose || route.start || { x: 0, y: 0, h: 0 };
  const body = { start: { x: sp.x, y: sp.y, h: (sp.h || 0) * 180 / Math.PI }, segments: base.segments };
  base.segments.forEach((sg, i) => {
    const s = route.segments[i];
    const cbs = (s.callbacks || []).map(c => (c.ms !== undefined ? { ms: c.ms, name: c.name || 'callback' } : { t: c.t, name: c.name || 'callback' }));
    if (cbs.length) sg.callbacks = cbs;
    const w = s.waitAfter ?? s.wait;
    if (Number.isFinite(w) && w > 0) sg.waitAfter = w;
  });
  for (const k of ['dt', 'stride', 'maxTime', 'settle', 'holdEnd', 'maxPower', 'robot', 'constants', 'pathConstraints']) if (opts[k] !== undefined) body[k] = opts[k];
  return body;
}

/** Server response -> trajectory array (radians). */
export function responseToTrajectory(resp) {
  const traj = (resp.ticks || []).map(k => ({ t: k.t, x: k.x, y: k.y, h: rad(k.h), vx: k.vx, vy: k.vy, w: rad(k.w || 0), p: k.p, seg: k.seg, phase: k.phase }));
  traj.source = 'pedro-core'; traj.approximate = false; traj.events = resp.events || []; traj.finished = !!resp.finished;
  traj.endReason = resp.endReason; traj.duration = resp.duration ?? traj.at(-1)?.t ?? 0; traj.info = resp.info || null; traj.error = null;
  return traj;
}

/** Crude kinematic stand-in (pathing.followSim), only used when the Java simulation cannot run. Clearly flagged. */
export function fallbackTrajectory(route, startPose, opts = {}, error = 'Pedro follow simulation unavailable') {
  const r = JSON.parse(JSON.stringify(route));
  if (startPose) r.start = { ...r.start, ...startPose };
  const dt = opts.dt ?? 0.01, stride = Math.max(1, opts.stride ?? 2), maxT = opts.maxTime ?? 60, state = {}, traj = [];
  let t = 0, prev = null, n = 0;
  for (; t <= maxT; t += dt, n++) {
    const q = followSim(r, dt, state, { vmax: opts.vmax ?? 60, aMax: opts.aMax ?? 90 });
    const vx = prev ? (q.x - prev.x) / dt : 0, vy = prev ? (q.y - prev.y) / dt : 0;
    if (n % stride === 0 || q.done) traj.push({ t: +t.toFixed(4), x: q.x, y: q.y, h: q.h, vx, vy, w: 0, p: [0, 0, 0, 0], seg: 0, phase: q.done ? 3 : 0 });
    prev = q; if (q.done) break;
  }
  traj.source = 'js-fallback'; traj.approximate = true; traj.events = []; traj.finished = !!prev?.done; traj.endReason = traj.finished ? 'finished' : 'timeout';
  traj.duration = traj.at(-1)?.t ?? 0; traj.info = null; traj.error = error;
  return traj;
}

/** Runs the real Pedro Follower over `route`. opts: {fetchImpl, signal, fallback=true, dt, stride, maxTime, settle, holdEnd, maxPower, robot, constants, pathConstraints}. */
export async function followRoute(route, startPose, opts = {}) {
  if (!route || !route.segments || !route.segments.length) throw new Error('followRoute: route has no segments');
  const f = opts.fetchImpl || defFetch();
  try {
    if (!f) throw new Error('fetch unavailable');
    const res = await f(FOLLOW_URL, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(followRequest(route, startPose, opts)), signal: opts.signal });
    let body = null;
    try { body = await res.json(); } catch { /* handled below */ }
    if (!res.ok || !body || body.error) throw new Error(body?.error || 'HTTP ' + res.status);
    return responseToTrajectory(body);
  } catch (e) {
    if (opts.fallback === false || e?.name === 'AbortError') throw e;
    return fallbackTrajectory(route, startPose, opts, String(e?.message || e));
  }
}

/** Interpolated state at time t (clamped to the trajectory). */
export function sampleTrajectory(traj, t) {
  if (!traj.length) return null;
  if (t <= traj[0].t) return { ...traj[0] };
  const last = traj[traj.length - 1];
  if (t >= last.t) return { ...last };
  let lo = 0, hi = traj.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (traj[m].t <= t) lo = m; else hi = m; }
  const a = traj[lo], b = traj[hi], f = (t - a.t) / (b.t - a.t || 1);
  return { t, x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, h: wrap(a.h + wrap(b.h - a.h) * f), vx: a.vx + (b.vx - a.vx) * f, vy: a.vy + (b.vy - a.vy) * f, seg: a.seg, phase: a.phase };
}

/** Live stepping: `await live.start(route, startPose, opts)`, then `await live.step(n)` repeatedly (n ticks of dt each,
 *  returns {ticks,events,done}); `await live.stop()` frees the Java process (idle sessions expire after 5 min). */
export class PedroLiveFollower {
  constructor(fetchImpl = defFetch()) { this.fetchImpl = fetchImpl; this.id = null; this.done = false; this.info = null; this.t = 0; }
  async _post(url, body) {
    const res = await this.fetchImpl(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    let j = null; try { j = await res.json(); } catch { /* below */ }
    if (!res.ok || !j || j.error) throw new Error(j?.error || 'HTTP ' + res.status);
    return j;
  }
  async start(route, startPose, opts = {}) {
    const j = await this._post(LIVE_START_URL, followRequest(route, startPose, opts));
    this.id = j.id; this.info = j.info || null; this.done = false; this.t = 0;
    return responseToTrajectory({ ticks: j.tick ? [j.tick] : [], info: j.info });
  }
  async step(n = 10) {
    if (!this.id) throw new Error('PedroLiveFollower: not started');
    const j = await this._post(LIVE_STEP_URL, { id: this.id, n });
    this.done = !!j.done; this.t = j.t ?? this.t;
    const traj = responseToTrajectory({ ticks: j.ticks, events: j.events });
    traj.done = this.done; traj.endReason = j.endReason;
    return traj;
  }
  async stop() { if (!this.id) return; const id = this.id; this.id = null; try { await this._post(LIVE_STOP_URL, { id }); } catch { /* process may already be gone */ } }
}
