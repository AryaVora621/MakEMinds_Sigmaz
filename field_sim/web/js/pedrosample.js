// Client for the REAL Pedro 2.1.0 sampler (POST /api/pedro/sample, see pedro_bridge.py) with caching, stale-request
// aborting and a JS fallback (pathing.sampleRoute) when the server is unavailable. Pure ES module; fetch is injectable.
import { sampleRoute, routeToBridgeJson, routeLength, wrapAngle } from './pathing.js';

const rad = d => d * Math.PI / 180;
export const SAMPLE_URL = '/api/pedro/sample';

/** Converts a bridge response to the sampleRoute shape: flat [{x,y,h(rad),t,seg,s(total arc length)}] (+ per-segment info). */
export function bridgeToSamples(resp) {
  const samples = [], segments = [];
  (resp.segments || []).forEach((sg, i) => {
    const off = Number.isFinite(sg.offset) ? sg.offset : 0, list = [];
    (sg.samples || []).forEach((q, j) => {
      const last = samples.at(-1);
      if (i > 0 && j === 0 && last && Math.hypot(last.x - q.x, last.y - q.y) < 1e-6) return;  // shared join, listed once
      const p = { x: q.x, y: q.y, h: wrapAngle(rad(q.h)), t: q.t, seg: i, s: off + (q.s || 0) };
      samples.push(p); list.push(p);
    });
    segments.push({ type: sg.type, mode: sg.mode, length: sg.length, offset: off, samples: list });
  });
  const total = Number.isFinite(resp.total) ? resp.total : (samples.at(-1)?.s ?? 0);
  return { total, segments, samples };
}

function fallbackResult(route, step, error) {
  const samples = sampleRoute(route, step), segs = [];
  samples.forEach(p => { if (p.seg >= 0) (segs[p.seg] ||= { type: route.segments[p.seg].type, mode: route.segments[p.seg].heading.mode, samples: [] }).samples.push(p); });
  return { ok: true, source: 'js-fallback', total: routeLength(route), segments: segs, samples, error: error || 'Pedro sampler unavailable', segCount: route.segments.length };
}

/** One un-cached request. Throws on network / server error. */
async function fetchPedro(route, step, fetchImpl, signal) {
  const res = await fetchImpl(SAMPLE_URL, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(routeToBridgeJson(route, step)), signal });
  if (!res.ok) throw new Error('HTTP ' + res.status);
  const body = await res.json();
  if (!body || body.error) throw new Error(body?.error || 'empty response');
  const conv = bridgeToSamples(body);
  return { ok: true, source: 'pedro-core', ...conv, segCount: route.segments.length };
}

const hashStr = s => { let h = 5381; for (let i = 0; i < s.length; i++) h = ((h << 5) + h + s.charCodeAt(i)) | 0; return (h >>> 0).toString(36) + ':' + s.length; };
/** Hash of everything the sampler depends on. */
export function hashRoute(route, step = 1) { return hashStr(JSON.stringify(routeToBridgeJson(route, step))); }

export class PedroSampler {
  constructor(fetchImpl = globalThis.fetch?.bind(globalThis), { debounceMs = 0, cacheSize = 32, step = 1 } = {}) {
    this.fetchImpl = fetchImpl; this.debounceMs = debounceMs; this.cacheSize = cacheSize; this.step = step;
    this.cache = new Map(); this.latest = null; this.inFlight = 0; this._ctl = null; this._timer = null; this._tok = 0; this._cancel = null; this.lastSource = null; this.lastError = null;
  }
  _get(k) { if (!this.cache.has(k)) return null; const v = this.cache.get(k); this.cache.delete(k); this.cache.set(k, v); return v; }
  _put(k, v) { this.cache.set(k, v); while (this.cache.size > this.cacheSize) this.cache.delete(this.cache.keys().next().value); }
  get busy() { return this.inFlight > 0 || !!this._timer; }
  /** Requests samples for the route (snapshot taken now). Resolves {aborted:true, ok:false} if superseded by a newer request. */
  request(route) {
    const step = this.step, tok = ++this._tok;
    if (this._ctl) { this._ctl.abort(); this._ctl = null; }
    if (this._timer) { clearTimeout(this._timer); this._timer = null; this._cancel?.({ ok: false, aborted: true }); this._cancel = null; }
    if (!route.segments.length) {
      const r = { ok: true, source: 'empty', total: 0, segments: [], samples: sampleRoute(route, step), segCount: 0 };
      this.latest = r; return Promise.resolve(r);
    }
    const key = hashRoute(route, step), hit = this._get(key);
    if (hit) { this.latest = hit; this.lastSource = hit.source; return Promise.resolve(hit); }
    const snap = JSON.parse(JSON.stringify(route));           // route is mutated in place by the editor
    return new Promise(resolve => {
      const run = async () => {
        this._timer = null; this._cancel = null;
        const ctl = typeof AbortController !== 'undefined' ? new AbortController() : null; this._ctl = ctl; this.inFlight++;
        let out;
        try {
          if (!this.fetchImpl) throw new Error('fetch unavailable');
          out = await fetchPedro(snap, step, this.fetchImpl, ctl?.signal);
          this._put(key, out); this.lastError = null;
        } catch (e) {
          if (tok !== this._tok || e?.name === 'AbortError') { this.inFlight--; return resolve({ ok: false, aborted: true }); }
          out = fallbackResult(snap, step, String(e?.message || e)); this.lastError = out.error;
        }
        this.inFlight--;
        if (tok !== this._tok) return resolve({ ok: false, aborted: true });
        if (this._ctl === ctl) this._ctl = null;
        this.latest = out; this.lastSource = out.source; resolve(out);
      };
      if (this.debounceMs > 0) { this._cancel = resolve; this._timer = setTimeout(run, this.debounceMs); this._timer.unref?.(); } else run();
    });
  }
}

let shared = null;
/** One-shot sampling through a shared cache: {ok, source:'pedro-core'|'js-fallback', total, segments, samples, error?}. */
export async function samplePedro(route, { step = 1, signal, fetchImpl } = {}) {
  if (!shared || fetchImpl) { const s = new PedroSampler(fetchImpl ?? globalThis.fetch?.bind(globalThis), { step }); if (fetchImpl) return runOnce(s, route, step, signal); shared = s; }
  return runOnce(shared, route, step, signal);
}
async function runOnce(s, route, step, signal) {
  if (signal?.aborted) return { ok: false, aborted: true };
  s.step = step;
  const r = await s.request(route);
  return signal?.aborted ? { ok: false, aborted: true } : r;
}
