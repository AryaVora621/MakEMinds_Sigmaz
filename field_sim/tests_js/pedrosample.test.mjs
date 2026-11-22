import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';
import { PedroSampler, samplePedro, bridgeToSamples, hashRoute } from '../web/js/pedrosample.js';

function route() {
  const r = P.newRoute({ start: { x: 10, y: 10, h: Math.PI / 2 } });
  P.addLine(r, 10, 50); P.addCurve(r, 50, 80, 10, 80); return r;
}
const bridge = {
  start: { x: 10, y: 10, h: 90 }, total: 20,
  segments: [
    { type: 'line', mode: 'linear', length: 10, offset: 0, samples: [{ x: 0, y: 0, h: 90, t: 0, s: 0 }, { x: 0, y: 10, h: 180, t: 1, s: 10 }] },
    { type: 'line', mode: 'tangent', length: 10, offset: 10, samples: [{ x: 0, y: 10, h: 0, t: 0, s: 0 }, { x: 10, y: 10, h: 0, t: 1, s: 10 }] },
  ],
};
const okFetch = (calls = []) => async (url, init) => { calls.push({ url, body: JSON.parse(init.body) }); return { ok: true, json: async () => bridge }; };

test('bridgeToSamples converts degrees to radians, offsets arc length and de-duplicates joins', () => {
  const r = bridgeToSamples(bridge);
  assert.equal(r.samples.length, 3); assert.equal(r.total, 20);
  assert.ok(Math.abs(Math.abs(r.samples[1].h) - Math.PI) < 1e-9);
  assert.deepEqual(r.samples.map(p => p.seg), [0, 0, 1]); assert.equal(r.samples[2].s, 20);
});
test('PedroSampler posts degrees to /api/pedro/sample and reports pedro-core', async () => {
  const calls = [], s = new PedroSampler(okFetch(calls));
  const res = await s.request(route());
  assert.equal(res.source, 'pedro-core'); assert.equal(res.ok, true);
  assert.equal(calls[0].url, '/api/pedro/sample'); assert.equal(calls[0].body.start.h, 90); assert.equal(calls[0].body.segments.length, 2);
  assert.equal(s.latest, res);
});
test('cache hit avoids a second request; LRU evicts oldest', async () => {
  const calls = [], s = new PedroSampler(okFetch(calls), { cacheSize: 2 });
  const r = route(); await s.request(r); await s.request(r); assert.equal(calls.length, 1);
  for (const x of [20, 30, 40]) { const q = route(); q.segments[0].points[1].x = x; await s.request(q); }
  assert.equal(s.cache.size, 2); await s.request(r); assert.equal(calls.length, 5);
});
test('falls back to the JS sampler when fetch throws', async () => {
  const s = new PedroSampler(async () => { throw new Error('offline'); });
  const res = await s.request(route());
  assert.equal(res.source, 'js-fallback'); assert.ok(res.samples.length > 10); assert.match(res.error, /offline/); assert.equal(s.latest, res);
});
test('falls back on {error} body and on HTTP error; fallback is not cached', async () => {
  let n = 0; const s = new PedroSampler(async () => { n++; return n === 1 ? { ok: true, json: async () => ({ error: 'bad' }) } : { ok: false, status: 500 }; });
  const a = await s.request(route()); assert.equal(a.source, 'js-fallback'); assert.match(a.error, /bad/);
  const b = await s.request(route()); assert.equal(b.source, 'js-fallback'); assert.match(b.error, /500/); assert.equal(n, 2);
});
test('a newer request supersedes a stale one; latest keeps the last good result', async () => {
  const pending = [];
  const f = (url, init) => new Promise((res, rej) => { init.signal?.addEventListener('abort', () => rej(Object.assign(new Error('x'), { name: 'AbortError' }))); pending.push(res); });
  const s = new PedroSampler(f), r1 = route(), r2 = route(); r2.segments[0].points[1].x = 33;
  const p1 = s.request(r1), p2 = s.request(r2);
  await new Promise(r => setImmediate(r));
  pending[1]({ ok: true, json: async () => bridge });
  assert.equal((await p1).aborted, true); const second = await p2;
  assert.equal(second.source, 'pedro-core'); assert.equal(s.latest, second);
});
test('debounce collapses rapid requests into one fetch', async () => {
  const calls = [], s = new PedroSampler(okFetch(calls), { debounceMs: 10 });
  const a = s.request(route()); const q = route(); q.segments[0].points[1].x = 12; const b = s.request(q);
  assert.equal((await a).aborted, true); assert.equal((await b).source, 'pedro-core'); assert.equal(calls.length, 1);
});
test('empty route needs no network; hashRoute is stable and sensitive', async () => {
  const s = new PedroSampler(async () => { throw new Error('must not be called'); });
  const res = await s.request(P.newRoute()); assert.equal(res.samples.length, 1);
  const r = route(), h = hashRoute(r); assert.equal(hashRoute(route()), h); r.segments[1].points[1].y += 1; assert.notEqual(hashRoute(r), h);
});
test('samplePedro with injected fetch returns the same shape', async () => {
  const res = await samplePedro(route(), { fetchImpl: okFetch() });
  assert.equal(res.source, 'pedro-core'); assert.ok(Array.isArray(res.segments)); assert.equal(res.total, 20);
  const dead = await samplePedro(route(), { fetchImpl: async () => { throw new Error('no'); } }); assert.equal(dead.source, 'js-fallback');
  const ctl = new AbortController(); ctl.abort(); assert.equal((await samplePedro(route(), { fetchImpl: okFetch(), signal: ctl.signal })).aborted, true);
});
test('LIVE (optional): real Pedro sampler on 127.0.0.1:8783', async t => {
  let up = false;
  try { up = (await (await fetch('http://127.0.0.1:8783/api/pedro/status', { signal: AbortSignal.timeout(800) })).json()).available; } catch { /* no server */ }
  if (!up) return t.skip('server not running');
  const real = (u, i) => fetch('http://127.0.0.1:8783' + u, i);
  const res = await samplePedro(route(), { fetchImpl: real });
  assert.equal(res.source, 'pedro-core'); assert.ok(Math.abs(res.total - P.routeLength(route())) < 1.5, `${res.total}`);
});
