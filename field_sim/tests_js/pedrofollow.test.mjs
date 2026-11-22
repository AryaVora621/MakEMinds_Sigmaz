import test from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import * as P from '../web/js/pathing.js';
import { followRoute, followRequest, responseToTrajectory, sampleTrajectory, fallbackTrajectory, PedroLiveFollower } from '../web/js/pedrofollow.js';

function route() {
  const r = P.newRoute({ start: { x: 24, y: 24, h: 0 } });
  P.addLine(r, 72, 24); P.addCurve(r, 72, 72, 100, 30);
  r.segments[0].callbacks = [{ t: 0.5, name: 'mid' }];
  r.segments[0].waitAfter = 0.5;
  r.segments[0].heading = { mode: 'linear', reversed: false, start: 0, end: Math.PI / 2 };
  return r;
}
const resp = {
  ok: true, source: 'pedro-core', finished: true, endReason: 'finished', duration: 1,
  info: { dt: 0.005, stride: 2 }, events: [{ t: 0.5, kind: 'callback', name: 'mid', seg: 0 }],
  ticks: [
    { t: 0, x: 0, y: 0, h: 0, vx: 0, vy: 0, w: 0, p: [0, 0, 0, 0], seg: 0, phase: 0 },
    { t: 1, x: 10, y: 20, h: 90, vx: 10, vy: 20, w: 45, p: [1, 1, 1, 1], seg: 1, phase: 3 },
  ],
};
const okFetch = (calls = []) => async (url, init) => { calls.push({ url, body: JSON.parse(init.body) }); return { ok: true, status: 200, json: async () => resp }; };

test('followRequest converts radians to degrees and carries callbacks / waits / options', () => {
  const b = followRequest(route(), { x: 5, y: 6, h: Math.PI }, { maxTime: 12, constants: { mass: 9 }, dt: 0.01 });
  assert.deepEqual(b.start, { x: 5, y: 6, h: 180 });
  assert.equal(b.segments.length, 2); assert.equal(b.segments[0].heading.end, 90);
  assert.deepEqual(b.segments[0].callbacks, [{ t: 0.5, name: 'mid' }]);
  assert.equal(b.segments[0].waitAfter, 0.5); assert.equal(b.segments[1].waitAfter, undefined);
  assert.equal(b.maxTime, 12); assert.equal(b.constants.mass, 9); assert.equal(b.dt, 0.01);
});

test('followRoute posts to /api/pedro/follow and returns a radian trajectory flagged pedro-core', async () => {
  const calls = [];
  const traj = await followRoute(route(), undefined, { fetchImpl: okFetch(calls) });
  assert.equal(calls[0].url, '/api/pedro/follow'); assert.equal(calls[0].body.start.x, 24);
  assert.equal(traj.length, 2); assert.equal(traj.source, 'pedro-core'); assert.equal(traj.approximate, false);
  assert.ok(Math.abs(traj[1].h - Math.PI / 2) < 1e-9); assert.ok(Math.abs(traj[1].w - Math.PI / 4) < 1e-9);
  assert.equal(traj.events[0].name, 'mid'); assert.equal(traj.finished, true); assert.equal(traj.duration, 1);
  assert.deepEqual(Object.keys(traj[0]).slice(0, 6), ['t', 'x', 'y', 'h', 'vx', 'vy']);
});

test('followRoute falls back to a clearly flagged JS approximation when the server fails', async () => {
  const bad = async () => ({ ok: false, status: 503, json: async () => ({ error: 'Pedro core jar or JDK not found' }) });
  const traj = await followRoute(route(), undefined, { fetchImpl: bad });
  assert.equal(traj.source, 'js-fallback'); assert.equal(traj.approximate, true);
  assert.match(traj.error, /JDK/); assert.ok(traj.length > 10);
  const end = traj.at(-1); assert.ok(Math.hypot(end.x - 72, end.y - 72) < 1);
  await assert.rejects(followRoute(route(), undefined, { fetchImpl: bad, fallback: false }), /JDK/);
  const thrown = async () => { throw new TypeError('network down'); };
  assert.equal((await followRoute(route(), undefined, { fetchImpl: thrown })).source, 'js-fallback');
});

test('followRoute rejects an empty route and a missing fetch falls back', async () => {
  await assert.rejects(followRoute(P.newRoute(), undefined, { fetchImpl: okFetch() }), /no segments/);
  const saved = globalThis.fetch; globalThis.fetch = undefined;
  try { assert.equal((await followRoute(route())).source, 'js-fallback'); } finally { globalThis.fetch = saved; }
});

test('sampleTrajectory interpolates and clamps, wrapping heading the short way', () => {
  const traj = responseToTrajectory({ ticks: [
    { t: 0, x: 0, y: 0, h: 170, vx: 0, vy: 0 }, { t: 1, x: 10, y: 0, h: -170, vx: 10, vy: 0 }] });
  const m = sampleTrajectory(traj, 0.5);
  assert.equal(m.x, 5); assert.ok(Math.abs(Math.abs(m.h) - Math.PI) < 1e-9);
  assert.equal(sampleTrajectory(traj, -1).x, 0); assert.equal(sampleTrajectory(traj, 9).x, 10);
  assert.equal(sampleTrajectory([], 1), null);
});

test('PedroLiveFollower start/step/stop use the live endpoints', async () => {
  const calls = [];
  const f = async (url, init) => {
    const body = JSON.parse(init.body); calls.push({ url, body });
    if (url.endsWith('/start')) return { ok: true, json: async () => ({ id: 's1', info: { dt: 0.005 }, tick: resp.ticks[0] }) };
    if (url.endsWith('/step')) return { ok: true, json: async () => ({ done: true, endReason: 'finished', t: 1, events: resp.events, ticks: resp.ticks }) };
    return { ok: true, json: async () => ({ ok: true }) };
  };
  const live = new PedroLiveFollower(f);
  const first = await live.start(route(), undefined, { dt: 0.005 });
  assert.equal(first.length, 1); assert.equal(live.id, 's1');
  const st = await live.step(50);
  assert.equal(st.length, 2); assert.equal(st.done, true); assert.equal(live.done, true);
  assert.deepEqual(calls[1].body, { id: 's1', n: 50 });
  await live.stop(); assert.equal(calls[2].url, '/api/pedro/live/stop'); assert.equal(live.id, null);
  await assert.rejects(live.step(1), /not started/);
});

test('fallbackTrajectory reaches the route end', () => {
  const t = fallbackTrajectory(route());
  assert.equal(t.approximate, true); assert.equal(t.finished, true);
});

// Integration against the real JVM simulation; skipped when the JDK / Pedro jar / venv is not available.
const here = path.dirname(fileURLToPath(import.meta.url)), root = path.join(here, '..');
const py = [path.join(root, '.venv', 'Scripts', 'python.exe'), path.join(root, '.venv', 'bin', 'python')].find(existsSync);
function realFollow(req) {
  const code = 'import sys,json,pedro_bridge as b; print(json.dumps(b.follow(json.loads(sys.stdin.read()))))';
  const r = spawnSync(py, ['-I', '-c', 'import sys; sys.path.insert(0, ' + JSON.stringify(root) + '); ' + code], { input: JSON.stringify(req), encoding: 'utf8', timeout: 120000 });
  return JSON.parse(r.stdout);
}
const probe = py ? spawnSync(py, ['-I', '-c', 'import sys; sys.path.insert(0, ' + JSON.stringify(root) + '); import pedro_bridge as b; print(b.follow_available())'], { encoding: 'utf8' }).stdout.trim() : '';
test('real Pedro Follower drives a line, curve, wait and callback to the end pose', { skip: probe !== 'True' && 'JDK / Pedro jar / venv not available' }, () => {
  const r = P.newRoute({ start: { x: 24, y: 24, h: 0 } });
  P.addLine(r, 72, 24); P.addCurve(r, 72, 72, 110, 40);
  r.segments[0].heading = { mode: 'constant', reversed: false, start: 0 };
  r.segments[1].heading = { mode: 'tangent', reversed: false };
  r.segments[0].callbacks = [{ t: 0.5, name: 'mid' }]; r.segments[0].waitAfter = 0.4;
  const resp2 = realFollow(followRequest(r, undefined, { maxTime: 30 }));
  assert.ok(!resp2.error, resp2.error);
  const traj = responseToTrajectory(resp2);
  assert.equal(traj.finished, true); assert.equal(traj.source, 'pedro-core'); assert.equal(traj.info.realtimeClock, false);
  const end = traj.at(-1);
  assert.ok(Math.hypot(end.x - 72, end.y - 72) < 2, 'ends at the route end: ' + JSON.stringify(end));
  assert.ok(Math.max(...traj.map(k => Math.hypot(k.vx, k.vy))) < 90, 'speed below the drivetrain limit');
  const kinds = traj.events.map(e => e.kind + ':' + e.name);
  assert.ok(kinds.includes('callback:mid')); assert.ok(kinds.some(k => k.startsWith('waitStart')) && kinds.some(k => k.startsWith('waitEnd')));
  const wStart = traj.events.find(e => e.kind === 'waitStart').t, wEnd = traj.events.find(e => e.kind === 'waitEnd').t;
  assert.ok(Math.abs(wEnd - wStart - 0.4) < 0.02);
  const held = traj.filter(k => k.t > wStart + 0.2 && k.t < wEnd); assert.ok(held.every(k => Math.hypot(k.vx, k.vy) < 5), 'holds while waiting');
});
