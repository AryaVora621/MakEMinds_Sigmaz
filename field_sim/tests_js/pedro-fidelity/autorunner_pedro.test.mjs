// AutoRunner followerModel 'pedro' with a mock host; the 'auto' (default) behaviour is unchanged.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AutoRunner } from '../../web/js/autorunner.js';
import { newAuto, addNode } from '../../web/js/autos.js';

const line = (a, b) => ({ type: 'line', points: [{ x: a[0], y: a[1] }, { x: b[0], y: b[1] }] });
function sampler(route) {
  const a = route.segments[0].points[0], b = route.segments.at(-1).points.at(-1);
  const L = Math.hypot(b.x - a.x, b.y - a.y), out = [];
  for (let s = 0; s <= L + 1e-9; s += 1) out.push({ x: a.x + (b.x - a.x) * s / L, y: a.y + (b.y - a.y) * s / L, h: 0.5, s });
  if (out.at(-1).s < L - 1e-6) out.push({ x: b.x, y: b.y, h: 0.5, s: L });
  return out;
}
function fakeHost() {
  const h = {
    pose: { x: 0, y: 0, h: 0 }, vel: [0, 0, 0], balls: 3, calls: [],
    getPose: () => h.pose, setPose: p => { h.pose = { ...p }; }, setVelocity: (a, b, c) => { h.vel = [a, b, c]; },
    intake: on => h.calls.push('intake:' + on), shoot: a => { h.calls.push('shoot'); if (a?.all) h.balls = 0; return true; },
    ballCount: () => h.balls, hopperFull: () => h.balls >= 3, hopperEmpty: () => h.balls === 0, flywheelReady: () => true, aimed: () => true,
    setAim: () => {}, setRpm: () => {}, flower: () => {}, gate: () => {}, park: () => h.calls.push('park'),
    snapshot: () => ({ pose: { ...h.pose }, balls: h.balls }), restore: s => { h.pose = { ...s.pose }; h.balls = s.balls; },
  };
  return h;
}
function twoPathAuto(settings) {
  const a = newAuto({ start: { x: 24, y: 24, h: 0 } });
  a.paths.p1 = { segments: [line([24, 24], [24, 84])], markers: [] };
  a.paths.p2 = { segments: [line([24, 84], [84, 84])], markers: [] };
  addNode(a, null, { type: 'follow', path: 'p1' });
  addNode(a, null, { type: 'follow', path: 'p2' });
  addNode(a, null, { type: 'command', name: 'park' });
  if (settings) a.settings = { ...(a.settings || {}), ...settings };
  return a;
}
const run = (r, max = 40) => { for (let i = 0; i < max * 60 && r.state === 'running'; i++) r.step(1 / 60); };
const mk = (auto, opts = {}) => { const host = fakeHost(), r = new AutoRunner({ host, pathSampler: sampler, ...opts }); r.load(auto); return { r, host }; };

test("followerModel 'pedro' completes both follows with a mock host and ends near the path end", () => {
  const { r, host } = mk(twoPathAuto(), { followerModel: 'pedro' });
  r.start(); run(r);
  assert.equal(r.state, 'done');
  assert.ok(Math.hypot(host.pose.x - 84, host.pose.y - 84) < 2, `end ${host.pose.x},${host.pose.y}`);
  assert.ok(host.calls.includes('park'));
  assert.ok(r.time > 2.5 && r.time < 15, `time ${r.time}`);
});

test("auto.settings.followerModel = 'pedro' overrides the constructor default", () => {
  const { r, host } = mk(twoPathAuto({ followerModel: 'pedro' }));
  r.start(); run(r);
  assert.equal(r.state, 'done');
  assert.ok(Math.hypot(host.pose.x - 84, host.pose.y - 84) < 2);
  // the closed loop is not an exact sampler replay: it does not land on (84, 84) bit-exactly
  assert.notDeepEqual([host.pose.x, host.pose.y], [84, 84]);
});

test("default ('auto') behaviour is unchanged: exact sampler replay in the same time as before", () => {
  const { r, host } = mk(twoPathAuto());
  r.start(); run(r);
  assert.equal(r.state, 'done');
  assert.deepEqual([host.pose.x, host.pose.y], [84, 84]);
  assert.ok(r.time > 3.2 && r.time < 3.8, `time ${r.time}`);   // two 60 in paths, trapezoid sampler
});

test("'pedro': a host push (collision) is respected and fed back into the follower", () => {
  const { r, host } = mk(twoPathAuto(), { followerModel: 'pedro' });
  r.start(); r.step(0.5);
  const before = { ...host.pose };
  assert.ok(before.y > 24, 'robot moved');
  host.pose = { ...host.pose, x: host.pose.x + 8 };   // pushed 8 in sideways
  r.step(1 / 60);
  assert.ok(host.pose.x - before.x > 5, `push kept: x ${before.x} -> ${host.pose.x}`);
  run(r);
  assert.equal(r.state, 'done');
  assert.ok(Math.hypot(host.pose.x - 84, host.pose.y - 84) < 2.5, `recovered to ${host.pose.x},${host.pose.y}`);
});

test("'pedro': stop() zeroes velocity and reset()/start() run again", () => {
  const { r, host } = mk(twoPathAuto(), { followerModel: 'pedro' });
  r.start(); r.step(0.5); assert.ok(Math.hypot(host.vel[0], host.vel[1]) > 1);
  r.stop(); assert.deepEqual(host.vel, [0, 0, 0]);
  r.reset(); r.start(); run(r);
  assert.equal(r.state, 'done');
});
