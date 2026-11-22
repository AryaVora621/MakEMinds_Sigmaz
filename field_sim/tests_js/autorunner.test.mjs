import test from 'node:test';
import assert from 'node:assert/strict';
import { AutoRunner, buildTimeline } from '../web/js/autorunner.js';
import { newAuto, addNode, addMarker, ensureIds } from '../web/js/autos.js';

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
    pose: { x: 0, y: 0, h: 0 }, vel: [0, 0, 0], balls: 3, calls: [], snaps: 0,
    getPose: () => h.pose, setPose: p => { h.pose = { ...p }; }, setVelocity: (a, b, c) => { h.vel = [a, b, c]; },
    intake: on => h.calls.push('intake:' + on), shoot: a => { h.calls.push('shoot:' + JSON.stringify(a)); if (a?.all) h.balls = 0; return true; },
    ballCount: () => h.balls, hopperFull: () => h.balls >= 3, hopperEmpty: () => h.balls === 0, flywheelReady: () => true, aimed: () => true,
    setAim: () => {}, setRpm: () => {}, flower: () => {}, gate: () => {}, park: () => h.calls.push('park'),
    snapshot: () => { h.snaps++; return { pose: { ...h.pose }, balls: h.balls }; }, restore: s => { h.pose = { ...s.pose }; h.balls = s.balls; },
  };
  return h;
}
function fullAuto() {
  const a = newAuto({ start: { x: 24, y: 24, h: 0 } });
  a.paths.p1 = { segments: [line([24, 24], [24, 84])], markers: [] };
  a.paths.p2 = { segments: [line([24, 84], [84, 84])], markers: [] };
  addNode(a, null, { type: 'command', name: 'intakeOn' });
  addNode(a, null, { type: 'follow', path: 'p1' });
  addNode(a, null, { type: 'follow', path: 'p2' });
  addNode(a, null, { type: 'command', name: 'shootAll' });
  addNode(a, null, { type: 'waitUntil', condition: 'ballCount<=0', timeout: 2 });
  addNode(a, null, { type: 'command', name: 'park' });
  addMarker(a, 'p2', 0.5, { type: 'command', name: 'intakeOff' });
  return a;
}
const run = (r, max = 30) => { for (let i = 0; i < max * 60 && r.state === 'running'; i++) r.step(1 / 60); };
const mk = (auto, host = fakeHost()) => { const r = new AutoRunner({ host, pathSampler: sampler }); r.load(auto); return { r, host }; };

test('full auto runs to done in expected time with expected side effects', () => {
  const { r, host } = mk(fullAuto());
  r.start(); assert.equal(r.state, 'running'); run(r);
  assert.equal(r.state, 'done');
  // two 60in paths (~1.67s each) + near-instant shootAll/waitUntil/park
  assert.ok(r.time > 3.2 && r.time < 3.8, `time ${r.time}`);
  assert.deepEqual(host.calls.filter(c => !c.startsWith('shoot')), ['intake:true', 'intake:false', 'park']);
  assert.ok(host.calls.some(c => c.includes('"all":true')));
  assert.deepEqual([host.pose.x, host.pose.y], [84, 84]);
  assert.deepEqual(host.vel, [0, 0, 0]);
});
test('pose follows path samples with heading', () => {
  const { r, host } = mk(fullAuto()); r.start();
  r.step(0.5); assert.ok(host.pose.y > 24 && host.pose.y < 84); assert.ok(Math.abs(host.pose.x - 24) < 1e-9);
  assert.ok(Math.abs(host.pose.h - 0.5) < 1e-9); assert.ok(host.vel[1] > 0);
});
test('marker fires mid second path', () => {
  const { r, host } = mk(fullAuto()); r.start();
  let t1 = -1; for (let i = 0; i < 600 && r.state === 'running'; i++) { r.step(1 / 60); if (t1 < 0 && host.calls.includes('intake:false')) t1 = r.time; }
  assert.ok(t1 > 1.67 + 0.6 && t1 < 1.67 + 1.1, `marker at ${t1}`);
});
test('state transitions idle->running->paused->running->done, stop->idle', () => {
  const { r } = mk(fullAuto()); assert.equal(r.state, 'idle');
  r.start(); assert.equal(r.state, 'running'); r.pause(); assert.equal(r.state, 'paused');
  const t = r.time; r.step(1); assert.equal(r.time, t, 'no progress while paused');
  r.resume(); assert.equal(r.state, 'running'); run(r); assert.equal(r.state, 'done');
  r.start(); r.stop(); assert.equal(r.state, 'idle');
  r.reset(); assert.equal(r.time, 0); assert.equal(r.trace.length, 0);
});
test('stop interrupts follow and zeroes velocity', () => {
  const { r, host } = mk(fullAuto()); r.start(); r.step(0.5); assert.ok(host.vel[1] > 0);
  r.stop(); assert.deepEqual(host.vel, [0, 0, 0]);
  assert.ok(r.trace.some(e => e.event === 'interrupt'));
  const p = { ...host.pose }; r.step(1); assert.deepEqual(host.pose, p);
});
test('seek is deterministic and restores host state', () => {
  const { r, host } = mk(fullAuto()); r.start(); run(r); assert.equal(r.state, 'done');
  r.seek(0.5); const a = { ...host.pose }; assert.equal(r.state, 'paused'); assert.ok(Math.abs(r.time - 0.5) < 1e-6);
  r.seek(2.5); r.seek(0.5); assert.deepEqual(host.pose, a);
  assert.equal(host.balls, 3, 'balls restored by snapshot');
  r.seek(100); assert.equal(r.state, 'done');
});
test('seek works without snapshot support (re-run)', () => {
  const host = fakeHost(); delete host.snapshot; delete host.restore;
  const { r } = mk(fullAuto(), host); r.seek(1); assert.equal(r.state, 'paused'); assert.ok(Math.abs(r.time - 1) < 1e-6);
});
test('unknown path is contained as an error state', () => {
  const a = newAuto(); addNode(a, null, { type: 'follow', path: 'nope' }); addNode(a, null, { type: 'command', name: 'park' });
  a.paths = {}; const { r, host } = mk(a); r.start();
  assert.equal(r.state, 'error'); assert.ok(r.errors.some(e => /nope/.test(e)));
  assert.ok(!host.calls.includes('park'), 'auto aborted after the error');
});
test('unknown command is an error, sampler failure recorded', () => {
  const a = newAuto(); addNode(a, null, { type: 'command', name: 'bogus' });
  const { r } = mk(a); r.start(); assert.equal(r.state, 'error');
  const b = fullAuto(); const r2 = new AutoRunner({ host: fakeHost(), pathSampler: () => { throw new Error('boom'); } }); r2.load(b);
  assert.ok(r2.errors.some(e => /boom/.test(e))); r2.start(); assert.equal(r2.state, 'error');
});
test('speed multiplier scales time', () => {
  const t = sp => { const { r } = mk(fullAuto()); r.speed = sp; r.start(); let n = 0; while (r.state === 'running' && n++ < 5000) r.step(1 / 60); return n; };
  const n1 = t(1), n2 = t(2), nq = t(0.5);
  assert.ok(Math.abs(n1 / n2 - 2) < 0.2, `${n1}/${n2}`); assert.ok(Math.abs(nq / n1 - 2) < 0.2);
});
test('async path sampler requires prepare()', async () => {
  const r = new AutoRunner({ host: fakeHost(), pathSampler: async rt => sampler(rt) }); r.load(fullAuto());
  r.start(); assert.equal(r.state, 'error');
  await r.prepare(); r.start(); assert.equal(r.state, 'running'); run(r); assert.equal(r.state, 'done');
});
test('timeElapsed uses the runner clock; parallel + race semantic', () => {
  const a = newAuto(); addNode(a, null, { type: 'waitUntil', condition: 'timeElapsed:1.5' });
  addNode(a, null, { type: 'command', name: 'park' });
  const { r, host } = mk(a); r.start(); run(r); assert.ok(r.time >= 1.5 && r.time < 1.6); assert.ok(host.calls.includes('park'));
});
test('activeNodes reflects the running node', () => {
  const a = fullAuto(); const { r } = mk(a); r.start(); r.step(0.5);
  const f1 = a.root.children[1].id; assert.ok(r.activeNodes.has(f1)); run(r); assert.equal(r.activeNodes.size, 0);
});
test('buildTimeline: sequential bars are consecutive and total matches', () => {
  const a = fullAuto(); const bars = buildTimeline(a, { pathTime: () => 2 });
  const leaf = bars.filter(b => b.kind !== 'group');
  const f = leaf.filter(b => b.kind === 'follow');
  assert.equal(f.length, 2); assert.ok(Math.abs(f[0].end - f[0].start - 2) < 0.03); assert.ok(Math.abs(f[1].start - f[0].end) < 0.03);
  assert.ok(bars.total > 7.1 && bars.total < 7.3, `total ${bars.total}`);
  assert.equal(bars.errors.length, 0);
  assert.ok(bars.some(b => b.kind === 'group' && b.group === 'sequential' && b.depth === 0));
  assert.ok(leaf.some(b => b.label.startsWith('until')));
});
test('buildTimeline: parallel group uses separate lanes; race ends at the shortest', () => {
  const a = newAuto(); a.paths.p1 = { segments: [line([24, 24], [24, 84])], markers: [] };
  addNode(a, null, { type: 'parallel', children: [{ type: 'follow', path: 'p1' }, { type: 'wait', seconds: 1 }, { type: 'wait', seconds: 3 }] });
  addNode(a, null, { type: 'race', children: [{ type: 'wait', seconds: 1 }, { type: 'wait', seconds: 5 }] });
  ensureIds(a);
  const bars = buildTimeline(a, { pathTime: () => 2 });
  const par = bars.filter(b => b.kind !== 'group' && b.start < 0.01);
  assert.equal(new Set(par.map(b => b.lane)).size, 3, 'three simultaneous leaves, three lanes');
  const raceBars = bars.filter(b => b.kind === 'wait' && b.start >= 2.9);
  assert.equal(raceBars.length, 2); assert.ok(raceBars.every(b => b.end - b.start < 1.1));
  assert.ok(Math.abs(bars.total - 4) < 0.05);
});
test('buildTimeline: waitUntil without timeout assumes 1s; error for unknown path', () => {
  const a = newAuto(); addNode(a, null, { type: 'waitUntil', condition: 'hopperFull' });
  const b = buildTimeline(a); assert.ok(Math.abs(b.total - 1) < 0.03);
  const c = newAuto(); addNode(c, null, { type: 'follow', path: 'zzz' });
  assert.ok(buildTimeline(c).errors.length > 0);
});

test('trajectorySampler: robot replays the time-parameterised trajectory; markers use its progress', () => {
  const a = newAuto({ start: { x: 0, y: 0, h: 0 } });
  a.paths.p = { segments: [line([0, 0], [100, 0])], markers: [] };
  addNode(a, null, { type: 'follow', path: 'p' }); addMarker(a, 'p', 0.5, { type: 'command', name: 'intakeOn' });
  // a fake follower: slow first half (10 in/s for 5 s), fast second half (50 in/s for 1 s)
  const traj = []; for (let i = 0; i <= 50; i++) traj.push({ t: i * 0.1, x: i, y: 0, h: 0 }); for (let i = 1; i <= 10; i++) traj.push({ t: 5 + i * 0.1, x: 50 + i * 5, y: 0, h: 0 });
  const host = fakeHost(), r = new AutoRunner({ host, pathSampler: sampler, trajectorySampler: () => traj }); r.load(a);
  r.start(); let tm = -1; for (let i = 0; i < 900 && r.state === 'running'; i++) { r.step(1 / 60); if (tm < 0 && host.calls.includes('intake:true')) tm = r.time; }
  assert.equal(r.state, 'done'); assert.ok(Math.abs(r.time - 6) < 0.05, 'ends at trajectory duration ' + r.time);
  assert.ok(tm > 4.9 && tm < 5.2, 'half-way marker at ' + tm);
  const bad = new AutoRunner({ host: fakeHost(), pathSampler: sampler, trajectorySampler: async () => { throw new Error('x'); } }); bad.load(a);
  return bad.prepare().then(() => { bad.start(); run(bad); assert.equal(bad.state, 'done'); assert.ok(bad.time < 5, 'falls back to trapezoid'); });
});

test('run lifecycle for the 3D host: snapshot (field reset) before the start pose, restore on stop/reset, re-run is identical', async () => {
  const order = [], host = fakeHost();
  const snap = host.snapshot, setPose = host.setPose;
  host.snapshot = () => { order.push('snapshot'); return snap(); };
  host.setPose = p => { order.push('setPose'); setPose(p); };
  host.restore = s => { order.push('restore'); host.pose = { ...s.pose }; host.balls = s.balls; };
  const { r } = mk(fullAuto(), host);
  await r.prepare(); r.start();
  assert.deepEqual(order.slice(0, 2), ['snapshot', 'setPose']);
  assert.ok(Math.hypot(host.pose.x - 24, host.pose.y - 24) < 1e-6);
  run(r); assert.equal(r.state, 'done'); const t1 = r.time;
  r.reset(); assert.equal(r.state, 'idle'); assert.ok(order.includes('restore'));
  r.start(); run(r); assert.equal(r.state, 'done'); assert.ok(Math.abs(r.time - t1) < 1e-6, 'restart reproduces the run');
  r.start(); r.step(0.5); r.stop(); assert.equal(r.state, 'idle'); assert.deepEqual(host.vel, [0, 0, 0]);
});

test('reset() calls the host resetHives hook after restoring; a missing or throwing hook is ignored', async () => {
  const order = [], host = fakeHost();
  host.restore = s => { order.push('restore'); host.pose = { ...s.pose }; host.balls = s.balls; };
  host.resetHives = () => { order.push('resetHives'); };
  const { r } = mk(fullAuto(), host);
  await r.prepare(); r.start(); run(r); assert.equal(r.state, 'done');
  order.length = 0;
  r.reset(); assert.equal(r.state, 'idle');
  assert.deepEqual(order, ['restore', 'resetHives'], 'hook runs once, after restore');
  // a stop that goes through stop() only (no reset) does not touch the hives
  r.start(); r.step(0.2); order.length = 0; r.stop(); assert.deepEqual(order, []);
  // a host without the hook, and a hook that throws, must not break reset()
  const bare = fakeHost(); delete bare.resetHives;
  const { r: r2 } = mk(fullAuto(), bare); await r2.prepare(); r2.start(); assert.doesNotThrow(() => r2.reset()); assert.equal(r2.state, 'idle');
  const boom = fakeHost(); boom.resetHives = () => { throw new Error('boom'); };
  const { r: r3 } = mk(fullAuto(), boom); await r3.prepare(); r3.start(); assert.doesNotThrow(() => r3.reset()); assert.equal(r3.state, 'idle');
});

test('a real-follower trajectory is replayed by time and pause holds the robot', async () => {
  const a = newAuto({ start: { x: 0, y: 0, h: 0 } });
  a.paths.p = { segments: [line([0, 0], [10, 0])], markers: [] }; addNode(a, null, { type: 'follow', path: 'p' });
  const host = fakeHost();
  const traj = [{ t: 0, x: 0, y: 0, h: 0 }, { t: 2, x: 10, y: 0, h: 0 }];
  const r = new AutoRunner({ host, pathSampler: sampler, trajectorySampler: () => traj }); r.load(a); await r.prepare(); r.start();
  r.step(1); assert.ok(Math.abs(host.pose.x - 5) < 0.2);
  r.pause(); r.step(1); assert.ok(Math.abs(host.pose.x - 5) < 0.2, 'paused: no motion');
  r.resume(); run(r); assert.equal(r.state, 'done'); assert.ok(Math.abs(host.pose.x - 10) < 1e-6);
});

// ---- trajectory validation (integrator, from Haiku audit t09) ----
const trajRunner = tr => { const a = fullAuto(); const r = new AutoRunner({ host: fakeHost(), pathSampler: sampler, trajectorySampler: () => tr }); r.load(a); return r; };
test('js-fallback (approximate) trajectory is not replayed as Pedro motion', () => {
  const tr = [{ t: 0, x: 0, y: 0, h: 0 }, { t: 1, x: 10, y: 0, h: 0 }]; tr.source = 'js-fallback'; tr.approximate = true;
  const r = trajRunner({ traj: tr, source: 'pedro-follower' }); assert.deepEqual(Object.keys(r.trajs), []);
});
test('unsorted trajectory times are sorted and T is the maximum', () => {
  const r = trajRunner([{ t: 0, x: 0, y: 0, h: 0 }, { t: 1, x: 5, y: 0, h: 0 }, { t: 0.5, x: 3, y: 0, h: 0 }]);
  const t = Object.values(r.trajs)[0]; assert.equal(t.T, 1); assert.deepEqual(t.tr.map(p => p.t), [0, 0.5, 1]);
});
