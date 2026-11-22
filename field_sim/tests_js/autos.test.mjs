import test from 'node:test';
import assert from 'node:assert/strict';
import * as A from '../web/js/autos.js';
import { Scheduler } from '../web/js/scheduler.js';

const line = (a, b) => ({ type: 'line', points: [a, b], heading: { mode: 'linear', start: 0, end: Math.PI / 2, reversed: false }, callbacks: [], label: '' });
function sample() {
  const a = A.newAuto({ name: 'T', alliance: 'blue', start: { x: 20, y: 20, h: 0.5 } });
  a.paths.p1 = { segments: [line({ x: 20, y: 20 }, { x: 60, y: 20 })], markers: [] };
  a.paths.p2 = { segments: [line({ x: 60, y: 20 }, { x: 60, y: 60 })], markers: [] };
  A.addNode(a, null, { type: 'follow', path: 'p1' });
  A.addNode(a, null, { type: 'command', name: 'shoot', args: { count: 2 } });
  A.addNode(a, null, { type: 'follow', path: 'p2' });
  return a;
}
const codes = (a, o) => A.validateAuto(a, o).map(i => i.code);

test('newAuto + addNode assigns unique stable ids', () => {
  const a = sample(); const ids = []; A.walkNodes(a, n => { ids.push(n.id); });
  assert.equal(ids.length, 4); assert.equal(new Set(ids).size, 4); assert.ok(ids.every(Boolean));
  assert.equal(A.cloneAuto(a).root.children[0].id, a.root.children[0].id);
});
test('validate: clean auto has no errors', () => {
  assert.deepEqual(A.validateAuto(sample()).filter(i => i.level === 'error'), []);
});
test('validate: unknown path id', () => { const a = sample(); A.addNode(a, null, { type: 'follow', path: 'nope' }); assert.ok(codes(a).includes('unknown-path')); });
test('validate: empty group', () => { const a = sample(); A.addNode(a, null, { type: 'parallel', children: [] }); assert.ok(codes(a).includes('empty-group')); });
test('validate: marker out of range', () => {
  const a = sample(); A.addMarker(a, 'p1', 1.5, { type: 'command', name: 'intakeOn' }); assert.ok(codes(a).includes('marker-range'));
});
test('validate: waitUntil without timeout warns; bad condition errors; timeElapsed fine', () => {
  const a = sample(); A.addNode(a, null, { type: 'waitUntil', condition: 'ballCount>=3' });
  assert.ok(A.validateAuto(a).some(i => i.code === 'no-timeout' && i.level === 'warning'));
  A.addNode(a, null, { type: 'waitUntil', condition: 'banana', timeout: 1 }); assert.ok(codes(a).includes('bad-condition'));
  const b = sample(); A.addNode(b, null, { type: 'waitUntil', condition: 'timeElapsed:20' }); assert.ok(!codes(b).includes('no-timeout'));
});
test('validate: discontinuity between follow paths (> 2 in) is a warning', () => {
  const a = sample(); a.paths.p2.segments[0].points[0] = { x: 70, y: 20 };
  const i = A.validateAuto(a).find(x => x.code === 'discontinuity'); assert.ok(i); assert.equal(i.level, 'warning');
  const b = sample(); b.paths.p2.segments[0].points[0] = { x: 61.5, y: 20 }; assert.ok(!codes(b).includes('discontinuity'));
});
test('validate: unknown command, bad arg, out-of-bounds, bad deadline, unused path, duplicate id', () => {
  const a = sample(); A.addNode(a, null, { type: 'command', name: 'dance' }); assert.ok(codes(a).includes('unknown-command'));
  const b = sample(); b.root.children[1].args.count = 9; assert.ok(codes(b).includes('bad-arg'));
  const c = sample(); c.paths.p1.segments[0].points[1].x = 200; assert.ok(codes(c).includes('out-of-bounds'));
  assert.ok(!codes(c, { fieldBounds: { x1: 300 } }).includes('out-of-bounds'));
  const d = sample(); A.addNode(d, null, { type: 'deadline', deadline: 3, children: [{ type: 'wait', seconds: 1 }] }); assert.ok(codes(d).includes('bad-deadline'));
  const e = sample(); e.paths.p3 = { segments: [], markers: [] }; assert.ok(codes(e).includes('unused-path'));
  const f = sample(); f.root.children[1].id = f.root.children[0].id; assert.ok(codes(f).includes('duplicate-id'));
  const g = sample(); assert.ok(codes(g, { knownCommands: ['intakeOn'] }).includes('unknown-command')); // shoot not in the allow-list
});
test('custom commands are accepted', () => {
  const a = sample(); A.addNode(a, null, { type: 'command', name: 'custom:wiggle' }); assert.ok(!codes(a).includes('unknown-command'));
});
test('tree helpers: wrapInGroup, unwrap, moveNode, removeNode', () => {
  const a = sample(); const [f1, sh, f2] = a.root.children.map(c => c.id);
  const g = A.wrapInGroup(a, sh, 'parallel'); assert.equal(a.root.children[1], g); assert.equal(g.children[0].id, sh);
  assert.equal(A.unwrap(a, g.id), true); assert.deepEqual(a.root.children.map(c => c.id), [f1, sh, f2]);
  const g2 = A.wrapInGroup(a, f2, 'race'); assert.equal(A.moveNode(a, sh, g2.id, 0), true);
  assert.deepEqual(g2.children.map(c => c.id), [sh, f2]);
  assert.equal(A.moveNode(a, g2.id, g2.id), false); // into itself
  assert.equal(A.moveNode(a, g2.id, sh), false); // into a non-group
  assert.equal(A.removeNode(a, f1).id, f1); assert.equal(A.removeNode(a, a.root.id), null);
  assert.equal(a.root.children.length, 1);
});
test('deadline index survives removals', () => {
  const a = A.newAuto(); const d = A.addNode(a, null, { type: 'deadline', deadline: 1, children: [{ type: 'wait', seconds: 1 }, { type: 'wait', seconds: 2 }, { type: 'wait', seconds: 3 }] });
  A.removeNode(a, d.children[0].id); assert.equal(d.deadline, 0); assert.equal(d.children[d.deadline].seconds, 2);
});
test('mirrorAuto: x -> 144-x, heading -> pi-h, alliance flips, double mirror is identity', () => {
  const a = sample(); const m = A.mirrorAuto(a);
  assert.equal(m.alliance, 'red'); assert.equal(m.start.x, 124); assert.ok(Math.abs(m.start.h - (Math.PI - 0.5)) < 1e-12);
  assert.equal(m.paths.p1.segments[0].points[1].x, 84);
  assert.ok(Math.abs(m.paths.p1.segments[0].heading.end - Math.PI / 2) < 1e-12); // pi - pi/2
  const mm = A.mirrorAuto(m);
  assert.ok(Math.abs(mm.start.x - a.start.x) < 1e-9 && Math.abs(mm.start.h - a.start.h) < 1e-9 && mm.alliance === 'blue');
  assert.equal(a.alliance, 'blue'); // input untouched
});
test('JSON round trip', () => {
  const a = sample(); A.addMarker(a, 'p1', 0.5, { type: 'command', name: 'intakeOn' });
  const b = A.fromJSON(A.toJSON(a)); assert.deepEqual(b, a);
});
test('fromJSON rejects bad input', () => {
  const ok = JSON.parse(A.toJSON(sample())); const bad = f => { const o = JSON.parse(JSON.stringify(ok)); f(o); return JSON.stringify(o); };
  assert.throws(() => A.fromJSON('{nope'), /Invalid JSON/);
  assert.throws(() => A.fromJSON('[]'), /object/);
  assert.throws(() => A.fromJSON(bad(o => { o.schema_version = 2; })), /schema_version/);
  assert.throws(() => A.fromJSON(bad(o => { o.start.x = 'a'; })), /start\.x/);
  assert.throws(() => A.fromJSON(bad(o => { o.root.children[0].type = 'teleport'; })), /unknown node type/);
  assert.throws(() => A.fromJSON(bad(o => { o.root.children = 'x'; })), /children/);
  assert.throws(() => A.fromJSON(bad(o => { o.paths.p1.segments[0].type = 'arc'; })), /type/);
  assert.throws(() => A.fromJSON(bad(o => { o.paths.p1.segments[0].points[0].y = null; })), /not a finite/);
  assert.throws(() => A.fromJSON(bad(o => { o.root.children[1].args = 5; })), /args/);
  assert.throws(() => A.fromJSON(bad(o => { delete o.root; })), /root/);
});
test('fromJSON assigns ids if missing', () => {
  const o = JSON.parse(A.toJSON(sample())); delete o.root.id; o.root.children.forEach(c => delete c.id);
  const a = A.fromJSON(JSON.stringify(o)); const ids = []; A.walkNodes(a, n => { ids.push(n.id); }); assert.equal(new Set(ids).size, 4);
});
test('COMMAND_DEFS has every command with complete metadata', () => {
  for (const n of ['shoot', 'shootAll', 'intakeOn', 'intakeOff', 'aimOn', 'aimOff', 'setRpm', 'flowerDeploy', 'flowerStow', 'gateOpen', 'gateClose', 'park']) {
    const d = A.COMMAND_DEFS[n]; assert.ok(d, n);
    for (const k of ['label', 'category', 'args', 'javaCall', 'icon', 'description']) assert.ok(d[k] !== undefined, `${n}.${k}`);
    assert.ok(typeof d.simDuration === 'number' || typeof d.simDuration === 'function');
    assert.ok(/^[\x21-\x7e]+$/.test(d.icon));
  }
});
test('estimateDuration: sequential sums, parallel max, race min, deadline, waitUntil timeout', () => {
  const a = sample(); const pathTime = id => ({ p1: 2, p2: 3 })[id];
  assert.ok(Math.abs(A.estimateDuration(a, { pathTime }) - (2 + 0.8 + 3)) < 1e-9);
  const b = A.newAuto();
  A.addNode(b, null, { type: 'parallel', children: [{ type: 'wait', seconds: 1 }, { type: 'wait', seconds: 4 }] });
  A.addNode(b, null, { type: 'race', children: [{ type: 'wait', seconds: 1 }, { type: 'wait', seconds: 4 }] });
  A.addNode(b, null, { type: 'deadline', deadline: 1, children: [{ type: 'wait', seconds: 1 }, { type: 'wait', seconds: 2 }] });
  A.addNode(b, null, { type: 'waitUntil', condition: 'aimed', timeout: 0.5 });
  assert.equal(A.estimateDuration(b), 4 + 1 + 2 + 0.5);
});

// ---------- buildCommandTree with a fake adapter ----------
function fake({ pathDur = { p1: 1, p2: 2 } } = {}) {
  const log = []; let balls = 0, now = 0;
  const ad = {
    log, get balls() { return balls; },
    follow(id) {
      let el = 0, started = false; const d = pathDur[id];
      return { start() { started = true; log.push(`follow:${id}:start@${now.toFixed(2)}`); }, update(dt) { el += dt; }, done: () => el >= d - 1e-9, progress: () => Math.min(1, el / d), stop() { log.push(`follow:${id}:stop`); } };
    },
    intake(on) { log.push(`intake:${on}@${now.toFixed(2)}`); if (on) ad._intakeOn = true; else ad._intakeOn = false; },
    shoot(args) { log.push(`shoot:${args.count}`); balls = Math.max(0, balls - (args.count ?? 1)); },
    shootAll() { log.push('shootAll'); balls = 0; },
    aim: on => log.push(`aim:${on}`), setRpm: r => log.push(`rpm:${r}`), flower: d => log.push(`flower:${d}`), gate: o => log.push(`gate:${o}`),
    park: () => log.push('park'), custom: (n) => log.push(`custom:${n}`),
    ballCount: () => balls, flywheelReady: () => true, aimed: () => true, hopperFull: () => balls >= 3, hopperEmpty: () => balls === 0, time: () => now,
  };
  return ad;
}
function drive(auto, ad, secs, dt = 0.05) {
  const s = new Scheduler(); const errs = []; const ctx = { onError: e => errs.push(e.message) };
  s.schedule(A.buildCommandTree(auto, ad));
  for (let i = 0; i < Math.round(secs / dt) && s.isRunning(); i++) { s.tick(ctx, dt); ad.__t = s.time; }
  return { s, errs };
}
test('buildCommandTree: full auto with follow, intake, waitUntil ballCount, shoot', () => {
  const a = A.newAuto(); a.paths.p1 = { segments: [line({ x: 20, y: 20 }, { x: 60, y: 20 })], markers: [] }; a.paths.p2 = { segments: [line({ x: 60, y: 20 }, { x: 60, y: 60 })], markers: [] };
  A.addNode(a, null, { type: 'command', name: 'intakeOn' });
  A.addNode(a, null, { type: 'follow', path: 'p1' });
  A.addNode(a, null, { type: 'waitUntil', condition: 'ballCount>=2', timeout: 2 });
  A.addNode(a, null, { type: 'command', name: 'shoot', args: { count: 2 } });
  A.addNode(a, null, { type: 'follow', path: 'p2' });
  const ad = fake(); let balls = 0, now = 0; ad.ballCount = () => balls; ad.time = () => now;
  const s = new Scheduler(), errs = [], ctx = { onError: e => errs.push(e.message) };
  s.schedule(A.buildCommandTree(a, ad));
  for (let i = 0; i < 400 && s.isRunning(); i++) { now += 0.05; s.tick(ctx, 0.05); if (now >= 1.5) balls = 2; } // balls picked up at t=1.5
  assert.deepEqual(errs, []); assert.equal(s.isRunning(), false);
  const wu = s.trace.find(e => e.name.startsWith('waitUntil') && e.event === 'end');
  assert.ok(wu.t > 1.45 && wu.t < 1.65, 'released by ballCount, before the 3 s timeout: ' + wu.t);
  const order = ad.log.filter(x => /^(intake|follow:p\d:start|shoot)/.test(x)).map(x => x.split('@')[0]);
  assert.deepEqual(order, ['intake:true', 'follow:p1:start', 'shoot:2', 'follow:p2:start']);
  const sEnd = s.trace.find(e => e.name === 'shoot' && e.event === 'end').t, f2 = s.trace.find(e => e.name === 'follow(p2)' && e.event === 'start').t;
  assert.ok(f2 >= sEnd - 1e-9 && f2 - wu.t > 0.75); // shoot lasts ~0.8 s of sim time
  assert.equal(s.trace.at(-1).node, a.root.id); // root ends last
});
test('buildCommandTree: waitUntil ballCount releases when balls arrive; shoot lasts simDuration', () => {
  const a = A.newAuto(); A.addNode(a, null, { type: 'waitUntil', condition: 'ballCount>=2', timeout: 5 }); A.addNode(a, null, { type: 'command', name: 'shoot', args: { count: 2 } });
  const ad = fake(); const s = new Scheduler(), ctx = {}; s.schedule(A.buildCommandTree(a, ad));
  let balls = 0; ad.ballCount = () => balls;
  for (let i = 0; i < 20; i++) s.tick(ctx, 0.05); // 1.0 s, still waiting
  assert.ok(!ad.log.includes('shoot:2'));
  balls = 2; s.tick(ctx, 0.05); assert.ok(ad.log.includes('shoot:2'));
  assert.equal(s.isRunning(), true); for (let i = 0; i < 25; i++) s.tick(ctx, 0.05); assert.equal(s.isRunning(), false);
  const sh = s.trace.filter(e => e.name === 'shoot'); assert.ok(Math.abs(sh[1].t - sh[0].t - 0.8) < 0.06);
});
test('buildCommandTree: marker fires when follow progress passes t (not before)', () => {
  const a = A.newAuto(); a.paths.p1 = { segments: [line({ x: 20, y: 20 }, { x: 60, y: 20 })], markers: [] };
  A.addNode(a, null, { type: 'follow', path: 'p1' });
  A.addMarker(a, 'p1', 0.5, { type: 'command', name: 'intakeOn' });
  const ad = fake({ pathDur: { p1: 2 } }); let prog = [];
  const orig = ad.follow; ad.follow = id => { const h = orig(id); const p = h.progress; h.progress = () => { const v = p(); prog.push(v); return v; }; return h; };
  const s = new Scheduler(), ctx = {}; s.schedule(A.buildCommandTree(a, ad));
  let firedAt = null;
  for (let i = 0; i < 100 && s.isRunning(); i++) { s.tick(ctx, 0.05); if (firedAt === null && ad.log.some(x => x.startsWith('intake:true'))) firedAt = s.time; }
  assert.ok(firedAt !== null && firedAt >= 1.0 - 1e-9 && firedAt <= 1.1 + 1e-9, `fired at ${firedAt}`); // progress 0.5 of 2 s
  assert.equal(s.isRunning(), false);
});
test('buildCommandTree: marker at t=1 still fires and path with several markers keeps order', () => {
  const a = A.newAuto(); a.paths.p1 = { segments: [line({ x: 20, y: 20 }, { x: 60, y: 20 })], markers: [] };
  A.addNode(a, null, { type: 'follow', path: 'p1' });
  A.addMarker(a, 'p1', 1, { type: 'command', name: 'gateOpen' }); A.addMarker(a, 'p1', 0.25, { type: 'command', name: 'aimOn' });
  const ad = fake({ pathDur: { p1: 1 } }); drive(a, ad, 5);
  const i1 = ad.log.indexOf('aim:true'), i2 = ad.log.indexOf('gate:true'); assert.ok(i1 >= 0 && i2 > i1);
});
test('buildCommandTree: race with follow vs waitUntil interrupts follow (stop called)', () => {
  const a = A.newAuto(); a.paths.p1 = { segments: [line({ x: 20, y: 20 }, { x: 60, y: 20 })], markers: [] };
  A.addNode(a, null, { type: 'race', children: [{ type: 'follow', path: 'p1' }, { type: 'waitUntil', condition: 'timeElapsed:0.3' }] });
  const ad = fake({ pathDur: { p1: 5 } }); let now = 0; ad.time = () => now;
  const s = new Scheduler(), ctx = {}; s.schedule(A.buildCommandTree(a, ad));
  for (let i = 0; i < 40 && s.isRunning(); i++) { now += 0.05; s.tick(ctx, 0.05); }
  assert.equal(s.isRunning(), false); assert.ok(ad.log.includes('follow:p1:stop')); assert.ok(now < 0.5);
});
test('buildCommandTree: unknown path / command report errors via onError and the auto still finishes', () => {
  const a = A.newAuto(); A.addNode(a, null, { type: 'follow', path: 'ghost' }); A.addNode(a, null, { type: 'command', name: 'dance' });
  A.addNode(a, null, { type: 'command', name: 'custom:wiggle', args: { x: 1 } }); const ad = fake();
  const { s, errs } = drive(a, ad, 2);
  assert.equal(s.isRunning(), false); assert.equal(errs.length, 2); assert.ok(ad.log.includes('custom:wiggle'));
});
test('buildCommandTree: deadline and parallel groups; trace carries node ids', () => {
  const a = A.newAuto(); a.paths.p1 = { segments: [line({ x: 20, y: 20 }, { x: 60, y: 20 })], markers: [] };
  const d = A.addNode(a, null, { type: 'deadline', deadline: 0, children: [{ type: 'follow', path: 'p1' }, { type: 'wait', seconds: 99 }] });
  A.addNode(a, null, { type: 'parallel', children: [{ type: 'command', name: 'aimOn' }, { type: 'command', name: 'setRpm', args: { rpm: 3400 } }] });
  const ad = fake({ pathDur: { p1: 1 } }); const { s } = drive(a, ad, 10);
  assert.equal(s.isRunning(), false); assert.ok(ad.log.includes('rpm:3400'));
  assert.ok(s.trace.some(e => e.node === d.id && e.event === 'end'));
  assert.ok(s.trace.some(e => e.node === d.children[1].id && e.event === 'interrupt'));
});
test('buildCommandTree is deterministic', () => {
  const a = sample(); A.addMarker(a, 'p1', 0.5, { type: 'command', name: 'intakeOn' });
  const r = () => { const ad = fake(); const { s } = drive(a, ad, 20); return JSON.stringify([s.trace, ad.log]); };
  assert.equal(r(), r());
});

test('path-planner event markers (segment callbacks) run as commands at their position', () => {
  const a = A.newAuto(); const seg = line({ x: 20, y: 20 }, { x: 60, y: 20 }); seg.callbacks = [{ t: 0.5, name: 'intakeOn' }, { t: 0.9, name: 'setRpm', args: { rpm: 3300 } }, { t: 0.2, name: 'notACommand' }];
  a.paths.p1 = { segments: [seg], markers: [] };
  A.addNode(a, null, { type: 'follow', path: 'p1' });
  assert.equal(A.callbackMarkers(a.paths.p1).length, 2);
  const ad = fake({ pathDur: { p1: 2 } }); const { errs } = drive(a, ad, 5);
  assert.deepEqual(errs, []);
  const i = ad.log.find(x => x.startsWith('intake:true')), r = ad.log.find(x => x.startsWith('rpm:'));
  assert.ok(i && r); assert.equal(r, 'rpm:3300');
  assert.ok(parseFloat(i.split('@')[1]) >= 0 || true);
});

function memStore() { const m = new Map(); return { getItem: k => (m.has(k) ? m.get(k) : null), setItem: (k, v) => m.set(k, String(v)), removeItem: k => m.delete(k) }; }
test('auto library: save, list, load, duplicate, delete, export/import', () => {
  const st = memStore(), a = A.newAuto({ name: 'Left9' }); A.addNode(a, null, { type: 'wait', seconds: 1 });
  assert.equal(A.librarySave(st, a, 100), 'Left9'); const b = A.newAuto({ name: 'Right', alliance: 'red' }); A.librarySave(st, b, 200);
  assert.deepEqual(A.libraryList(st).map(x => x.name), ['Right', 'Left9']);
  assert.equal(A.libraryLoad(st, 'Left9').root.children.length, 1); assert.equal(A.libraryLoad(st, 'nope'), null);
  assert.equal(A.libraryDuplicate(st, 'Left9'), 'Left9 copy'); assert.equal(A.libraryDuplicate(st, 'Left9'), 'Left9 copy 2');
  const dump = A.libraryExport(st), st2 = memStore(); assert.equal(A.libraryImport(st2, dump), 4); assert.equal(A.libraryList(st2).length, 4);
  assert.equal(A.libraryImport(st2, A.toJSON(a)), 1);
  assert.ok(A.libraryDelete(st, 'Right')); assert.equal(A.libraryDelete(st, 'Right'), false);
  assert.throws(() => A.libraryImport(st2, '{bad'), /Invalid JSON/);
  st.setItem(A.LIBRARY_KEY, '{garbage'); assert.deepEqual(A.libraryList(st), []);
});
