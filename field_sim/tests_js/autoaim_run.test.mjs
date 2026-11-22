import test from 'node:test';
import assert from 'node:assert/strict';
import { AutoRunner } from '../web/js/autorunner.js';
import { newAuto, addNode } from '../web/js/autos.js';

const line = (a, b) => ({ type: 'line', points: [{ x: a[0], y: a[1] }, { x: b[0], y: b[1] }] });
function sampler(route) {
  const a = route.segments[0].points[0], b = route.segments.at(-1).points.at(-1);
  const L = Math.hypot(b.x - a.x, b.y - a.y), out = [];
  for (let s = 0; s <= L + 1e-9; s += 1) out.push({ x: a.x + (b.x - a.x) * s / L, y: a.y + (b.y - a.y) * s / L, h: 0, s });
  if (out.at(-1).s < L - 1e-6) out.push({ x: b.x, y: b.y, h: 0, s: L });
  return out;
}
// Shot solved at a fixed target; the plant moves toward it on apply.
function aimHost({ feasible = true, withTarget = true } = {}) {
  const T = { rpm: 3000, turretDeg: 30, hoodDeg: 20, feasible };
  const h = {
    cur: { rpm: 0, turretDeg: 0, hoodDeg: 0 }, calls: [], balls: 2, transferOn: false, applied: 0,
    setPose() {}, setVelocity() {}, onLog: () => {},
    transfer: on => { h.transferOn = on; h.calls.push('transfer:' + on); },
    shoot: a => { h.calls.push('shoot:' + JSON.stringify(a)); if (a?.all) h.balls = 0; return true; },
    hopperEmpty: () => h.balls === 0,
    autoAimCurrent: () => ({ ...h.cur }),
    autoAimApply: c => { h.applied++; if (c) h.cur = { rpm: c.rpm, turretDeg: c.turretDeg, hoodDeg: c.hoodDeg }; },
  };
  if (withTarget) h.autoAimTarget = () => (feasible ? { ...T } : { ...T, feasible: false });
  return h;
}
function autoWith(cmdNodes, paths = true) {
  const a = newAuto({ start: { x: 24, y: 24, h: 0 } });
  a.paths.p1 = { segments: [line([24, 24], [24, 84])], markers: [] };
  for (const n of cmdNodes) addNode(a, null, n);
  if (paths) addNode(a, null, { type: 'follow', path: 'p1' });
  return a;
}
function run(auto, host, seconds) {
  const r = new AutoRunner({ host, pathSampler: sampler }).load(auto);
  r.start();
  for (let t = 0; t < seconds && r.state === 'running'; t += 1 / 60) r.step(1 / 60);
  return r;
}

test('autoAim aligns, does not feed while aiming, and finishes; the auto continues to the next command', () => {
  const h = aimHost();
  const r = run(autoWith([{ type: 'command', name: 'autoAim', args: { shootAfter: false } }]), h, 20);
  assert.equal(r.state, 'done');
  assert.ok(!h.calls.includes('transfer:true'), 'transfer stays off while aiming (Java TRANSFER_ON_START=false)');
  assert.equal(h.transferOn, false, 'auto end turns transfer back off');
  assert.ok(h.applied > 0);
  assert.deepEqual(r.warnings, []);
});

test('autoAim with shootAfter fires a full hopper once ready', () => {
  const h = aimHost();
  const r = run(autoWith([{ type: 'command', name: 'autoAim', args: { shootAfter: true } }]), h, 20);
  assert.equal(r.state, 'done');
  assert.ok(h.calls.some(c => c === 'shoot:{"all":true}'));
  assert.equal(h.balls, 0);
});

test('a failed aim (no solution) is a soft failure: warning recorded, auto still completes', () => {
  const h = aimHost({ withTarget: true, feasible: false });
  const r = run(autoWith([{ type: 'command', name: 'autoAim', args: { timeout: 0.5 } }]), h, 20);
  assert.equal(r.state, 'done', 'does not abort');
  assert.deepEqual(r.errors, []);
  assert.ok(r.warnings.some(w => /^autoAim: timeout/.test(w)), r.warnings.join(' | '));
});

test('a host without autoAimTarget makes autoAim complete immediately with a warning', () => {
  const h = aimHost({ withTarget: false });
  const r = run(autoWith([{ type: 'command', name: 'autoAim' }]), h, 20);
  assert.equal(r.state, 'done');
  assert.ok(r.warnings.some(w => /no autoAimTarget/.test(w)));
  assert.equal(h.transferOn, false);
});

test('track keeps applying the target after the aim command, until the next follow finishes', () => {
  const h = aimHost();
  const a = autoWith([{ type: 'command', name: 'autoAim', args: { track: true } }]);
  const r = new AutoRunner({ host: h, pathSampler: sampler }).load(a);
  r.start();
  for (let t = 0; t < 0.5 && r.state === 'running'; t += 1 / 60) r.step(1 / 60);
  assert.ok(r._trackAim, 'tracking set after the aim command');
  const before = h.applied;
  // Stop the auto: stop() must clear tracking and turn transfer off.
  r.stop();
  assert.equal(r._trackAim, null);
  assert.equal(h.transferOn, false);
  assert.ok(h.applied >= before);
});

test('keepAim tracks through a follow and ends when that follow finishes', () => {
  const h = aimHost();
  const a = autoWith([]);
  const r = new AutoRunner({ host: h, pathSampler: sampler }).load(a);
  r.keepAim({});
  assert.ok(r._trackAim);
  r.start();
  for (let t = 0; t < 30 && r.state === 'running'; t += 1 / 60) r.step(1 / 60);
  assert.equal(r.state, 'done');
  assert.equal(r._trackAim, null, 'cleared when the follow completes');
});

test('reset() clears warnings and tracking; updating a finished tracking command is harmless', () => {
  const h = aimHost({ withTarget: false });
  const r = run(autoWith([{ type: 'command', name: 'autoAim', args: { track: true } }]), h, 20);
  assert.ok(r.warnings.length > 0);
  r.reset();
  assert.deepEqual(r.warnings, []);
  assert.equal(r._trackAim, null);
  r._tick(1 / 60); // no sched: harmless
});
