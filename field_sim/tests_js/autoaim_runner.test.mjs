import test from 'node:test';
import assert from 'node:assert/strict';
import { AutoRunner } from '../web/js/autorunner.js';
import { newAuto, addNode } from '../web/js/autos.js';

// Runner-level tests for the autoAim command: drives the adapter directly (autos.js may not map
// type 'autoAim' everywhere), plus one end-to-end case through buildCommandTree.

const DT = 1 / 60;
const line = (a, b) => ({ type: 'line', points: [{ x: a[0], y: a[1] }, { x: b[0], y: b[1] }] });
function sampler(route) {
  const a = route.segments[0].points[0], b = route.segments.at(-1).points.at(-1);
  const L = Math.hypot(b.x - a.x, b.y - a.y), out = [];
  for (let s = 0; s <= L + 1e-9; s += 1) out.push({ x: a.x + (b.x - a.x) * s / L, y: a.y + (b.y - a.y) * s / L, h: 0, s });
  if (out.at(-1).s < L - 1e-6) out.push({ x: b.x, y: b.y, h: 0, s: L });
  return out;
}

/**
 * Fake host. The plant moves 10% of the way to each applied command (converges),
 * or never moves when frozen. `moving` makes the solved turret angle drift every read.
 */
function aimHost({ frozen = false, feasible = true, withTarget = true, moving = false } = {}) {
  const h = {
    cur: { rpm: 0, turretDeg: 0, hoodDeg: 0 }, applied: [], calls: [], balls: 2, transferOn: false,
    setPose() {}, setVelocity() {}, onLog: () => {},
    transfer: on => { h.transferOn = on; h.calls.push('transfer:' + on); },
    shoot: a => { h.calls.push('shoot:' + JSON.stringify(a)); if (a?.all) h.balls = 0; return true; },
    hopperEmpty: () => h.balls === 0,
    autoAimCurrent: () => ({ ...h.cur }),
    autoAimApply: c => {
      h.applied.push({ ...c });
      if (frozen || !c) return;
      for (const k of ['rpm', 'turretDeg', 'hoodDeg']) h.cur[k] += (c[k] - h.cur[k]) * 0.1;
    },
  };
  if (withTarget) {
    let n = 0;
    h.autoAimTarget = () => {
      const turretDeg = moving ? 10 + 0.5 * n : 30;
      n++;
      return { rpm: 3000, turretDeg, hoodDeg: 20, feasible };
    };
  }
  return h;
}

function emptyRunner(host) {
  const auto = newAuto({ start: { x: 24, y: 24, h: 0 } });
  return new AutoRunner({ host, pathSampler: sampler }).load(auto);
}

test('adapter autoAim does not turn transfer on at start (Java TRANSFER_ON_START=false)', () => {
  const h = aimHost();
  const r = emptyRunner(h);
  r._adapter().autoAim({ shootAfter: true });
  assert.ok(!h.calls.includes('transfer:true'), h.calls.join(','));
  assert.equal(h.transferOn, false);
});

test('autoAim{shootAfter} completes once the plants converge and fires shoot({all:true}) exactly once', () => {
  const h = aimHost();
  const r = emptyRunner(h);
  const cmd = r._adapter().autoAim({ shootAfter: true });
  let n = 0;
  while (!cmd.done() && n++ < 20 * 60) r._aimCmd.update(DT);
  assert.equal(cmd.done(), true, 'command finished');
  assert.ok(n < 20 * 60, 'finished well before the 20 s guard');
  assert.equal(r._aimCmd.status().result, 'ready', 'finished by readiness, not timeout');
  assert.equal(h.calls.filter(c => c === 'shoot:{"all":true}').length, 1, 'fired once');
  assert.deepEqual(r.warnings, []);
});

test('a frozen plant fails the aim: completes by 4 s, warning recorded, runner state not error', () => {
  const h = aimHost({ frozen: true });
  const r = emptyRunner(h);
  const cmd = r._adapter().autoAim({ shootAfter: false });
  let t = 0;
  while (!cmd.done() && t < 6) { r._aimCmd.update(DT); t += DT; }
  assert.equal(cmd.done(), true, 'completes');
  assert.ok(t <= 4 + DT + 1e-9, `completed at ${t.toFixed(3)} s`);
  assert.ok(r.warnings.length > 0, 'warning recorded');
  assert.match(r.warnings[0], /^autoAim: timeout after 4s/);
  assert.notEqual(r.state, 'error');
  assert.equal(h.calls.some(c => c.startsWith('shoot')), false, 'no shot without shootAfter');
});

test('a host without autoAimTarget makes adapter autoAim complete immediately with a warning', () => {
  const h = aimHost({ withTarget: false });
  const r = emptyRunner(h);
  const cmd = r._adapter().autoAim({ shootAfter: true });
  assert.equal(cmd.done(), true);
  assert.ok(r.warnings.some(w => /no autoAimTarget/.test(w)), r.warnings.join(' | '));
  assert.equal(h.calls.includes('transfer:true'), false, 'nothing started');
  assert.notEqual(r.state, 'error');
});

test('track: after the aim finishes, step() keeps applying the moving target until stop()', () => {
  const h = aimHost({ moving: true });
  const auto = newAuto({ start: { x: 24, y: 24, h: 0 } });
  addNode(auto, null, { type: 'wait', seconds: 5 });
  const r = new AutoRunner({ host: h, pathSampler: sampler }).load(auto);
  r.start();
  r._adapter().autoAim({ track: true });
  for (let i = 0; i < 60; i++) r.step(DT);
  assert.equal(r.state, 'running');
  assert.ok(r._trackAim, 'tracking still active after the aim finished');
  const at05 = h.applied.at(-1).turretDeg;
  for (let i = 0; i < 60; i++) r.step(DT);
  const at10 = h.applied.at(-1).turretDeg;
  assert.ok(at10 > at05, `target moved ${at05} -> ${at10}`);
  // Applied targets never go backwards while tracking.
  for (let i = 1; i < h.applied.length; i++) assert.ok(h.applied[i].turretDeg >= h.applied[i - 1].turretDeg);
  const n = h.applied.length;
  r.stop();
  assert.equal(r._trackAim, null);
  assert.equal(h.transferOn, false, 'stop turns transfer off');
  r.step(1);
  assert.equal(h.applied.length, n, 'no more applies after stop');
});

test('e2e: autoAim then shootAll in the command tree; a failed aim does not stop the auto', () => {
  const h = aimHost({ frozen: true });
  const auto = newAuto({ start: { x: 24, y: 24, h: 0 } });
  addNode(auto, null, { type: 'command', name: 'autoAim', args: { shootAfter: false } });
  addNode(auto, null, { type: 'command', name: 'shootAll' });
  const r = new AutoRunner({ host: h, pathSampler: sampler }).load(auto);
  r.start();
  for (let i = 0; i < 30 * 60 && r.state === 'running'; i++) r.step(DT);
  assert.equal(r.state, 'done', `state ${r.state} errors ${r.errors.join(' | ')}`);
  assert.ok(r.warnings.some(w => /^autoAim: timeout/.test(w)), r.warnings.join(' | '));
  assert.ok(h.calls.includes('shoot:{"all":true}'), 'shootAll still ran after the failed aim');
  assert.ok(r.time >= 3.99 && r.time < 6, `auto ended at ${r.time}`); // ~4 s aim timeout, shootAll is near-instant
  assert.equal(h.transferOn, false);
});
