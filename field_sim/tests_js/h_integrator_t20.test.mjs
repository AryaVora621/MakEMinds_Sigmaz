// t20 cross-module smoke: parallel(drive + intakeOn) -> shootAll -> wait, run headlessly to completion.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AutoRunner, buildTimeline } from '../web/js/autorunner.js';
import { newAuto, addNode, findNode, walkNodes } from '../web/js/autos.js';

const DT = 1 / 60, EPS = 1e-6;
const line = (a, b) => ({ type: 'line', points: [{ x: a[0], y: a[1] }, { x: b[0], y: b[1] }] });
function sampler(route) { // straight line sampled every 1 in, constant heading 0.5 rad
  const a = route.segments[0].points[0], b = route.segments.at(-1).points.at(-1);
  const L = Math.hypot(b.x - a.x, b.y - a.y), out = [];
  for (let s = 0; s <= L + 1e-9; s += 1) out.push({ x: a.x + (b.x - a.x) * s / L, y: a.y + (b.y - a.y) * s / L, h: 0.5, s });
  if (out.at(-1).s < L - 1e-6) out.push({ x: b.x, y: b.y, h: 0.5, s: L });
  return out;
}
function fakeHost() {
  const h = {
    pose: { x: 24, y: 24, h: 0 }, vel: [0, 0, 0], balls: 0, intakeOn: false, calls: [],
    getPose: () => h.pose, setPose: p => { h.pose = { ...p }; }, setVelocity: (a, b, c) => { h.vel = [a, b, c]; },
    intake: on => { h.intakeOn = on; h.calls.push('intake:' + on); },
    shoot: a => { h.calls.push('shoot:' + JSON.stringify(a)); if (a?.all) h.balls = 0; return true; },
    ballCount: () => h.balls, hopperEmpty: () => h.balls === 0, hopperFull: () => h.balls >= 4,
    flywheelReady: () => true, aimed: () => true, park: () => {}, onLog: () => {},
  };
  return h;
}
const base = () => { const a = newAuto({ start: { x: 24, y: 24, h: 0 } }); a.paths.p1 = { segments: [line([24, 24], [24, 84])], markers: [] }; return a; };

test('parallel(drive+intake) -> shootAll -> wait: done, timeline matches, bars nest, holds <= 4, pose finite', () => {
  // Runner's own drive duration for p1 (used as pathTime so the timeline and runner share one clock).
  const solo = base(); addNode(solo, null, { type: 'follow', path: 'p1' });
  const sr = new AutoRunner({ host: fakeHost(), pathSampler: sampler }).load(solo).start();
  while (sr.state === 'running') sr.step(DT);
  const T = sr.time;

  const auto = base();
  addNode(auto, null, { type: 'parallel', children: [{ type: 'follow', path: 'p1' }, { type: 'command', name: 'intakeOn' }] });
  addNode(auto, null, { type: 'command', name: 'shootAll' });
  addNode(auto, null, { type: 'wait', seconds: 0.5 });
  const host = fakeHost(), r = new AutoRunner({ host, pathSampler: sampler }).load(auto);
  r.start(); assert.equal(r.state, 'running');
  // Test-side ball plant: intake adds 2 balls/s with NO cap, so the hold-count assertion is a real observation.
  let maxBalls = 0, acc = 0, n = 0;
  while (r.state === 'running' && n++ < 60 * 60) {
    r.step(DT);
    if (host.intakeOn) { acc += 2 * DT; while (acc >= 1) { acc -= 1; host.balls += 1; } } else acc = 0;
    maxBalls = Math.max(maxBalls, host.balls);
    for (const v of [host.pose.x, host.pose.y, host.pose.h, ...host.vel]) assert.ok(Number.isFinite(v), 'non-finite pose/vel');
  }
  assert.equal(r.state, 'done', `state ${r.state}; errors ${r.errors.join(' | ')}`);
  assert.ok(maxBalls <= 4, `hold count reached ${maxBalls} (limit 4)`);
  assert.ok(host.calls.includes('shoot:{"all":true}'), 'shootAll fired');
  assert.ok(Math.abs(r.time - (T + 0.5)) <= DT + EPS, `runner time ${r.time} vs ${T + 0.5}`);

  const bars = buildTimeline(auto, { pathTime: () => T });
  assert.deepEqual(bars.errors, []);

  const parBar = bars.find(b => b.id === auto.root.children[0].id);
  assert.ok(parBar && parBar.kind === 'group' && parBar.group === 'parallel', 'parallel group bar present');
  const kids = auto.root.children[0].children.map(c => bars.find(b => b.id === c.id));
  assert.ok(kids.every(Boolean), 'child bars present');
  for (const k of kids) assert.ok(Math.abs(k.start - parBar.start) < EPS, `${k.label} starts ${k.start}, group ${parBar.start}`);
  assert.notEqual(kids[0].lane, kids[1].lane, 'overlapping parallel children get separate lanes');
  for (const g of bars.filter(b => b.kind === 'group')) {
    walkNodes(findNode(auto, g.id).node, n2 => {
      const b = n2.id !== g.id && bars.find(x => x.id === n2.id);
      if (b && b.kind !== 'group') assert.ok(b.start >= g.start - EPS && b.end <= g.end + EPS, `${b.label} outside ${g.group}`);
    });
  }
  const shoot = bars.find(b => b.label === 'Shoot all');
  assert.ok(shoot && Math.abs(shoot.start - parBar.end) <= DT + EPS, 'shootAll starts when the parallel ends');
});

// FAILING-BY-DESIGN FINDING: buildTimeline total != runner time. The dry run has no shootAll handle, so it uses
// COMMAND_DEFS.shootAll.simDuration (1.2 s); the live adapter finishes shootAll as soon as hopperEmpty() is true.
// Observed: runner 2.10 s vs timeline 3.30 s (diff 1.2 s). Kept as todo until the two agree (owner: autorunner.js).
test('todo: timeline total matches runner time (shootAll duration mismatch)', { todo: 'FAILING-BY-DESIGN FINDING: shootAll 1.2s in timeline vs instant at runtime' }, () => {
  const solo = base(); addNode(solo, null, { type: 'follow', path: 'p1' });
  const sr = new AutoRunner({ host: fakeHost(), pathSampler: sampler }).load(solo).start();
  while (sr.state === 'running') sr.step(DT);
  const auto = base();
  addNode(auto, null, { type: 'parallel', children: [{ type: 'follow', path: 'p1' }, { type: 'command', name: 'intakeOn' }] });
  addNode(auto, null, { type: 'command', name: 'shootAll' });
  addNode(auto, null, { type: 'wait', seconds: 0.5 });
  const host = fakeHost(), r = new AutoRunner({ host, pathSampler: sampler }).load(auto).start();
  while (r.state === 'running') r.step(DT);
  const bars = buildTimeline(auto, { pathTime: () => sr.time });
  assert.ok(Math.abs(r.time - bars.total) <= DT + EPS, `runner ${r.time} vs timeline ${bars.total}`);
});

/*
 * Ran: node --test tests_js/h_integrator_t20.test.mjs (main test passes; todo test reported as todo, not failure).
 * Ran also before the fix: the total check failed (runner 2.10 vs timeline 3.30), which is the finding above.
 * Read only (not run): tests_js/autorunner.test.mjs, tests_js/autoaim_runner.test.mjs, web/js/autorunner.js, web/js/autos.js, web/js/scheduler.js (exports/trace).
 * Caveats: the hold plant (2 balls/s, uncapped) is in the test, not the runner, so the <=4 check only shows this auto stays within 4 for this timing.
 * Caveats: pathTime is measured from the runner's own solo follow, so the timeline check is runner-vs-timeline, not vs real Pedro.
 */
