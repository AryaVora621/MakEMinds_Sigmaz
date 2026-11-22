import test from 'node:test';
import assert from 'node:assert/strict';
import { AutoRunner } from '../web/js/autorunner.js';
import { newAuto, addNode } from '../web/js/autos.js';

const line = (a, b) => ({ type: 'line', points: [{ x: a[0], y: a[1] }, { x: b[0], y: b[1] }] });
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
// straight 60-in path along x; the trajectory (when given) replaces the trapezoid profile
function sampler(route) {
  const a = route.segments[0].points[0], b = route.segments.at(-1).points.at(-1);
  const L = Math.hypot(b.x - a.x, b.y - a.y), out = [];
  for (let s = 0; s <= L + 1e-9; s += 1) out.push({ x: a.x + (b.x - a.x) * s / L, y: a.y + (b.y - a.y) * s / L, h: 0, s });
  if (out.at(-1).s < L - 1e-6) out.push({ x: b.x, y: b.y, h: 0, s: L });
  return out;
}
function build(traj) {
  // auto start heading matches the trajectory's first sample so the start teleport is not counted as a seam jump
  const a = newAuto({ start: { x: 0, y: 0, h: traj?.[0]?.h ?? 0 } });
  a.paths.p = { segments: [line([0, 0], [60, 0])], markers: [] };
  addNode(a, null, { type: 'follow', path: 'p' });
  const host = { poses: [], setPose(p) { this.poses.push({ ...p }); }, setVelocity() {} };
  const r = new AutoRunner({ host, pathSampler: sampler, trajectorySampler: () => traj });
  r.load(a);
  return { r, host };
}
const runToEnd = (r, max = 60) => { for (let i = 0; i < max * 60 && r.state === 'running'; i++) r.step(1 / 60); };
const finite = p => Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.h);

test('heading crossing the +pi/-pi seam is replayed with no jump above pi/2 per tick', () => {
  const traj = [];
  for (let i = 0; i <= 120; i++) traj.push({ t: i * 0.05, x: i * 0.5, y: 0, h: wrap(3.0 + i * 0.02) }); // 3.0 rad rising past pi
  const { r, host } = build(traj);
  r.start(); runToEnd(r);
  assert.equal(r.state, 'done');
  const hs = host.poses.map(p => p.h);
  assert.ok(hs.every(Number.isFinite), 'no NaN heading');
  assert.ok(hs.some(h => h > 2.9) && hs.some(h => h < -2.9), 'run actually crosses the seam');
  for (let i = 1; i < hs.length; i++) {
    const d = Math.abs(wrap(hs[i] - hs[i - 1]));
    assert.ok(d <= Math.PI / 2, `heading jump ${d} at pose ${i}`);
  }
});

test('duplicate timestamps in the trajectory: runs to the end with finite poses', () => {
  const traj = [
    { t: 0, x: 0, y: 0, h: 0 }, { t: 0, x: 0, y: 0, h: 0 },
    { t: 1, x: 10, y: 0, h: 0.2 }, { t: 1, x: 10, y: 0, h: 0.2 },
    { t: 2, x: 20, y: 0, h: 0.4 },
  ];
  const { r, host } = build(traj);
  r.start(); runToEnd(r);
  assert.equal(r.state, 'done');
  assert.ok(r.time < 2.05, `ends at trajectory duration ${r.time}`);
  assert.ok(host.poses.every(finite), 'no NaN pose');
});

test('single-sample trajectory is rejected and the trapezoid fallback runs cleanly', () => {
  const { r, host } = build([{ t: 0, x: 5, y: 0, h: 1 }]);
  r.start(); runToEnd(r);
  assert.equal(r.state, 'done');
  assert.ok(host.poses.every(finite), 'no NaN pose');
});

for (const [label, traj] of [['empty array', []], ['{traj: []}', { traj: [] }], ['null', null]]) {
  test(`empty trajectory (${label}) does not throw and yields finite poses`, () => {
    const { r, host } = build(traj);
    assert.doesNotThrow(() => { r.start(); runToEnd(r); });
    assert.equal(r.state, 'done');
    assert.ok(host.poses.length > 0 && host.poses.every(finite), 'no NaN pose');
  });
}

// ---- run record (task t10, integrator) ----
// Ran: cd field_sim && node --test tests_js/h_integrator_t10.test.mjs
// Result: 6 tests, 6 pass, 0 fail (ran once; first run failed on a test setup issue, the start pose h=0 vs trajectory h=3.0,
//   fixed in the test by matching the start heading; no autorunner.js bug found, no todo/skip needed).
