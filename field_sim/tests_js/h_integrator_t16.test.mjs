// t16: AutoRunner lifecycle edge cases on a fake host (shots live in host state). { todo } = FAILING-BY-DESIGN FINDING.
import test from 'node:test'; import assert from 'node:assert/strict';
import { AutoRunner } from '../web/js/autorunner.js'; import { newAuto, addNode } from '../web/js/autos.js';
const sampler = r => { const a = r.segments[0].points[0], b = r.segments.at(-1).points.at(-1), L = Math.hypot(b.x - a.x, b.y - a.y), o = []; for (let s = 0; s <= L + 1e-9; s += 1) o.push({ x: a.x + (b.x - a.x) * s / L, y: a.y + (b.y - a.y) * s / L, h: 0, s }); o.push({ x: b.x, y: b.y, h: 0, s: L }); return o; };
function host() { const h = { pose: { x: 0, y: 0, h: 0 }, balls: 3, shots: 0, restores: 0, calls: [], throwOnce: null };
  return Object.assign(h, { getPose: () => h.pose, setPose: p => { h.pose = { ...p }; }, setVelocity: () => {}, intake: on => h.calls.push('intake:' + on),
    shoot: a => { h.calls.push('shoot:' + JSON.stringify(a)); const n = a?.all ? h.balls : Math.min(h.balls, a?.count ?? 1); h.balls -= n; h.shots += n; return n > 0; },
    ballCount: () => h.balls, hopperFull: () => h.balls >= 3, hopperEmpty: () => h.balls === 0, flywheelReady: () => true, aimed: () => true,
    park: () => { if (h.throwOnce) { const e = h.throwOnce; h.throwOnce = null; throw e; } h.calls.push('park'); },
    snapshot: () => ({ pose: { ...h.pose }, balls: h.balls, shots: h.shots }), restore: s => { h.restores++; Object.assign(h, { pose: { ...s.pose }, balls: s.balls, shots: s.shots }); } }); }
function auto(extra) { const a = newAuto({ start: { x: 24, y: 24, h: 0 } }); // 60in follow, shootAll, [extra], park
  a.paths.p1 = { segments: [{ type: 'line', points: [{ x: 24, y: 24 }, { x: 24, y: 84 }] }], markers: [] };
  addNode(a, null, { type: 'follow', path: 'p1' }); addNode(a, null, { type: 'command', name: 'shootAll' }); if (extra) addNode(a, null, extra); addNode(a, null, { type: 'command', name: 'park' }); return a; }
async function mk(h = host(), a = auto()) { const r = new AutoRunner({ host: h, pathSampler: sampler }).load(a); await r.prepare(); return { r, h }; }
const runAll = r => { for (let i = 0; i < 1000 && r.state === 'running'; i++) r.step(1 / 60); return r; };
const fresh = async () => { const f = await mk(); f.r.start(); runAll(f.r); return f; };
test('start twice in a row behaves like a single start', async () => {
  const { r, h } = await mk(); r.start(); r.start(); assert.equal(r.state, 'running'); runAll(r);
  const f = await fresh(); assert.equal(r.state, 'done'); assert.deepEqual(h.calls, f.h.calls); assert.ok(Math.abs(r.time - f.r.time) < 1e-9); });
test('pause before first step then resume matches an unpaused run; pause/resume on idle is a no-op', async () => {
  const idle = await mk(); idle.r.pause().resume(); assert.equal(idle.r.state, 'idle');
  const { r, h } = await mk(); r.start(); r.pause(); r.step(5); assert.equal(r.time, 0); r.resume(); runAll(r);
  const f = await fresh(); assert.equal(r.state, 'done'); assert.ok(Math.abs(r.time - f.r.time) < 1e-9); assert.deepEqual(h.calls, f.h.calls); });
test('reset while running restores the snapshot exactly once and returns to idle', async () => {
  const { r, h } = await mk(); r.start(); r.step(0.5); r.reset();
  assert.equal(h.restores, 1); assert.equal(r.state, 'idle'); assert.deepEqual(h.pose, { x: 0, y: 0, h: 0 }); assert.equal(h.balls, 3); });
test('seek backwards after completion re-runs shots without double counting', async () => {
  const { r, h } = await mk(); r.start(); runAll(r); assert.equal(h.shots, 3);
  r.seek(0.2); assert.equal(h.shots, 0); assert.equal(h.balls, 3); r.seek(100); assert.equal(h.shots, 3); assert.equal(h.balls, 0); });
// FAILING-BY-DESIGN FINDING: start() re-snapshots host state, overwriting the pristine snapshot with the mid-run state.
test('FAILING-BY-DESIGN FINDING: reset after a second start() restores the pre-run state', { todo: 'start() overwrites snapshot' }, async () => {
  const { r, h } = await mk(); r.start(); r.step(0.5); r.start(); r.reset(); assert.deepEqual(h.pose, { x: 0, y: 0, h: 0 }); });
test('stop mid-run then start gives the same host-call log as a fresh runner', async () => {
  const { r, h } = await mk(); r.start(); r.step(0.5); r.stop(); assert.equal(r.state, 'idle');
  h.calls.length = 0; r.start(); runAll(r); const f = await fresh();
  assert.deepEqual(h.calls, f.h.calls); assert.ok(Math.abs(r.time - f.r.time) < 1e-9); });
test('a throwing command sets state error, aborts the auto, and the runner recovers after reset', async () => {
  const h = host(); h.throwOnce = new Error('park jam'); const { r } = await mk(h); r.start(); runAll(r);
  assert.equal(r.state, 'error'); assert.ok(r.errors.some(e => /park jam/.test(e)));
  r.reset(); r.start(); runAll(r); assert.equal(r.state, 'done'); assert.ok(h.calls.includes('park')); });
test('step(0) advances nothing; step(1000) finishes without skipping commands or looping', async () => {
  const { r, h } = await mk(); r.start(); r.step(0); assert.equal(r.time, 0); assert.equal(r.state, 'running');
  const t0 = Date.now(); r.step(1000); assert.ok(Date.now() - t0 < 2000, 'bounded loop');
  assert.equal(r.state, 'done'); assert.equal(h.shots, 3); assert.deepEqual(h.calls.filter(c => c === 'park'), ['park']); assert.ok(r.time > 1.5 && r.time < 2.5); });
test('step(1000) on a blocked waitUntil stays running and advances exactly 1000 s', async () => {
  const { r } = await mk(host(), auto({ type: 'waitUntil', condition: 'ballCount>=99' }));
  r.start(); r.step(1000); assert.equal(r.state, 'running'); assert.ok(Math.abs(r.time - 1000) < 1e-3, `time ${r.time}`); });
/* RUN LOG: node --test tests_js/h_integrator_t16.test.mjs. Result: 8 pass, 0 fail, 1 todo (FAILING-BY-DESIGN FINDING: reset after second start() restored pose (24, 36.625), not (0, 0)). */
