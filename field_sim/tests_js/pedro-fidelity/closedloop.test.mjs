// Offline simulateBridgeRequest + live PedroDriveService. Units: in, s, rad (trajectory h/w are radians).
import test from 'node:test';
import assert from 'node:assert/strict';
import { simulateBridgeRequest, PedroDriveService, PHASE, contactVelocity } from '../../web/js/physics/pedro-fidelity/closedloop.js';
import { groupsFromBridgeRequest } from '../../web/js/physics/pedro-fidelity/pedropath.js';
import { resolveParams } from '../../web/js/physics/pedro-fidelity/params.js';

const line = (x0, y0, x1, y1, extra = {}) => ({ type: 'line', points: [{ x: x0, y: y0 }, { x: x1, y: y1 }], heading: { mode: 'constant', start: 0 }, ...extra });
const body48 = () => ({ start: { x: 0, y: 0, h: 0 }, segments: [line(0, 0, 48, 0)] });

test('simulateBridgeRequest: 48 in line finishes, ends within 1 in, holds, pedro-js source', () => {
  const tr = simulateBridgeRequest(body48());
  assert.equal(tr.source, 'pedro-js'); assert.equal(tr.approximate, false);
  assert.equal(tr.finished, true); assert.equal(tr.endReason, 'finished');
  const last = tr.at(-1);
  assert.ok(Math.hypot(last.x - 48, last.y) < 1, `end ${last.x},${last.y}`);
  assert.equal(last.mode, 'hold'); assert.equal(last.phase, PHASE.DONE);
  assert.ok(tr.duration > 1 && tr.duration < 6, `duration ${tr.duration}`);
  const kinds = tr.events.map(e => e.kind);
  assert.ok(kinds.includes('pathStart') && kinds.includes('pathEnd') && kinds.at(-1) === 'done', kinds.join(','));
  assert.ok(Math.abs(last.h) <= Math.PI + 1e-9, 'h is in radians within [-PI, PI]');
});

test('determinism: two runs produce identical output', () => {
  const b = { start: { x: 10, y: 10, h: 0 }, segments: [
    { type: 'curve', points: [{ x: 10, y: 10 }, { x: 40, y: 10 }, { x: 40, y: 50 }], heading: { mode: 'tangent' }, waitAfter: 0.3, callbacks: [{ t: 0.5, name: 'c' }] },
    line(40, 50, 90, 50, { heading: { mode: 'linear', start: 0, end: 90 } }),
  ] };
  const a = simulateBridgeRequest(b), c = simulateBridgeRequest(b);
  assert.equal(JSON.stringify([...a]), JSON.stringify([...c]));
  assert.equal(JSON.stringify(a.events), JSON.stringify(c.events));
  assert.ok(a.events.some(e => e.kind === 'waitStart') && a.events.some(e => e.kind === 'waitEnd'));
});

test('simulateBridgeRequest ends with endReason timeout when maxTimeS is too short', () => {
  const tr = simulateBridgeRequest(body48(), { maxTimeS: 0.5 });
  assert.equal(tr.finished, false); assert.equal(tr.endReason, 'timeout');
  assert.ok(tr.events.some(e => e.kind === 'timeout'));
});

function mockHost(pose) {
  const h = { pose: { ...pose }, vel: [0, 0, 0], getPose: () => h.pose, setPose: p => { h.pose = { ...p }; }, setVelocity: (a, b, c) => { h.vel = [a, b, c]; } };
  return h;
}

// Round-2 note. The original single test ("corrects a 3 in push to < 0.5 in within 2 s" with library defaults) only
// passed because of a host-sync bug: the push was turned into +d/L = 300 in/s (capped 122 in/s) of velocity, which flung
// the robot ~25-27 in past the hold point before it came back. With the bug fixed (contactVelocity: a correction never
// injects velocity) and the REAL 2.1.0 defaults (defaults(): translational P 0.1, F 0; hold scaling 0.45) the hold
// controller cannot reach 0.5 in on this plant: measured rest error 1.806 in lateral, where the commanded lateral effort
// is cy = 0.1297, below the plant's static breakaway effort coulombY*tauY/vFreeY = 59.7819*0.2/77.39 = 0.1545 (so the
// robot is held by static friction, not by a service defect). With the bug fixed and the old (wrong) constants it also
// rests outside 0.5 in. So the < 0.5 in / 2 s spec is kept, but checked with a hold controller able to meet it
// (translational P 0.3, a typical tuned value), and the library-default case checks what is physically guaranteed.
function holdAndPush(params, push) {
  const P = resolveParams(params), host = mockHost({ x: 24, y: 24, h: 0 });
  const svc = new PedroDriveService({ host, params, startPose: { x: 24, y: 24, h: 0 } });
  svc.followGroups(groupsFromBridgeRequest({ start: { x: 24, y: 24, h: 0 }, segments: [line(24, 24, 60, 24)] }, P.constraints));
  assert.equal(svc.isBusy(), true);
  for (let i = 0; i < 1200 && svc.isBusy(); i++) svc.tick(0.01);
  assert.equal(svc.isBusy(), false);
  assert.ok(Math.hypot(host.pose.x - 60, host.pose.y - 24) < 1, `end ${host.pose.x},${host.pose.y}`);
  for (let i = 0; i < 50; i++) svc.tick(0.01);            // settle in hold
  assert.equal(svc.follower.isHolding(), true);
  const y0 = host.pose.y;
  host.pose = { ...host.pose, y: host.pose.y + push };     // external push (collision)
  svc.tick(0.01);
  assert.ok(svc.plant.pose.y > y0 + push - 0.5, `service accepted the host pose (y ${svc.plant.pose.y})`);
  let maxDev = 0, settledAt = null;
  for (let i = 1; i <= 200; i++) {                         // 2 s
    svc.tick(0.01);
    const e = Math.hypot(host.pose.x - 60, host.pose.y - 24);
    maxDev = Math.max(maxDev, Math.abs(host.pose.y - 24));
    if (e >= 0.5) settledAt = null; else if (settledAt === null) settledAt = i * 0.01;
  }
  return { P, svc, host, maxDev, settledAt, y0 };
}

test('PedroDriveService: finishes, keeps HOLDING, and (tuned hold P 0.3) corrects a 3 in push to < 0.5 in within 2 s, no overshoot', () => {
  const r = holdAndPush({ pedro: { coefficientsTranslationalPIDF: { P: 0.3 } } }, 3);
  assert.ok(Math.hypot(r.host.pose.x - 60, r.host.pose.y - 24) < 0.5, `after 2 s: ${r.host.pose.x},${r.host.pose.y}`);
  assert.ok(r.settledAt !== null && r.settledAt <= 2, `settled at ${r.settledAt}`);
  assert.ok(r.maxDev <= Math.abs(r.y0 - 24) + 3 + 0.05, `max deviation ${r.maxDev} exceeds the push: the sync injected velocity`);
});

test('PedroDriveService: library-default hold after a 3 in push never overshoots and rests only inside the static-friction deadband', () => {
  const r = holdAndPush({}, 3), P = r.P;
  for (let i = 0; i < 300; i++) { r.svc.tick(0.01); r.maxDev = Math.max(r.maxDev, Math.abs(r.host.pose.y - 24)); }   // +3 s: weak default hold creeps
  assert.ok(r.maxDev <= Math.abs(r.y0 - 24) + 3 + 0.05, `max deviation ${r.maxDev} exceeds the push: the sync injected velocity`);
  const e = Math.abs(r.host.pose.y - 24);
  assert.ok(e < 3 - 0.5, `the hold pulled back (rest error ${e})`);
  if (e >= 0.5) {   // not corrected: then it must be static friction holding it (effort below breakaway), and at rest
    const breakY = P.plant.coulombY * P.plant.tauY / P.plant.vFreeY;
    assert.ok(Math.abs(r.svc.plant.efforts().cy) <= breakY + 1e-9, `effort ${r.svc.plant.efforts().cy} > breakaway ${breakY} yet not moving`);
    assert.ok(Math.hypot(r.svc.plant.velocity.vx, r.svc.plant.velocity.vy) < 1e-6, 'at rest');
  }
});

test('contactVelocity: a wall undoing the loop motion zeroes v; a push never adds velocity; never reverses', () => {
  const L = 0.01;
  let r = contactVelocity({ vx: 40, vy: 0, w: 0 }, -0.4, 0, 0, L);           // wall undoes 40 in/s * 10 ms
  assert.ok(Math.abs(r.vx) < 1e-9 && r.vy === 0);
  r = contactVelocity({ vx: 0, vy: 0, w: 0 }, 0, 3, 0.2, L);                  // push of a resting robot
  assert.deepEqual(r, { vx: 0, vy: 0, w: 0 });
  r = contactVelocity({ vx: 40, vy: 0, w: 0 }, -3, 0, 0, L);                  // big correction: stops, does not reverse
  assert.ok(Math.abs(r.vx) < 1e-9);
  r = contactVelocity({ vx: 40, vy: 10, w: 2 }, -0.1, 0, -0.01, L);           // partial: removes only |d|/L = 10 in/s, w 1 rad/s
  assert.ok(Math.abs(r.vx - 30) < 1e-9 && r.vy === 10 && Math.abs(r.w - 1) < 1e-9);
  r = contactVelocity({ vx: 40, vy: 0, w: 0 }, 1, 0, 0, L);                   // correction along v: unchanged
  assert.equal(r.vx, 40);
});

test('PedroDriveService: stop() zeroes host velocity and leaves the service idle; progress is monotone 0..1', () => {
  const P = resolveParams(), host = mockHost({ x: 0, y: 0, h: 0 });
  const svc = new PedroDriveService({ host });
  svc.followGroups(groupsFromBridgeRequest(body48(), P.constraints));
  let last = 0;
  for (let i = 0; i < 100; i++) { svc.tick(0.01); const p = svc.progress(); assert.ok(p >= last - 1e-9 && p <= 1); last = p; }
  assert.ok(last > 0.05);
  svc.stop();
  assert.deepEqual(host.vel, [0, 0, 0]); assert.equal(svc.isBusy(), false);
});

test('PedroDriveService: waitAfter holds between groups before the next one starts', () => {
  const P = resolveParams(), host = mockHost({ x: 0, y: 0, h: 0 });
  const svc = new PedroDriveService({ host });
  svc.followGroups(groupsFromBridgeRequest({ start: { x: 0, y: 0, h: 0 }, segments: [line(0, 0, 20, 0, { waitAfter: 0.5 }), line(20, 0, 40, 0)] }, P.constraints));
  for (let i = 0; i < 2000 && svc.isBusy(); i++) svc.tick(0.01);
  const ev = svc.events.map(e => e.kind), ws = svc.events.find(e => e.kind === 'waitStart'), we = svc.events.find(e => e.kind === 'waitEnd');
  assert.ok(ev.includes('done'), ev.join(','));
  assert.ok(we.t - ws.t >= 0.49, `wait ${we.t - ws.t}`);
  assert.ok(Math.hypot(host.pose.x - 40, host.pose.y) < 1);
});
