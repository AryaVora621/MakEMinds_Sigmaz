// PedroFollower closed loop against the MecanumPlant (no host). Units: in, s, rad.
import test from 'node:test';
import assert from 'node:assert/strict';
import { resolveParams } from '../../web/js/physics/pedro-fidelity/params.js';
import { SimClock, smallestAngleDiff } from '../../web/js/physics/pedro-fidelity/pedromath.js';
import { MecanumPlant } from '../../web/js/physics/pedro-fidelity/plant.js';
import { SimLocalizer } from '../../web/js/physics/pedro-fidelity/localizer.js';
import { MecanumDrivetrain } from '../../web/js/physics/pedro-fidelity/mecanumdrive.js';
import { PedroFollower } from '../../web/js/physics/pedro-fidelity/follower.js';
import { groupsFromBridgeRequest } from '../../web/js/physics/pedro-fidelity/pedropath.js';

function rig(overrides = {}, pose = { x: 0, y: 0, h: 0 }) {
  const P = resolveParams(overrides), clock = new SimClock(), plant = new MecanumPlant(P, pose);
  const localizer = new SimLocalizer(plant, P.localizer, clock), drivetrain = new MecanumDrivetrain(P);
  const f = new PedroFollower({ params: P, clock, localizer, drivetrain });
  f.setStartingPose(pose);
  const dt = P.loop.dt;
  const r = { P, clock, plant, f, dt, t: 0, wall: null, wallHitT: null };
  // spec 5a order: clock -> plant (with last powers) -> follower.update -> new powers
  r.tick = () => {
    clock.advance(dt); plant.step(dt); r.t += dt;
    if (r.wall !== null && plant.x >= r.wall) {   // immovable wall at x = wall: the robot cannot go further
      if (r.wallHitT === null) r.wallHitT = r.t;
      plant.syncTo({ x: r.wall, y: plant.y, h: plant.h }, { vx: 0, vy: 0, w: 0 });
    }
    f.update(); plant.setPowers(f.powers());
  };
  r.until = (cond, maxS) => { while (r.t < maxS && !cond()) r.tick(); return r.t; };
  return r;
}
const line = (x0, y0, x1, y1, extra = {}) => ({ type: 'line', points: [{ x: x0, y: y0 }, { x: x1, y: y1 }], heading: { mode: 'constant', start: 0 }, ...extra });
const chainOf = (r, body) => groupsFromBridgeRequest(body, r.P.constraints)[0].chain;

test('48 in line finishes within 1 in, then HOLDS (isBusy false, mode hold)', () => {
  const r = rig();
  r.f.followChain(chainOf(r, { start: { x: 0, y: 0, h: 0 }, segments: [line(0, 0, 48, 0)] }));
  r.tick();
  assert.equal(r.f.isBusy(), true);
  r.until(() => !r.f.isBusy(), 10);
  assert.equal(r.f.isBusy(), false, 'path should finish');
  assert.ok(Math.hypot(r.plant.x - 48, r.plant.y) < 1, `end ${r.plant.x},${r.plant.y}`);
  const tEnd = r.t;
  while (r.t < tEnd + 1) r.tick();
  assert.equal(r.f.isHolding(), true);
  assert.equal(r.f.telemetry().mode, 'hold');
  assert.ok(Math.hypot(r.plant.x - 48, r.plant.y) < 1);
});

test('wall at 90% of the path: stuck triggers and the follower ends within stuckTimeout + timeoutConstraint + 0.2 s', () => {
  const r = rig(); r.wall = 0.9 * 48;
  r.f.followChain(chainOf(r, { start: { x: 0, y: 0, h: 0 }, segments: [line(0, 0, 48, 0)] }));
  let stuckSeen = false;
  while (r.t < 10 && r.f.isBusy()) { r.tick(); if (r.f.isRobotStuck()) stuckSeen = true; }
  assert.notEqual(r.wallHitT, null, 'robot reached the wall');
  assert.equal(stuckSeen, true, 'stuck detection fired');
  assert.equal(r.f.isBusy(), false);
  const budget = r.P.pedro.stuckTimeout / 1000 + r.P.constraints.timeoutConstraint / 1000 + 0.2;   // 0.5 + 0.1 + 0.2 s
  assert.ok(r.t - r.wallHitT <= budget, `ended ${r.t - r.wallHitT} s after wall contact, budget ${budget}`);
});

test('turnTo(PI/2) ends with |headingError| < 0.01 rad', () => {
  const r = rig();
  r.f.turnTo(Math.PI / 2); r.tick();
  assert.equal(r.f.isTurning(), true); assert.equal(r.f.isBusy(), true);
  r.until(() => !r.f.isBusy(), 8);
  assert.equal(r.f.isBusy(), false); assert.equal(r.f.isTurning(), false);
  assert.ok(Math.abs(r.f.headingError()) < 0.01, `headingError ${r.f.headingError()}`);
  assert.ok(smallestAngleDiff(r.plant.h, Math.PI / 2) < 0.03, `plant h ${r.plant.h}`);
});

test('chain advance: a 2-path chain visits chainIndex 0 then 1 and finishes at the end', () => {
  const r = rig();
  r.f.followChain(chainOf(r, { start: { x: 0, y: 0, h: 0 }, segments: [line(0, 0, 30, 0), line(30, 0, 30, 30)] }));
  const seen = new Set();
  while (r.t < 10 && r.f.isBusy()) { r.tick(); seen.add(r.f.getChainIndex()); }
  assert.ok(seen.has(0) && seen.has(1), [...seen].join(','));
  assert.ok(Math.hypot(r.plant.x - 30, r.plant.y - 30) < 1.5, `end ${r.plant.x},${r.plant.y}`);
});

const cbBody = () => ({ start: { x: 0, y: 0, h: 0 }, segments: [
  line(0, 0, 30, 0, { callbacks: [{ t: 0.5, name: 'a' }, { ms: 200, name: 'tmr' }] }),
  line(30, 0, 60, 0, { callbacks: [{ t: 0.5, name: 'b' }] }),
] });
function runCallbacks(overrides) {
  const r = rig(overrides), fired = [];
  r.f.onCallback((name, seg, pathIndex) => fired.push({ name, seg, pathIndex, t: r.t }));
  r.f.followChain(chainOf(r, cbBody()));
  while (r.t < 10 && r.f.isBusy()) r.tick();
  return fired;
}

test('callbacks (intended semantics): parametric on path 0 and path 1 fire, temporal fires ~ms after the path started', () => {
  const fired = runCallbacks({});
  const names = fired.map(c => c.name);
  assert.ok(names.includes('a') && names.includes('b') && names.includes('tmr'), names.join(','));
  assert.ok(names.indexOf('a') < names.indexOf('b'));
  const tmr = fired.find(c => c.name === 'tmr');
  assert.ok(tmr.t >= 0.15 && tmr.t <= 0.3, `temporal fired at ${tmr.t}`);
  assert.equal(fired.find(c => c.name === 'b').pathIndex, 1);
});

test('callbacks (emulate210CallbackBug): path-0 parametric callbacks never fire (Pedro 2.1.0 getNextPathCallbacks)', () => {
  const fired = runCallbacks({ sim: { emulate210CallbackBug: true } });
  assert.ok(!fired.some(c => c.name === 'a'), 'path-0 callback "a" must not fire under the bug');
});
