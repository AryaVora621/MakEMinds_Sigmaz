import test, { beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { resetBodyIds, World } from '../../web/js/physics/rigid-body-solver/index.js';
import { runFor, makeWall, makeRobot, penetrationOf, ROBOT_HALF_M as H } from './helpers.mjs';

beforeEach(() => resetBodyIds());

const speed = (b) => Math.hypot(b.linearVelocity.x, b.linearVelocity.y);

test('driving into a wall at 1 m/s: small penetration, at rest, contact force near the traction limit, slipping', () => {
  const w = new World();
  makeWall(w, 1, 0, 0.5, 5); // face at x = 0.5
  const r = makeRobot(w, { x: 0.1, y: 0, target: { vx: 1, vy: 0, om: 0 } });
  runFor(w, 2);
  const limit = Math.min(0.7 * 15 * 9.81, 180); // N: muWheel*m*g, PLACEHOLDER muWheel 0.7 from the spec
  assert.ok(penetrationOf(w, r) < 0.012, 'penetration ' + penetrationOf(w, r));
  assert.ok(speed(r) < 0.05, 'speed ' + speed(r));
  const f = w.contactForceOf(r).magnitude;
  assert.ok(Math.abs(f - limit) / limit < 0.15, `contact force ${f} N vs ${limit} N`);
  assert.equal(r.drive.slipping, true);
});

test('free acceleration reaches the 1 m/s target within 0.5 s with traction to spare', () => {
  const w = new World();
  const r = makeRobot(w, { x: 0, y: 0, target: { vx: 1, vy: 0, om: 0 } });
  runFor(w, 0.5);
  assert.ok(Math.abs(r.linearVelocity.x - 1) < 0.05, 'vx ' + r.linearVelocity.x);
  assert.ok(r.drive.usage < 1, 'usage ' + r.drive.usage);
  assert.equal(r.drive.slipping, false);
});

// Start in contact: with a run-up, A's momentum would carry both robots even when the traction limits are equal.
function pushingMatch(massB, seconds) {
  const w = new World();
  const a = makeRobot(w, { x: 0, y: 0, target: { vx: 1, vy: 0, om: 0 } });
  const b = makeRobot(w, { x: 2 * H, y: 0, mass: massB, target: { vx: 0, vy: 0, om: 0 } });
  const x0 = b.position.x;
  runFor(w, seconds);
  return { a, b, moved: b.position.x - x0 };
}

test('pushing match: equal mass, the braking defender is moved less than 0.15 m in 2 s', () => {
  const { moved } = pushingMatch(15, 2);
  assert.ok(moved < 0.15, 'moved ' + moved + ' m');
});

test('pushing match: a 7.5 kg defender has less traction and is pushed to > 0.2 m/s by 1.5 s', () => {
  const { b } = pushingMatch(7.5, 1.5);
  assert.ok(b.linearVelocity.x > 0.2, 'speed ' + b.linearVelocity.x);
});

test('a kinematic robot pushes a dynamic one without slowing down', () => {
  const w = new World();
  const k = makeRobot(w, { x: 0, y: 0, type: 'kinematic', target: { vx: 1, vy: 0, om: 0 } });
  const d = makeRobot(w, { x: 2 * H + 0.2, y: 0, drive: false });
  runFor(w, 2);
  assert.ok(Math.abs(k.linearVelocity.x - 1) < 1e-6, 'kinematic vx ' + k.linearVelocity.x);
  assert.ok(d.position.x > 1, 'pushed body x ' + d.position.x);
  assert.ok(d.linearVelocity.x > 0.9, 'pushed body vx ' + d.linearVelocity.x);
});
