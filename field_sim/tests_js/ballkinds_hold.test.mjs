import test from 'node:test';
import assert from 'node:assert/strict';
import { BallWorld, BALL, MAX_HELD } from '../web/js/balls.js';

const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 };
const DIMS = { halfX: 9, halfY: 9, height: 18 };
const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
const g = DIMS.halfX + BALL.r + 1;   // centre distance from the robot centre: ball surface 1 in from the roller face

// Six intake slots, all inside the mouth width (|ly| <= halfY - 0.5) and spaced 6 in apart so none overlap at spawn.
// Slots 0-2 sit at the front roller, slots 3-5 at the rear roller.
const SLOTS = [[g, -6], [g, 0], [g, 6], [-g, -6], [-g, 0], [-g, 6]];
function layout(w, kinds) {
  kinds.forEach((k, i) => {
    const [lx, ly] = SLOTS[i];
    const c = Math.cos(robot.h), s = Math.sin(robot.h);
    const pos = [robot.x + c * lx - s * ly, robot.y + s * lx + c * ly, BALL.r];
    w.addBall(k === 'nectar' ? 'nectar_red' : 'pollen', pos, [0, 0, 0], k);
  });
}
const spin = (w, n = 120) => { for (let i = 0; i < n; i++) { w.setRobot(robot, DIMS); w.step(1 / 60); } };
const freeWorld = () => { const w = new BallWorld({ bounds }); w.setIntake({ enabled: true, space: Infinity }); return w; };

test('MAX_HELD is the single hold limit of 4 (matchrules holdLimit, G407)', () => {
  assert.equal(MAX_HELD, 4);
});

test('intake of [pollen, nectar, pollen, nectar] holds 4 with correct per-kind counts', () => {
  const w = freeWorld();
  const got = []; w.onIntake = (b) => got.push(b);
  layout(w, ['pollen', 'nectar', 'pollen', 'nectar']);
  spin(w);
  assert.equal(got.length, 4, 'taken ' + got.length);
  assert.equal(w.held.length, 4);
  assert.deepEqual(w.heldKindCounts(), { pollen: 2, nectar: 2 });
  // The hold FIFO records the same balls, in intake order, with their kinds and types.
  assert.deepEqual(w.held.map(h => h.id), got.map(b => b.id));
  assert.deepEqual(w.held.map(h => h.kind), got.map(b => b.kind));
  assert.ok(w.held.every(h => (h.kind === 'nectar') === /nectar/.test(h.type)));
});

test('a 5th ball is refused (combined hold limit) and is neither loaded nor removed from the world', () => {
  const w = freeWorld();
  const got = []; w.onIntake = (b) => got.push(b);
  layout(w, ['pollen', 'nectar', 'pollen', 'nectar', 'pollen']);
  const fifth = w.balls[4];
  spin(w);
  assert.equal(got.length, MAX_HELD, 'taken ' + got.length);
  assert.equal(w.held.length, MAX_HELD);
  assert.equal(fifth.state, 'free', 'refused ball must not be loaded');
  assert.ok(w.balls.includes(fifth), 'refused ball stays in the world');
  assert.ok(!got.includes(fifth));
});

test('hold limit is combined: 4 nectar fill it and a pollen is refused', () => {
  const w = freeWorld();
  layout(w, ['nectar', 'nectar', 'nectar', 'nectar', 'pollen']);
  const pollen = w.balls[4];
  spin(w);
  assert.equal(w.held.length, MAX_HELD);
  assert.deepEqual(w.heldKindCounts(), { pollen: 0, nectar: 4 });
  assert.equal(pollen.state, 'free');
});

test('shots dequeue in FIFO order and each launched ball keeps its kind', () => {
  const w = freeWorld();
  const got = []; w.onIntake = (b) => got.push(b);
  layout(w, ['pollen', 'nectar', 'nectar', 'pollen']);
  spin(w);
  assert.equal(got.length, 4);
  const intakeKinds = got.map(b => b.kind);

  const fired = [];
  for (let i = 0; i < 4; i++) fired.push(w.fireHeld([10, 10, BALL.r], [0, 0, 0]));
  assert.equal(w.held.length, 0);
  assert.deepEqual(w.heldKindCounts(), { pollen: 0, nectar: 0 });
  assert.equal(w.fireHeld([10, 10, BALL.r], [0, 0, 0]), null, 'nothing left to fire');

  // Shot order is the intake (FIFO) order and each launched ball keeps its kind, type and radius.
  assert.deepEqual(fired.map(b => b.kind), intakeKinds);
  assert.deepEqual(fired.map(b => b.type), intakeKinds.map(k => (k === 'nectar' ? 'nectar_red' : 'pollen')));
  assert.deepEqual(fired.map(b => b.r), intakeKinds.map(k => (k === 'nectar' ? BALL.nectarR : BALL.r)));
  assert.ok(fired.every(b => b.fired && b.state === 'free'));
});

test('dequeueHeld removes the oldest entry first', () => {
  const w = freeWorld();
  layout(w, ['nectar', 'pollen', 'pollen']);
  spin(w);
  assert.equal(w.held.length, 3, 'three balls held');
  const order = w.held.map(h => h.kind);
  assert.equal(w.dequeueHeld().kind, order[0]);
  assert.equal(w.dequeueHeld().kind, order[1]);
  assert.equal(w.held.length, 1);
  assert.deepEqual(w.heldKindCounts(), { pollen: order[2] === 'pollen' ? 1 : 0, nectar: order[2] === 'nectar' ? 1 : 0 });
});

test('a shot frees a hold slot so a refused ball can then be taken', () => {
  const w = freeWorld();
  const got = []; w.onIntake = (b) => got.push(b);
  layout(w, ['pollen', 'nectar', 'pollen', 'nectar', 'pollen']);
  const fifth = w.balls[4];
  spin(w);
  assert.equal(got.length, MAX_HELD);
  assert.equal(fifth.state, 'free');
  const out = w.fireHeld([10, 10, BALL.r], [0, 0, 0]);
  assert.ok(out && out.fired);
  assert.equal(w.held.length, MAX_HELD - 1);
  // Put the refused ball back on its slot and let the roller take it.
  const [lx, ly] = SLOTS[0];
  fifth.x = robot.x + lx; fifth.y = robot.y + ly; fifth.z = BALL.r;
  fifth.vx = fifth.vy = fifth.vz = 0; fifth.sleeping = false; fifth.sleepT = 0;
  spin(w, 60);
  assert.equal(w.held.length, MAX_HELD);
  assert.equal(w.held[MAX_HELD - 1].id, fifth.id);
  assert.equal(w.held[MAX_HELD - 1].kind, 'pollen');
});

test('dequeueHeld on an empty hold returns null and counts are zero', () => {
  const w = new BallWorld({ bounds });
  assert.equal(w.dequeueHeld(), null);
  assert.deepEqual(w.heldKindCounts(), { pollen: 0, nectar: 0 });
});

test('resetField clears the hold FIFO', () => {
  const w = freeWorld();
  layout(w, ['pollen', 'nectar']);
  spin(w);
  assert.equal(w.held.length, 2);
  w.resetField({ balls: [] });
  assert.equal(w.held.length, 0);
});
