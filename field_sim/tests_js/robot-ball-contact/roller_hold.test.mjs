// Tests for roller, hold queue, capture and gate (spec sections 4, 6). Units: in, s, kg.
import assert from 'node:assert/strict';
import { RBC_PARAMS } from '../../web/js/physics/robot-ball-contact/params.js';
import { buildRobotShape, toLocal } from '../../web/js/physics/robot-ball-contact/robot_geometry.js';
import { IntakeController } from '../../web/js/physics/robot-ball-contact/roller.js';
import { HoldQueue } from '../../web/js/physics/robot-ball-contact/hold_queue.js';
import { createRobotBallContact } from '../../web/js/physics/robot-ball-contact/index.js';

const DIMS = { halfX: 9, halfY: 9, height: 18 };
const H = 1 / 240;
const stubReaction = { addImpulse() {} };
const mkBall = (id, kind, x, y, z, extra = {}) => {
  const r = kind === 'nectar' ? 1.7995 : 1.391;
  return { id, type: kind, kind, state: 'free', x, y, z: z ?? r, vx: 0, vy: 0, vz: 0, r, massKg: kind === 'nectar' ? 0.041 : 0.025, ...extra };
};

// Roller test: ball directly below the front roller axle (local x = 9.5, z = 4), robot at origin, at rest.
function rollerRun(rpm, kind, h, steps) {
  const robot = { x: 0, y: 0, h: 0, vx: 0, vy: 0, om: 0 };
  const shape = buildRobotShape(DIMS, RBC_PARAMS);
  const ic = new IntakeController(RBC_PARAMS, null);
  ic.setEnabled(true);
  ic.setRpm('front', rpm);
  const hold = new HoldQueue(4);
  const b = mkBall(1, kind, 9.5, 0, null);
  let minVx = Infinity, maxVx = -Infinity, maxSpeed = 0;
  for (let i = 0; i < steps; i++) {
    ic.step([b], robot, DIMS, shape, hold, stubReaction, h);
    b.x += b.vx * h; b.y += b.vy * h; b.z += b.vz * h;
    if (b.z < b.r) { b.z = b.r; if (b.vz < 0) b.vz = 0; } // floor
    minVx = Math.min(minVx, b.vx); maxVx = Math.max(maxVx, b.vx);
    maxSpeed = Math.max(maxSpeed, Math.hypot(b.vx, b.vy, b.vz));
    assert.ok(Number.isFinite(b.vx) && Number.isFinite(b.vz) && Number.isFinite(b.x), 'non-finite state');
  }
  return { b, minVx, maxVx, maxSpeed };
}

// Facade sweep: robot drives +x at 20 in/s over 6 pollen balls in a row.
function sweep() {
  const rbc = createRobotBallContact({});
  const robot = { x: 0, y: 0, h: 0, vx: 20, vy: 0, om: 0 };
  const balls = [];
  const ys = [-7, -4.2, -1.4, 1.4, 4.2, 7];
  ys.forEach((y, i) => balls.push(mkBall(i + 1, 'pollen', 16, y)));
  const jam = new Set(), capd = [];
  let minLx = Infinity;
  for (let i = 0; i < 720; i++) {
    robot.vx = 20; robot.vy = 0; robot.om = 0;
    robot.x += robot.vx * H;
    const res = rbc.step(balls, robot, DIMS, H, {});
    for (const [b] of res.captured) capd.push(b);
    for (const b of res.jammed) jam.add(b);
    for (const b of balls) {
      if (b.state !== 'free') continue;
      b.x += b.vx * H; b.y += b.vy * H; b.z += b.vz * H;
      if (b.z < b.r) { b.z = b.r; if (b.vz < 0) b.vz = 0; }
      const [lx, ly] = toLocal(robot, b.x, b.y);
      if (jam.has(b) && Math.abs(ly) < 8) minLx = Math.min(minLx, lx);
    }
  }
  return { rbc, balls, jam, capd, minLx };
}

export const tests = [
  { name: 'roller: positive rpm pulls a resting pollen ball inward (local -x)', fn() {
    const { minVx } = rollerRun(1150, 'pollen', H, 50);
    assert.ok(minVx < -1, `expected inward velocity, min vx = ${minVx}`);
  } },
  { name: 'roller: negative rpm pushes the ball outward (local +x)', fn() {
    const { maxVx } = rollerRun(-1150, 'pollen', H, 50);
    assert.ok(maxVx > 1, `expected outward velocity, max vx = ${maxVx}`);
  } },
  { name: 'roller: implicit spring stays finite and bounded at h = 0.004 with nectar', fn() {
    const { maxSpeed } = rollerRun(1150, 'nectar', 0.004, 50);
    assert.ok(maxSpeed < 500, `max speed ${maxSpeed}`);
  } },
  { name: 'hold queue: FIFO, limit 4, refuses when full', fn() {
    const q = new HoldQueue(4);
    for (let i = 1; i <= 4; i++) assert.ok(q.push(mkBall(i, 'pollen', 0, 0)));
    assert.equal(q.full, true);
    assert.equal(q.canAccept(), false);
    assert.equal(q.push(mkBall(5, 'pollen', 0, 0)), null);
    assert.equal(q.shift().id, 1);
    assert.equal(q.count, 3);
    assert.equal(q.snapshot().map((e) => e.id).join(), '2,3,4');
    q.clear();
    assert.equal(q.count, 0);
  } },
  { name: 'capture and hold: sweeping 6 balls captures exactly 4; 5th and 6th stay free, jammed, stopped by the gate', fn() {
    const { rbc, balls, jam, capd, minLx } = sweep();
    assert.equal(capd.length, 4, `captured ${capd.length}`);
    assert.equal(rbc.hold.count, 4);
    assert.equal(balls.filter((b) => b.state === 'intaken').length, 4);
    const free = balls.filter((b) => b.state === 'free');
    assert.equal(free.length, 2);
    for (const b of free) assert.ok(jam.has(b), 'free ball should have been listed as jammed');
    // gate face at local x = halfX - pocketDepth = 6; centre never deeper than 6 + r - 0.05
    assert.ok(minLx >= 6 + 1.391 - 0.05, `ball centre reached local x = ${minLx}`);
  } },
];
