import test, { beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  resetBodyIds, createFieldWorld, addRobot, setRobotCommand, readRobot, teleportRobot, stepWorld, IN_TO_M,
} from '../../web/js/physics/rigid-body-solver/index.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const colliders = JSON.parse(fs.readFileSync(path.join(here, '..', '..', 'web', 'data', 'colliders.json'), 'utf8'));
const B = colliders.bounds;

beforeEach(() => resetBodyIds());

function expectedStaticCount() {
  const zcap = colliders.z_cap || 18; // params.field.zCapIn
  const keep = (o) => !(o.zMin >= zcap || o.zMax <= 0);
  return (colliders.boxes || []).filter(keep).length + (colliders.circles || []).filter(keep).length
    + (colliders.polygons || []).filter(keep).length + 4;
}

test('createFieldWorld builds statics for every filtered collider plus 4 walls', () => {
  const fw = createFieldWorld(colliders);
  assert.equal(fw.statics.length, expectedStaticCount());
  assert.ok(fw.statics.every((b) => b.type === 'static'));
  assert.equal(fw.world.bodies.length, fw.statics.length);
});

test('IN_TO_M is exactly 0.0254', () => {
  assert.equal(IN_TO_M, 0.0254);
});

test('robot driven +x at 60 in/s for 4 s stays inside the bounds and reports a push-back normal', () => {
  const fw = createFieldWorld(colliders);
  const robot = addRobot(fw.world, { x: 72, y: 72, h: 0 });
  let sawHit = null;
  for (let i = 0; i < 240; i++) {
    setRobotCommand(robot, { vx: 60, vy: 0, om: 0 });
    stepWorld(fw.world, 1 / 60);
    const r = readRobot(fw.world, robot);
    if (r.hit && !sawHit) sawHit = r;
  }
  const r = readRobot(fw.world, robot);
  const half = 9; // in, default robot half size
  assert.ok(r.x > 72 && r.x < 144, 'x in inches ' + r.x);
  assert.ok(r.x + half <= B.maxX + 0.5 && r.x - half >= B.minX - 0.5);
  assert.ok(r.y + half <= B.maxY + 0.5 && r.y - half >= B.minY - 0.5);
  assert.equal(r.hit, true);
  assert.ok(r.normals.length >= 1);
  assert.ok(r.normals.some(([nx]) => nx < -0.9), 'normals ' + JSON.stringify(r.normals));
  assert.ok(Math.abs(r.h) <= Math.PI);
});

test('setRobotCommand rotates a field-frame command into the body frame (+y left)', () => {
  const fw = createFieldWorld(colliders);
  const robot = addRobot(fw.world, { x: 72, y: 72, h: Math.PI / 2 });
  setRobotCommand(robot, { vx: 10, vy: 0, om: 0.5 });
  // Facing +y field: body +x = field +y, body +y (left) = field -x. Field +x = body -y.
  assert.ok(Math.abs(robot.drive.target.vx) < 1e-9);
  assert.ok(Math.abs(robot.drive.target.vy - (-10 * IN_TO_M)) < 1e-9);
  assert.equal(robot.drive.target.om, 0.5);

  setRobotCommand(robot, { vx: 0, vy: 10, om: 0 }); // field +y = body +x
  assert.ok(Math.abs(robot.drive.target.vx - 10 * IN_TO_M) < 1e-9);
  assert.ok(Math.abs(robot.drive.target.vy) < 1e-9);
});

test('readRobot returns inches in the field frame', () => {
  const fw = createFieldWorld(colliders);
  const robot = addRobot(fw.world, { x: 30, y: 40, h: 1 });
  const r = readRobot(fw.world, robot);
  assert.ok(Math.abs(r.x - 30) < 1e-9 && Math.abs(r.y - 40) < 1e-9 && Math.abs(r.h - 1) < 1e-9);
  assert.ok(Math.abs(robot.position.x - 30 * IN_TO_M) < 1e-12);
  assert.equal(r.hit, false);
  assert.deepEqual(r.normals, []);
});

test('teleportRobot moves the robot and zeroes its velocity', () => {
  const fw = createFieldWorld(colliders);
  const robot = addRobot(fw.world, { x: 30, y: 30, h: 0 });
  setRobotCommand(robot, { vx: 40, vy: 10, om: 1 });
  for (let i = 0; i < 20; i++) stepWorld(fw.world, 1 / 60);
  assert.ok(Math.hypot(robot.linearVelocity.x, robot.linearVelocity.y) > 0.1);
  teleportRobot(robot, { x: 60, y: 20, h: 0.5 });
  const r = readRobot(fw.world, robot);
  assert.ok(Math.abs(r.x - 60) < 1e-9 && Math.abs(r.y - 20) < 1e-9 && Math.abs(r.h - 0.5) < 1e-9);
  assert.equal(robot.linearVelocity.x, 0);
  assert.equal(robot.linearVelocity.y, 0);
  assert.equal(robot.angularVelocity, 0);
});
