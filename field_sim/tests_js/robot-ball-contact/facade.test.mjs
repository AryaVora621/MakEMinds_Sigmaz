// Tests for the facade step() result shape and captured-ball state (spec section 6).
import assert from 'node:assert/strict';
import { createRobotBallContact } from '../../web/js/physics/robot-ball-contact/index.js';
import * as api from '../../web/js/physics/robot-ball-contact/index.js';

const H = 1 / 240;
const DIMS = { halfX: 9, halfY: 9, height: 18 };

export const tests = [
  { name: 'index re-exports the public API', fn() {
    for (const k of ['RBC_PARAMS', 'makeParams', 'mixFriction', 'buildRobotShape', 'robotContacts', 'pocketTest',
      'solveRobotBallContact', 'RobotReactionAccumulator', 'detectPinch', 'resolvePinch', 'underFrame',
      'IntakeController', 'rollerContact', 'HoldQueue', 'createRobotBallContact']) {
      assert.ok(k in api, `missing export ${k}`);
    }
  } },
  { name: 'facade step returns the sec-6 result shape; captured balls become intaken', fn() {
    const rbc = createRobotBallContact({});
    const robot = { x: 0, y: 0, h: 0, vx: 20, vy: 0, om: 0 };
    const ball = { id: 1, type: 'pollen', kind: 'pollen', state: 'free', x: 14, y: 0, z: 1.391, vx: 0, vy: 0, vz: 0, r: 1.391, massKg: 0.025 };
    let captured = null, last = null;
    for (let i = 0; i < 600 && !captured; i++) {
      robot.vx = 20; robot.x += robot.vx * H;
      last = rbc.step([ball], robot, DIMS, H, {});
      if (last.captured.length) captured = last.captured;
      ball.x += ball.vx * H; ball.y += ball.vy * H; ball.z += ball.vz * H;
      if (ball.z < ball.r) { ball.z = ball.r; if (ball.vz < 0) ball.vz = 0; }
    }
    assert.ok(Array.isArray(last.captured) && Array.isArray(last.jammed) && Array.isArray(last.blocks));
    assert.equal(typeof last.plowForceN, 'number');
    assert.ok('front' in last.stalled && 'rear' in last.stalled);
    assert.ok(['dvx', 'dvy', 'dom'].every((k) => typeof last.reaction[k] === 'number'));
    assert.ok(captured, 'ball should have been captured');
    assert.equal(captured[0][0], ball);
    assert.ok(captured[0][1] === 'front' || captured[0][1] === 'rear');
    assert.equal(ball.state, 'intaken');
    assert.equal(rbc.hold.count, 1);
    rbc.reset();
    assert.equal(rbc.hold.count, 0);
  } },
  { name: 'setRobotMass changes the reaction magnitude', fn() {
    const dv = (m) => {
      const rbc = createRobotBallContact({ params: { shape: { sides: [] } } });
      rbc.setRobotMass(m);
      const robot = { x: 0, y: 0, h: 0, vx: 40, vy: 0, om: 0 };
      const ball = { id: 1, type: 'pollen', kind: 'pollen', state: 'free', x: 9 + 1.391 - 0.01, y: 0, z: 1.391, vx: 0, vy: 0, vz: 0, r: 1.391, massKg: 0.025 };
      const res = rbc.step([ball], robot, DIMS, H, {});
      return res.reaction.dvx;
    };
    const a = dv(5), b = dv(30);
    assert.ok(a < 0 && b < 0 && Math.abs(a) > Math.abs(b), `${a} ${b}`);
  } },
];
