// Tests for reaction accumulator, pinch, plowing, under-frame (spec section 5). Units: in, s, kg.
import assert from 'node:assert/strict';
import { RBC_PARAMS } from '../../web/js/physics/robot-ball-contact/params.js';
import { RobotReactionAccumulator } from '../../web/js/physics/robot-ball-contact/reaction.js';
import { detectPinch, resolvePinch, underFrame } from '../../web/js/physics/robot-ball-contact/pinch.js';
import { createRobotBallContact } from '../../web/js/physics/robot-ball-contact/index.js';

const H = 1 / 240;
const DIMS = { halfX: 9, halfY: 9, height: 18 };
const pollen = (extra = {}) => ({ x: 0, y: 0, z: 1.391, vx: 0, vy: 0, vz: 0, r: 1.391, kind: 'pollen', massKg: 0.025, ...extra });

function plow(N) {
  const rbc = createRobotBallContact({ params: { shape: { sides: [] }, roller: { axleHeightIn: 100 } } });
  const robot = { x: 0, y: 0, h: 0, vx: 20, vy: 0, om: 0 };
  const balls = [];
  for (let i = 0; i < N; i++) {
    balls.push({ id: i, type: 'pollen', kind: 'pollen', state: 'free', x: 9 + 1.391, y: -7.25 + 2.9 * i, z: 1.391, vx: 0, vy: 0, vz: 0, r: 1.391, massKg: 0.025 });
  }
  let total = 0;
  for (let s = 0; s < 240; s++) {
    robot.vx = 20; robot.vy = 0; robot.om = 0;
    robot.x += robot.vx * H;
    const res = rbc.step(balls, robot, DIMS, H, {});
    total += res.plowForceN;
    for (const b of balls) {
      b.x += b.vx * H; b.y += b.vy * H;
      if (b.vx > 0) b.vx = Math.max(0, b.vx - 30 * H); // rollFriction-like decel, 30 in/s^2 (ASSUMED test value)
    }
  }
  return total;
}

export const tests = [
  { name: 'pinch: ball between static wall (+x) and robot face moving -x is blocked; applyTo clips closing speed', fn() {
    const robot = { x: 0, y: 0, h: Math.PI, vx: -100, vy: 0, om: 0, massKg: 15, inertia: 1000 };
    const ball = pollen({ x: 0 });
    const rc = { nx: -1, ny: 0, nz: 0, pen: 0.2, rx: 5, ry: 0, tag: 'body' };
    const st = [{ nx: 1, ny: 0, nz: 0, pen: 0.1 }];
    const p = detectPinch(ball, robot, rc, st, H, RBC_PARAMS);
    assert.ok(p && p.blocked && !p.ejected);
    assert.ok(p.maxClosingSpeed >= 0);
    const blk = resolvePinch(ball, robot, p, RBC_PARAMS);
    assert.ok(blk);
    const acc = new RobotReactionAccumulator(RBC_PARAMS);
    acc.addBlock(blk);
    acc.applyTo(robot, H);
    const closing = robot.vx * blk.nx + robot.vy * blk.ny;
    assert.ok(closing <= p.maxClosingSpeed + 1e-6, `closing ${closing} > max ${p.maxClosingSpeed}`);
    assert.ok(Math.abs(robot.vx) < 100, 'robot closing speed should be reduced');
  } },
  { name: 'pinch: over-squish ejects the ball with vz >= popSpeed', fn() {
    const robot = { x: 0, y: 0, h: 0, vx: 0, vy: 0, om: 0, massKg: 15, inertia: 1000 };
    const ball = pollen();
    const rc = { nx: -1, ny: 0, nz: 0, pen: 1.0, rx: 5, ry: 0, tag: 'body' };
    const p = detectPinch(ball, robot, rc, [{ nx: 1, ny: 0, nz: 0, pen: 0.5 }], H, RBC_PARAMS);
    assert.ok(p && p.ejected);
    resolvePinch(ball, robot, p, RBC_PARAMS);
    assert.ok(ball.vz >= RBC_PARAMS.pinch.popSpeedInS);
  } },
  { name: 'reaction accumulator: dv = J/m, plowForce from normal impulses', fn() {
    const acc = new RobotReactionAccumulator(RBC_PARAMS);
    acc.addImpulse(-1, 0, 6, 0, true, 1);
    const r = acc.result({ massKg: 15, inertia: 1500 }, H);
    assert.ok(Math.abs(r.dvx - (-1 / 15)) < 1e-12);
    assert.ok(Math.abs(r.plowForce - 1 / H) < 1e-9);
    assert.ok(Math.abs(r.plowForceN - 1 / H / RBC_PARAMS.units.N_TO_SIM) < 1e-9);
  } },
  { name: 'plow: summed push force increases with number of pushed balls (1 < 3 < 6)', fn() {
    const f1 = plow(1), f3 = plow(3), f6 = plow(6);
    assert.ok(f1 > 0, `f1=${f1}`);
    assert.ok(f3 > f1, `f3=${f3} f1=${f1}`);
    assert.ok(f6 > f3, `f6=${f6} f3=${f3}`);
  } },
  { name: 'underFrame is false for pollen and nectar at default clearance', fn() {
    const robot = { x: 0, y: 0, h: 0 };
    assert.equal(underFrame(pollen(), robot, DIMS), false);
    assert.equal(underFrame(pollen({ r: 1.7995, kind: 'nectar' }), robot, DIMS), false);
  } },
];
