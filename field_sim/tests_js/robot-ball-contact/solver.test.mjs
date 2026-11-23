// Tests for params/mixing/solver (spec sections 1, 3). Units: in, s, kg. Run via run_all.mjs.
import assert from 'node:assert/strict';
import { RBC_PARAMS, makeParams } from '../../web/js/physics/robot-ball-contact/params.js';
import { mixFriction, mixRestitution, rollerSurfaceSpeed, robotInertia } from '../../web/js/physics/robot-ball-contact/mixing.js';
import { buildRobotShape, robotContacts } from '../../web/js/physics/robot-ball-contact/robot_geometry.js';
import { solveRobotBallContact } from '../../web/js/physics/robot-ball-contact/solver.js';

const near = (a, b, tol = 1e-6, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} ${a} vs ${b} (tol ${tol})`);

const DIMS = { halfX: 9, halfY: 9, height: 18 };
const R = 1.391;     // pollen radius, in
const MB = 0.025;    // pollen mass, kg
const MR = 15;       // robot mass, kg

// A pollen ball at rest touching the body face (x = halfX - pocketDepth = 6) of a robot at the origin, h = 0.
function faceSetup(vRobot) {
  const robot = { x: 0, y: 0, h: 0, vx: vRobot, vy: 0, om: 0, massKg: MR, inertia: robotInertia(MR, 9, 9) };
  const ball = { x: 6 + R - 0.01, y: 0, z: R, vx: 0, vy: 0, vz: 0, r: R, kind: 'pollen', massKg: MB };
  const shape = buildRobotShape(DIMS, RBC_PARAMS);
  const cs = robotContacts(ball, robot, shape, 2);
  assert.ok(cs.length >= 1, 'expected a contact with the front body face');
  assert.equal(cs[0].tag, 'body');
  return { robot, ball, c: cs[0] };
}

export const tests = [
  { name: 'legacy parity: infinite-mass robot, ball leaves at 1.35*40 = 54 in/s', fn() {
    const p = makeParams({ robot: { legacyInfiniteMass: true } });
    const { robot, ball, c } = faceSetup(40);
    const J = solveRobotBallContact(ball, robot, c, p);
    near(ball.vx, 54, 1e-6);
    assert.ok(J.impact);
    assert.equal(ball._c, true);
  } },
  { name: 'finite mass: vb = (1+e)mR v/(mR+mb), robot dv = -(1+e) mb v/(mR+mb), momentum conserved', fn() {
    const v = 40, e = 0.35;
    const { robot, ball, c } = faceSetup(v);
    const J = solveRobotBallContact(ball, robot, c, RBC_PARAMS);
    near(ball.vx, (1 + e) * MR * v / (MR + MB), 1e-6, 'ball vx');
    const dv = -J.jx / MR; // reaction on robot is -J
    near(dv, -(1 + e) * MB * v / (MR + MB), 1e-6, 'robot dv');
    near(MB * ball.vx + MR * (v + dv), MR * v, 1e-6, 'momentum');
  } },
  { name: 'below restitution threshold (5 in/s): eEff = 0, vb = mR v/(mR+mb)', fn() {
    const v = 5;
    const { robot, ball, c } = faceSetup(v);
    const J = solveRobotBallContact(ball, robot, c, RBC_PARAMS);
    assert.equal(J.impact, false);
    near(ball.vx, MR * v / (MR + MB), 1e-6);
  } },
  { name: 'friction cone |jt| <= mu*jn on an oblique hit; mu = sqrt(0.5*0.5)', fn() {
    const robot = { x: 0, y: 0, h: 0, vx: 40, vy: 0, om: 0, massKg: MR, inertia: robotInertia(MR, 9, 9) };
    const a = Math.PI / 6;
    const ball = { x: 0, y: 0, z: R, vx: 0, vy: 0, vz: 0, r: R, kind: 'pollen', massKg: MB };
    const c = { nx: Math.cos(a), ny: Math.sin(a), nz: 0, pen: 0, rx: 6, ry: 0, tag: 'body' };
    const J = solveRobotBallContact(ball, robot, c, RBC_PARAMS);
    assert.ok(J.jn > 0);
    assert.ok(Math.abs(J.jt) > 0, 'oblique hit should produce friction');
    assert.ok(Math.abs(J.jt) <= 0.5 * J.jn + 1e-12, `|jt|=${Math.abs(J.jt)} mu*jn=${0.5 * J.jn}`);
  } },
  { name: 'yawing robot throws the ball tangentially with the sign of om x r', fn() {
    for (const om of [2, -2]) {
      const robot = { x: 0, y: 0, h: 0, vx: 0, vy: 0, om, massKg: MR, inertia: robotInertia(MR, 9, 9) };
      const ball = { x: 0, y: 0, z: R, vx: 0, vy: 0, vz: 0, r: R, kind: 'pollen', massKg: MB };
      // lever (6, -3*sgn): point velocity = (-om*ry, om*rx) = (3*|om|, 6*om); its +x part drives into the ball (n = +x)
      const sgn = Math.sign(om);
      const c = { nx: 1, ny: 0, nz: 0, pen: 0, rx: 6, ry: -3 * sgn, tag: 'body' };
      solveRobotBallContact(ball, robot, c, RBC_PARAMS);
      // (om x r)_y = om*rx
      assert.ok(ball.vy * (om * 6) > 0, `vy=${ball.vy} should have sign of om*rx=${om * 6}`);
    }
  } },
  { name: 'mixing and helpers', fn() {
    near(mixFriction(0.5, 0.5), 0.5, 1e-12);
    near(mixRestitution(0.2, 0.35), 0.35, 1e-12);
    near(rollerSurfaceSpeed(1150, 2.8346), 170.7, 0.1);
    near(robotInertia(15, 9, 9), 15 * (18 ** 2 + 18 ** 2) / 12, 1e-12);
  } },
];
