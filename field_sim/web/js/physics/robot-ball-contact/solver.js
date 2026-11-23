// solver.js - ball vs robot impulse (sequential-impulse form, Box2D/dyn4j style) with finite robot mass.
// Units: in, s, kg, impulse kg*in/s, sim force kg*in/s^2. The solver mutates ONLY the ball; the robot
// reaction is the negative of the returned impulse (caller gives it to the reaction accumulator).
// Sources: impulse form Box2D contact_solver.c (COMPUTED/SOURCED in spec); material mixing in mixing.js.
import { RBC_PARAMS } from './params.js';
import { mixFriction, mixRestitution, effectiveRestitution, ballProps } from './mixing.js';

/** Planar velocity [vx, vy] (in/s) of the robot point at lever (rx, ry) in from the robot centre. */
export function robotPointVelocity(robot, rx, ry) {
  const om = robot.om || 0;
  return [(robot.vx || 0) - om * ry, (robot.vy || 0) + om * rx];
}

/**
 * Resolve one ball-robot contact. contact = {nx,ny,nz,pen,rx,ry,...} (normal robot -> ball).
 * Returns impulse ON THE BALL: { jx, jy, jz, jn, jt, rx, ry, impact, vn }.
 */
export function solveRobotBallContact(ball, robot, contact, params = RBC_PARAMS) {
  const { nx, ny, nz, pen, rx, ry } = contact;
  const props = ballProps(ball.kind, params);
  const r = ball.r;
  const mb = ball.massKg ?? props.massKg;
  const kappa = props.inertiaFactor;
  const Ib = kappa * mb * r * r;

  // 1. positional projection (robot does not move positionally)
  if (pen > 0) { ball.x += nx * pen; ball.y += ny * pen; ball.z += nz * pen; }
  ball._c = true;

  const zero = { jx: 0, jy: 0, jz: 0, jn: 0, jt: 0, rx, ry, impact: false, vn: 0 };

  const e = mixRestitution(ball.e ?? props.restitution, params.robot.restitution);
  const mu = mixFriction(ball.mu ?? props.friction, params.robot.friction);

  // 2. contact-point velocities
  const spin = typeof ball.wx === 'number';
  const wx = spin ? ball.wx : 0, wy = spin ? (ball.wy || 0) : 0, wz = spin ? (ball.wz || 0) : 0;
  const bx = -r * nx, by = -r * ny, bz = -r * nz; // lever from ball centre to contact point
  const vbcx = ball.vx + (wy * bz - wz * by);
  const vbcy = ball.vy + (wz * bx - wx * bz);
  const vbcz = ball.vz + (wx * by - wy * bx);
  const [vRx, vRy] = robotPointVelocity(robot, rx, ry);
  const vrx = vbcx - vRx, vry = vbcy - vRy, vrz = vbcz;
  const vn = vrx * nx + vry * ny + vrz * nz;
  zero.vn = vn;
  if (vn >= 0) return zero;

  // 4. robot inverse terms
  const legacy = !!params.robot.legacyInfiniteMass;
  const mR = robot.massKg ?? params.robot.massKg;
  const invM = (legacy || !Number.isFinite(mR) || mR <= 0) ? 0 : 1 / mR;
  let IR = robot.inertia;
  if (!Number.isFinite(IR) && Number.isFinite(mR) && Number.isFinite(robot.halfX) && Number.isFinite(robot.halfY)) {
    IR = mR * ((2 * robot.halfX) ** 2 + (2 * robot.halfY) ** 2) / 12;
  }
  const invI = (legacy || !Number.isFinite(IR) || IR <= 0) ? 0 : 1 / IR;

  const rn = rx * ny - ry * nx;
  const Kn = 1 / mb + invM * (nx * nx + ny * ny) + invI * rn * rn;
  const eEff = effectiveRestitution(e, -vn, params.contact.restitutionThresholdInS);
  const Jn = Math.max(0, -(1 + eEff) * vn / Kn);

  // 5. Coulomb friction (impulse along t opposes slip)
  let Jt = 0, tx = 0, ty = 0, tz = 0;
  const vtx = vrx - vn * nx, vty = vry - vn * ny, vtz = vrz - vn * nz;
  const vtm = Math.hypot(vtx, vty, vtz);
  if (vtm > 1e-9) {
    tx = vtx / vtm; ty = vty / vtm; tz = vtz / vtm;
    const rt = rx * ty - ry * tx;
    const Kt = (1 + 1 / kappa) / mb + invM * (tx * tx + ty * ty) + invI * rt * rt;
    Jt = -Math.min(vtm / Kt, mu * Jn);
  }

  // 6. apply to ball
  const jx = Jn * nx + Jt * tx, jy = Jn * ny + Jt * ty, jz = Jn * nz + Jt * tz;
  ball.vx += jx / mb; ball.vy += jy / mb; ball.vz += jz / mb;
  if (spin) {
    ball.wx = wx + (by * jz - bz * jy) / Ib;
    ball.wy = wy + (bz * jx - bx * jz) / Ib;
    ball.wz = wz + (bx * jy - by * jx) / Ib;
  }
  return { jx, jy, jz, jn: Jn, jt: Jt, rx, ry, impact: eEff > 0, vn };
}
