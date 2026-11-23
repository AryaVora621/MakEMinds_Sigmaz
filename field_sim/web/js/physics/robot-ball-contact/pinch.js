// pinch.js - ball pinched between a robot face and a static surface, plus under-frame trap check.
// Units: in, s, kg; stiffness N/m = kg/s^2 so F[sim force kg*in/s^2] = k * delta[in] exactly.
// Constants live in params.js (RBC_PARAMS.pinch, RBC_PARAMS.trap, ball.*), tagged ASSUMED there.
import { RBC_PARAMS } from './params.js';
import { ballProps } from './mixing.js';

/**
 * Detect a pinch between the robot contact (normal robot->ball) and a static contact.
 * @param {object} ball {r, kind, vx, vy, vz}
 * @param {object} robot {vx, vy, om}
 * @param {object} robotContact {nx, ny, pen, rx, ry}
 * @param {Array<{nx:number,ny:number,nz?:number,pen:number}>} staticContacts
 * @param {number} h substep [s]
 * @returns {null|{blocked:boolean, ejected:boolean, nx:number, ny:number, depth:number, force:number, maxClosingSpeed:number, tx:number, ty:number, tsign:number}}
 */
export function detectPinch(ball, robot, robotContact, staticContacts, h, params = RBC_PARAMS) {
  if (!robotContact || !staticContacts || !staticContacts.length) return null;
  const nrx = robotContact.nx, nry = robotContact.ny;
  // pick the deepest static contact whose normal opposes the robot normal
  let s = null;
  for (const c of staticContacts) {
    if (nrx * c.nx + nry * c.ny < -params.pinch.cosOpposed && (!s || c.pen > s.pen)) s = c;
  }
  if (!s) return null;
  // robot velocity at the contact point [in/s]
  const rx = robotContact.rx || 0, ry = robotContact.ry || 0;
  const om = robot.om || 0;
  const vpx = (robot.vx || 0) - om * ry, vpy = (robot.vy || 0) + om * rx;
  const closing = Math.max(0, vpx * nrx + vpy * nry); // speed toward ball [in/s]
  const depth = robotContact.pen + s.pen + closing * h; // [in]
  const bp = ballProps(ball.kind, params);
  const r = ball.r ?? bp.radiusIn;
  const allowed = bp.maxSquishFrac * 2 * r; // [in]
  // static tangent n_s x zhat = (ny, -nx); sign follows robot tangential velocity
  const tx = s.ny, ty = -s.nx;
  const vt = vpx * tx + vpy * ty;
  const tsign = vt >= 0 ? 1 : -1;
  const out = { blocked: false, ejected: false, nx: nrx, ny: nry, depth, force: 0, maxClosingSpeed: 0, tx, ty, tsign };
  if (depth > params.pinch.overSquishEject * allowed) {
    out.ejected = true;
    return out;
  }
  out.blocked = true;
  if (depth <= allowed) {
    out.force = bp.stiffnessNpm * depth; // sim force
    out.maxClosingSpeed = Math.max(0, (allowed - depth) / (h > 0 ? h : 1));
  } else {
    out.force = bp.stiffnessNpm * allowed;
    out.maxClosingSpeed = 0;
  }
  return out;
}

/**
 * Apply eject velocity to the ball when pinch.ejected. Returns a block for
 * RobotReactionAccumulator.addBlock, or null when nothing blocks.
 * Eject: vz = max(vz, popSpeed); the ball planar speed is added along the static tangent
 * (sign follows the robot tangential velocity at the contact).
 */
export function resolvePinch(ball, robot, pinch, params = RBC_PARAMS) {
  if (!pinch) return null;
  if (pinch.ejected) {
    ball.vz = Math.max(ball.vz || 0, params.pinch.popSpeedInS);
    const sp = Math.hypot(ball.vx || 0, ball.vy || 0);
    ball.vx = (ball.vx || 0) + pinch.tsign * pinch.tx * sp;
    ball.vy = (ball.vy || 0) + pinch.tsign * pinch.ty * sp;
    return null;
  }
  if (!pinch.blocked) return null;
  return { nx: pinch.nx, ny: pinch.ny, depth: pinch.depth, force: pinch.force, maxClosingSpeed: pinch.maxClosingSpeed };
}

/**
 * Informational: ball fits under the chassis (2r < clearance [in]) with centre inside the footprint.
 * @param {{x:number,y:number,h:number}} robot @param {{halfX:number,halfY:number}} dims
 */
export function underFrame(ball, robot, dims, clearanceIn = RBC_PARAMS.trap.chassisClearanceIn) {
  const r = ball.r ?? ballProps(ball.kind).radiusIn;
  if (!(2 * r < clearanceIn)) return false;
  const c = Math.cos(robot.h || 0), s = Math.sin(robot.h || 0);
  const dx = ball.x - robot.x, dy = ball.y - robot.y;
  const lx = c * dx + s * dy, ly = -s * dx + c * dy;
  return Math.abs(lx) <= dims.halfX && Math.abs(ly) <= dims.halfY;
}
