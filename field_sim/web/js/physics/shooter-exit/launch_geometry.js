/* Launch geometry for the shooter-exit model: hood map, ball exit point and launch velocity.
 * Units: inches, radians (hood/elevation maps in DEGREES), seconds. Pedro frame.
 * Everything here is opt-in: with opts.model !== 'physical' (or PARAMS.geom.ENABLED_PHYSICAL false) the
 * hood map is the identity, the exit offset is zero and the exit sits at [pivotWorld, sh.exitHeight], which gives
 * exactly the numbers of shotrules forwardShot.
 * All geometry constants are ASSUMED (see params.js PARAM_SOURCES): measure AXLE_FWD_IN / AXLE_UP_IN from CAD. */

import { PARAMS } from './params.js';
import { compressionOf } from './exit_model.js';

const RAD = Math.PI / 180;

/** True when the physical exit geometry/hood map is active. opts {model, params}. */
export function geomEnabled(opts = {}) {
  const params = opts.params || PARAMS;
  return opts.model === 'physical' && !!params.geom.ENABLED_PHYSICAL;
}

function gapOf(kind, opts, params) {
  return opts.gapIn != null ? opts.gapIn : params.compression.GAP_IN[kind];
}

/** Hood actuator angle [deg] -> launch elevation [deg]: ELEV_OFFSET + ELEV_GAIN*hood - THROWDOWN*c (identity when disabled). */
export function hoodToElevDeg(hoodDeg, kind, opts = {}) {
  if (!geomEnabled(opts)) return hoodDeg;
  const params = opts.params || PARAMS, h = params.hood;
  const c = compressionOf(kind, opts.gapIn, params);
  return h.ELEV_OFFSET_DEG + h.ELEV_GAIN * hoodDeg - h.THROWDOWN_DEG_PER_UNIT_C * c;
}

/** Exact inverse of hoodToElevDeg. */
export function elevToHoodDeg(elevDeg, kind, opts = {}) {
  if (!geomEnabled(opts)) return elevDeg;
  const params = opts.params || PARAMS, h = params.hood;
  const c = compressionOf(kind, opts.gapIn, params);
  return (elevDeg - h.ELEV_OFFSET_DEG + h.THROWDOWN_DEG_PER_UNIT_C * c) / h.ELEV_GAIN;
}

/** Ball-centre path radius about the wheel axle [in]: wheel radius + gap/2. */
function armIn(kind, opts, params) {
  const rW = opts.wheelR != null ? opts.wheelR : params.wheel.R_IN;
  return rW + gapOf(kind, opts, params) / 2;
}

/** Exit point relative to the turret pivot, turret frame [in]: fwd along the aim direction, up above sh.exitHeight.
 *  The ball leaves tangent to its centre path (radius ARM) around the axle, over the top:
 *  fwd = AXLE_FWD - ARM*sin(elev), up = AXLE_UP + ARM*cos(elev) - Z_REF.
 *  AXLE_UP null = auto so that up = 0 at 45 deg for pollen at its default gap. */
export function exitOffset(elevRad, kind, opts = {}) {
  if (!geomEnabled(opts)) return { fwd: 0, up: 0 };
  const params = opts.params || PARAMS, g = params.geom;
  const arm = armIn(kind, opts, params);
  let axleUp = g.AXLE_UP_IN;
  if (axleUp == null) {
    const armPollen = armIn('pollen', { wheelR: opts.wheelR, params }, params);
    axleUp = g.Z_REF_IN - armPollen * Math.cos(Math.PI / 4);
  }
  return {
    fwd: g.AXLE_FWD_IN - arm * Math.sin(elevRad),
    up: axleUp + arm * Math.cos(elevRad) - g.Z_REF_IN,
  };
}

/** Turret pivot in the field frame: pose + R(h)*(sh.pivotX, sh.pivotY) (same as shotrules pivotWorld). */
export function pivotWorld(pose, sh) {
  const c = Math.cos(pose.h), s = Math.sin(pose.h), px = sh.pivotX || 0, py = sh.pivotY || 0;
  return [pose.x + c * px - s * py, pose.y + s * px + c * py];
}

/** World exit point [x, y, z] in inches: pivot + fwd*(cos az, sin az), sh.exitHeight + up. */
export function exitPointWorld(pose, sh, azRad, elevRad, kind, opts = {}) {
  const [px, py] = pivotWorld(pose, sh);
  if (!geomEnabled(opts)) return [px, py, sh.exitHeight];
  const o = exitOffset(elevRad, kind, opts);
  return [px + o.fwd * Math.cos(azRad), py + o.fwd * Math.sin(azRad), sh.exitHeight + o.up];
}

/** Ball launch velocity [in/s]. vBase = robot velocity + om x (exit - robot centre) + omegaTurret x (exit - pivot).
 *  robotVel {vx,vy,om} defaults to pose (as in forwardShot). turretRateDegS is the turret rate about +z [deg/s]. */
export function launchVelocityWorld({ pose, robotVel, sh, azRad, elevRad, speed, exit, turretRateDegS = 0 }) {
  const rv = robotVel || pose;
  const om = rv.om || 0, omT = (turretRateDegS || 0) * RAD;
  const [px, py] = pivotWorld(pose, sh);
  const vx0 = (rv.vx || 0) - om * (exit[1] - pose.y) - omT * (exit[1] - py);
  const vy0 = (rv.vy || 0) + om * (exit[0] - pose.x) + omT * (exit[0] - px);
  const vBase = [vx0, vy0, 0];
  const ce = Math.cos(elevRad);
  const vel = [vx0 + Math.cos(azRad) * ce * speed, vy0 + Math.sin(azRad) * ce * speed, Math.sin(elevRad) * speed];
  return { vel, vBase };
}

/** Callback for Core.solveExit: (elevRad, azAbsRad) -> exit minus pivot in the field frame {dx, dy, dz} [in]. */
export function makeExitFn(pose, sh, kind, opts = {}) {
  return (elevRad, azAbsRad) => {
    const o = exitOffset(elevRad, kind, opts);
    return { dx: o.fwd * Math.cos(azAbsRad), dy: o.fwd * Math.sin(azAbsRad), dz: o.up };
  };
}
