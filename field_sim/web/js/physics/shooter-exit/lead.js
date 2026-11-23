// lead.js - latency look-ahead and shoot-on-the-move helpers (spec section 5). Pure ES module, no DOM.
// Units: inches, seconds, radians (turret feedforward in deg and deg/s).
// Sources: constant-twist arc is exact kinematics. CMD_LATENCY_S and FEED_DELAY_S are ASSUMED (see params.js
// PARAM_SOURCES; measure with high-speed video or encoder timestamps).
import { PARAMS } from './params.js';

const RAD2DEG = 180 / Math.PI;

/** Seconds from "fire decided" to "ball leaves the wheels": command latency + feed delay + wheel contact time. */
export function releaseLeadTime({
  cmdLatencyS = PARAMS.timing.CMD_LATENCY_S,
  feedDelayS = PARAMS.timing.FEED_DELAY_S,
  contactTimeS = 0,
} = {}) {
  return cmdLatencyS + feedDelayS + contactTimeS;
}

/** Pose after t seconds of a constant robot-frame twist. pose {x,y,h} (in, rad); vel {vx,vy,om} is the FIELD-frame
 *  velocity at t = 0 (in/s, rad/s); it rotates with the heading as the robot turns. Optional accel {ax,ay} (in/s^2,
 *  field frame) adds 0.5*a*t^2 to position and a*t to velocity. */
export function predictPose(pose, vel, t, accel) {
  const vx = vel.vx || 0, vy = vel.vy || 0, om = vel.om || 0;
  let x, y, nvx, nvy;
  if (Math.abs(om) < 1e-6) {
    x = pose.x + vx * t; y = pose.y + vy * t; nvx = vx; nvy = vy;
  } else {
    const th = om * t, s = Math.sin(th), c = Math.cos(th);
    x = pose.x + (vx * s - vy * (1 - c)) / om;
    y = pose.y + (vy * s + vx * (1 - c)) / om;
    nvx = vx * c - vy * s; nvy = vx * s + vy * c;
  }
  const h = pose.h + om * t;
  if (accel) {
    const ax = accel.ax || 0, ay = accel.ay || 0;
    x += 0.5 * ax * t * t; y += 0.5 * ay * t * t;
    nvx += ax * t; nvy += ay * t;
  }
  return { x, y, h, vx: nvx, vy: nvy, om };
}

/** Turret feedforward to hold field aim while the chassis yaws at om (rad/s): the solver already aims from the predicted
 *  pose, so the angle is unchanged and only the rate term is added. */
export function turretFeedforward(turretDegSol, omRadS) {
  return { turretDeg: turretDegSol, turretRateDegS: 0 - omRadS * RAD2DEG };
}
