// adapters.js - pure helpers between the IMU/heading estimate and the rest of the sim.
// Units: inches, radians (ccw positive, heading 0 = +x). Pedro wire format uses heading in DEGREES.
// No state, no imports beyond ./angles.js and ./params.js (DEG).

import { wrapPi, angDiff, bodyToWorld } from './angles.js';
import { DEG } from './params.js';

/**
 * Extra headingOffset (rad) to add to MecanumDrive.step so that field-centric drive rotates by the
 * ESTIMATED heading while the driver still rotates by the true state.h (spec sec. 6 step 3).
 * [D] sign convention from the spec: extra = -angDiff(hEst, hTrue).
 */
export function fieldCentricExtraOffset(hTrue, hEst) {
  const d = angDiff(hEst, hTrue);
  return d === 0 ? 0 : -d;
}

/**
 * Turret aim error. The turret solves its aim from the ESTIMATED pose (world bearing to the target from the
 * estimated turret position, commanded relative to the estimated body heading), but the shot leaves from the
 * TRUE pose, so the true world aim angle is aimEst - hEst + hTrue.
 * Returns aimErrRad (true aim minus ideal true bearing, wrapped), missLateralIn = rangeIn*sin(aimErr)
 * (lateral miss at the target, inches) and rangeIn (true turret-to-target distance, inches).
 * Exactly zero error when estPose equals truePose.
 */
export function turretAimError({ truePose, estPose, target, turretOffsetBody = { f: 0, l: 0 } }) {
  const off = turretOffsetBody;
  const oe = bodyToWorld(off.f, off.l, estPose.h);
  const ot = bodyToWorld(off.f, off.l, truePose.h);
  const aimEst = Math.atan2(target.y - (estPose.y + oe.dy), target.x - (estPose.x + oe.dx));
  const tdx = target.x - (truePose.x + ot.dx);
  const tdy = target.y - (truePose.y + ot.dy);
  const aimTrue = Math.atan2(tdy, tdx);
  const rangeIn = Math.hypot(tdx, tdy);
  // (aimEst - hEst + hTrue) - aimTrue, arranged so identical inputs give a bit-exact 0
  const aimErrRad = wrapPi(angDiff(aimEst, aimTrue) - angDiff(estPose.h, truePose.h));
  return { aimErrRad, missLateralIn: rangeIn * Math.sin(aimErrRad), rangeIn };
}

/** Sim pose (rad) -> Pedro wire pose (heading in degrees, wrapped to (-180,180]). */
export function toPedroPose(pose) {
  return { x: pose.x, y: pose.y, h: wrapPi(pose.h) / DEG };
}

/** Pedro wire pose (heading in degrees) -> sim pose (rad). */
export function fromPedroPose(p) {
  return { x: p.x, y: p.y, h: p.h * DEG };
}
