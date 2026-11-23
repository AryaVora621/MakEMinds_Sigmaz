// kinematics.js - pod geometry and Pinpoint pose reconstruction. Pure ES module, no DOM.
// Units: inches, radians, seconds (pinpointBodyDelta/poseExp only need consistent length units, mm is fine).
// Robot frame: +x forward, +y left, heading 0 = +x field, CCW positive.

const TWO_PI = 2 * Math.PI;

/** Wrap angle to (-pi, pi]. */
export function wrapAngle(a) {
  let r = a % TWO_PI;
  if (r <= -Math.PI) r += TWO_PI;
  else if (r > Math.PI) r -= TWO_PI;
  return r;
}

/** World-frame velocity -> robot body frame (vf forward, vl left). */
export function worldToBody(vx, vy, h) {
  const c = Math.cos(h), s = Math.sin(h);
  return { vf: vx * c + vy * s, vl: -vx * s + vy * c };
}

/** Robot body frame -> world frame. */
export function bodyToWorld(vf, vl, h) {
  const c = Math.cos(h), s = Math.sin(h);
  return { vx: vf * c - vl * s, vy: vf * s + vl * c };
}

/** Unit sensing axis of a pod in the robot frame. Forward pod: (cos a, sin a). Strafe pod: (-sin a, cos a). */
export function podAxis(pod, kind) {
  const a = (pod && pod.angleRad) || 0;
  const c = Math.cos(a), s = Math.sin(a);
  return kind === 'strafe' ? { ax: -s, ay: c } : { ax: c, ay: s };
}

/** Ground rate (in/s) of the pod contact point along the pod axis. pod = {x, y, angleRad} (in, robot frame). */
export function podGroundRate(pod, kind, vf, vl, om) {
  const { ax, ay } = podAxis(pod, kind);
  const vcx = vf - om * pod.y;
  const vcy = vl + om * pod.x;
  return ax * vcx + ay * vcy;
}

/**
 * Pinpoint body-frame displacement from pod deltas and heading delta.
 * fwdPodY = configured sideways (y) position of the forward pod; strPodX = configured forward (x) position of the strafe pod.
 * dxB = dF + Yf*dth ; dyB = dS - Xs*dth. Units follow the inputs.
 */
export function pinpointBodyDelta(dF, dS, dth, fwdPodY, strPodX) {
  return { dxB: dF + fwdPodY * dth, dyB: dS - strPodX * dth };
}

/** Constant-curvature pose exponential. Returns a NEW {x, y, h} with h wrapped. */
export function poseExp(pose, dxB, dyB, dth) {
  let A, B;
  if (Math.abs(dth) < 1e-6) {
    A = 1 - (dth * dth) / 6;
    B = dth / 2 - (dth * dth * dth) / 24;
  } else {
    A = Math.sin(dth) / dth;
    B = (1 - Math.cos(dth)) / dth;
  }
  const lx = A * dxB - B * dyB;
  const ly = B * dxB + A * dyB;
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  return {
    x: pose.x + lx * c - ly * s,
    y: pose.y + lx * s + ly * c,
    h: wrapAngle(pose.h + dth),
  };
}

/** Error of belief relative to truth (belief - truth). dh wrapped. */
export function poseError(truth, belief) {
  const dx = belief.x - truth.x;
  const dy = belief.y - truth.y;
  return { dx, dy, dh: wrapAngle(belief.h - truth.h), dist: Math.hypot(dx, dy) };
}
