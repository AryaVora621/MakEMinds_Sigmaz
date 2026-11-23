// angles.js - angle and SE(2) helpers. Radians, ccw positive, heading 0 = +x. Inches for lengths.

const PI = Math.PI;
const TWO_PI = 2 * Math.PI;

/** Wrap to (-pi, pi]. Values already in range are returned unchanged (bit-exact). wrapPi(-pi) === pi. */
export function wrapPi(a) {
  if (a > -PI && a <= PI) return a;
  let r = a - TWO_PI * Math.floor((a + PI) / TWO_PI); // [-pi, pi)
  if (r <= -PI) r += TWO_PI;
  return r;
}

/** wrapPi(a - b). */
export function angDiff(a, b) {
  return wrapPi(a - b);
}

/** a + 2*pi*k, with integer k chosen so the result is closest to prev. */
export function unwrapNear(prev, a) {
  return a + TWO_PI * Math.round((prev - a) / TWO_PI);
}

/**
 * SE(2) exponential step. Body deltas df (forward), dl (left), heading change dh,
 * applied at pose {x,y,h}. Returned h = h + dh (NOT wrapped).
 */
export function se2Exp(pose, df, dl, dh) {
  let dxr, dyr;
  if (Math.abs(dh) < 1e-9) {
    dxr = df;
    dyr = dl;
  } else {
    const A = Math.sin(dh) / dh;
    const B = (1 - Math.cos(dh)) / dh;
    dxr = A * df - B * dl;
    dyr = B * df + A * dl;
  }
  const c = Math.cos(pose.h);
  const s = Math.sin(pose.h);
  return {
    x: pose.x + dxr * c - dyr * s,
    y: pose.y + dxr * s + dyr * c,
    h: pose.h + dh,
  };
}

/** World-frame delta -> body frame (f forward, l left) at heading h. */
export function worldToBody(dx, dy, h) {
  const c = Math.cos(h);
  const s = Math.sin(h);
  return { f: dx * c + dy * s, l: -dx * s + dy * c };
}

/** Body-frame delta -> world frame at heading h. */
export function bodyToWorld(f, l, h) {
  const c = Math.cos(h);
  const s = Math.sin(h);
  return { dx: f * c - l * s, dy: f * s + l * c };
}

/** Round v to a multiple of lsb. lsb <= 0 returns v unchanged. */
export function quantize(v, lsb) {
  if (!(lsb > 0)) return v;
  return Math.round(v / lsb) * lsb;
}
