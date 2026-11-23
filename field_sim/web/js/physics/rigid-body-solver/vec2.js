// vec2.js - small pure 2D helpers; every vector function returns a new {x,y}.
// Rotation q = {c,s} (cos, sin). Transform xf = {p:{x,y}, q:{c,s}}.

export function v(x = 0, y = 0) { return { x, y }; }
export function add(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
export function sub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
export function scale(a, s) { return { x: a.x * s, y: a.y * s }; }
export function neg(a) { return { x: -a.x, y: -a.y }; }
export function dot(a, b) { return a.x * b.x + a.y * b.y; }
export function cross(a, b) { return a.x * b.y - a.y * b.x; }
export function crossSV(s, a) { return { x: -s * a.y, y: s * a.x }; }
export function crossVS(a, s) { return { x: s * a.y, y: -s * a.x }; }
export function lengthSq(a) { return a.x * a.x + a.y * a.y; }
export function length(a) { return Math.sqrt(a.x * a.x + a.y * a.y); }
export function normalize(a) {
  const l = Math.sqrt(a.x * a.x + a.y * a.y);
  if (l < 1e-12) return { x: 0, y: 0 };
  return { x: a.x / l, y: a.y / l };
}
export function rightPerp(a) { return { x: a.y, y: -a.x }; }
export function leftPerp(a) { return { x: -a.y, y: a.x }; }
export function makeRot(angle) { return { c: Math.cos(angle), s: Math.sin(angle) }; }
export function rotate(q, a) { return { x: q.c * a.x - q.s * a.y, y: q.s * a.x + q.c * a.y }; }
export function invRotate(q, a) { return { x: q.c * a.x + q.s * a.y, y: -q.s * a.x + q.c * a.y }; }
export function transformPoint(xf, a) {
  return { x: xf.q.c * a.x - xf.q.s * a.y + xf.p.x, y: xf.q.s * a.x + xf.q.c * a.y + xf.p.y };
}
export function invTransformPoint(xf, a) {
  const dx = a.x - xf.p.x, dy = a.y - xf.p.y;
  return { x: xf.q.c * dx + xf.q.s * dy, y: -xf.q.s * dx + xf.q.c * dy };
}
export function clamp(x, lo, hi) { return x < lo ? lo : x > hi ? hi : x; }
// Wrap to (-PI, PI].
export function wrapAngle(a) {
  const TWO_PI = 2 * Math.PI;
  let r = a % TWO_PI;
  if (r > Math.PI) r -= TWO_PI;
  else if (r <= -Math.PI) r += TWO_PI;
  return r;
}
