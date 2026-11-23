// analytic.js - closed-form vertical flight under gravity + quadratic drag (a_drag = -k|v|v). g in in/s^2, k in 1/in.
export function terminalSpeed(g, k) { return Math.sqrt(g / k); }
export function apexTime(v0, g, k) { const vt = terminalSpeed(g, k); return (vt / g) * Math.atan(v0 / vt); }
export function apexHeight(v0, g, k) { const vt = terminalSpeed(g, k); return Math.log(1 + (v0 * v0) / (vt * vt)) / (2 * k); }
// Rising, valid for t <= apexTime. Returns height above start (in) and velocity (in/s).
export function risingState(v0, t, g, k) {
  const vt = terminalSpeed(g, k), p0 = Math.atan(v0 / vt), phi = p0 - g * t / vt;
  return { z: Math.log(Math.cos(phi) / Math.cos(p0)) / k, v: vt * Math.tan(phi) };
}
// Falling from rest. drop in inches, speed in in/s.
export function fallingState(t, g, k) {
  const vt = terminalSpeed(g, k), x = g * t / vt;
  return { drop: Math.log(Math.cosh(x)) / k, speed: vt * Math.tanh(x) };
}
