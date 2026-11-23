// interp.js - render interpolation helpers (timestep-determinism, spec section 3).
// Render-only: results are never hashed. Units: angles in radians. No DOM, no randomness.
// Source: Fiedler "Fix Your Timestep!" https://gafferongames.com/post/fix_your_timestep/ (state = curr*alpha + prev*(1-alpha)).

const TWO_PI = 2 * Math.PI;

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

// Wrap to (-pi, pi].
export function wrapAngle(a) {
  if (!Number.isFinite(a)) return a;
  let r = a % TWO_PI;
  if (r > Math.PI) r -= TWO_PI;
  else if (r <= -Math.PI) r += TWO_PI;
  return r === 0 ? 0 : r; // normalise -0
}

// Signed shortest rotation from a to b, in (-pi, pi].
export function angleDiff(a, b) {
  return wrapAngle(b - a);
}

export function lerpAngle(a, b, t) {
  return wrapAngle(a + angleDiff(a, b) * t);
}

// {x, y, h} (h = heading, rad); other keys pass through from p1.
export function lerpPose(p0, p1, t) {
  if (!p0) return { ...p1 };
  const out = { ...p1 };
  out.x = lerp(p0.x, p1.x, t);
  out.y = lerp(p0.y, p1.y, t);
  out.h = lerpAngle(p0.h, p1.h, t);
  return out;
}

function isAngleKey(key, angleKeys) {
  if (typeof key !== 'string') return false;
  if (angleKeys.includes(key)) return true;
  return key.endsWith('h') || key.endsWith('heading') || key.endsWith('yaw');
}

function blend(prev, curr, t, angleKeys, angle) {
  if (typeof curr === 'number') {
    if (typeof prev !== 'number') return curr;
    return angle ? lerpAngle(prev, curr, t) : lerp(prev, curr, t);
  }
  if (Array.isArray(curr)) {
    const ok = Array.isArray(prev) && prev.length === curr.length;
    return curr.map((c, i) => blend(ok ? prev[i] : undefined, c, t, angleKeys, angle));
  }
  if (curr && typeof curr === 'object') {
    const out = {};
    const p = prev && typeof prev === 'object' && !Array.isArray(prev) ? prev : null;
    for (const k of Object.keys(curr)) {
      out[k] = blend(p ? p[k] : undefined, curr[k], t, angleKeys, isAngleKey(k, angleKeys));
    }
    return out;
  }
  return curr; // strings, booleans, null, undefined taken from curr
}

// New object; inputs are never mutated. prev null/undefined -> deep copy of curr.
export function lerpSnapshot(prev, curr, t, { angleKeys = [] } = {}) {
  return blend(prev, curr, t, angleKeys, false);
}
