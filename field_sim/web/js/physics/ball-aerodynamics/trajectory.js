// trajectory.js - drag/spin-aware ball flight integration for aiming (ESM, pure, no DOM).
// Units: inches, seconds, rad/s. Integrates with stepRK4 and airborne = true always.
import { AERO } from './params.js';
import { stepRK4 } from './integrator.js';
import { launchSpin } from './spin.js';

function makeState(init, model, slip) {
  const p = init.pos, v = init.vel;
  let w = init.spin;
  if (!w && model.lift && model.lift.enabled) w = launchSpin(v[0], v[1], v[2], model.radiusIn, slip);
  return {
    x: p[0], y: p[1], z: p[2], vx: v[0], vy: v[1], vz: v[2],
    wx: w ? w[0] : 0, wy: w ? w[1] : 0, wz: w ? w[2] : 0,
  };
}

// Flies init under model. Samples [t,x,y,z,vx,vy,vz] at t=0 and after every step.
// Stops at tMax, or when z < zStop while descending (vz < 0).
export function simulateFlight(init, model, opts = {}) {
  const h = opts.h ?? AERO.solver.h;             // s
  const tMax = opts.tMax ?? AERO.solver.tMax;    // s
  const zStop = opts.zStop ?? -Infinity;         // in
  const slip = opts.slip ?? AERO.lift.spinSlip;  // dimensionless
  const s = makeState(init, model, slip);
  const samples = [[0, s.x, s.y, s.z, s.vx, s.vy, s.vz]];
  let apexZ = s.z, t = 0, n = 0;
  const nMax = Math.ceil(tMax / h - 1e-9);
  while (n < nMax) {
    stepRK4(s, h, model, true);
    n++; t = n * h;
    samples.push([t, s.x, s.y, s.z, s.vx, s.vy, s.vz]);
    if (s.z > apexZ) apexZ = s.z;
    if (s.z < zStop && s.vz < 0) break;
  }
  return { samples, apexZ, tEnd: t };
}

function hermite(p0, p1, m0, m1, f) {
  const f2 = f * f, f3 = f2 * f;
  return (2 * f3 - 3 * f2 + 1) * p0 + (f3 - 2 * f2 + f) * m0 + (-2 * f3 + 3 * f2) * p1 + (f3 - f2) * m1;
}
function hermiteD(p0, p1, m0, m1, f) {
  const f2 = f * f;
  return (6 * f2 - 6 * f) * p0 + (3 * f2 - 4 * f + 1) * m0 + (-6 * f2 + 6 * f) * p1 + (3 * f2 - 2 * f) * m1;
}

// First crossing of z = zTarget (descending by default). Linear fraction, then one Hermite Newton refinement.
// Returns {t,pos,vel,apexZ} or null if no crossing before tMax.
export function crossHeight(init, model, zTarget, opts = {}) {
  const h = opts.h ?? AERO.solver.h;
  const tMax = opts.tMax ?? AERO.solver.tMax;
  const slip = opts.slip ?? AERO.lift.spinSlip;
  const descending = opts.descending ?? true;
  const s = makeState(init, model, slip);
  let apexZ = s.z;
  const nMax = Math.ceil(tMax / h - 1e-9);
  for (let n = 0; n < nMax; n++) {
    const a = [s.x, s.y, s.z, s.vx, s.vy, s.vz];
    stepRK4(s, h, model, true);
    if (s.z > apexZ) apexZ = s.z;
    const hit = descending ? (a[2] > zTarget && s.z <= zTarget) : (a[2] < zTarget && s.z >= zTarget);
    if (!hit) continue;
    const b = [s.x, s.y, s.z, s.vx, s.vy, s.vz];
    let f = (zTarget - a[2]) / (b[2] - a[2]);
    const m0 = a[5] * h, m1 = b[5] * h;
    const d = hermiteD(a[2], b[2], m0, m1, f);
    if (Math.abs(d) > 1e-12) {
      const fn = f - (hermite(a[2], b[2], m0, m1, f) - zTarget) / d;
      if (fn >= 0 && fn <= 1) f = fn;
    }
    const pos = [0, 1, 2].map(i => hermite(a[i], b[i], a[i + 3] * h, b[i + 3] * h, f));
    pos[2] = zTarget;
    const vel = [0, 1, 2].map(i => a[i + 3] + (b[i + 3] - a[i + 3]) * f);
    return { t: (n + f) * h, pos, vel, apexZ };
  }
  return null;
}
