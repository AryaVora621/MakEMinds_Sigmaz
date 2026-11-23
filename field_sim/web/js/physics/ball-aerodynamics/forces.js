// forces.js - ball accelerations (gravity, quadratic drag, Magnus lift). Units: inches, seconds, rad/s, in/s^2.
// All constants come from the model object (built in air.js); nothing is hard-coded here.
import { reynolds } from './air.js';

// Cd(Re): subcritical below reCritLo, supercritical above reCritHi, cosine blend between (values ASSUMED, see params.js).
export function cdOfRe(re, cd) {
  if (re <= cd.reCritLo) return cd.subcritical;
  if (re >= cd.reCritHi) return cd.supercritical;
  const s = (1 - Math.cos(Math.PI * (re - cd.reCritLo) / (cd.reCritHi - cd.reCritLo))) / 2;
  return cd.subcritical + (cd.supercritical - cd.subcritical) * s;
}

// CL(S) = min(clMax, S/(a*S+b)), S = r|w|/|v| dimensionless (Sawicki/Hubbard/Stronge form, ASSUMED).
export function clOfSpin(S, lift) {
  if (!(S > 0)) return 0;
  return Math.min(lift.clMax, S / (lift.a * S + lift.b));
}

// Backspin axis = normalize(v x z_hat) from horizontal v. v=+x -> [0,-1,0] (then w x v = +z, lift).
export function backspinAxis(vx, vy, vz) {
  const hs = Math.sqrt(vx * vx + vy * vy);
  if (hs < 1e-9) return [0, 0, 0];
  return [vy / hs, -vx / hs, 0];
}

// out = [ax,ay,az] in/s^2. Gravity always; drag and Magnus only when airborne.
export function accel(model, vx, vy, vz, wx, wy, wz, airborne, out) {
  let ax = 0, ay = 0, az = -model.g;
  if (airborne) {
    const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
    let k = model.kDrag;
    if (model.mode === 'physical' && model.cd.dragCrisis) {
      k = model.base * cdOfRe(reynolds(sp, model.radiusIn, model.nu), model.cd);
    }
    ax -= k * sp * vx; ay -= k * sp * vy; az -= k * sp * vz;
    const lift = model.lift;
    if (lift.enabled && sp > 1e-9) {
      const wm = Math.sqrt(wx * wx + wy * wy + wz * wz);
      if (wm > 1e-9) {
        let CL = clOfSpin(model.radiusIn * wm / sp, lift);
        if (lift.inverseMagnus) {
          const re = reynolds(sp, model.radiusIn, model.nu);
          if (re > model.cd.reCritLo && re < model.cd.reCritHi) CL = -CL;
        }
        const f = model.base * CL * sp / wm; // base*CL*|v| * (w/|w|) x v
        ax += f * (wy * vz - wz * vy);
        ay += f * (wz * vx - wx * vz);
        az += f * (wx * vy - wy * vx);
      }
    }
  }
  out[0] = ax; out[1] = ay; out[2] = az;
  return out;
}
