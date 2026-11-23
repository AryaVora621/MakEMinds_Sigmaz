// fuse.js - 20 A ATO blade fuse, Miner's-rule damage against the datasheet time-current curve.
import { FUSE } from './params.js';

// Open time [s] at constant current iA [A]; Infinity if I/rated <= noTripFrac.
export function fuseOpenTime(iA, p = FUSE) {
  const I = Math.abs(iA);
  const r = I / p.ratedA;
  if (r <= p.noTripFrac) return Infinity;
  const c = p.curve;
  let t;
  if (r < c[0][0]) {
    t = c[0][1] * (c[0][0] - p.noTripFrac) / (r - p.noTripFrac);
  } else {
    let i = 0;
    while (i < c.length - 2 && r > c[i + 1][0]) i++; // last segment extrapolates past the end
    const [r0, t0] = c[i], [r1, t1] = c[i + 1];
    const f = (Math.log(r) - Math.log(r0)) / (Math.log(r1) - Math.log(r0));
    t = Math.exp(Math.log(t0) + f * (Math.log(t1) - Math.log(t0)));
  }
  return Math.min(t, p.i2tA2s / (I * I));
}

export class Fuse {
  constructor(p = FUSE) { this.p = p; this.reset(); }
  reset() { this.state = { damage: 0, blown: false, tripT: null, t: 0 }; }
  // returns true when blown (latched)
  step(iA, dt) {
    const s = this.state, p = this.p;
    s.t += dt;
    if (!p.enabled || s.blown) return s.blown;
    const r = Math.abs(iA) / p.ratedA;
    if (r <= p.noTripFrac) s.damage *= Math.exp(-dt / p.coolTauS);
    else s.damage += dt / fuseOpenTime(iA, p);
    if (s.damage >= 1) { s.blown = true; s.tripT = s.t; }
    return s.blown;
  }
}
