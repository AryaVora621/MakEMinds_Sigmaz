// sensor.js - battery voltage sensor (Lynx ADC style) and the FTC voltage-compensation helper.
// SI units: V, s. Constants from params.js SENSOR ([A], ADC resolution not found).
import { SENSOR } from './params.js';

export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class VoltageSensor {
  constructor(p = SENSOR) {
    this.p = p;
    this.reset();
  }

  reset() {
    this.rng = mulberry32(this.p.seed);
    this.state = { filtered: null, last: 0 };
  }

  _gauss() {
    const u1 = Math.max(this.rng(), 1e-12), u2 = this.rng();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  }

  sample(vBus, dt) {
    const p = this.p, s = this.state;
    let v = vBus;
    if (p.lpTauS > 0) {
      if (s.filtered === null) s.filtered = vBus;
      else s.filtered += (1 - Math.exp(-dt / p.lpTauS)) * (vBus - s.filtered);
      v = s.filtered;
    } else {
      s.filtered = vBus;
    }
    if (p.noiseSigmaV > 0) v += p.noiseSigmaV * this._gauss();
    if (p.quantV > 0) v = Math.round(v / p.quantV) * p.quantV;
    s.last = v;
    return v;
  }
}

// FTC pattern: power * 12 / V, clamped to [-1, 1]. Dimensionless motor power.
export function compensatedPower(power, vMeasured, nominalV = 12) {
  const c = power * nominalV / Math.max(vMeasured, 1);
  return Math.min(1, Math.max(-1, c));
}
