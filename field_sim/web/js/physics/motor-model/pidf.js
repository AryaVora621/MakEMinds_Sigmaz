// pidf.js - velocity PIDF in power units, run at a fixed control period and held in between.
// Units: target/measured in ticks/s, output = motor power in [-1,1].
export class VelocityPIDF {
  constructor({ kP, kI = 0, kD = 0, kF = 0, iClamp = 1, periodS = 0.02, voltageCompensation = false, nominalV = 12 }) {
    Object.assign(this, { kP, kI, kD, kF, iClamp, periodS, voltageCompensation, nominalV });
    this.reset();
  }
  update(dtS, target, measured, busV = 12) {
    this._acc += dtS;
    if (this._acc + 1e-12 < this.periodS) return this._out;
    const T = this._acc;
    this._acc = 0;
    const e = target - measured;
    this._I += e * T;
    if (this.kI !== 0) {
      const lim = this.iClamp / Math.abs(this.kI);
      this._I = Math.max(-lim, Math.min(lim, this._I));
    }
    const D = this._hasPrev ? (e - this._ePrev) / T : 0;
    this._ePrev = e; this._hasPrev = true;
    let out = this.kF * target + this.kP * e + this.kI * this._I + this.kD * D;
    if (this.voltageCompensation && busV > 0) out *= this.nominalV / busV;
    this._out = Math.max(-1, Math.min(1, out));
    return this._out;
  }
  get output() { return this._out; }
  reset() { this._acc = 0; this._I = 0; this._ePrev = 0; this._hasPrev = false; this._out = 0; }
}
