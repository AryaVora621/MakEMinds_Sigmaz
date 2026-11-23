// imu.js - Pinpoint internal IMU yaw model. Units: radians, rad/s, seconds.
// Every random draw comes from the injected rng. No DOM, no external imports.
// All imu.* parameter values are [ASSUMPTION] (see params.js); this file only applies them.

export class Imu {
  constructor(params, rng) {
    this._p = params.imu;
    this._rng = rng;
    this.reset(0);
  }

  // Zero heading (or set it) and the bias random-walk state.
  reset(headingRad = 0) {
    this._heading = headingRad; // rad, unwrapped, unquantized
    this._bias = 0;             // rad/s, random-walk part only
    this._rate = 0;             // rad/s, last measured rate
  }

  // om: true yaw rate (rad/s); dt: sub-step (s).
  advance(om, dt) {
    if (!(dt > 0)) return;
    const p = this._p;
    const g = this._rng;
    this._bias += p.biasWalkRadSPerSqrtS * Math.sqrt(dt) * g.gauss();
    const meas = om * (1 + p.scaleError) + this._bias + p.biasRadS +
      p.noiseRadPerSqrtS * g.gauss() / Math.sqrt(dt);
    this._rate = meas;
    this._heading += meas * dt;
  }

  get headingRad() {
    const q = this._p.quantRad;
    return q > 0 ? Math.round(this._heading / q) * q : this._heading;
  }

  get rateRadS() { return this._rate; }
}
