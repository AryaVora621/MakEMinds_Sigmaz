// Force-based four-wheel drivetrain (spec section 8). SI units, body frame (+x fwd, +y left, omega CCW).
import { computeNormalLoads } from './loads.js';
import { selectMu, } from './frictionlaw.js';
import { stepWheelMotor, applyCoulombTorque } from './motor.js';
import { idealWheelSpeeds } from './kinematics.js';
import { makeContactOut, solveWheelContacts } from './contactsolver.js';

export class WheelFrictionDrivetrain {
  /** @param {object} cfg DrivetrainConfig from geometry.js buildDrivetrainConfig */
  constructor(cfg) {
    this.cfg = cfg;
    this.body = { vx: 0, vy: 0, om: 0 };            // m/s, m/s, rad/s (body frame)
    this.wheelOmega = new Float64Array(4);           // rad/s
    this.muScale = cfg.friction.gripScale;           // dimensionless grip multiplier
    this._loads = new Float64Array(4);               // N
    this._muEff = new Float64Array(4);
    this._prevSlip = new Float64Array(4);            // m/s
    this._powers = new Float64Array(4);
    this._torque = new Float64Array(4);              // N*m, last motor torque at wheel
    this._aFilt = [0, 0];                            // m/s^2, filtered body acceleration
    this._out = makeContactOut();
    this._tipping = false;
    this._substeps = 0;
    this._u = new Float64Array(4);
    this._jEff = new Float64Array(4);                // kg*m^2, per-substep wheel response inertia J + h*k
  }

  reset() {
    this.body.vx = 0; this.body.vy = 0; this.body.om = 0;
    this.wheelOmega.fill(0);
    this._prevSlip.fill(0);
    this._powers.fill(0);
    this._torque.fill(0);
    this._aFilt[0] = 0; this._aFilt[1] = 0;
    this._tipping = false;
    this._substeps = 0;
  }

  setMuScale(s) { this.muScale = s; }

  setBodyVelocity(vx, vy, om, matchWheels = false) {
    this.body.vx = vx; this.body.vy = vy; this.body.om = om;
    if (matchWheels) {
      idealWheelSpeeds(this.body, this.cfg, this._u);
      for (let i = 0; i < 4; i++) this.wheelOmega[i] = this._u[i] / this.cfg.wheels[i].radiusM;
    }
  }

  getBodyVelocity() { return { vx: this.body.vx, vy: this.body.vy, om: this.body.om }; }

  /**
   * Advance dt seconds. powers: length-4 [-1,1] in FL,FR,BL,BR. opts {extForceBodyN:[fx,fy]|null, extTorqueNm}.
   * Returns {vx, vy, om, dHeading} in SI body frame at the end heading.
   */
  step(powers, dt, opts = {}) {
    const cfg = this.cfg, sv = cfg.solver, body = this.body;
    if (!(dt > 0)) return { vx: body.vx, vy: body.vy, om: body.om, dHeading: 0 };
    dt = Math.min(dt, sv.maxDtS);
    const n = Math.min(sv.maxSubsteps, Math.ceil(dt / sv.maxSubstepS));
    const h = dt / n;
    const ext = opts.extForceBodyN || null;
    const fx = ext ? ext[0] : 0, fy = ext ? ext[1] : 0, tq = opts.extTorqueNm || 0;
    const M = cfg.massKg, I = cfg.inertiaKgM2;
    const tau = sv.accelFilterTauS;
    const kf = Math.min(1, h / tau);
    let dHeading = 0;
    for (let i = 0; i < 4; i++) this._powers[i] = powers[i] || 0;
    this._substeps = n;

    for (let s = 0; s < n; s++) {
      // a. normal loads
      const ld = computeNormalLoads(cfg, this._aFilt, this._loads);
      const loads = ld.loads;
      this._tipping = ld.tipping;
      // b + c. friction coefficient and motor
      for (let i = 0; i < 4; i++) {
        const w = cfg.wheels[i];
        this._muEff[i] = selectMu(this._prevSlip[i], w, cfg.friction, this.muScale);
        const r = stepWheelMotor(this.wheelOmega[i], this._powers[i], h, cfg.motor, w.inertiaKgM2);
        let om_i = r.omega;
        // backward-Euler response inertia (J + h*k), see motor.js stepWheelMotor
        this._jEff[i] = r.inertiaEffKgM2;
        if (w.crrDrive > 0) om_i = applyCoulombTorque(om_i, w.crrDrive * loads[i] * w.radiusM, h, r.inertiaEffKgM2);
        this.wheelOmega[i] = om_i;
        this._torque[i] = r.torqueNm;
      }
      // d. external impulse
      body.vx += fx * h / M;
      body.vy += fy * h / M;
      body.om += tq * h / I;
      // e. contact solve
      const vx0 = body.vx, vy0 = body.vy;
      solveWheelContacts(body, this.wheelOmega, loads, this._muEff, cfg, h, this._out, this._jEff);
      // f. rotating-frame correction (express in the body frame at the end of the substep)
      const th = body.om * h;
      const c = Math.cos(th), sn = Math.sin(th);
      const nvx = c * body.vx + sn * body.vy;
      const nvy = -sn * body.vx + c * body.vy;
      body.vx = nvx; body.vy = nvy;
      dHeading += th;
      // g. filtered acceleration (previous velocity rotated into the same frame)
      const ovx = c * vx0 + sn * vy0, ovy = -sn * vx0 + c * vy0;
      this._aFilt[0] += ((nvx - ovx) / h - this._aFilt[0]) * kf;
      this._aFilt[1] += ((nvy - ovy) / h - this._aFilt[1]) * kf;
      // h. remember slip for the next Stribeck selection
      for (let i = 0; i < 4; i++) this._prevSlip[i] = this._out.slip[i];
    }
    return { vx: body.vx, vy: body.vy, om: body.om, dHeading };
  }

  getTelemetry() {
    const o = this._out;
    const wheels = [];
    for (let i = 0; i < 4; i++) {
      wheels.push({
        name: this.cfg.wheels[i].name,
        normalN: this._loads[i],
        slipMS: o.slip[i],
        slipRatio: o.slipRatio[i],
        gripUsed: o.gripUsed[i],
        saturated: o.saturated[i] === 1,
        omegaRadS: this.wheelOmega[i],
        motorTorqueNm: this._torque[i],
        power: this._powers[i],
        forceN: [o.forceN[i][0], o.forceN[i][1]],
      });
    }
    return { wheels, accelBody: [this._aFilt[0], this._aFilt[1]], tipping: this._tipping, substeps: this._substeps };
  }
}
