// flywheel_plant.js - rigid-shaft flywheel plant: N DC motors + velocity PIDF (ZOH) + losses + shot loads.
// Pure ES module, SI units internally (rad/s, N*m, kg*m^2, s, V, A). rpm only at the API edge.
// Implements FLYWHEEL-DYNAMICS SPEC 2f. See params.js for the source tag of every constant.
import { makeFlywheelParams, validateFlywheelParams, RPM_TO_RADS, RADS_TO_RPM } from './params.js';
import { totalInertia } from './inertia.js';
import { makeDcMotor, driveTorqueAtWheelNm, batteryBusVoltage } from './motor.js';
import { coulombTorqueNm, lossTorquesNm, brakeTorqueAtWheelNm } from './losses.js';
import { deriveFeedforward, VelocityController, VelocityMeasurement } from './controller.js';
import { makeBall, predictShot } from './shot_transfer.js';

const fin = (x) => typeof x === 'number' && Number.isFinite(x);

/**
 * Flywheel plant. Construct with partial overrides of FLYWHEEL_PARAMS.
 * state.rpm is the true shaft speed [rev/min]; it is not clamped to maxRpm (only the command is).
 */
export class FlywheelPlant {
  /** @param {object} [params] partial overrides for makeFlywheelParams */
  constructor(params) {
    const p = makeFlywheelParams(params);
    const errs = validateFlywheelParams(p);
    if (errs.length) throw new Error('Invalid flywheel params: ' + errs.join('; '));
    this.params = p;
    this.motor = makeDcMotor(p.motor);
    this.inertia = totalInertia(p);
    this.jKgM2 = this.inertia.jTotalKgM2;
    this.coulombNm = coulombTorqueNm(p.losses, this.motor, p.drive);
    this.ff = deriveFeedforward(this.motor, p.drive, this.coulombNm);
    this.controller = new VelocityController(p.controller, this.ff);
    this.measurement = new VelocityMeasurement(p.controller);
    this._shotCount = 0;
    this.reset(0);
  }

  /** Reset shaft speed [rpm], filter, loads, energy, integrator and time. */
  reset(rpm = 0) {
    const w = fin(rpm) ? rpm * RPM_TO_RADS : 0;
    this._w = w;
    this._loads = [];
    this._ctrlAccum = 0;
    this._firstCtrl = true;
    this._held = { voltageV: 0, float: true, saturated: false };
    this._busV = this.params.battery.voltageV;
    this._targetRadS = 0;
    this._energy = { driveJ: 0, lossJ: 0, loadJ: 0 };
    this.controller.reset();
    this.measurement.reset(w);
    this.state = {
      timeS: 0, omegaRadS: w, rpm: w * RADS_TO_RPM, rpmMeasured: w * RADS_TO_RPM,
      targetRpm: 0, voltageV: 0, float: true, currentA: 0, busV: this._busV,
      loadTorqueNm: 0, activeLoads: 0, energy: this._energy,
    };
  }

  /** Set the commanded speed [rpm]; non-finite ignored, clamped to [0, limits.maxRpm]. */
  setTargetRpm(rpm) {
    if (!fin(rpm)) return;
    const t = Math.min(Math.max(rpm, 0), this.params.limits.maxRpm);
    this.state.targetRpm = t;
    this._targetRadS = t * RPM_TO_RADS;
  }

  /** Advance dtS [s]. Non-finite or <= 0 is a no-op. */
  step(dtS) {
    if (!fin(dtS) || dtS <= 0) return this.state;
    const p = this.params;
    const n = Math.max(1, Math.ceil(dtS / p.integrator.maxSubstepS));
    const h = dtS / n;
    const period = p.controller.periodS;
    const st = this.state;
    let w = this._w;
    let I = 0, T_drive = 0, T_load = 0;
    for (let i = 0; i < n; i++) {
      // 1. measurement filter
      const wm = this.measurement.update(h, w);
      // 2. zero-order-hold controller
      this._ctrlAccum += h;
      if (this._firstCtrl || this._ctrlAccum >= period - 1e-12) {
        this._ctrlAccum = this._firstCtrl ? 0 : this._ctrlAccum - period;
        this._firstCtrl = false;
        this._held = this.controller.compute(this._targetRadS, wm, period, this._busV);
      }
      // 3. applied voltage and drive torque
      const V = Math.min(Math.max(this._held.voltageV, -this._busV), this._busV);
      if (this._held.float) {
        T_drive = p.losses.zeroPowerBehavior === 'brake' ? brakeTorqueAtWheelNm(w, this.motor, p.drive) : 0;
        I = 0;
      } else {
        const d = driveTorqueAtWheelNm(this.motor, p.drive, V, w);
        T_drive = d.torqueNm; I = d.currentA;
      }
      // 4. shot loads
      T_load = 0;
      for (const L of this._loads) {
        T_load += L.torqueNm * Math.min(h, L.remainingS) / h;
        L.remainingS -= h;
      }
      if (this._loads.length) this._loads = this._loads.filter((L) => L.remainingS > 1e-12);
      // 5. losses and integration
      const loss = lossTorquesNm(w, { losses: p.losses, motor: this.motor, drive: p.drive, radiusM: p.wheel.radiusM });
      const T_nf = T_drive - T_load;
      let wNew, sgn, tLoss;
      if (Math.abs(w) < 1e-6 && Math.abs(T_nf) <= loss.coulombNm) {
        wNew = 0; sgn = 0; tLoss = 0;
      } else {
        sgn = Math.abs(w) >= 1e-6 ? Math.sign(w) : Math.sign(T_nf);
        tLoss = loss.totalNm;
        wNew = w + h * (T_nf - sgn * tLoss) / this.jKgM2;
        if (Math.sign(wNew) !== sgn && T_drive * sgn >= 0) wNew = 0;
      }
      // 6. energy accounting
      const wAvg = 0.5 * (w + wNew);
      this._energy.driveJ += T_drive * wAvg * h;
      this._energy.loadJ += T_load * wAvg * h;
      this._energy.lossJ += sgn * tLoss * wAvg * h;
      // 7. bus voltage and state
      this._busV = batteryBusVoltage(p.battery, I);
      w = wNew;
    }
    this._w = w;
    st.timeS += dtS;
    st.omegaRadS = w;
    st.rpm = w * RADS_TO_RPM;
    st.rpmMeasured = this.measurement.value * RADS_TO_RPM;
    st.voltageV = this._held.voltageV;
    st.float = !!this._held.float;
    st.currentA = I;
    st.busV = this._busV;
    st.loadTorqueNm = T_load;
    st.activeLoads = this._loads.length;
    return st;
  }

  /**
   * Fire a ball: the predicted wheel angular impulse is applied as a constant load over the contact time.
   * @param {string|object} [ball] ball kind or descriptor
   * @param {{massScale?:number, etaScale?:number}} [opts] multiplicative ball variation
   */
  fire(ball, opts) {
    const shot = this.params.shot;
    const b = makeBall(ball, shot);
    const ms = opts && fin(opts.massScale) ? opts.massScale : 1;
    const es = opts && fin(opts.etaScale) ? opts.etaScale : 1;
    b.massKg *= ms;
    b.etaScale *= es;
    const r = predictShot(this._w, this.jKgM2, b, shot, this.params.wheel.radiusM);
    if (r.wheelAngularImpulseNms > 0 && r.contactTimeS > 0) {
      this._loads.push({ torqueNm: r.wheelAngularImpulseNms / r.contactTimeS, remainingS: r.contactTimeS });
      this.state.activeLoads = this._loads.length;
    }
    this._shotCount += 1;
    return { ...r, shotId: this._shotCount, timeS: this.state.timeS };
  }

  /** Rotational kinetic energy 0.5*J*w^2 [J]. */
  kineticEnergyJ() {
    return 0.5 * this.jKgM2 * this._w * this._w;
  }
}
