// gyro.js - single-axis (yaw) gyro error model. Pure ES module, no DOM.
// Units: angles rad, rates rad/s, time s, acceleration in g (1 g = 386.09 in/s^2).
// Model (spec section 2): omega_s = (1+eps)*omega + b0 + b_gm + b_rw + tempCoeff*dT + gSens*aG + noise,
// then clip to full scale, quantize to the rate LSB, integrate. All error magnitudes come from the ImuConfig
// built by params.js resolveImuConfig() (internal units); sources are tagged in params.js. Nothing is hardcoded
// here except the numeric guards below.
// Randomness: only the Rng passed in. Draw order is fixed: at reset b0, scaleErr, scaleAsym, b_gm;
// per substep w1 (gm), w2 (rw), w3 (white); at calibration end one draw (b0); per shock one draw.
import { wrapPi, quantize } from './angles.js';

const IN_PER_M = 39.37;        // [D] inches per metre (unit conversion)
const MIN_HISTORY_S = 1.0;     // [spec] ring buffer covers >= 1 s of sensor time
const MAX_SUBSTEPS = 64;       // [spec] clamp on substeps per advance()

export class GyroModel {
  constructor(config, rng) {
    this.cfg = config;
    this.rng = rng;
    this._status = 'NOT_READY';
    this._clearState();
  }

  _clearState() {
    this.tSensor = 0;       // s, internal sensor clock
    this.tOn = 0;           // s, time since power-on (temperature ramp)
    this.b0 = 0;            // rad/s, residual bias after calibration
    this.bGm = 0;           // rad/s, Gauss-Markov bias state
    this.bRw = 0;           // rad/s, random-walk bias state
    this.scaleErr = 0;
    this.scaleAsym = 0;
    this.tempDeltaK = 0;    // K
    this.satEvents = 0;
    this.totalMeas = 0;     // rad, integrated measured angle (no offset)
    this.totalTrue = 0;     // rad, integrated true angle
    this.yawOffset = 0;     // rad
    this.frozenTotal = 0;   // rad, reported total while CALIBRATING
    this.calibElapsed = 0;  // s
    this.calibSum = 0;      // rad (integral of true rate over the window)
    this.hT = [];           // history times (s)
    this.hV = [];           // history totalMeas (rad)
  }

  reset(t = 0) {
    const c = this.cfg, r = this.rng;
    this._clearState();
    this.tSensor = t;
    this.b0 = c.biasInitSigmaRad * r.gauss();
    this.scaleErr = c.scaleErrSigma * r.gauss();
    this.scaleAsym = c.scaleAsymSigma * r.gauss();
    this.bGm = c.gmSigmaRad * r.gauss();
    this.hT.push(t);
    this.hV.push(0);
    this._status = c.calibDurationS > 0 ? 'CALIBRATING' : 'READY';
  }

  startCalibration() {
    if (this._status === 'NOT_READY') return;
    this.frozenTotal = this.totalMeas + this.yawOffset;
    this.calibElapsed = 0;
    this.calibSum = 0;
    this._status = this.cfg.calibDurationS > 0 ? 'CALIBRATING' : 'READY';
  }

  get status() { return this._status; }
  get time() { return this.tSensor; }

  advance(dt, omegaStartRad, omegaEndRad, ctx = { linAccelG: 0, flywheelOn: false }) {
    if (this._status === 'NOT_READY' || !(dt > 0)) return;
    const c = this.cfg, r = this.rng;
    const n = Math.min(MAX_SUBSTEPS, Math.max(1, Math.ceil(dt * c.internalHz)));
    const dts = dt / n;
    const linG = (ctx && ctx.linAccelG) || 0;
    const vib = (ctx && ctx.flywheelOn) ? c.vibrationNoiseMult : 1; // [S7]
    const tau = c.gmTauS > 0 ? c.gmTauS : Infinity;
    const aGm = Math.exp(-dts / tau);
    const gmDrive = c.gmSigmaRad * Math.sqrt(Math.max(0, 1 - Math.exp(-2 * dts / tau)));
    const rwDrive = c.biasRwRad * Math.sqrt(dts);
    const noiseStd = c.noiseDensityRad * vib / Math.sqrt(dts); // rad/s per sample [S2]
    for (let i = 0; i < n; i++) {
      const frac = (i + 0.5) / n;
      const wt = omegaStartRad + (omegaEndRad - omegaStartRad) * frac; // true rate at substep midpoint
      const w1 = r.gauss(), w2 = r.gauss(), w3 = r.gauss();
      this.bGm = aGm * this.bGm + gmDrive * w1;
      this.bRw += rwDrive * w2;
      const tMid = this.tOn + 0.5 * dts;
      this.tempDeltaK = c.tempRiseK * (1 - Math.exp(-tMid / c.tempTauS));
      const eps = this.scaleErr + this.scaleAsym * Math.sign(wt);
      const ws = (1 + eps) * wt + this.b0 + this.bGm + this.bRw
        + c.tempCoeffRad * this.tempDeltaK + c.gSensRad * linG + noiseStd * w3;
      let wc = ws;
      if (wc > c.fullScaleRad) { wc = c.fullScaleRad; this.satEvents++; }
      else if (wc < -c.fullScaleRad) { wc = -c.fullScaleRad; this.satEvents++; }
      const wq = c.rateLsbRad > 0 ? Math.round(wc / c.rateLsbRad) * c.rateLsbRad : wc;
      this.totalMeas += wq * dts;
      this.totalTrue += wt * dts;
      this.tSensor += dts;
      this.tOn += dts;
      this.hT.push(this.tSensor);
      this.hV.push(this.totalMeas);
      if (this._status === 'CALIBRATING') {
        this.calibSum += wt * dts;
        this.calibElapsed += dts;
        if (this.calibElapsed >= c.calibDurationS - 1e-12) this._finishCalibration();
      }
    }
    this._prune();
  }

  _finishCalibration() {
    const omegaBar = this.calibElapsed > 0 ? this.calibSum / this.calibElapsed : 0;
    this.b0 = this.cfg.biasInitSigmaRad * this.rng.gauss() - omegaBar;
    this.yawOffset = this.frozenTotal - this.totalMeas; // reported heading continues from the frozen value
    this._status = 'READY';
  }

  _prune() {
    const keepFrom = this.tSensor - Math.max(MIN_HISTORY_S, this.cfg.latencyS) - 0.5;
    if (this.hT.length < 64 || this.hT[0] >= keepFrom) return;
    let lo = 0, hi = this.hT.length - 1; // first index with t >= keepFrom
    while (lo < hi) { const m = (lo + hi) >> 1; if (this.hT[m] < keepFrom) lo = m + 1; else hi = m; }
    const drop = Math.min(lo, this.hT.length - 2);
    if (drop > 0) { this.hT.splice(0, drop); this.hV.splice(0, drop); }
  }

  shock({ impactSpeedInS = 0, peakRateRad = 0, durationS = 0 } = {}) {
    if (this._status === 'NOT_READY') return;
    const c = this.cfg;
    let d = 0;
    if (impactSpeedInS > 0 && c.shockSigmaRad > 0) {
      d += c.shockSigmaRad * (impactSpeedInS / IN_PER_M) * this.rng.gauss(); // [A]
    }
    if (Math.abs(peakRateRad) > c.fullScaleRad && durationS > 0) {
      d -= (Math.abs(peakRateRad) - c.fullScaleRad) * durationS * Math.sign(peakRateRad); // [D]
      this.satEvents++;
    }
    if (d !== 0) {
      this.totalMeas += d;
      this.hV[this.hV.length - 1] = this.totalMeas; // jump appears at the current sensor time
    }
  }

  // Interpolate totalMeas at sensor time tq; returns {v, rate}.
  _lookup(tq) {
    const T = this.hT, V = this.hV, n = T.length;
    if (n === 1) return { v: V[0], rate: 0 };
    if (tq <= T[0]) tq = T[0];
    if (tq >= T[n - 1]) tq = T[n - 1];
    let lo = 0, hi = n - 1; // first index with T >= tq
    while (lo < hi) { const m = (lo + hi) >> 1; if (T[m] < tq) lo = m + 1; else hi = m; }
    const i1 = Math.max(1, lo), i0 = i1 - 1;
    const span = T[i1] - T[i0];
    if (!(span > 0)) return { v: V[i1], rate: 0 };
    const f = (tq - T[i0]) / span;
    return { v: V[i0] + (V[i1] - V[i0]) * f, rate: (V[i1] - V[i0]) / span };
  }

  read(tNow) {
    const c = this.cfg;
    const tq = tNow - c.latencyS;
    if (this._status === 'NOT_READY') return { yaw: 0, total: 0, rate: 0, t: tq, status: 'NOT_READY' };
    if (this._status === 'CALIBRATING') {
      const total = this.frozenTotal;
      return { yaw: wrapPi(quantize(total, c.headingLsbRad)), total, rate: 0, t: tq, status: 'CALIBRATING' };
    }
    const { v, rate } = this._lookup(tq);
    const total = v + this.yawOffset;
    return { yaw: wrapPi(quantize(total, c.headingLsbRad)), total, rate, t: tq, status: 'READY' };
  }

  resetYaw() { this.setYaw(0); }

  setYaw(rad) {
    this.yawOffset = rad - this.totalMeas;
    if (this._status === 'CALIBRATING') this.frozenTotal = rad;
  }

  getDiagnostics() {
    return {
      b0: this.b0, bGm: this.bGm, bRw: this.bRw,
      scaleErr: this.scaleErr, scaleAsym: this.scaleAsym,
      tempDeltaK: this.tempDeltaK, satEvents: this.satEvents,
      totalTrueRad: this.totalTrue, totalMeasRad: this.totalMeas,
      errorRad: this.totalMeas - this.totalTrue,
    };
  }
}
