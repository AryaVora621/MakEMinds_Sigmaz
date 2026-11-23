// estimator.js - odometry pod model, 2-state heading Kalman filter, synthetic vision, dead-reckoning pose estimator.
// Pure ES module, no DOM. Units: inches, seconds, radians (ccw positive, heading 0 = +x).
// The estimator never reads the true heading: it gets body deltas plus a GyroReading
// {yaw, total, rate, t, status}. See IMU-HEADING SPEC section 3. Every numeric parameter comes in through
// the resolved config objects of params.js (tags there: [A] assumed, [S*] sourced, [R] recalled).
import { wrapPi, se2Exp } from './angles.js';

/** Odometry pods (two dead wheels, forward and lateral) with scale error, sqrt(distance) noise, slip and tick quantization. */
export class OdometryModel {
  /** @param {{podScaleSigma:number, podNoiseInPerRtIn:number, podSlipSigmaIn:number, ticksPerIn:number}} odoCfg
   *  @param {{normal:Function, gauss:Function}} rng */
  constructor(odoCfg, rng) {
    this.cfg = odoCfg;
    this.rng = rng;
    // scale errors, dimensionless fractions, drawn once at construction [A sigma in params.js]
    this.sF = rng.normal(0, odoCfg.podScaleSigma);
    this.sL = rng.normal(0, odoCfg.podScaleSigma);
    this.accF = 0; // ticks carried
    this.accL = 0;
  }

  /** @param {number} df forward true delta (in) @param {number} dl lateral true delta (in) */
  measure(df, dl, ctx = { collision: false }) {
    const c = this.cfg, r = this.rng;
    let dfm = (1 + this.sF) * df + c.podNoiseInPerRtIn * Math.sqrt(Math.abs(df)) * r.gauss();
    let dlm = (1 + this.sL) * dl + c.podNoiseInPerRtIn * Math.sqrt(Math.abs(dl)) * r.gauss();
    if (ctx && ctx.collision) {
      dfm += r.normal(0, c.podSlipSigmaIn);
      dlm += r.normal(0, c.podSlipSigmaIn);
    }
    if (c.ticksPerIn > 0) {
      this.accF += dfm * c.ticksPerIn;
      this.accL += dlm * c.ticksPerIn;
      const tf = Math.trunc(this.accF), tl = Math.trunc(this.accL);
      this.accF -= tf;
      this.accL -= tl;
      dfm = tf / c.ticksPerIn;
      dlm = tl / c.ticksPerIn;
    }
    return { df: dfm, dl: dlm };
  }

  get scale() { return { sF: this.sF, sL: this.sL }; }
}

/** 2-state filter: x = [c, beta]; c = correction added to gyro heading (rad), beta = gyro rate bias estimate (rad/s). */
export class HeadingKF {
  /** @param {{qc:number, qb:number, p0c?:number, p0b:number}} o  qc rad^2/s, qb (rad/s)^2/s, p0c rad^2, p0b (rad/s)^2 */
  constructor({ qc, qb, p0c = 0, p0b }) {
    this.qc = qc; this.qb = qb;
    this.c = 0; this.beta = 0;
    this.p00 = p0c; this.p01 = 0; this.p10 = 0; this.p11 = p0b;
  }

  predict(dt) {
    this.c -= this.beta * dt;
    // P = F P F^T + Q, F = [[1,-dt],[0,1]]
    const p00 = this.p00 - dt * (this.p01 + this.p10) + dt * dt * this.p11;
    const p01 = this.p01 - dt * this.p11;
    const p10 = this.p10 - dt * this.p11;
    this.p00 = p00 + this.qc * dt;
    this.p01 = p01;
    this.p10 = p10;
    this.p11 = this.p11 + this.qb * dt;
  }

  update(innovRad, sigmaRad) {
    const S = this.p00 + sigmaRad * sigmaRad;
    if (!(S > 0)) return;
    const k0 = this.p00 / S, k1 = this.p10 / S;
    this.c += k0 * innovRad;
    this.beta += k1 * innovRad;
    const p00 = this.p00 - k0 * this.p00;
    const p01 = this.p01 - k0 * this.p01;
    const p10 = this.p10 - k1 * this.p00;
    const p11 = this.p11 - k1 * this.p01;
    this.p00 = p00; this.p01 = p01; this.p10 = p10; this.p11 = p11;
  }

  get correction() { return this.c; }
  get biasEst() { return this.beta; }
  get P() { return [[this.p00, this.p01], [this.p10, this.p11]]; }
}

/** Build a heading KF matched to the gyro config. */
export function makeHeadingKF(imuCfg) {
  return new HeadingKF({
    qc: imuCfg.noiseDensityRad * imuCfg.noiseDensityRad,
    qb: imuCfg.biasRwRad * imuCfg.biasRwRad + 2 * imuCfg.gmSigmaRad * imuCfg.gmSigmaRad / imuCfg.gmTauS,
    p0c: 0,
    p0b: imuCfg.biasInitSigmaRad * imuCfg.biasInitSigmaRad,
  });
}

/** Synthetic AprilTag measurement of the true pose. Draw order: x, y, h. */
export function sampleVision(truePose, visionCfg, rng, t) {
  return {
    x: truePose.x + rng.normal(0, visionCfg.posSigmaIn),
    y: truePose.y + rng.normal(0, visionCfg.posSigmaIn),
    h: truePose.h + rng.normal(0, visionCfg.headingSigmaRad),
    t,
    sigmaXY: visionCfg.posSigmaIn,
    sigmaH: visionCfg.headingSigmaRad,
  };
}

export class PoseEstimator {
  constructor({ imuCfg, estimatorCfg, odometryCfg, visionCfg, rng }) {
    this.imuCfg = imuCfg;
    this.cfg = estimatorCfg;
    this.odoCfg = odometryCfg;
    this.visionCfg = visionCfg;
    this.rng = rng;
    this._odo = new OdometryModel(odometryCfg, rng);
    this.reset({ x: 0, y: 0, h: 0 }, 0);
  }

  reset(pose, t = 0) {
    this.kf = makeHeadingKF(this.imuCfg);
    this.pose = { x: pose.x, y: pose.y, h: pose.h }; // h unwrapped
    this.hGyroPrev = null;
    this.cPrev = 0;
    this.pxx = 0; // in^2
    this.t = t;
    this.hist = [];
    this.truthPose = { x: pose.x, y: pose.y, h: pose.h };
  }

  /**
   * Anchor the gyro reference to a reading taken at the same instant as the reset pose. Without this the
   * estimator anchors lazily on the first READY reading inside step(), which already contains the first
   * frame's rotation, so that rotation would be lost for good (one frame of yaw rate, e.g. 0.017 rad at
   * 1.7 rad/s, dt = 0.01 s). The lazy anchor in step() remains for a gyro that is still CALIBRATING at reset.
   */
  anchorGyro(reading) {
    if (!reading || reading.status !== 'READY') return;
    this.hGyroPrev = reading.total;
    this.cPrev = this.kf.correction;
  }

  get _truthMode() { return this.cfg.mode === 'truth'; }

  step(dt, t, bodyDelta, reading, ctx = { collision: false, measureOdometry: true }) {
    this.t = t;
    if (this._truthMode || reading == null) return; // pass-through: pose comes from setTruePose
    if (reading.status !== 'READY') return;          // hold pose, no prediction [S6 RR fallback]

    const measure = !ctx || ctx.measureOdometry !== false;
    let df = bodyDelta.f, dl = bodyDelta.l;
    if (measure) {
      const m = this._odo.measure(df, dl, ctx || {});
      df = m.df; dl = m.dl;
    }

    if (this.hGyroPrev === null) {
      // first READY reading: anchor, no heading increment yet
      this.hGyroPrev = reading.total;
      this.cPrev = this.kf.correction;
    }

    if (this.cfg.useKF) this.kf.predict(dt);
    const c = this.cfg.useKF ? this.kf.correction : 0;
    const dh = (reading.total + c) - (this.hGyroPrev + this.cPrev);
    this.hGyroPrev = reading.total;
    this.cPrev = c;

    const ds = Math.hypot(df, dl);
    const phh = Math.max(this.kf.p00, 0);
    this.pxx += this.odoCfg.podNoiseInPerRtIn * this.odoCfg.podNoiseInPerRtIn * ds + ds * ds * phh;

    this.pose = se2Exp(this.pose, df, dl, dh);
    this._pushHist(t, reading.total);
  }

  _pushHist(t, hGyro) {
    this.hist.push({ t, hGyro, x: this.pose.x, y: this.pose.y });
    const tmin = t - this.cfg.historyS;
    let k = 0;
    while (k < this.hist.length - 1 && this.hist[k + 1].t <= tmin) k++;
    if (k > 0) this.hist.splice(0, k);
  }

  _histAt(t) {
    const h = this.hist;
    if (h.length === 0 || t < h[0].t - 1e-12 || t > h[h.length - 1].t + 1e-12) return null;
    for (let i = h.length - 1; i >= 0; i--) {
      if (h[i].t <= t) {
        if (i === h.length - 1) return h[i];
        const a = h[i], b = h[i + 1];
        const f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
        return { t, hGyro: a.hGyro + f * (b.hGyro - a.hGyro), x: a.x + f * (b.x - a.x), y: a.y + f * (b.y - a.y) };
      }
    }
    return h[0];
  }

  addVisionPose(m) {
    if (this._truthMode || this.hGyroPrev === null) return false;
    const at = this._histAt(m.t);
    if (!at) return false;
    if (this.cfg.useKF) {
      const innov = wrapPi(m.h - (at.hGyro + this.kf.correction));
      this.kf.update(innov, m.sigmaH);
      const dc = this.kf.correction - this.cPrev;
      this.cPrev = this.kf.correction;
      this.pose.h += dc;
    }
    const s2 = m.sigmaXY * m.sigmaXY;
    const alpha = this.pxx + s2 > 0 ? this.pxx / (this.pxx + s2) : 0;
    this.pose.x += alpha * (m.x - at.x);
    this.pose.y += alpha * (m.y - at.y);
    this.pxx *= (1 - alpha);
    return true;
  }

  setTruePose(pose) {
    this.truthPose = { x: pose.x, y: pose.y, h: pose.h };
    if (this._truthMode) this.pose = { x: pose.x, y: pose.y, h: pose.h };
  }

  getPose() {
    return { x: this.pose.x, y: this.pose.y, h: wrapPi(this.pose.h) };
  }

  getHeadingUnwrapped() { return this.pose.h; }

  getCovariance() {
    return { pxx: this.pxx, phh: this.kf.p00, biasEst: this.kf.beta };
  }

  get odometry() { return this._odo; }
}
