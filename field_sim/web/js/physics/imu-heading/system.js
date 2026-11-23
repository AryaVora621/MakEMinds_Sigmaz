// system.js - ImuHeadingSystem facade: one call per sim frame (spec section 6).
// Units: inches, s, rad (ccw positive, heading 0 = +x). The TRUE state is never modified here.
// enabled=false or estimator mode 'truth' => the estimate equals the truth exactly.

import { createRng } from './rng.js';
import { angDiff, worldToBody } from './angles.js';
import {
  SYSTEM_DEFAULTS,
  resolveImuConfig,
  resolveOdometryConfig,
  resolveVisionConfig,
  resolveEstimatorConfig,
} from './params.js';
import { GyroModel } from './gyro.js';
import { PoseEstimator, sampleVision } from './estimator.js';
import { fieldCentricExtraOffset } from './adapters.js';

const G_IN_S2 = 386.09; // standard gravity in in/s^2 [sourced: 9.80665 m/s^2 / 0.0254 m/in]
const MAX_QUEUED_VISION = 64; // cap on undelivered vision samples [A]

export class ImuHeadingSystem {
  constructor(opts = {}) {
    const o = { ...SYSTEM_DEFAULTS, ...opts };
    this._enabled = !!o.enabled;
    this._hostHz = o.hostHz; // Hz
    // ideal preset: raw (unmeasured) odometry so the estimate equals the truth, unless pod overrides are given
    this._idealOdo = o.preset === 'ideal' && !Object.keys(o.odometryOverrides || {}).length;
    if (!(this._hostHz > 0)) throw new Error('hostHz must be > 0');
    this._cfg = {
      imu: resolveImuConfig(o.preset, o.imuOverrides || {}),
      odometry: resolveOdometryConfig(o.odometryOverrides || {}),
      vision: resolveVisionConfig(o.visionOverrides || {}),
      estimator: resolveEstimatorConfig(o.estimatorOverrides || {}),
      hostHz: this._hostHz,
      enabled: this._enabled,
    };
    const root = createRng(o.seed);
    this._gyro = new GyroModel(this._cfg.imu, root.fork('gyro'));
    this._odoRng = root.fork('odo');
    this._visionRng = root.fork('vision');
    this._est = new PoseEstimator({
      imuCfg: this._cfg.imu,
      estimatorCfg: this._cfg.estimator,
      odometryCfg: this._cfg.odometry,
      visionCfg: this._cfg.vision,
      rng: this._odoRng,
    });
    this.reset({ x: 0, y: 0, h: 0 }, 0);
  }

  _passThrough() {
    return !this._enabled || this._cfg.estimator.mode === 'truth';
  }

  reset(truePose, t = 0, { skipCalibration = true } = {}) {
    this._t = t;
    this._truth = { x: truePose.x, y: truePose.y, h: truePose.h };
    this._prev = { ...this._truth };
    this._vPrev = { f: 0, l: 0 };
    this._hostTimer = 0;
    this._acc = { f: 0, l: 0, collision: false };
    this._visionQueue = [];
    this._lastVisionT = -Infinity;
    this._gyro.reset(t);
    if (skipCalibration) {
      // Robot assumed still pre-match: run the calibration window with zero true rate.
      for (let i = 0; i < 1000 && this._gyro.status === 'CALIBRATING'; i++) {
        this._gyro.advance(0.05, 0, 0, { linAccelG: 0, flywheelOn: false });
      }
    }
    this._gyro.setYaw(truePose.h);
    this._est.reset({ ...this._truth }, t);
    // the reset pose and this gyro reading describe the same instant: anchor the estimator to it
    this._est.anchorGyro(this._gyro.read(this._gyro.time));
  }

  step(dt, trueState, events = {}) {
    if (!(dt > 0)) return;
    const collisions = events.collisions || [];
    const prev = this._prev;
    const dh = angDiff(trueState.h, prev.h); // rad, exact true rotation over this frame
    // True yaw rate fed to the gyro over this frame: the constant frame-mean rate dh/dt (rad/s).
    // The host truth is a sequence of discrete poses, so the only rate the sim actually knows is the mean
    // rate per frame. A ramp from the previous frame's rate (the earlier version) integrates
    // dt*(w_prev + w)/2, which is NOT dh: across a 0 -> 1.7 rad/s step at dt = 0.01 s it loses
    // 0.01*1.7/2 = 0.0085 rad of TRUE rotation, a fake error charged to every preset (even 'ideal') and
    // also corrupting gyro.totalTrue. With a constant rate the gyro's true-rate integral equals sum(dh)
    // exactly, so all reported gyro error comes from the sensor model itself (scale, asymmetry, bias,
    // Gauss-Markov / random walk, temperature, g-sensitivity, noise, clipping, LSB quantization, latency),
    // which still act per internal substep for the non-ideal presets.
    const omegaEnd = dh / dt; // rad/s
    const omegaStart = omegaEnd; // rad/s (constant over the frame; see above)
    const body = worldToBody(trueState.x - prev.x, trueState.y - prev.y, prev.h + 0.5 * dh);
    const vf = body.f / dt; // in/s
    const vl = body.l / dt;
    const linAccelG = Math.hypot(vf - this._vPrev.f, vl - this._vPrev.l) / dt / G_IN_S2;

    this._t += dt;
    const t = this._t;
    this._truth = { x: trueState.x, y: trueState.y, h: trueState.h };
    this._prev = { x: trueState.x, y: trueState.y, h: trueState.h };
    this._vPrev = { f: vf, l: vl };
    if (!this._enabled) return;

    // (b) collisions -> gyro shock + odometry slip flag
    let collided = false;
    for (const c of collisions) {
      this._gyro.shock(c);
      collided = true;
    }
    // (c) gyro internal integration
    this._gyro.advance(dt, omegaStart, omegaEnd, { linAccelG, flywheelOn: !!events.flywheelOn });

    if (this._cfg.estimator.mode === 'truth') {
      this._est.setTruePose(this._truth);
      return;
    }

    // (d) host loop gating with the accumulated odometry delta
    this._acc.f += body.f;
    this._acc.l += body.l;
    this._acc.collision = this._acc.collision || collided;
    this._hostTimer += dt;
    const period = 1 / this._hostHz; // s
    if (this._hostTimer >= period - 1e-9) {
      this._hostTimer -= period;
      if (this._hostTimer > period || this._hostTimer < 0) this._hostTimer = 0;
      const reading = this._gyro.read(this._gyro.time);
      this._est.step(
        period,
        t,
        { f: this._acc.f, l: this._acc.l },
        reading,
        { collision: this._acc.collision, measureOdometry: !this._idealOdo },
      );
      this._acc = { f: 0, l: 0, collision: false };
    }

    // (e) vision: generate when due, deliver after latency
    const vc = this._cfg.vision;
    if (vc.enabled) {
      if (events.visionVisible && t - this._lastVisionT >= 1 / vc.rateHz - 1e-9) {
        this._lastVisionT = t;
        this._visionQueue.push(sampleVision(this._truth, vc, this._visionRng, t));
        if (this._visionQueue.length > MAX_QUEUED_VISION) this._visionQueue.shift();
      }
      while (this._visionQueue.length && this._visionQueue[0].t + vc.latencyS <= t + 1e-9) {
        this._est.addVisionPose(this._visionQueue.shift());
      }
    }
  }

  getEstimatedPose() {
    if (this._passThrough()) return { ...this._truth };
    return this._est.getPose();
  }
  getTruePose() {
    return { ...this._truth };
  }
  getHeadingError() {
    return angDiff(this.getEstimatedPose().h, this._truth.h);
  }
  getPositionError() {
    const e = this.getEstimatedPose();
    return Math.hypot(e.x - this._truth.x, e.y - this._truth.y);
  }
  getDriverHeadingOffset() {
    return fieldCentricExtraOffset(this._truth.h, this.getEstimatedPose().h);
  }
  getGyroReading() {
    return this._gyro.read(this._gyro.time);
  }
  resetYaw() {
    this._gyro.resetYaw();
  }
  setYaw(rad) {
    this._gyro.setYaw(rad);
  }
  recalibrate() {
    this._gyro.startCalibration();
  }

  getTelemetry() {
    return {
      t: this._t,
      truePose: this.getTruePose(),
      estPose: this.getEstimatedPose(),
      headingErrDeg: (this.getHeadingError() * 180) / Math.PI,
      posErrIn: this.getPositionError(),
      gyro: this._gyro.getDiagnostics(),
      cov: this._est.getCovariance(),
      status: this._gyro.status,
    };
  }

  get gyro() {
    return this._gyro;
  }
  get estimator() {
    return this._est;
  }
  get config() {
    return this._cfg;
  }
}
