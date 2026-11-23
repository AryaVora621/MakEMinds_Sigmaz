// odometry.js - OdometrySim facade: truth motion in, believed (Pinpoint/Pedro) pose out.
// Units: inches, radians, seconds. No DOM; imports only from this folder.
import { makeOdoParams } from './params.js';
import { worldToBody, poseError } from './kinematics.js';
import { makeRng } from './rng.js';
import { PodWheel } from './podwheel.js';
import { Imu } from './imu.js';
import { PinpointDevice } from './pinpoint.js';
import { PinpointHost } from './host.js';
import { PinpointLocalizerSim } from './localizer.js';

export class OdometrySim {
  constructor({ preset = 'nominal', overrides = {} } = {}) {
    this._params = makeOdoParams(preset, overrides);
    const p = this._params;
    this._rng = makeRng(p.seed);
    this._fwd = new PodWheel('forward', p, this._rng);
    this._str = new PodWheel('strafe', p, this._rng);
    this._imu = new Imu(p, this._rng);
    this._dev = new PinpointDevice(p, { fwdPod: this._fwd, strPod: this._str, imu: this._imu });
    this._host = new PinpointHost(this._dev, p, this._rng);
    this._loc = new PinpointLocalizerSim(this._dev, this._host);
    this._simT = 0;
    this._nextTick = 0;
    this._hostAcc = 0;
    this._lastTruth = null;
    this.reset({ x: 0, y: 0, h: 0 });
  }

  get params() { return this._params; }
  get device() { return this._dev; }
  get host() { return this._host; }
  get localizer() { return this._loc; }
  get pods() { return { fwd: this._fwd, str: this._str }; }
  get imu() { return this._imu; }

  reset(pose) {
    this._fwd.reset();
    this._str.reset();
    this._imu.reset(0);
    this._simT = 0;
    this._hostAcc = 0;
    const fw = this._params.firmware;
    this._nextTick = fw.rateHz > 0 ? 1 / fw.rateHz : 0;
    this._dev.clearHistory();
    this._loc.setStartPose(pose); // device.setPosition + clearHistory + cache
    this._dev.tick(0);
    this._loc.update(0);
    this._lastTruth = { x: pose.x, y: pose.y, h: pose.h, vx: 0, vy: 0, om: 0 };
  }

  stepPhysics(dt, truth, events = {}) {
    if (!(dt > 0)) return;
    const { vx, vy, om } = truth;
    // b. mid-step body twist drives slip detection
    const hm = truth.h + om * dt / 2;
    const b0 = worldToBody(vx, vy, hm);
    this._fwd.beginStep(dt, b0.vf, b0.vl, om, events);
    this._str.beginStep(dt, b0.vf, b0.vl, om, events);

    // c. firmware sub-stepping
    const fw = this._params.firmware;
    if (fw.rateHz > 0) {
      const Ts = 1 / fw.rateHz;
      let remaining = dt, tIn = 0;
      while (remaining > 1e-12) {
        const h = Math.min(remaining, this._nextTick - this._simT);
        const step = h > 0 ? h : remaining; // guard against a non-advancing step
        const b = worldToBody(vx, vy, truth.h + om * (tIn + step / 2));
        this._fwd.advance(step, b.vf, b.vl, om);
        this._str.advance(step, b.vf, b.vl, om);
        this._imu.advance(om, step);
        this._simT += step; remaining -= step; tIn += step;
        if (this._simT >= this._nextTick - 1e-12) {
          this._dev.tick(this._simT);
          const jit = fw.rateJitterFrac ? fw.rateJitterFrac * this._rng.gauss() : 0;
          this._nextTick += Math.max(Ts / 2, Ts * (1 + jit));
        }
      }
    } else {
      const b = worldToBody(vx, vy, hm);
      this._fwd.advance(dt, b.vf, b.vl, om);
      this._str.advance(dt, b.vf, b.vl, om);
      this._imu.advance(om, dt);
      this._simT += dt;
      this._dev.tick(this._simT);
    }

    // d. host loop
    const per = this._params.host.loopPeriodS;
    this._hostAcc += dt;
    if (per <= 0) {
      this._loc.update(this._simT);
      this._hostAcc = 0;
    } else if (this._hostAcc >= per) {
      this._loc.update(this._simT);
      this._hostAcc -= per;
    }
    this._lastTruth = { ...truth }; // e.
  }

  getPose() { return this._loc.getPose(); }
  getVelocity() { return this._loc.getVelocity(); }
  setPose(pose) { this._loc.setPose(pose); }
  getTruthError(truthPose) { return poseError(truthPose, this.getPose()); }

  diagnostics() {
    const s = this._dev.snapshot();
    return {
      simTimeS: this._simT,
      fwdCount: this._fwd.count,
      strCount: this._str.count,
      fwdSlip: this._fwd.slipFraction,
      strSlip: this._str.slipFraction,
      fwdLiftoff: this._fwd.inLiftoff,
      strLiftoff: this._str.inLiftoff,
      imuHeadingRad: this._imu.headingRad,
      dataAgeS: this._host.lastDataAgeS,
      fwLoopTimeUs: s ? s.loopTimeUs : 0,
      fwFrequencyHz: s ? s.frequencyHz : 0,
      status: this._dev.status,
    };
  }
}
