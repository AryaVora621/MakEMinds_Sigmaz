// podwheel.js - one passive odometry pod plus its quadrature counter.
// Units: inches, seconds, in/s. Counter in encoder ticks.
// All slip/noise parameters are [ASSUMPTION] (see params.js); this file only applies them.
import { podGroundRate } from './kinematics.js';
import { MM_PER_IN } from './params.js';

export class PodWheel {
  // kind: 'forward' | 'strafe'; params: resolved makeOdoParams result; rng: makeRng result.
  constructor(kind, params, rng) {
    if (kind !== 'forward' && kind !== 'strafe') throw new Error('PodWheel kind must be forward or strafe: ' + kind);
    this.kind = kind;
    this._pod = kind === 'forward' ? params.truth.forwardPod : params.truth.strafePod;
    this._slip = params.slip;
    this._counter = params.counter;
    this._noise = params.noise;
    this._tpmm = params.ticksPerMmTruth; // ticks per mm of true wheel travel
    this._rng = rng;
    this.reset();
  }

  reset() {
    this._ticksF = 0;
    this._travel = 0;
    this._groundTravel = 0;
    this._gPrev = null;
    this._sTotal = 0;
    this._liftoffS = 0;
    this._collSlipS = 0;
    this._collSlipFrac = 0;
    this._lastRate = 0;
    this._s = 0; // slip from accel/scrub/pushing, excluding collision term
  }

  // Once per physics step. vf, vl in in/s (robot frame), om in rad/s.
  beginStep(dt, vf, vl, om, events = {}) {
    const sl = this._slip;
    const g = podGroundRate(this._pod, this.kind, vf, vl, om);
    const aAxis = this._gPrev === null || !(dt > 0) ? 0 : (g - this._gPrev) / dt;
    this._gPrev = g;
    if (!sl.enabled) {
      this._sTotal = 0; this._s = 0; this._liftoffS = 0; this._collSlipS = 0;
      return;
    }
    let s = sl.accelGain * Math.max(0, Math.abs(aAxis) - sl.accelThreshInS2) +
            sl.scrubGain * Math.max(0, Math.abs(om) - sl.scrubThreshRadS);
    s = Math.min(Math.max(s, 0), sl.maxSlip);
    const sev = events.collision ?? 0;
    if (sev >= sl.liftoffSeverityMin) {
      this._liftoffS = sl.liftoffMaxS * sev;
    } else if (sev > 0) {
      this._collSlipS = sl.collisionSlipDurationS;
      this._collSlipFrac = sl.collisionSlipFrac * sev;
    }
    if (events.pushing) s += sl.pushingSlipFrac;
    this._s = s;
    this._sTotal = Math.min(sl.maxSlip, s + (this._collSlipS > 0 ? this._collSlipFrac : 0));
  }

  // Once per firmware sub-step.
  advance(subDt, vf, vl, om) {
    if (!(subDt > 0)) return;
    const sl = this._slip;
    const g = podGroundRate(this._pod, this.kind, vf, vl, om);
    let rate;
    if (this._liftoffS > 0) {
      // Wheel off the ground: spin decays exponentially from its last rate.
      const r0 = this._lastRate ?? 0;
      rate = r0 * Math.exp(-subDt / sl.coastTauS);
      this._liftoffS -= subDt;
    } else {
      rate = g * (1 - this._sTotal);
    }
    this._lastRate = rate;
    this._collSlipS -= subDt;

    const d = rate * subDt;
    this._travel += d;
    this._groundTravel += g * subDt;
    const nz = this._noise.travelNoiseInPerSqrtIn;
    if (nz > 0) this._travel += nz * Math.sqrt(Math.abs(g * subDt)) * this._rng.gauss();

    // A wheel larger by (1+e) under-reads by 1/(1+e).
    this._ticksF += this._pod.encoderSign * d * MM_PER_IN * this._tpmm / this._pod.diameterScale;
  }

  get count() { return this._counter.quantize ? (Math.floor(this._ticksF) | 0) : this._ticksF; }
  get ticksF() { return this._ticksF; }
  get travelIn() { return this._travel; }
  get groundTravelIn() { return this._groundTravel; }
  get slipFraction() { return this._sTotal; }
  get inLiftoff() { return this._liftoffS > 0; }
}
