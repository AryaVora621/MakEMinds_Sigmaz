// controllers.js - velocity controllers for the flywheel sim. Pure ES module.
//  SdkFlywheelPIDF : line-for-line port of TeamCode/.../flywheel/FlywheelPIDF.java [REPO].
//  clampTargetTicks: port of Flywheel.clampTargetTicks (Flywheel.java lines 60-65) [REPO].
//  HubVelocityPIDF : ASSUMED/UNVERIFIED RUN_USING_ENCODER parallel PIDF (form, units and defaults not verified).
//  ReadyGate       : Flywheel.atTarget() style tolerance gate [REPO] with optional settle-loop count.
import { defaultParams } from './params.js';

const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

export class SdkFlywheelPIDF {
  /** @param gains sdkPidf group: kS V, kV V/(tick/s), kA V/(tick/s^2), kP, kI, kD, iMaxVolts V, nominalV V */
  constructor(gains) {
    this.g = { ...defaultParams().sdkPidf, ...(gains || {}) };
    this.lastVolts = 0;
    this.reset();
  }

  /** target, velocity in ticks/s; batteryVoltage V; dt s. Returns power in -1..1. */
  calculate(target, velocity, batteryVoltage, dt) {
    const g = this.g;
    if (target === 0) { this.reset(); this.lastVolts = 0; return 0; }
    const error = target - velocity;
    const targetRate = this._first ? 0 : (target - this._lastTarget) / dt;
    const errorRate = this._first ? 0 : (error - this._lastError) / dt;

    this._integral += error * dt;
    if (g.kI !== 0) {
      const lim = g.iMaxVolts / Math.abs(g.kI);   // |kI| so a negative kI does not invert the bounds
      this._integral = Math.max(-lim, Math.min(lim, this._integral));
    }

    const volts = g.kS * Math.sign(target) + g.kV * target + g.kA * targetRate
      + g.kP * error + g.kI * this._integral + g.kD * errorRate;
    this.lastVolts = volts;

    this._lastError = error;
    this._lastTarget = target;
    this._first = false;
    // 0 / NaN battery reading would give Infinity or NaN power; fall back to nominal V (NaN > 1 is false).
    const v = batteryVoltage > 1 ? batteryVoltage : g.nominalV;
    const power = volts / v;
    if (Number.isNaN(power)) return 0;
    return Math.max(-1, Math.min(1, power));
  }

  reset() {
    this._integral = 0;
    this._lastError = 0;
    this._lastTarget = 0;
    this._first = true;
  }
}

/** Target ticks/s actually used: !(t>0) -> 0; else raised to minRpm, capped at maxRpm (cap wins). */
export function clampTargetTicks(ticksPerSecond, minRpm, maxRpm, ticksPerRev) {
  if (!(ticksPerSecond > 0)) return 0;
  const maxTicks = maxRpm * ticksPerRev / 60;
  const minTicks = Math.min(minRpm * ticksPerRev / 60, maxTicks);
  return Math.min(maxTicks, Math.max(ticksPerSecond, minTicks));
}

/** [ASSUMED] hub-side velocity PIDF, parallel form, called once per hub period. */
export class HubVelocityPIDF {
  constructor(hub) {
    this.h = { ...defaultParams().hubPidf, ...(hub || {}) };
    this.target = 0;
    this._integ = 0;
    this._ePrev = 0;
    this._first = true;
  }

  setTarget(ticksPerS) {
    this.target = Number.isFinite(ticksPerS) ? ticksPerS : 0;
    if (this.target === 0) this._clearState();
  }

  /** measuredTicksPerS ticks/s, periodS s. Returns power in -1..1. */
  tick(measuredTicksPerS, periodS) {
    const h = this.h;
    if (this.target === 0) { this._clearState(); return 0; }
    const T = periodS > 0 ? periodS : h.periodS;
    const e = this.target - measuredTicksPerS;
    this._integ += e * T;
    if (h.kI !== 0) {
      const lim = h.outputScale / Math.abs(h.kI);
      this._integ = clamp(this._integ, -lim, lim);
    }
    const deriv = this._first ? 0 : (e - this._ePrev) / T;
    this._ePrev = e;
    this._first = false;
    const out = h.kP * e + h.kI * this._integ + h.kD * deriv + h.kF * this.target;
    const p = out / h.outputScale;
    if (Number.isNaN(p)) return 0;
    return clamp(p, -1, 1);
  }

  reset() { this.target = 0; this._clearState(); }

  _clearState() { this._integ = 0; this._ePrev = 0; this._first = true; }
}

/** [UNVERIFIED] tuning-guide rule kF = scale / maxTicksPerS (2800 ticks/s -> 11.70). */
export function hubKfFromMaxTicks(maxTicksPerS, scale = 32767) {
  return scale / maxTicksPerS;
}

export class ReadyGate {
  /** @param ready group: mode 'ticks'|'frac', tolTicks ticks/s, fracTol, settleLoops */
  constructor(ready) {
    this.r = { ...defaultParams().ready, ...(ready || {}) };
    this._count = 0;
  }

  update(targetTicks, measuredTicks) {
    const r = this.r;
    let inTol = false;
    if (targetTicks !== 0) {
      const err = Math.abs(targetTicks - measuredTicks);
      inTol = r.mode === 'ticks'
        ? err <= r.tolTicks
        : err / Math.max(Math.abs(targetTicks), 1) < r.fracTol;
    }
    this._count = inTol ? this._count + 1 : 0;
    return this.ready;
  }

  get ready() { return this._count >= this.r.settleLoops; }

  reset() { this._count = 0; }
}
