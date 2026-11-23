// controller.js - velocity PIDF controller and encoder-velocity filter (flywheel-dynamics spec 2d).
// Pure ES module, no DOM. SI units: rad/s, V, s. Caller supplies the sampling cadence (ZOH is done by the plant).
// Constants live in params.js; feedforward is DERIVED here from the motor model:
//   kF = G*Ke/eta_d  (back-EMF, V per rad/s), kS = T_c*R/(count*Kt*G*eta_d)  (V to overcome Coulomb friction).

const fin = (x) => (typeof x === 'number' && Number.isFinite(x));
const num = (x, d = 0) => (fin(x) ? x : d);

/**
 * Derive feedforward gains from the motor model.
 * @param {{R:number, Kt:number, Ke:number, count:number}} motor
 * @param {{gearRatio:number, efficiency:number}} drive
 * @param {number} coulombNm wheel-shaft Coulomb torque, N*m
 * @returns {{kSVolts:number, kFVoltsPerRadS:number}} V and V/(rad/s)
 */
export function deriveFeedforward(motor, drive, coulombNm) {
  const G = num(drive && drive.gearRatio, 1);
  const eta = num(drive && drive.efficiency, 1);
  const m = motor || {};
  const den = num(m.count) * num(m.Kt) * G * eta;
  const kF = eta > 0 ? G * num(m.Ke) / eta : 0;
  const kS = den > 0 ? Math.max(0, num(coulombNm)) * num(m.R) / den : 0;
  return { kSVolts: kS, kFVoltsPerRadS: kF };
}

/** Velocity PIDF controller with clamped output and conditional-integration anti-windup. */
export class VelocityController {
  /**
   * @param {object} ctrlParams params.controller
   * @param {{kSVolts:number, kFVoltsPerRadS:number}} ff derived feedforward
   */
  constructor(ctrlParams, ff) {
    this.p = ctrlParams || {};
    const f = ff || {};
    this.kS = fin(this.p.kSVolts) ? this.p.kSVolts : num(f.kSVolts);
    this.kF = fin(this.p.kFVoltsPerRadS) ? this.p.kFVoltsPerRadS : num(f.kFVoltsPerRadS);
    this.integ = 0;
  }

  /** Clear the integrator. */
  reset() { this.integ = 0; }

  /**
   * One controller evaluation.
   * @param {number} targetRadS rad/s
   * @param {number} measuredRadS rad/s
   * @param {number} periodS controller period, s
   * @param {number} maxVoltageV available bus voltage, V
   * @returns {{voltageV:number, float:boolean, saturated:boolean}}
   */
  compute(targetRadS, measuredRadS, periodS, maxVoltageV) {
    const p = this.p;
    const wt = num(targetRadS);
    const wm = num(measuredRadS);
    const h = Math.max(0, num(periodS));
    const vmax = Math.max(0, num(maxVoltageV));
    const stopCoast = (p.stopMode === undefined ? 'coast' : p.stopMode) === 'coast';
    if (wt <= 0 && stopCoast) {
      this.integ = 0;
      return { voltageV: 0, float: true, saturated: false };
    }
    const e = wt - wm;
    const sgn = wt > 0 ? 1 : (wt < 0 ? -1 : 0);
    let v = this.kS * sgn + this.kF * wt + num(p.kPVoltsPerRadS) * e + num(p.kIVoltsPerRad) * this.integ;
    const lo = p.allowReverse ? -vmax : 0;
    const clamped = Math.min(vmax, Math.max(lo, v));
    const saturated = clamped !== v;
    if (!saturated) this.integ += e * h;
    v = Number.isFinite(clamped) ? clamped : 0;
    return { voltageV: v, float: v === 0 && !p.allowReverse, saturated };
  }
}

/** First-order low-pass model of encoder velocity measurement. */
export class VelocityMeasurement {
  /** @param {object} ctrlParams params.controller (uses measFilterTauS, s) */
  constructor(ctrlParams) {
    this.tau = Math.max(0, num(ctrlParams && ctrlParams.measFilterTauS));
    this.value = 0;
  }

  /** @param {number} [omegaRadS=0] initial filter value, rad/s */
  reset(omegaRadS = 0) { this.value = num(omegaRadS); }

  /**
   * Advance the filter: m += (w - m)*(1 - exp(-h/tau)); tau = 0 gives m = w.
   * @param {number} hS substep, s
   * @param {number} trueOmegaRadS true wheel speed, rad/s
   * @returns {number} measured speed, rad/s
   */
  update(hS, trueOmegaRadS) {
    const w = num(trueOmegaRadS);
    const h = num(hS);
    if (h <= 0) return this.value;
    if (this.tau <= 0) this.value = w;
    else this.value += (w - this.value) * (1 - Math.exp(-h / this.tau));
    return this.value;
  }
}
