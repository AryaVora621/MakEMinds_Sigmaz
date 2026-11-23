// dcmotor.js - brushed DC gearmotor electrical/torque model (quasi-static, inductance ignored).
// Units: SI (rad/s, N*m, A, V, degC). See spec section 3.
//   Thermal scaling: R' = R*rS (winding temp), Kt' = Kt*kS, Ke' = Ke*kS (housing temp).
//   Power-flow branch: motoring torque loses eta, back-driven (regen/brake) torque gains 1/eta.
import { THERMAL } from './params.js';
import { ThermalTwoNode } from './thermal.js';

const sign = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);

export class MotorUnit {
  constructor(gm, { thermal = THERMAL, ambientC } = {}) {
    this.gm = gm;
    this.thermal = new ThermalTwoNode(thermal, ambientC ?? thermal.ambientC);
    this._diodeV = thermal.diodeDropV; // ASSUMED 0.7 V per diode (params.js)
    this._lastStatorA = 0;
  }

  /** Pure: does not mutate. omegaOut mechanism rad/s, busV V, duty [-1,1]. */
  compute(omegaOut, busV, duty, zeroPower = 'BRAKE') {
    const { motor: m, G, n, eta } = this.gm;
    const kS = this.thermal.kScale();
    const R = m.R * this.thermal.rScale();
    const Kt = m.Kt * kS;
    const Ke = m.Ke * kS;
    const wm = omegaOut * G;
    let V, I;
    if (duty !== 0) {
      V = duty * busV;
      I = (V - Ke * wm) / R;
    } else if (zeroPower === 'FLOAT') {
      V = 0;
      const e = Ke * wm;
      const clampV = busV + 2 * this._diodeV;
      I = Math.abs(e) > clampV ? -sign(wm) * (Math.abs(e) - busV - 2 * this._diodeV) / R : 0;
    } else {
      V = 0; // BRAKE: leads shorted
      I = -Ke * wm / R;
    }
    const Tem = Kt * I;
    const f = Tem * wm >= 0 ? eta : 1 / eta;
    const torqueOut = n * G * Tem * f;
    const dTdOmega = (zeroPower === 'FLOAT' && duty === 0 && I === 0) ? 0 : -n * G * G * Kt * Ke / R * f;
    return {
      torqueOut,
      frictionOut: n * G * m.Tf,
      dTdOmega,
      statorA: I,
      supplyA: duty === 0 ? 0 : I * Math.abs(duty),
      appliedV: V,
      omegaMotor: wm,
      windingPowerW: I * I * R, // per motor, extra field used by commit()
    };
  }

  commit(res, dtS) {
    this.thermal.step(dtS, res.windingPowerW);
    this._lastStatorA = res.statorA;
  }
  get lastStatorA() { return this._lastStatorA; }
  get windingC() { return this.thermal.windingC; }
  get overTemp() { return this.thermal.overTemp; }
  reset() { this.thermal.reset(); this._lastStatorA = 0; }
}

/** Duty that holds omegaOut against torqueOutNm (>=0 means driving along +omega sign convention of torque). */
export function feedforwardDuty(gm, omegaOut, busV, torqueOutNm = 0) {
  const { motor: m, G, n, eta } = gm;
  const wm = omegaOut * G;
  const Tm = torqueOutNm / (n * G * eta) + m.Tf * sign(omegaOut);
  const I = Tm / m.Kt;
  const d = (m.Ke * wm + m.R * I) / busV;
  return Math.max(-1, Math.min(1, d));
}

/** Cold-motor steady torque at output, minus Coulomb friction (for tests and HUD). */
export function steadyStateTorqueOut(gm, omegaOut, busV, duty) {
  const { motor: m, G, n, eta } = gm;
  const wm = omegaOut * G;
  const I = (duty * busV - m.Ke * wm) / m.R;
  return n * G * eta * (m.Kt * I - m.Tf * sign(omegaOut));
}
