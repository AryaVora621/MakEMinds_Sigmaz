// dcmotor.js - brushed DC motor + flywheel inertia + battery model (pure, no DOM, no randomness).
// Units: V, A, ohm, N*m, rad/s, kg*m^2, s, J. Constants come from params.js (tagged there).
// Current is quasi-static (inductance ignored). Integration is semi-implicit Euler.

const TWO_PI = 2 * Math.PI;

function num(x, d = 0) {
  return Number.isFinite(x) ? x : d;
}

/** Derive R, Kt, Ke, friction terms from catalogue motor data [DERIVED]. */
export function deriveMotorConstants(motor) {
  const R = motor.nominalV / motor.stallCurrentA; // ohm
  const Kt = motor.stallTorqueNm / motor.stallCurrentA; // N*m/A
  const omegaFreeRadS = (motor.freeRpm * TWO_PI) / 60; // rad/s
  const Ke = (motor.nominalV - motor.freeCurrentA * R) / omegaFreeRadS; // V*s/rad
  const tauFricFreeNm = Kt * motor.freeCurrentA; // N*m
  const tauCoulombNm = motor.coulombFrac * tauFricFreeNm;
  const bViscousNmS = ((1 - motor.coulombFrac) * tauFricFreeNm) / omegaFreeRadS;
  return { R, Kt, Ke, omegaFreeRadS, tauFricFreeNm, tauCoulombNm, bViscousNmS };
}

/**
 * Motor current [A]. coast -> 0 (open circuit). Otherwise I = (V - Ke*w)/R;
 * brake is volts = 0, which gives I = -Ke*w/R.
 */
export function motorCurrent(k, volts, omegaMotorRadS, coast = false) {
  if (coast) return 0;
  return (num(volts) - k.Ke * num(omegaMotorRadS)) / k.R;
}

/** Friction torque [N*m], same sign as omega: Coulomb (tanh smoothed) + viscous. */
export function frictionTorque(k, omegaMotorRadS, coulombSmoothRadS) {
  const w = num(omegaMotorRadS);
  const s = coulombSmoothRadS > 0 ? coulombSmoothRadS : 1;
  return k.tauCoulombNm * Math.tanh(w / s) + k.bViscousNmS * w;
}

export class FlywheelPlant {
  constructor(params) {
    this.params = params;
    this.k = deriveMotorConstants(params.motor);
    this.motorCount = Math.max(1, Math.round(num(params.flywheel.motorCount, 1)));
    this.gear = params.flywheel.gearRatio; // motor rev per wheel rev
    this.J = params.flywheel.inertiaKgM2; // kg*m^2 reflected to wheel
    this.smooth = params.motor.coulombSmoothRadS;
    this.state = { omegaWheelRadS: 0, thetaMotorRad: 0, currentA: new Array(this.motorCount).fill(0), ballTorqueNm: 0 };
    this._balls = []; // {torque N*m, remainingS}
  }

  get rpm() {
    return (this.state.omegaWheelRadS * 60) / TWO_PI;
  }

  reset(rpm = 0) {
    this.state.omegaWheelRadS = (num(rpm) * TWO_PI) / 60;
    this.state.thetaMotorRad = 0;
    this.state.currentA = new Array(this.motorCount).fill(0);
    this.state.ballTorqueNm = 0;
    this._balls = [];
  }

  /** Constant-torque impulse removing energyJ of kinetic energy at the current speed over durationS. */
  addBallLoad(energyJ, durationS) {
    const E = num(energyJ);
    const w = this.state.omegaWheelRadS;
    if (!(E > 0) || !(w > 0)) return;
    const dur = durationS > 1e-6 ? durationS : 1e-6;
    const wAfter = Math.sqrt(Math.max(0, w * w - (2 * E) / this.J));
    const dL = this.J * (w - wAfter); // N*m*s
    this._balls.push({ torque: dL / dur, remainingS: dur });
  }

  /** Advance h seconds. Returns total motor current [A] summed over motors. */
  step(h, drives) {
    h = num(h);
    if (!(h > 0)) return this.state.currentA.reduce((a, b) => a + b, 0);
    const s = this.state;
    const wW = s.omegaWheelRadS;
    const wM = this.gear * wW;
    const fric = frictionTorque(this.k, wM, this.smooth);
    let torqueW = 0;
    let total = 0;
    for (let i = 0; i < this.motorCount; i++) {
      const d = (drives && drives[i]) || { volts: 0, coast: true };
      const I = motorCurrent(this.k, d.volts, wM, !!d.coast);
      s.currentA[i] = I;
      total += I;
      torqueW += this.gear * (this.k.Kt * I - fric);
    }
    // ball load (only for the part of h it is still active)
    let ballT = 0;
    for (const b of this._balls) {
      const frac = Math.min(1, b.remainingS / h);
      ballT += b.torque * frac;
      b.remainingS -= h;
    }
    this._balls = this._balls.filter((b) => b.remainingS > 1e-12);
    s.ballTorqueNm = ballT;
    const wFree = wW + (h * torqueW) / this.J;
    let wNew = wFree - (h * ballT) / this.J;
    // ball never drives the wheel backwards
    if (wNew < 0 && wFree >= 0 && wW >= 0) wNew = 0;
    if (!Number.isFinite(wNew)) wNew = 0;
    s.omegaWheelRadS = wNew;
    s.thetaMotorRad += this.gear * wNew * h;
    return total;
  }
}

export class Battery {
  constructor(battery) {
    this.p = battery;
    this.voltage = battery.openCircuitV;
  }

  /** Terminal voltage from total current drawn; returns and stores it. */
  update(totalCurrentA) {
    const I = Math.max(0, num(totalCurrentA));
    this.voltage = Math.max(this.p.minV, this.p.openCircuitV - this.p.internalOhm * I);
    return this.voltage;
  }

  reset() {
    this.voltage = this.p.openCircuitV;
  }
}
