// motor.js - DC motor model (terminal inductance neglected). SI units: V, A, ohm, rad/s, N*m.
// Constants come from params.js (MOTOR_5203 is [S] goBILDA 5203 listing, INTAKE is [A]).
import { MOTOR_5203, INTAKE } from './params.js';

// Derive R, kE, kT, friction from the catalogue stall/free figures.
export function deriveMotor(spec = MOTOR_5203) {
  const rOhm = spec.vRated / spec.stallA;                    // ohm
  const omegaFree = spec.freeRpm * 2 * Math.PI / 60;         // rad/s
  const kE = (spec.vRated - spec.freeA * rOhm) / omegaFree;  // V*s/rad
  const kT = kE;                                             // N*m/A (SI: kT == kE)
  const frictionNm = kT * spec.freeA;                        // N*m
  return { rOhm, kE, kT, frictionNm, omegaFree, vRated: spec.vRated };
}

// Winding current (A), positive = motoring. Float (duty 0) gives 0.
export function motorCurrent(duty, vBus, omega, m) {
  if (duty === 0) return 0;
  return (duty * vBus - m.kE * omega) / m.rOhm;
}

// Bus-side current (A); negative is regen.
export function motorBusCurrent(duty, vBus, omega, m) {
  if (duty === 0) return 0;
  return duty * motorCurrent(duty, vBus, omega, m);
}

// Shaft torque (N*m) with Coulomb friction opposing rotation.
export function motorTorque(iMotor, omega, m) {
  return m.kT * iMotor - Math.sign(omega) * m.frictionNm;
}

// Bus load, linear in V: Ibus = g*V - a.
export function motorBusLoad(duty, omega, m, n = 1) {
  if (duty === 0) return { kind: 'linear', g: 0, a: 0 };
  return {
    kind: 'linear',
    g: n * duty * duty / m.rOhm,
    a: n * duty * m.kE * omega / m.rOhm,
  };
}

// Generic motor + inertia + constant load torque (intake). Euler, substeps <= 0.002 s.
export class DcMotorSim {
  constructor(opts = {}) {
    const {
      spec = MOTOR_5203,
      nMotors = INTAKE.nMotors,
      gearRatio = INTAKE.gearRatio,
      inertiaKgM2 = INTAKE.inertiaKgM2,
      loadNm = INTAKE.loadNm,
    } = opts;
    this.m = deriveMotor(spec);
    this.n = nMotors;
    this.gearRatio = gearRatio;
    this.inertia = inertiaKgM2;
    this.loadNm = loadNm;
    this.state = { omega: 0, duty: 0, iMotor: 0, iBus: 0 };
  }

  // omega is the motor shaft speed (rad/s). loadNm is referred to the motor shaft.
  step(duty, vBus, dt) {
    const s = this.state, m = this.m;
    const d = Math.max(-1, Math.min(1, duty));
    const nSub = Math.max(1, Math.ceil(dt / 0.002 - 1e-9));
    const h = dt / nSub;
    let iBusSum = 0, iMotor = 0;
    for (let k = 0; k < nSub; k++) {
      iMotor = motorCurrent(d, vBus, s.omega, m);
      const tau = d === 0 ? 0 : this.n * motorTorque(iMotor, s.omega, m);
      // load opposes motion; at rest it cannot drive the shaft backwards
      const load = Math.sign(s.omega) * this.loadNm;
      let w = s.omega + (tau - load) / this.inertia * h;
      if (s.omega === 0 && Math.abs(tau) <= this.loadNm) w = 0;
      s.omega = w;
      iBusSum += this.n * d * iMotor;
    }
    s.duty = d;
    s.iMotor = iMotor;
    s.iBus = nSub > 0 ? iBusSum / nSub : 0;
    return { omega: s.omega, iMotor: s.iMotor, iBus: s.iBus };
  }

  busLoad() {
    return motorBusLoad(this.state.duty, this.state.omega, this.m, this.n);
  }
}
