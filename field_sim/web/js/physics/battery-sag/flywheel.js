// flywheel.js - voltage-dependent flywheel: feedforward kF + P on velocity, duty-saturated.
// SI inside; omega is motor shaft speed (rad/s); wheel rpm = omega*60/(2pi)/gearRatio.
import { MOTOR_5203, FLYWHEEL } from './params.js';
import { deriveMotor, motorBusLoad } from './motor.js';

const RPM_TO_RADS = 2 * Math.PI / 60;

export class FlywheelSim {
  constructor(opts = {}) {
    const { params = FLYWHEEL, spec = MOTOR_5203 } = opts;
    this.p = params;
    this.m = deriveMotor(spec);
    this.state = { omega: 0, rpm: 0, duty: 0, iMotor: 0, iBus: 0, targetRpm: 0 };
  }

  setTargetRpm(rpm) { this.state.targetRpm = Math.max(0, rpm || 0); }

  step(dt, vBus, { enabled = true } = {}) {
    const s = this.state, p = this.p, m = this.m;
    const nSub = Math.max(1, Math.ceil(dt / p.substepMaxS - 1e-9));
    const h = dt / nSub;
    const wt = s.targetRpm * RPM_TO_RADS * p.gearRatio;
    let kF = 1 / m.omegaFree;
    if (p.compensation === 'nominal12') kF *= 12 / Math.max(vBus, 1);
    const active = enabled && s.targetRpm !== 0;
    let iBusSum = 0, d = 0, I = 0;
    for (let k = 0; k < nSub; k++) {
      d = active ? Math.max(0, Math.min(1, kF * wt + p.kP * (wt - s.omega))) : 0;
      I = d ? (d * vBus - m.kE * s.omega) / m.rOhm : 0;
      const tau = p.nMotors * (m.kT * I - Math.sign(s.omega) * m.frictionNm)
        - p.viscousNmPerRadS * s.omega;
      s.omega = Math.max(0, s.omega + tau / p.inertiaKgM2 * h);
      iBusSum += p.nMotors * d * I;
    }
    s.duty = d;
    s.iMotor = I;
    s.iBus = iBusSum / nSub;
    s.rpm = s.omega / RPM_TO_RADS / p.gearRatio;
    return { rpm: s.rpm, iBus: s.iBus, duty: s.duty };
  }

  kick(frac = this.p.rpmDipFrac) {
    const s = this.state;
    s.omega *= (1 - frac);
    s.rpm = s.omega / RPM_TO_RADS / this.p.gearRatio;
  }

  // Load for the bus solve, from the last substep's duty and speed.
  busLoad() {
    return motorBusLoad(this.state.duty, this.state.omega, this.m, this.p.nMotors);
  }

  atSpeed(tolRpm = 50) {
    const s = this.state;
    return s.targetRpm > 0 && Math.abs(s.rpm - s.targetRpm) <= tolRpm;
  }

  reset() {
    Object.assign(this.state, { omega: 0, rpm: 0, duty: 0, iMotor: 0, iBus: 0, targetRpm: 0 });
  }
}
