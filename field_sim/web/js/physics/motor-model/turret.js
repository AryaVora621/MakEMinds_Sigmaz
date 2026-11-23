// turret.js - LEGACY DC-motor turret plant (P controller on degrees, hard stops).
// NOTE: the real robot's turret is now driven by two Axon SERVOS, not a DC motor. This plant is kept as an
// optional DC option only; it replaces the old 540 deg/s rate limit + kp 14 with a motor-driven model.
// Constants come from params.js TURRET (ASSUMPTIONS tagged there). Units: SI internally, degrees at the API edge.
import { TURRET, SIM } from './params.js';
import { gearmotorFromConfig } from './derive.js';
import { MotorUnit } from './dcmotor.js';
import { RotaryMechanism } from './rotary.js';
import { PowerSystem } from './hub.js';

const DEG = Math.PI / 180;
const clamp = (x, lo, hi) => Math.min(Math.max(x, lo), hi);

export class DcTurretPlant {
  constructor({ params = {}, power = null, name = 'turret' } = {}) {
    this.params = { ...TURRET, ...params };
    const p = this.params;
    this.name = name;
    this._ownsPower = !power;
    this.power = power || new PowerSystem();
    this.power.register(name, { zeroPower: p.zeroPower });
    this.gm = gearmotorFromConfig(p);
    this.motor = new MotorUnit(this.gm);
    this.rot = new RotaryMechanism({
      motor: this.motor, J: p.J, minTheta: p.minDeg * DEG, maxTheta: p.maxDeg * DEG, power: this.power, name,
    });
    this._targetDeg = 0;
    this._duty = 0;
  }

  setTargetDeg(deg) {
    const d = Number.isFinite(deg) ? deg : 0;
    this._targetDeg = clamp(d, this.params.minDeg, this.params.maxDeg);
  }
  get targetDeg() { return this._targetDeg; }
  get deg() { return this.rot.theta / DEG; }
  get rateDegS() { return this.rot.omega / DEG; }

  /** Advance dtS seconds; the P controller runs every substep. Returns deg. */
  step(dtS) {
    const dt = Math.min(dtS, SIM.maxFrameDtS);
    if (!(dt > 0)) return this.deg;
    const n = Math.ceil(dt / SIM.substepS - 1e-9);
    const h = dt / n;
    for (let i = 0; i < n; i++) {
      this.power.setPower(this.name, clamp(this.params.kpPowerPerDeg * (this._targetDeg - this.deg), -1, 1));
      this._duty = this.power.duty(this.name);
      this.rot.substep(h, this._duty, this.power.busVoltage(), this.power.zeroPower(this.name));
    }
    if (this._ownsPower) this.power.endFrame(dt);
    return this.deg;
  }

  atTarget(tolDeg = 2) { return Math.abs(this.deg - this._targetDeg) <= tolDeg; }

  telemetry() {
    return {
      deg: this.deg, targetDeg: this._targetDeg, duty: this._duty, statorA: this.motor.lastStatorA,
      windingC: this.motor.windingC,
    };
  }

  reset(deg = 0) {
    this.rot.reset(deg * DEG, 0);
    this.power.setPower(this.name, 0);
    this._targetDeg = deg; this._duty = 0;
  }
}
