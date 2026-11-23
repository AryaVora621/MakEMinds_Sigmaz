// flywheel.js - shooter flywheel plant: RotaryMechanism + QuadEncoder + VelocityPIDF on a (shared or private) PowerSystem.
// Units: SI internally (rad, rad/s, kg, kg*m^2, J, s); rpm and ticks/s only at the API edge.
// All constants come from params.js FLYWHEEL (sourced/ASSUMPTION tags live there). See spec section 8.
// Shot energy: E = 0.5*m*v^2*(1+rotFactor)/shotEff (rotFactor 2/3 hollow sphere, shotEff 0.70: both ASSUMPTIONS).
import { FLYWHEEL, SIM } from './params.js';
import { gearmotorFromConfig, rpmToRadS, radSToRpm } from './derive.js';
import { MotorUnit } from './dcmotor.js';
import { RotaryMechanism } from './rotary.js';
import { PowerSystem } from './hub.js';
import { QuadEncoder, radSToTicksPerSec } from './encoder.js';
import { VelocityPIDF } from './pidf.js';

export class FlywheelPlant {
  constructor({ params = {}, power = null, name = 'flywheel' } = {}) {
    this.params = { ...FLYWHEEL, ...params, pidf: { ...FLYWHEEL.pidf, ...(params.pidf || {}) } };
    const p = this.params;
    this.name = name;
    this._ownsPower = !power;
    this.power = power || new PowerSystem();
    this.power.register(name, { zeroPower: p.zeroPower });
    this.gm = gearmotorFromConfig(p);
    this.motor = new MotorUnit(this.gm);
    this.rot = new RotaryMechanism({ motor: this.motor, J: p.wheelJ + p.extraJ, power: this.power, name });
    this.encoder = new QuadEncoder(this.gm.ticksPerMechRev);
    const pf = p.pidf;
    this.pidf = new VelocityPIDF({
      kP: pf.kP, kI: pf.kI, kD: pf.kD, kF: pf.kF ?? 1 / this.gm.freeTicksPerSec,
      iClamp: pf.iClamp, periodS: pf.periodS, voltageCompensation: pf.voltageCompensation,
    });
    this._target = 0;
    this._t = 0;
    this._ctlAcc = 0;
    this._duty = 0;
  }

  setTargetRpm(rpm) {
    const r = Number.isFinite(rpm) ? rpm : 0;
    this._target = Math.min(Math.max(r, 0), this.params.maxRpm);
  }
  get targetRpm() { return this._target; }
  get omega() { return this.rot.omega; }
  get rpm() { return radSToRpm(this.rot.omega); }

  /** Advance dtS seconds; returns rpm. */
  step(dtS) {
    const dt = Math.min(dtS, SIM.maxFrameDtS);
    if (!(dt > 0)) return this.rpm;
    const n = Math.ceil(dt / SIM.substepS - 1e-9);
    const h = dt / n;
    const period = this.params.pidf.periodS;
    const tgtTps = radSToTicksPerSec(rpmToRadS(this._target), this.gm.ticksPerMechRev);
    for (let i = 0; i < n; i++) {
      this._t += h;
      this._ctlAcc += h;
      if (this._ctlAcc >= period - 1e-12) {
        this._ctlAcc -= period;
        this.encoder.sample(this._t, this.rot.theta);
        if (this._target <= 0) { this.pidf.reset(); this.power.setPower(this.name, 0); }
        else {
          // output held between control ticks; PIDF gates on its own period (exactly one period passed here)
          const out = this.pidf.update(period, tgtTps, this.encoder.velocity, this.power.busVoltage());
          this.power.setPower(this.name, Math.max(0, out)); // a flywheel is never driven in reverse
        }
      }
      this._duty = this.power.duty(this.name);
      this.rot.substep(h, this._duty, this.power.busVoltage(), this.power.zeroPower(this.name));
    }
    if (this._ownsPower) this.power.endFrame(dt);
    return this.rpm;
  }

  ready(tolFrac = this.params.readyTolFrac) {
    return this._target > 0 && Math.abs(this.rpm - this._target) <= tolFrac * this._target;
  }

  /** Remove the ball's energy from the wheel. exitSpeedMps from shot_core. */
  shoot({ ballMassKg = this.params.defaultBallMassKg, exitSpeedMps }) {
    const p = this.params;
    const energyJ = 0.5 * ballMassKg * exitSpeedMps * exitSpeedMps * (1 + p.ballRotFactor) / p.shotEff;
    const rpmBefore = this.rpm;
    this.rot.removeEnergy(energyJ);
    return { rpmBefore, rpmAfter: this.rpm, energyJ };
  }

  surfaceSpeedMps() { return this.rot.omega * this.params.wheelDiameterM / 2; }

  telemetry() {
    return {
      rpm: this.rpm, targetRpm: this._target, duty: this._duty, statorA: this.motor.lastStatorA,
      windingC: this.motor.windingC, encoderTps: this.encoder.velocity, busV: this.power.busVoltage(),
    };
  }

  reset() {
    this.rot.reset();
    this.encoder.reset();
    this.pidf.reset();
    this.power.setPower(this.name, 0);
    this._target = 0; this._t = 0; this._ctlAcc = 0; this._duty = 0;
  }
}
