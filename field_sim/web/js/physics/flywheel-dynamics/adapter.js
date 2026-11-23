// adapter.js - glue between FlywheelPlant and an aimcontrol.js-style ShooterPlant (duck-typed, no import).
import { FlywheelPlant } from './flywheel_plant.js';

/** Metres to inches [in/m] (1/0.0254). */
export const M_TO_IN = 1 / 0.0254;

/**
 * Monkeypatch a ShooterPlant instance so rpm comes from the physical flywheel model.
 * step(dt, cmd): original step runs with cmd.rpm removed; target set only if cmd.rpm finite; flywheel steps; state.rpm written.
 * kick(ball): fires the flywheel, writes state.rpm, stores plant.lastShot, returns the shot.
 * @param {{step:Function, kick?:Function, state:{rpm:number}}} shooterPlant
 * @param {object} [params] overrides for FlywheelPlant
 */
export function wrapShooterPlant(shooterPlant, params) {
  const fw = new FlywheelPlant(params);
  const origStep = shooterPlant.step;
  const origKick = shooterPlant.kick;
  if (shooterPlant.state && Number.isFinite(shooterPlant.state.rpm)) fw.reset(shooterPlant.state.rpm);
  shooterPlant.flywheel = fw;
  shooterPlant.lastShot = null;
  shooterPlant._origStep = origStep;
  shooterPlant._origKick = origKick;
  shooterPlant.step = function (dt, cmd) {
    let c = cmd;
    let rpmCmd = null;
    if (cmd && typeof cmd === 'object') {
      const { rpm, ...rest } = cmd;
      c = rest; rpmCmd = rpm;
    }
    const ret = typeof origStep === 'function' ? origStep.call(shooterPlant, dt, c) : undefined;
    if (Number.isFinite(rpmCmd)) fw.setTargetRpm(rpmCmd);
    fw.step(dt);
    shooterPlant.state.rpm = fw.state.rpm;
    return ret;
  };
  shooterPlant.kick = function (ball) {
    const shot = fw.fire(ball);
    shooterPlant.state.rpm = fw.state.rpm;
    shooterPlant.lastShot = shot;
    return shot;
  };
  return shooterPlant;
}

/** Standalone shooter wrapper around FlywheelPlant. */
export class FlywheelShooterAdapter {
  /** @param {object} [params] */
  constructor(params) {
    this.flywheel = new FlywheelPlant(params);
    this.lastShot = null;
  }
  /** Step dtS [s] with optional rpm command (null holds target). Returns rpm. */
  step(dtS, cmdRpm = null) {
    if (Number.isFinite(cmdRpm)) this.flywheel.setTargetRpm(cmdRpm);
    this.flywheel.step(dtS);
    return this.flywheel.state.rpm;
  }
  /** Fire a ball; returns the shot result. */
  kick(ball) {
    this.lastShot = this.flywheel.fire(ball);
    return this.lastShot;
  }
  get rpm() { return this.flywheel.state.rpm; }
  /** Exit speed in inches/s. */
  exitSpeedInS(shot) { return (shot && Number.isFinite(shot.exitSpeedMps) ? shot.exitSpeedMps : 0) * M_TO_IN; }
}
