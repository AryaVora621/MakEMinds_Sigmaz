// derive.js - derives motor constants from datasheet endpoints and builds gearmotor constants.
// SI units. Kt = Ts/(Is - If) is a DELIBERATE deviation from WPILib Ts/Is so that both datasheet
// endpoints (free speed at If, stall at Is) hold exactly once Coulomb friction Tf = Kt*If is included.
// Inductance is ignored (quasi-static electrical model).
import { MOTORS, GEARBOXES, GEARBOX_STAGE_EFF } from './params.js';

export function rpmToRadS(rpm) { return rpm * 2 * Math.PI / 60; }
export function radSToRpm(w) { return w * 60 / (2 * Math.PI); }

export function deriveMotor(spec) {
  const V = spec.nominalV, Is = spec.stallCurrentA, If = spec.freeCurrentA, Ts = spec.stallTorqueNm;
  const wf = rpmToRadS(spec.freeSpeedRpm);
  const R = V / Is;               // ohm
  const Kv = wf / (V - R * If);   // (rad/s)/V
  const Ke = 1 / Kv;              // V*s/rad
  const Kt = Ts / (Is - If);      // N*m/A
  const Tf = Kt * If;             // N*m
  return Object.freeze({
    id: spec.id, nominalV: V, R, Kv, Ke, Kt, Tf,
    freeSpeedRadS: wf, stallTorqueNm: Ts, stallCurrentA: Is, freeCurrentA: If,
    rotorJ: spec.rotorJ, cprMotor: spec.cprMotor,
  });
}

function resolveMotor(m) {
  if (typeof m === 'string') {
    const s = MOTORS[m];
    if (!s) throw new Error('unknown motor: ' + m);
    return s;
  }
  return m;
}
function resolveGearbox(g) {
  if (typeof g === 'string') {
    const s = GEARBOXES[g];
    if (!s) throw new Error('unknown gearbox: ' + g);
    return s;
  }
  return g;
}

export function makeGearmotor(motorIdOrSpec, gearboxIdOrSpec, opts = {}) {
  const spec = resolveMotor(motorIdOrSpec);
  const gb = resolveGearbox(gearboxIdOrSpec);
  const n = opts.numMotors ?? 1;
  const extraRatio = opts.extraRatio ?? 1;
  const motor = deriveMotor(spec);
  const G = gb.ratio * extraRatio;
  const eta = opts.efficiency ?? gb.efficiency ?? Math.pow(GEARBOX_STAGE_EFF, gb.stages);
  const ticksPerMechRev = motor.cprMotor * G;
  const freeSpeedOutRadS = motor.freeSpeedRadS / G;
  return Object.freeze({
    id: spec.id + '/' + (typeof gearboxIdOrSpec === 'string' ? gearboxIdOrSpec : 'custom'),
    motor, n, gearboxRatio: gb.ratio, G, eta,
    rotorJReflected: n * motor.rotorJ * G * G,
    ticksPerMechRev,
    freeSpeedOutRadS,
    stallTorqueOutNm: n * G * eta * motor.stallTorqueNm,
    freeTicksPerSec: freeSpeedOutRadS / (2 * Math.PI) * ticksPerMechRev,
  });
}

export function gearmotorFromConfig(cfg) {
  return makeGearmotor(cfg.motor, cfg.gearbox, {
    numMotors: cfg.numMotors ?? cfg.numMotorsPerWheel ?? 1,
    extraRatio: cfg.extraRatio ?? 1,
  });
}
