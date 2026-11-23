// motor.js - DC motor model following the WPILib DCMotor construction (recalled from memory).
// SI units. DERIVED from datasheet values in params.js (which are DATASHEET-FROM-MEMORY).
import { RPM_TO_RADS } from './params.js';

/**
 * Build a per-motor electrical model.
 * R = Vnom/I_stall, Kt = T_stall/I_stall, Ke = (Vnom - I_free*R)/w_free, friction = Kt*I_free.
 * @param {object} mp params.motor
 * @returns {{nominalV:number,R:number,Kt:number,Ke:number,freeSpeedRadS:number,frictionTorqueNm:number,count:number,rotorInertiaKgM2:number}}
 */
export function makeDcMotor(mp) {
  const R = mp.nominalV / mp.stallCurrentA;                       // ohm
  const Kt = mp.stallTorqueNm / mp.stallCurrentA;                 // N*m/A
  const freeSpeedRadS = mp.freeSpeedRpm * RPM_TO_RADS;            // rad/s
  const Ke = (mp.nominalV - mp.freeCurrentA * R) / freeSpeedRadS; // V*s/rad
  return {
    nominalV: mp.nominalV, R, Kt, Ke, freeSpeedRadS,
    frictionTorqueNm: Kt * mp.freeCurrentA,
    count: mp.count, rotorInertiaKgM2: mp.rotorInertiaKgM2,
  };
}

/**
 * Drive torque at the wheel for an applied voltage. May be negative (regeneration).
 * @param {object} motor makeDcMotor result
 * @param {{gearRatio:number, efficiency:number}} drive
 * @param {number} voltageV applied to each motor, V
 * @param {number} omegaWheelRadS wheel speed, rad/s
 * @returns {{torqueNm:number, currentA:number}} currentA is the total of all motors
 */
export function driveTorqueAtWheelNm(motor, drive, voltageV, omegaWheelRadS) {
  if (!Number.isFinite(voltageV) || !Number.isFinite(omegaWheelRadS)) return { torqueNm: 0, currentA: 0 };
  const G = drive.gearRatio;
  const i = (voltageV - motor.Ke * G * omegaWheelRadS) / motor.R;
  return { torqueNm: motor.count * G * drive.efficiency * motor.Kt * i, currentA: motor.count * i };
}

/**
 * Bus voltage after battery sag: max(0, V - R_int*max(0, I)).
 * @param {{voltageV:number, internalResOhm:number}} battery
 * @param {number} totalCurrentA total motor current, A
 * @returns {number} volts
 */
export function batteryBusVoltage(battery, totalCurrentA) {
  const I = Number.isFinite(totalCurrentA) ? Math.max(0, totalCurrentA) : 0;
  return Math.max(0, battery.voltageV - battery.internalResOhm * I);
}
