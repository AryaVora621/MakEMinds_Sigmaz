/**
 * mecanumdrive.js - Pedro Pathing 2.1.0 Mecanum drivetrain, ported to plain JS.
 *
 * Source of record (cached 2.1.0 source jar, ftc module):
 *   Pedro:ftc/drivetrains/Mecanum.java  (lines cited per function below)
 *   Pedro:drivetrain/Drivetrain.java:104 (runDrive(Vector...) = runDrive(calculateDrive(...)))
 *
 * Units: powers are dimensionless motor commands in [-1, 1]; vectors are the
 * dimensionless "power vectors" Pedro uses (field frame, x right / y up);
 * angles are radians; velocities xVelocity / yVelocity are in/s; voltage is volts.
 *
 * Deviations from Pedro 2.1.0 (all deliberate, all listed here):
 *   1. calculateDrive does not mutate its input vectors. Java clamps the caller's
 *      Vector objects in place (Mecanum.java:112-118); no Pedro caller reads them afterwards.
 *   2. The voltage sensor is the injected voltage() function (Mecanum.java:222 getVoltage()).
 *   3. maxPowerScaling is clamped to [0, 1] in setMaxPowerScaling (documented Follower.setMaxPower
 *      contract: "caps the motor power from [0, 1]").
 *   4. Zero-voltage guard: if the injected voltage is <= 0 the compensation factor is 1
 *      (Java would divide by zero).
 *   5. If calculateDrive is called without opts.vectors, the library default vectors
 *      (xVelocity 81.34056 in/s, yVelocity 65.43028 in/s, Pedro:ftc/drivetrains/MecanumConstants.java)
 *      are used.
 * Motor direction, zero-power FLOAT/BRAKE bookkeeping and hardware are out of scope here:
 * the plant treats "all powers exactly 0" as FLOAT (Mecanum.java:69,210,218 float on construct/break).
 */

import {
  vPolar, vMag, vTheta, vAdd, vSub, vScale, vSetMag, vRotate,
  findNormalizingScaling, clamp, TAU,
} from './pedromath.js';

/** Library default drive velocities, in/s. Pedro:ftc/drivetrains/MecanumConstants.java (sourced). */
const DEFAULT_X_VELOCITY_INS = 81.34056;
const DEFAULT_Y_VELOCITY_INS = 65.43028;

/**
 * Wheel vectors in robot frame, order [LF, LR, RF, RR].
 * Pedro:ftc/drivetrains/Mecanum.java:72-77.
 * LF and RR sit at thF = theta of normalize(xVelocity, -yVelocity) (MecanumConstants.frontLeftVector
 * default is that vector), LR and RF at 2*PI - thF. Magnitude 1.
 * @param {number} xVelocity forward steady speed, in/s
 * @param {number} yVelocity strafe steady speed, in/s
 * @returns {{x:number,y:number}[]} four unit vectors
 */
export function mecanumWheelVectors(xVelocity, yVelocity) {
  const thF = vTheta({ x: xVelocity, y: -yVelocity });
  const a = vPolar(1, thF);
  const b = vPolar(1, TAU - thF);
  return [a, b, b, a].map((v) => ({ x: v.x, y: v.y }));
}

/**
 * Mixes corrective / heading / pathing vectors into four wheel powers.
 * Pedro:ftc/drivetrains/Mecanum.java:112-193 (line-for-line, including the Cramer's-rule solve
 * at lines 172-175 and the voltage factor at lines 224-232).
 * @param {{x:number,y:number}} corrective  centripetal + translational vector (power units)
 * @param {{x:number,y:number}} heading     vector along the robot heading whose signed magnitude is the turn power
 * @param {{x:number,y:number}} pathing     drive vector along the path tangent
 * @param {number} robotHeading rad
 * @param {object} [opts]
 * @param {number} [opts.maxPowerScaling=1]
 * @param {{x:number,y:number}[]} [opts.vectors] wheel vectors LF,LR,RF,RR (see mecanumWheelVectors)
 * @param {boolean} [opts.useVoltageCompensation=false] MecanumConstants.useVoltageCompensation (default false)
 * @param {number} [opts.nominalVoltage=12] V, MecanumConstants.nominalVoltage
 * @param {number} [opts.staticFrictionCoefficient=0.1] MecanumConstants.staticFrictionCoefficient
 * @param {number} [opts.voltage=12] V, present battery voltage
 * @returns {number[]} [LF, LR, RF, RR] powers, each |p| <= maxPowerScaling
 */
export function calculateDrive(corrective, heading, pathing, robotHeading, opts = {}) {
  const maxP = opts.maxPowerScaling ?? 1;
  const vectors = opts.vectors ?? mecanumWheelVectors(DEFAULT_X_VELOCITY_INS, DEFAULT_Y_VELOCITY_INS);
  const useVc = opts.useVoltageCompensation ?? false;
  const Vn = opts.nominalVoltage ?? 12;
  const sfc = opts.staticFrictionCoefficient ?? 0.1;
  const V = opts.voltage ?? 12;

  // Mecanum.java:112-118 clamp input magnitudes
  let corr = corrective, head = heading, path = pathing;
  if (vMag(corr) > maxP) corr = vSetMag(corr, maxP);
  if (vMag(head) > maxP) head = vSetMag(head, maxP);
  if (vMag(path) > maxP) path = vSetMag(path, maxP);

  let t0, t1; // truePathingVectors[0] = left side, [1] = right side
  if (vMag(corr) === maxP) {
    t0 = { x: corr.x, y: corr.y };
    t1 = { x: corr.x, y: corr.y };
  } else {
    const left = vSub(corr, head);
    const right = vAdd(corr, head);
    if (vMag(left) > maxP || vMag(right) > maxP) {
      const hs = Math.min(
        findNormalizingScaling(corr, head, maxP),
        findNormalizingScaling(corr, vScale(head, -1), maxP),
      );
      t0 = vSub(corr, vScale(head, hs));
      t1 = vAdd(corr, vScale(head, hs));
    } else {
      const leftP = vAdd(left, path);
      const rightP = vAdd(right, path);
      if (vMag(leftP) > maxP || vMag(rightP) > maxP) {
        const ps = Math.min(
          findNormalizingScaling(left, path, maxP),
          findNormalizingScaling(right, path, maxP),
        );
        t0 = vAdd(left, vScale(path, ps));
        t1 = vAdd(right, vScale(path, ps));
      } else {
        t0 = leftP;
        t1 = rightP;
      }
    }
  }
  t0 = vScale(t0, 2.0);
  t1 = vScale(t1, 2.0);

  const m = vectors.map((v) => vRotate(v, robotHeading));
  const w = [0, 0, 0, 0];
  w[0] = (m[1].x * t0.y - t0.x * m[1].y) / (m[1].x * m[0].y - m[0].x * m[1].y);
  w[1] = (m[0].x * t0.y - t0.x * m[0].y) / (m[0].x * m[1].y - m[1].x * m[0].y);
  w[2] = (m[3].x * t1.y - t1.x * m[3].y) / (m[3].x * m[2].y - m[2].x * m[3].y);
  w[3] = (m[2].x * t1.y - t1.x * m[2].y) / (m[2].x * m[3].y - m[3].x * m[2].y);

  if (useVc) {
    // Mecanum.java:228-231
    const vNorm = V > 0 ? (Vn - Vn * sfc) / (V - ((Vn * Vn) / V) * sfc) : 1;
    for (let i = 0; i < 4; i++) w[i] *= vNorm;
  }

  const wMax = Math.max(Math.abs(w[0]), Math.abs(w[1]), Math.abs(w[2]), Math.abs(w[3]));
  if (wMax > maxP) {
    for (let i = 0; i < 4; i++) w[i] = (w[i] / wMax) * maxP;
  }
  return w;
}

/**
 * Drivetrain wrapper: holds the last powers written (after motor caching) and the
 * max-power scale. Pedro:ftc/drivetrains/Mecanum.java:215-247 and Drivetrain.java:104.
 */
export class MecanumDrivetrain {
  /**
   * @param {object} params resolved params from params.js resolveParams()
   * @param {{voltage?:()=>number}} [hooks] voltage(): present battery volts (default params.plant.batteryVoc)
   */
  constructor(params, { voltage = () => params.plant.batteryVoc } = {}) {
    this._p = params;
    this._c = params.mecanum;
    this._voltage = voltage;
    this._vectors = mecanumWheelVectors(this._c.xVelocity, this._c.yVelocity);
    this._maxPower = 1.0; // Drivetrain.maxPowerScaling default 1.0
    this._last = [0, 0, 0, 0]; // Mecanum.java:61 lastMotorPowers
  }

  /** Drivetrain.setMaxPowerScaling; clamped to [0,1] (deviation 3). */
  setMaxPowerScaling(m) { this._maxPower = clamp(m, 0, 1); }
  getMaxPowerScaling() { return this._maxPower; }

  /** Mecanum.java:262 constants.xVelocity, in/s. */
  xVelocity() { return this._c.xVelocity; }
  /** Mecanum.java:267 constants.yVelocity, in/s. */
  yVelocity() { return this._c.yVelocity; }
  get vectors() { return this._vectors; }

  /**
   * Last powers actually written to the motors (after caching). A copy.
   * @returns {number[]} [LF, LR, RF, RR]
   */
  get powers() { return this._last.slice(); }

  /**
   * Drivetrain.java:104 + Mecanum.java:222-232 runDrive caching: write power i only if
   * |last_i - p_i| > motorCachingThreshold or (p_i == 0 and last_i != 0).
   * @returns {number[]} the powers now in effect
   */
  runDrive(corrective, heading, pathing, robotHeading) {
    const p = calculateDrive(corrective, heading, pathing, robotHeading, {
      maxPowerScaling: this._maxPower,
      vectors: this._vectors,
      useVoltageCompensation: this._c.useVoltageCompensation,
      nominalVoltage: this._c.nominalVoltage,
      staticFrictionCoefficient: this._c.staticFrictionCoefficient,
      voltage: this._voltage(),
    });
    const thr = this._c.motorCachingThreshold;
    for (let i = 0; i < 4; i++) {
      if (Math.abs(this._last[i] - p[i]) > thr || (p[i] === 0 && this._last[i] !== 0)) {
        this._last[i] = p[i];
      }
    }
    return this.powers;
  }

  /** Mecanum.java:215-221: all powers 0 and motors to FLOAT. */
  breakFollowing() { this._last = [0, 0, 0, 0]; }
}
