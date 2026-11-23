// Per-wheel DC motor model (SI: rad/s, N*m, s, kg*m^2). Pure functions, no imports.
// `motor` is cfg.motor: { freeSpeedRadS, stallTorqueNm (already x efficiency),
//   coulombTorqueNm, viscousNmPerRadS, voltageScale, zeroPowerBehavior }.
// Values behind these fields are tagged in params.js PROVENANCE (WEAK / ASSUMED).
//
// Loss bookkeeping (round 2 fix). freeSpeedRadS and stallTorqueNm are DATASHEET values: they are
// measured on the unloaded gearmotor output, so the gearmotor's own dry (coulombTorqueNm, Tc) and
// viscous (viscousNmPerRadS, B) friction are ALREADY inside them. Subtracting Tc and B*omega from
// the datasheet line a second time double-counted the losses: the free-spinning wheel settled at
// (Ts - Tc) / (Ts/wF + B) = 30.6 rad/s, 6.3% under the 32.67 rad/s datasheet free speed.
// The model now uses the INTERNAL electrical line (what the windings produce before friction):
//   Ts_e = Ts + Tc                     (at stall the output loses Tc to dry friction)
//   w0_e = Ts_e / (Ts/wF - B)          (chosen so net output is 0 exactly at wF)
// so that at full power the NET output Ts_e*(1 - w/w0_e) - B*w - Tc == Ts*(1 - w/wF) for w > 0,
// i.e. the datasheet line is reproduced exactly, and the free spin converges to wF.
// Same convention as physics/motor-model/derive.js (Kt*Istall - Tf = datasheet stall torque).
// Requires Ts/wF > B (else no positive free speed exists); electricalLine throws otherwise.

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

function isFloatIdle(power, motor) {
  return motor.zeroPowerBehavior === 'float' && power === 0;
}

/**
 * Internal electrical torque-speed line derived from the datasheet values and the friction terms.
 * @returns {{stallNm:number, freeRadS:number, slopeNmPerRadS:number}} stallNm N*m, freeRadS rad/s,
 *   slopeNmPerRadS = stallNm/freeRadS (back-EMF damping), N*m per rad/s
 */
export function electricalLine(motor) {
  const Ts = motor.stallTorqueNm, wF = motor.freeSpeedRadS;
  const Tc = motor.coulombTorqueNm || 0, B = motor.viscousNmPerRadS || 0;
  const slope = Ts / wF - B; // N*m per rad/s
  if (!(slope > 0)) throw new Error('wheel-friction: viscousNmPerRadS must be < stallTorqueNm / freeSpeedRadS');
  const stallNm = Ts + Tc;
  return { stallNm, freeRadS: stallNm / slope, slopeNmPerRadS: slope };
}

/**
 * Motor shaft torque on the wheel EXCLUDING Coulomb friction [N*m]: Ts_e*(p*vScale - omega/w0_e) - B*omega.
 * Float with p == 0: the back-EMF term is skipped, only viscous drag remains (-B*omega).
 */
export function motorTorqueNm(power, omegaRadS, motor) {
  const p = clamp(power, -1, 1);
  const B = motor.viscousNmPerRadS;
  if (isFloatIdle(p, motor)) return -B * omegaRadS;
  const e = electricalLine(motor);
  return e.stallNm * p * motor.voltageScale - e.slopeNmPerRadS * omegaRadS - B * omegaRadS;
}

/**
 * Coulomb (dry) torque as a clamp toward zero: omega - clamp(omega, -T*h/J, T*h/J).
 * Never reverses the sign of omega. Used for motor kS and traction crrDrive*N*R.
 */
export function applyCoulombTorque(omegaRadS, torqueNm, h, inertiaKgM2) {
  const lim = Math.abs(torqueNm) * h / inertiaKgM2;
  return omegaRadS - clamp(omegaRadS, -lim, lim);
}

/**
 * Implicit (backward-Euler) motor update over substep h [s], then Coulomb clamp.
 * Returns { omega [rad/s], torqueNm [N*m], inertiaEffKgM2 [kg*m^2] }.
 * inertiaEffKgM2 = J + h*k (k = back-EMF + viscous damping, N*m per rad/s) is the wheel's response
 * inertia for ANY further impulse in this substep under backward Euler:
 *   J (w' - w) = h*drive - h*k*w' - c*R*Lambda  =>  dw'/dLambda = -c*R / (J + h*k).
 * The contact solver must use it, otherwise back-EMF is evaluated at the free-spin predictor speed
 * instead of the end-of-substep speed (that split lost ~18% of motor torque for a rolling wheel).
 * Reported torque is the net shaft torque at the new omega including Coulomb friction;
 * it is 0 for float with power 0 (spec section 5).
 */
export function stepWheelMotor(omegaRadS, power, h, motor, inertiaKgM2) {
  const p = clamp(power, -1, 1);
  const B = motor.viscousNmPerRadS;
  const Tc = motor.coulombTorqueNm;
  const floatIdle = isFloatIdle(p, motor);
  let drive = 0, k = B;
  if (!floatIdle) {
    const e = electricalLine(motor);
    drive = e.stallNm * p * motor.voltageScale;
    k = e.slopeNmPerRadS + B;
  }
  const Jeff = inertiaKgM2 + h * k;
  let omega = (inertiaKgM2 * omegaRadS + h * drive) / Jeff;
  omega = applyCoulombTorque(omega, Tc, h, Jeff);
  let torqueNm = 0;
  if (!floatIdle) {
    const t = drive - k * omega;
    if (omega !== 0) torqueNm = t - Math.sign(omega) * Tc;
    else torqueNm = t - Math.sign(t) * Math.min(Tc, Math.abs(t));
  }
  return { omega, torqueNm, inertiaEffKgM2: Jeff };
}
