// motor.js - DC motor / wheel-shaft torque model (SI: V, A, N*m, rad/s, m/s).
// Frame convention (shared by the area): body x = forward, y = left, omega = CCW+.
// Constants come from ResolvedParams (params.js); nothing is hard-coded here.
// Model: V = p*batteryV, I = (V - ke*omegaM)/R (optionally current-limited),
// tauW = kt*I*G*eta - Coulomb*smoothSign(rim) - viscous*omegaW.
// Zero power: 'float' = open circuit (I = 0), 'brake' = shorted windings (V = 0, back-EMF braking).

/** tanh-smoothed sign. eps <= 0 (or non-finite) means the exact Math.sign. */
export function smoothSign(v, eps) {
  if (!(eps > 0)) return Math.sign(v);
  return Math.tanh(v / eps);
}

/** Armature current and motor-shaft torque. I = (V - ke*w)/R, clamped to +-limit if given. */
export function motorCurrentTorque(motor, volts, omegaMotorRadS, currentLimitA = null) {
  let currentA = (volts - motor.ke * omegaMotorRadS) / motor.resistanceOhm;
  if (currentLimitA != null) {
    const lim = Math.abs(currentLimitA);
    if (currentA > lim) currentA = lim;
    else if (currentA < -lim) currentA = -lim;
  }
  return { currentA, torqueNm: motor.kt * currentA };
}

/** Wheel-shaft torque (N*m) incl. gearing, efficiency, Coulomb + viscous losses and zero-power behaviour. */
export function wheelTorque(p, power, rimSpeedMps) {
  const pw = power > 1 ? 1 : power < -1 ? -1 : (power || 0);
  const omegaW = rimSpeedMps / p.wheelRadiusM;
  const omegaM = p.gearRatio * omegaW;
  let volts;
  let currentA;
  if (pw === 0 && p.zeroPowerBehavior === 'float') {
    volts = 0;
    currentA = 0;
  } else {
    volts = pw * p.batteryV; // brake at pw === 0 gives V = 0 (shorted windings)
    currentA = motorCurrentTorque(p.motor, volts, omegaM, p.currentLimitA).currentA;
  }
  const torqueNm = p.motor.kt * currentA * p.gearRatio * p.gearEfficiency
    - p.wheelCoulombTorqueNm * smoothSign(rimSpeedMps, p.coulombEpsMps)
    - p.wheelViscousNmsPerRad * omegaW;
  return { torqueNm, currentA, volts };
}
