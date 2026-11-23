// losses.js - flywheel shaft loss torques and analytic coast-down (flywheel-dynamics spec 2c).
// Pure ES module, no DOM. SI units: rad/s, N*m, kg*m^2, s, m.
// All loss torques are MAGNITUDES at the wheel shaft (>= 0) unless stated.
// Constants live in params.js (this file holds no tunable numbers).
// Source tags for the formulas used here:
//   Coulomb + viscous: ASSUMPTION model (values in params.js).
//   Motor friction torque = Kt*I_free: DATASHEET-FROM-MEMORY (goBILDA free current), see motor.js.
//   Windage: von Karman laminar disk, C_m = 3.87/sqrt(Re) for both faces, UNVERIFIED.
//   Brake torque: DERIVED, shorted windings i = -Ke*w_m/R, T = count*G*eta*Kt*i.

const fin = (x) => (typeof x === 'number' && Number.isFinite(x));
const num = (x, d = 0) => (fin(x) ? x : d);

/**
 * Constant (Coulomb) friction torque at the wheel shaft.
 * @param {object} losses params.losses
 * @param {{count:number, frictionTorqueNm:number}} motor makeDcMotor result
 * @param {{gearRatio:number}} drive params.drive
 * @returns {number} N*m, >= 0
 */
export function coulombTorqueNm(losses, motor, drive) {
  const bearing = Math.max(0, num(losses && losses.bearingCoulombNm));
  let motorPart = 0;
  if (losses && losses.motorFrictionFromFreeCurrent && motor) {
    motorPart = Math.max(0, num(motor.count) * num(motor.frictionTorqueNm) * num(drive && drive.gearRatio, 1));
  }
  return bearing + motorPart;
}

/**
 * Air drag (windage) torque on the spinning wheel, DERIVED from a laminar disk
 * moment coefficient. Re = |w| R^2 / nu; T = 0.5 rho w^2 R^5 C_m, C_m = c/sqrt(Re).
 * @param {number} omegaRadS wheel speed, rad/s
 * @param {object} windage params.losses.windage
 * @param {number} radiusM disk radius, m
 * @returns {number} N*m, >= 0 (0 when disabled or Re < 1)
 */
export function windageTorqueNm(omegaRadS, windage, radiusM) {
  if (!windage || !windage.enabled) return 0;
  const w = Math.abs(num(omegaRadS));
  const R = num(radiusM);
  const nu = num(windage.kinematicViscosityM2s);
  if (w === 0 || R <= 0 || nu <= 0) return 0;
  const re = w * R * R / nu;
  if (!(re >= 1)) return 0;
  const cm = num(windage.cmLaminarCoeff) / Math.sqrt(re);
  const t = 0.5 * num(windage.airDensityKgM3) * w * w * Math.pow(R, 5) * cm;
  return t > 0 && Number.isFinite(t) ? t : 0;
}

/**
 * All loss torque magnitudes at wheel speed omega.
 * @param {number} omegaRadS rad/s
 * @param {{losses:object, motor:object, drive:object, radiusM:number}} ctx
 * @returns {{coulombNm:number, viscousNm:number, windageNm:number, totalNm:number}} N*m, all >= 0
 */
export function lossTorquesNm(omegaRadS, ctx) {
  const c = ctx || {};
  const w = Math.abs(num(omegaRadS));
  const coulombNm = coulombTorqueNm(c.losses, c.motor, c.drive);
  const viscousNm = Math.max(0, num(c.losses && c.losses.viscousNmsPerRad)) * w;
  const windageNm = windageTorqueNm(w, c.losses && c.losses.windage, c.radiusM);
  return { coulombNm, viscousNm, windageNm, totalNm: coulombNm + viscousNm + windageNm };
}

/**
 * Electrical braking torque with the motor windings shorted (zeroPowerBehavior 'brake').
 * Signed: opposes omega. T = -count*G^2*eta*Kt*Ke*w/R.
 * @param {number} omegaWheelRadS rad/s
 * @param {{count:number, Kt:number, Ke:number, R:number}} motor
 * @param {{gearRatio:number, efficiency:number}} drive
 * @returns {number} N*m at the wheel
 */
export function brakeTorqueAtWheelNm(omegaWheelRadS, motor, drive) {
  if (!motor || !drive) return 0;
  const G = num(drive.gearRatio, 1);
  const eta = num(drive.efficiency, 1);
  const R = num(motor.R);
  if (R <= 0) return 0;
  const t = -num(motor.count) * G * G * eta * num(motor.Kt) * num(motor.Ke) * num(omegaWheelRadS) / R;
  return Number.isFinite(t) ? t : 0;
}

/**
 * Analytic coast-down speed under J dw/dt = -(Tc + b w), no windage, no drive.
 * w(t) = (w0 + Tc/b) exp(-b t/J) - Tc/b, clamped at 0; b = 0: w0 - Tc t/J.
 * @param {number} jKgM2 inertia, kg*m^2
 * @param {number} tcNm Coulomb torque, N*m
 * @param {number} bNmsPerRad viscous coefficient, N*m*s/rad
 * @param {number} omega0RadS initial speed (>= 0), rad/s
 * @param {number} tS elapsed time, s
 * @returns {number} rad/s
 */
export function coastDownOmega(jKgM2, tcNm, bNmsPerRad, omega0RadS, tS) {
  const J = num(jKgM2), tc = Math.max(0, num(tcNm)), b = Math.max(0, num(bNmsPerRad));
  const w0 = num(omega0RadS), t = num(tS);
  if (!(J > 0) || w0 <= 0) return 0;
  if (t <= 0) return w0;
  let w;
  if (b === 0) w = w0 - tc * t / J;
  else w = (w0 + tc / b) * Math.exp(-b * t / J) - tc / b;
  return w > 0 && Number.isFinite(w) ? w : 0;
}

/**
 * Time for the wheel to stop: (J/b) ln(1 + b w0/Tc), or J w0/Tc when b = 0.
 * @param {number} jKgM2 kg*m^2
 * @param {number} tcNm N*m
 * @param {number} bNmsPerRad N*m*s/rad
 * @param {number} omega0RadS rad/s
 * @returns {number} seconds; Infinity if Tc = 0 (never stops), 0 if w0 <= 0
 */
export function coastDownStopTimeS(jKgM2, tcNm, bNmsPerRad, omega0RadS) {
  const J = num(jKgM2), tc = Math.max(0, num(tcNm)), b = Math.max(0, num(bNmsPerRad));
  const w0 = num(omega0RadS);
  if (!(J > 0) || w0 <= 0) return 0;
  if (tc === 0) return Infinity;
  if (b === 0) return J * w0 / tc;
  return (J / b) * Math.log(1 + b * w0 / tc);
}
