// params.js - the single tunable-constants file for the mecanum-dynamics area, plus resolution into SI.
// FRAME CONVENTION: body frame x = robot forward, y = robot LEFT, z up, omega = counter-clockwise (+).
// Wheels in spec order FL, FR, BL, BR (robot_spec.json wheels[]); do not reorder.
// Units: DEFAULTS are user-facing (inches, lb); resolveParams() returns SI (m, kg, s, N, N*m, rad).
// Provenance tags per constant: SOURCED (with URL/citation, see SOURCES), SPEC (robot_spec.json), DERIVED, ASSUMPTION.
// Precedence in resolveParams: DEFAULTS < robot spec < spec.drive.dynamics < overrides.

export const UNITS = Object.freeze({
  IN_TO_M: 0.0254,       // m/in, exact
  LB_TO_KG: 0.45359237,  // kg/lb, exact
  G: 9.80665,            // m/s^2, standard gravity, exact
});

// goBILDA 5203 Yellow Jacket, 5.2:1 variant. SOURCED: goBILDA 5203-2402-0005 spec sheet (core-electronics listing).
// 1150 RPM no-load at 12 V, 9.2 A stall, 7.9 kgf*cm stall torque, both at the 5.2:1 output; converted to the bare motor shaft.
export const MOTOR_GOBILDA_5203 = Object.freeze({
  name: 'gobilda5203',
  stallTorqueNm: (7.9 * 0.0980665) / 5.2, // N*m, 0.14899 at the motor shaft. SOURCED (7.9 kgf*cm / 5.2)
  freeSpeedRadS: (1150 * 5.2 * 2 * Math.PI) / 60, // rad/s, 626.2. SOURCED (1150 RPM x 5.2 -> motor shaft)
  stallCurrentA: 9.2, // A. SOURCED
  nominalV: 12, // V. SOURCED
});

// Derive electrical constants: R = V/Istall, kt = T/Istall, ke = V/wfree. Free current ignored (not sourced).
export function motorConstants(motor) {
  const m = motor || MOTOR_GOBILDA_5203;
  const { stallTorqueNm, freeSpeedRadS, stallCurrentA, nominalV } = m;
  for (const [k, v] of Object.entries({ stallTorqueNm, freeSpeedRadS, stallCurrentA, nominalV })) {
    if (!Number.isFinite(v) || v <= 0) throw new Error(`motor.${k} must be a positive finite number (got ${v})`);
  }
  return {
    stallTorqueNm,
    freeSpeedRadS,
    stallCurrentA,
    nominalV,
    resistanceOhm: nominalV / stallCurrentA,   // ohm
    kt: stallTorqueNm / stallCurrentA,         // N*m/A
    ke: nominalV / freeSpeedRadS,              // V*s/rad
  };
}

export const DEFAULTS = Object.freeze({
  massLb: 31.4, // lb. SPEC chassis.mass_lb
  wheelRadiusIn: 2.047, // in. SPEC wheels[].radius (104 mm goBILDA GripForce)
  wheelsIn: Object.freeze([ // in, x forward, y left. SPEC wheels[]
    Object.freeze({ name: 'FL', x: 5.04, y: 6.35 }),
    Object.freeze({ name: 'FR', x: 5.04, y: -6.35 }),
    Object.freeze({ name: 'BL', x: -5.04, y: 6.35 }),
    Object.freeze({ name: 'BR', x: -5.04, y: -6.35 }),
  ]),
  footprintLIn: 18, // in, x extent. FTC 18x18 sizing rule; overridden by spec chassis.footprint bbox
  footprintWIn: 18, // in, y extent. same
  inertiaFactor: 1.0, // -, scales plate-formula yaw inertia. Choreo notes the plate formula can underestimate. ASSUMPTION
  yawInertiaKgM2: null, // kg*m^2, null = inertiaFactor*m*(L^2+W^2)/12 (Choreo plate formula)
  rollerAngleDeg: 45, // deg, roller axle vs wheel axle. SOURCED: Diegel 2002 / Gfrerrer 2008 / Wikipedia "Mecanum wheel"
  motor: MOTOR_GOBILDA_5203, // see above. SOURCED
  gearRatio: 19.2, // -, motor:wheel. ASSUMPTION: common FTC 312 RPM drive ratio
  gearEfficiency: 0.85, // -, drivetrain efficiency. ASSUMPTION
  wheelMassKg: 0.236, // kg per wheel. SOURCED: ServoCity GripForce 104 mm listing
  wheelInertiaKgM2: null, // kg*m^2, null = 0.5*wheelMassKg*r^2 (solid disk). ASSUMPTION
  rotorInertiaKgM2: 3.4e-6, // kg*m^2, motor rotor. ASSUMPTION (555-class rotor, ~40 g at r ~13 mm; not sourced)
  batteryV: 12.0, // V, nominal loaded battery. SOURCED (motor nominal voltage)
  currentLimitA: null, // A per motor, null = unlimited. ASSUMPTION
  mu: 0.7, // -, tire-floor friction coefficient. ASSUMPTION, needs measurement (gives ~164 in/s^2 peak accel)
  normalSplit: Object.freeze([0.25, 0.25, 0.25, 0.25]), // -, fraction of weight per wheel (sums to 1). ASSUMPTION: even
  kS: 0.06, // -, Coulomb friction as fraction of wheel stall torque. CALIBRATED to Pedro example forwardZeroPowerAcceleration -25.9 in/s^2 (low-confidence snippet)
  wheelViscousNmsPerRad: 0, // N*m*s/rad at wheel shaft. ASSUMPTION (none)
  crrBase: 0.0088, // -, floor rolling-resistance coefficient. DERIVED from Duda et al., Sensors 2025 (0 deg pull 1.040 N / 117.8 N)
  crrRoller: 0.045, // -, roller Coulomb loss along the roller free direction, per unit wheel load (net lateral force on the chassis = sqrt(1+sigma^2)*crrRoller*N). DERIVED from Duda et al. 2025 (60-90 deg, omni wheel)
  // N*s/m per wheel, roller viscous loss vs roller surface speed u = sqrt(1+sigma^2)*cy (see forces.js).
  // CALIBRATED (round 2, after the roller-speed kinematics fix): 0.6 gives strafe/forward steady speed 0.9095
  // (= 1/1.0995, gm0 strafe multiplier 1.1) and float lateral coast decel 66.7 in/s^2 (Pedro
  // lateralZeroPowerAcceleration -67 in/s^2) at the same time. Round-1 value 2.3 was fitted to the wrong u = c.d.
  rollerViscousNsPerM: 0.6,
  coulombEpsMps: 0.02, // m/s, tanh smoothing width for Coulomb terms. numerical choice
  zeroPowerBehavior: 'brake', // 'brake' | 'float'. ASSUMPTION: FTC drive default practice
  lateralMultiplier: 1.0, // -, strafe command scale. gm0 uses 1.1 (optional override); default ideal
  turnScale: 1.0, // -, turn command scale
  desaturateMode: 'max', // 'max' (WPILib) | 'gm0'
  maxSubstepS: 0.002, // s, integrator substep limit. numerical choice
  gravity: UNITS.G, // m/s^2. SOURCED: standard gravity
});

export const SOURCES = Object.freeze({
  massLb: 'robot_spec.json chassis.mass_lb',
  wheelRadiusIn: 'robot_spec.json wheels[].radius; goBILDA 104 mm GripForce',
  wheelsIn: 'robot_spec.json wheels[] x,y',
  footprintLIn: 'FTC 18 in sizing rule; robot_spec.json chassis.footprint bbox overrides',
  footprintWIn: 'FTC 18 in sizing rule; robot_spec.json chassis.footprint bbox overrides',
  inertiaFactor: 'ASSUMPTION; Choreo warns plate formula can underestimate (https://choreo.autos)',
  yawInertiaKgM2: 'Choreo plate formula m(L^2+W^2)/12',
  rollerAngleDeg: 'Diegel 2002; Gfrerrer 2008; https://en.wikipedia.org/wiki/Mecanum_wheel',
  motor: 'goBILDA 5203-2402-0005 spec sheet (core-electronics listing)',
  gearRatio: 'ASSUMPTION: 19.2:1 (312 RPM) common FTC drive ratio',
  gearEfficiency: 'ASSUMPTION',
  wheelMassKg: 'ServoCity GripForce 104 mm listing',
  wheelInertiaKgM2: 'ASSUMPTION: solid disk 0.5 m r^2',
  rotorInertiaKgM2: 'ASSUMPTION: 555-class rotor, not sourced',
  batteryV: 'motor nominal voltage 12 V',
  currentLimitA: 'ASSUMPTION: none',
  mu: 'ASSUMPTION, needs measurement',
  normalSplit: 'ASSUMPTION: even split; see weight-transfer area for dynamic split',
  kS: 'CALIBRATED: Pedro example forwardZeroPowerAcceleration -25.9 in/s^2; WPILib kS idea, F_s = kS*nGKt/(R r)',
  wheelViscousNmsPerRad: 'ASSUMPTION: none',
  crrBase: 'DERIVED: Duda et al., Sensors 2025, 1.040 N / 117.8 N',
  crrRoller: 'DERIVED: Duda et al., Sensors 2025, omni wheel 60-90 deg',
  rollerViscousNsPerM: 'CALIBRATED: gm0 strafe multiplier 1.1 (https://gm0.org) -> strafe/forward 0.909; Pedro lateralZeroPowerAcceleration -67 in/s^2',
  coulombEpsMps: 'numerical smoothing choice',
  zeroPowerBehavior: 'ASSUMPTION: FTC brake default',
  lateralMultiplier: 'gm0 uses 1.1 (https://gm0.org); default 1.0',
  turnScale: 'default 1.0',
  desaturateMode: 'WPILib MecanumDriveWheelSpeeds.desaturate; gm0 denominator',
  maxSubstepS: 'numerical choice; stability analysis in spec',
  gravity: 'standard gravity 9.80665 m/s^2',
});

const NUM_KEYS = [
  'massLb', 'wheelRadiusIn', 'footprintLIn', 'footprintWIn', 'inertiaFactor', 'rollerAngleDeg', 'gearRatio',
  'gearEfficiency', 'wheelMassKg', 'rotorInertiaKgM2', 'batteryV', 'mu', 'kS', 'wheelViscousNmsPerRad', 'crrBase',
  'crrRoller', 'rollerViscousNsPerM', 'coulombEpsMps', 'lateralMultiplier', 'turnScale', 'maxSubstepS', 'gravity',
];
const OPT_NUM_KEYS = ['yawInertiaKgM2', 'wheelInertiaKgM2', 'currentLimitA'];

function specDerived(spec) {
  const o = {};
  if (!spec || typeof spec !== 'object') return o;
  if (spec.chassis) {
    if (spec.chassis.mass_lb != null) o.massLb = spec.chassis.mass_lb;
    const fp = spec.chassis.footprint;
    if (Array.isArray(fp) && fp.length) {
      const xs = fp.map((p) => p[0]);
      const ys = fp.map((p) => p[1]);
      o.footprintLIn = Math.max(...xs) - Math.min(...xs);
      o.footprintWIn = Math.max(...ys) - Math.min(...ys);
    }
  }
  if (Array.isArray(spec.wheels) && spec.wheels.length) {
    o.wheelsIn = spec.wheels.map((w) => {
      const e = { name: w.name, x: w.x, y: w.y };
      if (w.sigma != null) e.sigma = w.sigma;
      return e;
    });
    const rs = spec.wheels.map((w) => w.radius).filter((r) => r != null);
    if (rs.length) o.wheelRadiusIn = rs.reduce((a, b) => a + b, 0) / rs.length;
  }
  return o;
}

function req(cond, msg) {
  if (!cond) throw new Error(`resolveParams: ${msg}`);
}

export function resolveParams(spec = null, overrides = {}) {
  const dyn = spec && spec.drive && typeof spec.drive.dynamics === 'object' && spec.drive.dynamics ? spec.drive.dynamics : {};
  const c = { ...DEFAULTS, ...specDerived(spec), ...dyn, ...(overrides || {}) };

  for (const k of NUM_KEYS) req(Number.isFinite(c[k]), `${k} must be a finite number (got ${c[k]})`);
  for (const k of OPT_NUM_KEYS) req(c[k] === null || c[k] === undefined || Number.isFinite(c[k]), `${k} must be null or a finite number (got ${c[k]})`);
  req(c.massLb > 0, 'massLb must be > 0');
  req(c.wheelRadiusIn > 0, 'wheelRadiusIn must be > 0');
  req(c.gearRatio > 0, 'gearRatio must be > 0');
  req(c.batteryV > 0, 'batteryV must be > 0');
  req(c.mu >= 0, 'mu must be >= 0');
  req(c.gearEfficiency > 0, 'gearEfficiency must be > 0');
  req(c.maxSubstepS > 0, 'maxSubstepS must be > 0');
  req(c.gravity > 0, 'gravity must be > 0');
  req(c.zeroPowerBehavior === 'brake' || c.zeroPowerBehavior === 'float', `zeroPowerBehavior must be 'brake' or 'float' (got ${c.zeroPowerBehavior})`);
  req(c.desaturateMode === 'max' || c.desaturateMode === 'gm0', `desaturateMode must be 'max' or 'gm0' (got ${c.desaturateMode})`);
  req(Array.isArray(c.wheelsIn) && c.wheelsIn.length === 4, 'exactly 4 wheels are required');
  req(Array.isArray(c.normalSplit) && c.normalSplit.length === 4 && c.normalSplit.every(Number.isFinite), 'normalSplit must be 4 finite numbers');
  req(Math.abs(c.normalSplit.reduce((a, b) => a + b, 0) - 1) <= 1e-6, 'normalSplit must sum to 1 (within 1e-6)');

  const { IN_TO_M, LB_TO_KG } = UNITS;
  const massKg = c.massLb * LB_TO_KG;
  const r = c.wheelRadiusIn * IN_TO_M;
  const cotDelta = 1 / Math.tan((c.rollerAngleDeg * Math.PI) / 180);
  const motor = Object.freeze(motorConstants(c.motor));

  const wheels = c.wheelsIn.map((w, i) => {
    req(Number.isFinite(w.x) && Number.isFinite(w.y), `wheel ${i} x/y must be finite`);
    const x = w.x * IN_TO_M;
    const y = w.y * IN_TO_M;
    let sigma;
    if (w.sigma != null) {
      sigma = w.sigma;
      req(Number.isFinite(sigma), `wheel ${i} sigma must be finite`);
    } else {
      req(w.x * w.y !== 0, `wheel ${i} has x*y == 0; roller sign needs an explicit sigma`);
      sigma = -Math.sign(w.x * w.y) * cotDelta;
    }
    return Object.freeze({ name: w.name ?? `W${i}`, x, y, sigma, k: sigma * x - y });
  });
  const kMax = Math.max(...wheels.map((w) => Math.abs(w.k)));
  req(kMax > 0, 'all wheel yaw levers are zero');

  const Lm = c.footprintLIn * IN_TO_M;
  const Wm = c.footprintWIn * IN_TO_M;
  const yawI = c.yawInertiaKgM2 ?? (c.inertiaFactor * massKg * (Lm * Lm + Wm * Wm)) / 12;
  req(Number.isFinite(yawI) && yawI > 0, 'yawInertiaKgM2 must be > 0');

  const jWheel = c.wheelInertiaKgM2 ?? 0.5 * c.wheelMassKg * r * r;
  const jw = jWheel + c.rotorInertiaKgM2 * c.gearRatio * c.gearRatio;

  return Object.freeze({
    massKg,
    yawInertiaKgM2: yawI,
    wheelRadiusM: r,
    wheels: Object.freeze(wheels),
    kMax,
    motor,
    gearRatio: c.gearRatio,
    gearEfficiency: c.gearEfficiency,
    wheelSpinInertiaKgM2: jw,
    batteryV: c.batteryV,
    currentLimitA: c.currentLimitA ?? null,
    mu: c.mu,
    gripScale: 1,
    normalForceN: Object.freeze(c.normalSplit.map((f) => massKg * c.gravity * f)),
    wheelCoulombTorqueNm: c.kS * motor.kt * motor.stallCurrentA * c.gearRatio * c.gearEfficiency,
    wheelViscousNmsPerRad: c.wheelViscousNmsPerRad,
    crrBase: c.crrBase,
    crrRoller: c.crrRoller,
    rollerViscousNsPerM: c.rollerViscousNsPerM,
    coulombEpsMps: c.coulombEpsMps,
    zeroPowerBehavior: c.zeroPowerBehavior,
    lateralMultiplier: c.lateralMultiplier,
    turnScale: c.turnScale,
    desaturateMode: c.desaturateMode,
    maxSubstepS: c.maxSubstepS,
    gravity: c.gravity,
  });
}
