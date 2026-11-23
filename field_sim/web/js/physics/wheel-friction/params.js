// params.js - the ONLY place tunable constants of the wheel-friction drivetrain live.
// Pure ES module, no DOM. Physics is SI internally (m, s, kg, N, rad); *In / *Lb fields are inputs converted by geometry.js.
// Every constant has a PROVENANCE entry: SOURCED | WEAK | ASSUMED | DERIVED.

export const UNITS = {
  IN_TO_M: 0.0254,        // m per inch
  LB_TO_KG: 0.45359237,   // kg per pound-mass
  G: 9.80665,             // m/s^2, standard gravity
};

// Wheel types. mu values are dimensionless, for FTC EVA foam tile, smooth side up.
export const WHEEL_TYPES = {
  mecanum: {
    axisAngleDeg: 45,     // deg, constrained roller-axis direction relative to the drive direction (sign set by position)
    muStatic: 0.60,       // dimensionless
    muKinetic: 0.51,      // dimensionless
    muLatRatio: 1.0,      // dimensionless, lateral/longitudinal mu (unused for roller wheels)
    crrFree: 0.084,       // dimensionless, free-roller rolling resistance coefficient (force/normal load)
    crrDrive: 0.0,        // dimensionless, driven-axis rolling resistance coefficient
  },
  omni: {
    axisAngleDeg: 0,      // deg
    muStatic: 0.60,       // dimensionless
    muKinetic: 0.50,      // dimensionless
    muLatRatio: 1.0,      // dimensionless
    crrFree: 0.06,        // dimensionless
    crrDrive: 0.0,        // dimensionless
  },
  traction: {
    axisAngleDeg: null,   // deg, null = two constrained axes (no free roller direction)
    muStatic: 0.95,       // dimensionless
    muKinetic: 0.80,      // dimensionless
    muLatRatio: 0.90,     // dimensionless, lateral mu / longitudinal mu
    crrDrive: 0.03,       // dimensionless, rolling resistance on the driven axis
    crrFree: 0,           // dimensionless
  },
};

export const FRICTION = {
  stribeckVelocityMS: 0.05,   // m/s, Stribeck characteristic slip speed
  stribeckExponent: 2,        // dimensionless, LuGre alpha
  stickVelocityMS: 0.01,      // m/s, Karnopp stick band
  slipRatioVMinMS: 0.05,      // m/s, slip-ratio denominator regularizer
  fieldGripScale: 1.0,        // dimensionless multiplier on mu (dust/wear)
  slipKnobGripLoss: 0.5,      // dimensionless, grip fraction lost at slip knob = 1
};

// goBILDA 5203 Yellow Jacket 19.2:1 drive motor
export const MOTOR = {
  freeSpeedRpm: 312,            // rev/min at wheel shaft
  stallTorqueNm: 2.383,         // N*m at wheel shaft (24.3 kg*cm)
  stallCurrentA: 9.2,           // A
  drivetrainEfficiency: 0.95,   // dimensionless, multiplies stall torque
  coulombTorqueNm: 0.08,        // N*m at wheel shaft, dry friction (WPILib kS analogue)
  viscousNmPerRadS: 0.002,      // N*m per rad/s at wheel shaft
  wheelInertiaKgM2: 6e-4,       // kg*m^2, wheel + reflected rotor
  voltageScale: 1.0,            // dimensionless, battery voltage / nominal
  zeroPowerBehavior: 'brake',   // 'brake' | 'float' (FTC DcMotor.ZeroPowerBehavior)
};

// Defaults copied from web/data/robot_spec.json; used when no spec is passed.
export const CHASSIS_DEFAULTS = {
  massLb: 31.4,               // lb
  cgOffsetIn: [0, 0],         // in, CG offset from robot centre [x forward, y left]
  cgHeightIn: 3.0,            // in, CG height above floor
  inertiaScale: 1.0,          // dimensionless multiplier on plate-model yaw inertia
  footprintIn: [15.0, 14.76], // in, [length x, width y] bounding box
  wheels: [                   // x,y,radius in inches, body frame (+x fwd, +y left)
    { name: 'FL', x: 5.04, y: 6.35, radius: 2.047, type: 'mecanum' },
    { name: 'FR', x: 5.04, y: -6.35, radius: 2.047, type: 'mecanum' },
    { name: 'BL', x: -5.04, y: 6.35, radius: 2.047, type: 'mecanum' },
    { name: 'BR', x: -5.04, y: -6.35, radius: 2.047, type: 'mecanum' },
  ],
};

export const SOLVER = {
  maxSubstepS: 1 / 500,   // s
  iterations: 10,         // count, sequential-impulse passes per substep
  maxDtS: 0.1,            // s, frame-spike cap
  maxSubsteps: 200,       // count
  accelFilterTauS: 0.03,  // s, low-pass tau on acceleration used for load transfer
};

export const DRIVER = {
  turnInputScale: 1.0,    // dimensionless multiplier on the turn stick
};

const W = 'WHEEL_TYPES.';
export const PROVENANCE = {
  'UNITS.IN_TO_M': 'DERIVED: exact definition, 1 in = 0.0254 m',
  'UNITS.LB_TO_KG': 'DERIVED: exact definition, 1 lb = 0.45359237 kg',
  'UNITS.G': 'DERIVED: standard gravity, 9.80665 m/s^2 (CGPM 1901)',

  [W + 'mecanum.axisAngleDeg']: 'DERIVED: 45 deg roller axis of a standard mecanum wheel',
  [W + 'mecanum.muStatic']: 'WEAK: AndyMark 4in SD mecanum on FTC soft tile gives mu_FR 0.51 and mu_side 0.67 (andymark.com/products/4-in-sd-mecanum-wheel, search summary only); Chief Delphi consensus 0.6 to 0.7',
  [W + 'mecanum.muKinetic']: 'WEAK: arXiv 1211.2323 braked-wheel mu_s = 0.51 +/- 0.03; Andy Baker 2006 dynamic/static ratio 0.6/0.7 = 0.857',
  [W + 'mecanum.muLatRatio']: 'ASSUMED: 1.0, unused for roller wheels (grip circle)',
  [W + 'mecanum.crrFree']: 'SOURCED: arXiv 1211.2323 free-roller mu_r = 0.084 +/- 0.006 (not measured on foam; plausible range 0.04 to 0.15)',
  [W + 'mecanum.crrDrive']: 'ASSUMED: 0, driven-axis resistance is covered by motor Coulomb torque',

  [W + 'omni.axisAngleDeg']: 'DERIVED: omni rollers are perpendicular to the drive direction (0 deg)',
  [W + 'omni.muStatic']: 'ASSUMED: redraiderrobotics.org/frc-calculator.html gives omni 0.5 on carpet; a VEX forum anecdote gives 0.93',
  [W + 'omni.muKinetic']: 'ASSUMED: same sources as omni.muStatic',
  [W + 'omni.muLatRatio']: 'ASSUMED: 1.0, unused for roller wheels',
  [W + 'omni.crrFree']: 'ASSUMED: no measured value for omni rollers on foam',
  [W + 'omni.crrDrive']: 'ASSUMED: 0, covered by motor Coulomb torque',

  [W + 'traction.axisAngleDeg']: 'DERIVED: null, a traction wheel has two constrained axes and no free roller',
  [W + 'traction.muStatic']: 'WEAK: VEX traction wheel 1.1 on carpet, redraider traction 0.9 on carpet, a disputed HiGrip figure of 1.067 on foam',
  [W + 'traction.muKinetic']: 'WEAK: kinetic/static ratio of 0.85 applied to muStatic; same sources as traction.muStatic',
  [W + 'traction.muLatRatio']: 'ASSUMED: UniTire states mu_x > mu_y is typical but gives no number',
  [W + 'traction.crrFree']: 'DERIVED: 0, traction wheel has no free-roller axis',
  [W + 'traction.crrDrive']: 'ASSUMED: 0.02 to 0.05 carpet-like prior (wheelchair studies: 0.0141 to 0.0203 on carpet)',

  'FRICTION.stribeckVelocityMS': 'ASSUMED: no source gave a value',
  'FRICTION.stribeckExponent': 'SOURCED: LuGre alpha = 2, arXiv 2504.12441',
  'FRICTION.stickVelocityMS': 'ASSUMED: Karnopp stick band',
  'FRICTION.slipRatioVMinMS': 'ASSUMED: regularizer after the Wikipedia Slip_(vehicle_dynamics) eps form',
  'FRICTION.fieldGripScale': 'ASSUMED: dust and wear have no sourced value; sweep range 0.7 to 1.1',
  'FRICTION.slipKnobGripLoss': "ASSUMED: mapping for the existing 0..1 'slip' knob",

  'MOTOR.freeSpeedRpm': 'WEAK: goBILDA vendor figure from memory, verify on gobilda.com 5203-2402-0019',
  'MOTOR.stallTorqueNm': 'WEAK: goBILDA vendor figure from memory (24.3 kg*cm), verify on gobilda.com 5203-2402-0019',
  'MOTOR.stallCurrentA': 'WEAK: goBILDA vendor figure from memory, verify on gobilda.com 5203-2402-0019',
  'MOTOR.drivetrainEfficiency': 'WEAK: JVN uses about 5% loss per stage',
  'MOTOR.coulombTorqueNm': 'ASSUMED: analogue of WPILib kS, which must be measured. Treated as gearmotor-internal friction already contained in the datasheet free speed / stall torque (motor.js electricalLine), so it shapes low-power and coast behaviour without lowering the datasheet line',
  'MOTOR.viscousNmPerRadS': 'ASSUMED: no measurement. Gearmotor-internal like coulombTorqueNm; must stay below stallTorqueNm/freeSpeedRadS (0.069 N*m*s/rad)',
  'MOTOR.wheelInertiaKgM2': 'ASSUMED: wheel 0.5*0.25 kg*0.052^2 = 3.4e-4 plus reflected rotor inertia',
  'MOTOR.voltageScale': 'ASSUMED: nominal battery voltage',
  'MOTOR.zeroPowerBehavior': 'ASSUMED: FTC DcMotor ZeroPowerBehavior used by most drivetrains; float also accepted',

  'CHASSIS_DEFAULTS.massLb': 'SOURCED: web/data/robot_spec.json chassis.mass_lb',
  'CHASSIS_DEFAULTS.cgOffsetIn': 'ASSUMED: CG at robot centre',
  'CHASSIS_DEFAULTS.cgOffsetIn.0': 'ASSUMED: CG at robot centre',
  'CHASSIS_DEFAULTS.cgOffsetIn.1': 'ASSUMED: CG at robot centre',
  'CHASSIS_DEFAULTS.cgHeightIn': 'ASSUMED: 3 in CG height',
  'CHASSIS_DEFAULTS.inertiaScale': 'ASSUMED: Beta8397 virtual_robot suggests about +25% for mecanum bots',
  'CHASSIS_DEFAULTS.footprintIn': 'SOURCED: web/data/robot_spec.json chassis.footprint bounding box',
  'CHASSIS_DEFAULTS.footprintIn.0': 'SOURCED: web/data/robot_spec.json chassis.footprint bbox length',
  'CHASSIS_DEFAULTS.footprintIn.1': 'SOURCED: web/data/robot_spec.json chassis.footprint bbox width',
  'CHASSIS_DEFAULTS.wheels': 'SOURCED: web/data/robot_spec.json wheels[] (y values marked placeholder there)',

  'SOLVER.maxSubstepS': 'ASSUMED: 500 Hz substep for stable stiff friction',
  'SOLVER.iterations': 'ASSUMED: sequential-impulse passes',
  'SOLVER.maxDtS': 'ASSUMED: frame-spike cap',
  'SOLVER.maxSubsteps': 'ASSUMED: cost cap per frame',
  'SOLVER.accelFilterTauS': 'ASSUMED: low-pass filter on acceleration used for load transfer',

  'DRIVER.turnInputScale': 'ASSUMED: 1.0, full stick maps to full rotational twist',
};
for (const w of CHASSIS_DEFAULTS.wheels) {
  for (const k of ['x', 'y', 'radius']) {
    PROVENANCE[`CHASSIS_DEFAULTS.wheels.${w.name}.${k}`] = 'SOURCED: web/data/robot_spec.json wheels[] (y is a placeholder there)';
  }
}

function isPlain(v) {
  return v !== null && typeof v === 'object' && !Array.isArray(v);
}
function merge(base, over) {
  for (const k of Object.keys(over)) {
    const ov = over[k];
    if (isPlain(ov) && isPlain(base[k])) merge(base[k], ov);
    else base[k] = Array.isArray(ov) ? structuredClone(ov) : ov;
  }
  return base;
}
function deepFreeze(o) {
  if (o !== null && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}

/** Deep-merge plain-object overrides into a fresh copy of the defaults (arrays replaced) and deep-freeze. */
export function makeParams(overrides = {}) {
  const base = structuredClone({ UNITS, WHEEL_TYPES, FRICTION, MOTOR, CHASSIS_DEFAULTS, SOLVER, DRIVER });
  if (overrides) merge(base, overrides);
  return deepFreeze(base);
}
