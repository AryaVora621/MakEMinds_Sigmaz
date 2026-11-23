// params.js - every tunable of the pedro-fidelity area, with units and a citation per value.
//
// Source of record: the Pedro Pathing 2.1.0 binaries the robot links (core-2.1.0.jar, ftc-2.1.0.aar in the Gradle
// cache). FollowerConstants: the no-arg ctor (FollowerConstants.java:251) calls defaults() (FollowerConstants.java:601),
// which OVERWRITES the field initializers. So the runtime values are the ones set in defaults(), NOT the field
// initializers and NOT the javadoc "Default Value" comments (all three disagree in 2.1.0). Verified 2026-11-22 by
// reflection on `new FollowerConstants()` / `new PathConstraints(0.99,100,1,1)` loaded from core-2.1.0.jar (JDK 17),
// and by javap of the MecanumConstants ctor in ftc-2.1.0.aar (it has no defaults() call; field initializers apply).
// TeamCode pedroPathing/Constants.java builds `new FollowerConstants()` / `new MecanumConstants()` with only names and
// directions set, so the robot runs these library defaults.
//
// Tags: "SOURCED" = read from the Pedro 2.1.0 source / TeamCode; "ESTIMATE" = assumed, to be replaced by calibrate.js
// fits from logged robot traces. Every ESTIMATE is flagged in SOURCES.
// Units: lengths in inches, time in seconds (ms where the key says so), angles in radians, mass in kg.
// Pure data + pure functions. No DOM, no imports.

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}
const isPlain = (x) => x !== null && typeof x === 'object' && !Array.isArray(x);
function clone(o) {
  if (Array.isArray(o)) return o.map(clone);
  if (isPlain(o)) { const r = {}; for (const k of Object.keys(o)) r[k] = clone(o[k]); return r; }
  return o;
}
function deepMerge(base, ov) {
  if (!isPlain(ov)) return base;
  for (const k of Object.keys(ov)) {
    if (ov[k] === undefined) continue;
    if (isPlain(ov[k]) && isPlain(base[k])) deepMerge(base[k], ov[k]);
    else base[k] = clone(ov[k]);
  }
  return base;
}

/** Pedro FollowerConstants as left by FollowerConstants.defaults() (run by the ctor). Java field names. */
export const PEDRO = deepFreeze({
  mass: 10.65,                                   // kg; only the centripetal multiplier
  forwardZeroPowerAcceleration: -41.278,         // in/s^2 (signed, negative); defaults() (dead field init: -34.62719)
  lateralZeroPowerAcceleration: -59.7819,        // in/s^2 (signed, negative); defaults() (dead field init: -78.15554)
  centripetalScaling: 0.0005,                    // Pedro centripetal multiplier

  // PIDF gains: power per unit error (in for translational, rad for heading, drive-error units for drive).
  coefficientsTranslationalPIDF: { P: 0.1, I: 0, D: 0, F: 0 },              // defaults() (dead field init: F 0.015)
  integralTranslational: { P: 0, I: 0, D: 0, F: 0.015 },                   // defaults() (dead field init: F 0)
  coefficientsHeadingPIDF: { P: 1, I: 0, D: 0, F: 0.01 },
  coefficientsDrivePIDF: { P: 0.025, I: 0, D: 0.00001, T: 0.6, F: 0.01 },

  coefficientsSecondaryTranslationalPIDF: { P: 0.3, I: 0, D: 0.01, F: 0.015 }, // defaults() (dead field init: F 0)
  integralSecondaryTranslational: { P: 0, I: 0, D: 0, F: 0 },                    // defaults() (dead field init: F 0.015)
  coefficientsSecondaryHeadingPIDF: { P: 5, I: 0, D: 0.08, F: 0.01 },
  coefficientsSecondaryDrivePIDF: { P: 0.02, I: 0, D: 0.000005, T: 0.6, F: 0.01 },

  predictiveBrakingCoefficients: { P: 0.15, kLinearBraking: 0.1, kQuadraticFriction: 0.001, maximumBrakingPower: 0.2 },
  usePredictiveBraking: false,

  useSecondaryTranslationalPIDF: false,
  useSecondaryHeadingPIDF: false,
  useSecondaryDrivePIDF: false,
  translationalPIDFSwitch: 3,                    // in
  headingPIDFSwitch: Math.PI / 20,               // rad
  drivePIDFSwitch: 20,                           // drive-error units (in/s)

  holdPointTranslationalScaling: 0.45,
  holdPointHeadingScaling: 0.35,
  BEZIER_CURVE_SEARCH_LIMIT: 10,                 // Newton steps
  turnHeadingErrorThreshold: 0.01,               // rad
  automaticHoldEnd: true,
  driveKalmanFilterModelCovariance: 6,
  driveKalmanFilterDataCovariance: 1,

  stuckVelocity: 1.0,                            // in/s   (Pedro:follower/Follower.java:528-539)
  stuckTValue: 0.8,                              // path parameter
  stuckTimeout: 500,                             // ms
});

/** Pedro ftc MecanumConstants (Pedro:ftc/drivetrains/MecanumConstants.java). */
export const MECANUM = deepFreeze({
  xVelocity: 81.34056,                           // in/s forward at full power (tuner mean |v|)
  yVelocity: 65.43028,                           // in/s lateral at full power
  maxPower: 1,
  motorCachingThreshold: 0.01,                   // power units
  useVoltageCompensation: false,
  nominalVoltage: 12.0,                          // V
  staticFrictionCoefficient: 0.1,
});

/** PathConstraints: TeamCode new PathConstraints(0.99, 100, 1, 1); the 4-arg ctor fills the rest (Pedro:paths/PathConstraints.java). */
export const CONSTRAINTS = deepFreeze({
  tValueConstraint: 0.99,                        // path parameter
  velocityConstraint: 0.1,                       // in/s
  translationalConstraint: 0.1,                  // in
  headingConstraint: 0.007,                      // rad
  timeoutConstraint: 100,                        // ms
  brakingStrength: 1,
  brakingStart: 1,
  BEZIER_CURVE_SEARCH_LIMIT: 10,
});

/** Drivetrain plant. Everything except gIn is an ESTIMATE or sim choice (see SOURCES). */
export const PLANT = deepFreeze({
  tauX: 0.18,                                    // s    ESTIMATE
  tauY: 0.20,                                    // s    ESTIMATE
  tauW: 0.10,                                    // s    ESTIMATE
  trackWidthIn: 15,                              // in   ESTIMATE
  wheelBaseIn: 15,                               // in   ESTIMATE
  muTraction: 0.8,                               // -    ESTIMATE
  gIn: 386.0886,                                 // in/s^2 standard gravity (9.80665 m/s^2 / 0.0254)
  plantNominalV: 12.0,                           // V    ESTIMATE (battery at which xVelocity was measured)
  batteryVoc: 12.0,                              // V    ESTIMATE
  batteryRintOhm: 0,                             // ohm  ESTIMATE (sag off)
  stallCurrentA: 9.2,                            // A    ESTIMATE (per motor)
  physicsDt: 0.001,                              // s    integration sub-step
});

export const LOOP = deepFreeze({
  dt: 0.010,                                     // s    ESTIMATE FTC loop with bulk reads
  jitter: 0,                                     // fraction of dt, uniform; 0 = deterministic
  seed: 1,
});

export const LOCALIZER = deepFreeze({
  posNoiseIn: 0,                                 // in, 1-sigma
  headNoiseRad: 0,                               // rad, 1-sigma
  velNoiseInS: 0,                                // in/s, 1-sigma
  latencyS: 0,                                   // s
  seed: 1,
});

export const SIM = deepFreeze({
  maxTimeS: 60,
  settleS: 0.3,
  trajStride: 2,
  followWatchdogS: 30,                           // s, sim-only (FTC auto length)
  poseSyncEpsIn: 0.01,
  poseSyncEpsRad: 0.001,
  emulate210CallbackBug: false,
});

const PJ = 'Pedro 2.1.0 core-2.1.0.jar (runtime value, verified by reflection) ';
const PF = 'Pedro 2.1.0 ftc-2.1.0.aar (verified by javap of the ctor) ';
const EST = 'ESTIMATE (unsourced; replace via calibrate.js fit from logged traces): ';

/** Citation per key, "<group>.<key>". ESTIMATE-prefixed entries are assumptions. */
export const SOURCES = deepFreeze((() => {
  const s = {};
  for (const k of Object.keys(PEDRO)) s['pedro.' + k] = PJ + 'follower/FollowerConstants.java defaults() (run by the ctor; overrides the field initializer and the javadoc default)';
  s['pedro.stuckVelocity'] = s['pedro.stuckTValue'] = s['pedro.stuckTimeout'] = PJ + 'follower/FollowerConstants.java; used at follower/Follower.java:528-539';
  s['pedro.forwardZeroPowerAcceleration'] = PJ + 'FollowerConstants.java defaults() (-41.278 / -59.7819; the field initializers -34.62719 / -78.15554 are overwritten); TeamCode runs the library default (pedroPathing/Constants.java sets none)';
  s['pedro.lateralZeroPowerAcceleration'] = s['pedro.forwardZeroPowerAcceleration'];
  for (const k of Object.keys(MECANUM)) s['mecanum.' + k] = PF + 'drivetrains/MecanumConstants.java field initializer';
  for (const k of Object.keys(CONSTRAINTS)) s['constraints.' + k] = PJ + 'paths/PathConstraints.java 4-arg ctor; TeamCode RobotSettings.PathFollowing (0.99, 100, 1, 1)';
  s['plant.tauX'] = s['plant.tauY'] = s['plant.tauW'] = EST + 'kept from the previous Java sim for continuity';
  s['plant.trackWidthIn'] = s['plant.wheelBaseIn'] = EST + 'robot geometry guess; may be replaced from web/js/robotspec.js';
  s['plant.muTraction'] = EST + 'rubber on foam tile; friction-circle limit aTractionMax = muTraction*gIn';
  s['plant.gIn'] = 'SOURCED: standard gravity 9.80665 m/s^2 / 0.0254 m/in = 386.0886 in/s^2';
  s['plant.plantNominalV'] = s['plant.batteryVoc'] = EST + 'battery voltage at which xVelocity was measured';
  s['plant.batteryRintOhm'] = EST + 'sag disabled by default';
  s['plant.stallCurrentA'] = EST + 'typical FTC drive motor stall current';
  s['plant.physicsDt'] = 'Chosen integration sub-step (1 ms)';
  s['loop.dt'] = EST + 'FTC loop with bulk reads; no source found';
  s['loop.jitter'] = s['loop.seed'] = 'Sim option, deterministic by default';
  for (const k of Object.keys(LOCALIZER)) s['localizer.' + k] = 'Sim option; 0 = exact truth (ideal Pinpoint-like localizer)';
  for (const k of Object.keys(SIM)) s['sim.' + k] = 'Sim option';
  s['sim.emulate210CallbackBug'] = 'Pedro PR #145: paths/callbacks/TemporalCallback.java:58,66 NANOSECONDS.convert(nanoTime, MILLISECONDS) bug present in 2.1.0';
  s['plant.coulombX'] = s['plant.coulombY'] = 'Derived: |forward/lateralZeroPowerAcceleration| (FLOAT coast decel, Pedro ZPA tuner)';
  s['plant.coulombW'] = EST + '((|fZPA|+|lZPA|)/2)/halfSumIn';
  return s;
})());

/**
 * Deep-merges overrides onto the defaults and computes derived plant fields.
 * Overrides are grouped: {pedro, mecanum, constraints, plant, loop, localizer, sim}. Top-level keys that name a SIM key
 * (maxTimeS, settleS, trajStride, ...) are accepted as sim overrides. Derived plant fields (halfSumIn, omegaMax, coulomb*)
 * may themselves be overridden via overrides.plant.
 */
export function resolveParams(overrides = {}) {
  const ov = isPlain(overrides) ? overrides : {};
  const r = {
    pedro: clone(PEDRO), mecanum: clone(MECANUM), constraints: clone(CONSTRAINTS),
    plant: clone(PLANT), loop: clone(LOOP), localizer: clone(LOCALIZER), sim: clone(SIM),
  };
  for (const g of Object.keys(r)) deepMerge(r[g], ov[g]);
  for (const k of Object.keys(SIM)) if (ov[k] !== undefined && !isPlain(ov[k])) r.sim[k] = ov[k];
  const p = r.plant, po = isPlain(ov.plant) ? ov.plant : {};
  const has = (k) => po[k] !== undefined;
  p.halfSumIn = has('halfSumIn') ? po.halfSumIn : (p.trackWidthIn + p.wheelBaseIn) / 2;
  p.omegaMax = has('omegaMax') ? po.omegaMax : r.mecanum.xVelocity / p.halfSumIn;
  p.coulombX = has('coulombX') ? po.coulombX : Math.abs(r.pedro.forwardZeroPowerAcceleration);
  p.coulombY = has('coulombY') ? po.coulombY : Math.abs(r.pedro.lateralZeroPowerAcceleration);
  p.coulombW = has('coulombW') ? po.coulombW : ((p.coulombX + p.coulombY) / 2) / p.halfSumIn;
  p.vFreeX = r.mecanum.xVelocity + p.coulombX * p.tauX;
  p.vFreeY = r.mecanum.yVelocity + p.coulombY * p.tauY;
  p.vFreeW = p.omegaMax + p.coulombW * p.tauW;
  p.aTractionMax = p.muTraction * p.gIn;
  return r;
}

/** Emits the request keys tools_java/PedroFollowSim.java reads (robot, constants, pathConstraints, dt). */
export function toJavaRequest(params) {
  const { pedro: f, mecanum: m, constraints: c, plant: p, loop } = params;
  const q = (o, keys) => keys.map((k) => o[k]);
  return {
    robot: {
      xVelocity: m.xVelocity, yVelocity: m.yVelocity, maxPower: m.maxPower,
      tauX: p.tauX, tauY: p.tauY, tauW: p.tauW, omegaMax: p.omegaMax,
      trackWidthIn: p.trackWidthIn, wheelBaseIn: p.wheelBaseIn,
      coulombX: p.coulombX, coulombY: p.coulombY, coulombW: p.coulombW,
      muTraction: p.muTraction, plantNominalV: p.plantNominalV, batteryVoc: p.batteryVoc,
    },
    constants: {
      mass: f.mass,
      forwardZeroPowerAcceleration: f.forwardZeroPowerAcceleration,
      lateralZeroPowerAcceleration: f.lateralZeroPowerAcceleration,
      centripetalScaling: f.centripetalScaling,
      holdPointTranslationalScaling: f.holdPointTranslationalScaling,
      holdPointHeadingScaling: f.holdPointHeadingScaling,
      usePredictiveBraking: f.usePredictiveBraking,
      automaticHoldEnd: f.automaticHoldEnd,
      translationalPIDF: q(f.coefficientsTranslationalPIDF, ['P', 'I', 'D', 'F']),
      headingPIDF: q(f.coefficientsHeadingPIDF, ['P', 'I', 'D', 'F']),
      drivePIDF: q(f.coefficientsDrivePIDF, ['P', 'I', 'D', 'T', 'F']),
      driveKalmanFilterModelCovariance: f.driveKalmanFilterModelCovariance,
      driveKalmanFilterDataCovariance: f.driveKalmanFilterDataCovariance,
    },
    pathConstraints: { tValue: c.tValueConstraint, timeoutMs: c.timeoutConstraint, braking: c.brakingStrength, brakingStart: c.brakingStart },
    dt: loop.dt,
  };
}
