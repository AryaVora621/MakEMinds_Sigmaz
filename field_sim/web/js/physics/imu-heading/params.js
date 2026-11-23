// params.js - single source of every tunable for the imu-heading area.
// Constants are stored in HUMAN units (deg/s, deg/h, deg/min, in, s). resolve*Config() converts ONCE
// into internal units (rad, rad/s, ...). No other module converts.
//
// Source tags (see PARAM_SOURCES):
//   [S1] navX2-MXP tech specs, https://pdocs.kauailabs.com/navx-mxp/intro/technical-specifications/ (medium)
//   [S2] Kalibr IMU noise model, https://github.com/ethz-asl/kalibr/wiki/IMU-noise-model (medium)
//   [S3] goBILDA Pinpoint user guide, snippet only (UNVERIFIED), >1 deg/min stationary drift = report
//   [S4] SparkFun OTOS Example3_Calibration.ino (medium); [S4b] SparkFun forum, ~1 deg per few min (low)
//   [S5] FTC docs IMU page (medium)   [S6] Road Runner quickstart Pinpoint localizer (low-medium)
//   [S7] Medium post, flywheel vibration makes Pinpoint readings creep (low, snippet only)
//   [A] assumed engineering estimate (override freely)   [R] recalled datasheet value, unverified
//   [D] derived in the spec

export const DEG = Math.PI / 180;

/** Human-unit IMU presets. Field units are in the field name (Dps = deg/s, Dph = deg/h). */
export const IMU_PRESETS = {
  // Every error term zero: estimate == truth bit-for-bit.
  ideal: {
    internalHz: 1000,          // Hz [A]
    latencyS: 0,               // s [A]
    fullScaleDps: Infinity,    // deg/s, no clipping [A]
    rateLsbDps: 0,             // deg/s, 0 = float output [A]
    headingLsbDeg: 0,          // deg, 0 = float output [A]
    noiseDensityDpsRtHz: 0,    // deg/s/sqrt(Hz) [A]
    biasInitSigmaDps: 0,       // deg/s [A]
    gmSigmaDph: 0,             // deg/h [A]
    gmTauS: 100,               // s [A] (unused when gmSigma = 0)
    biasRwDpsPerRtS: 0,        // deg/s/sqrt(s) [A]
    scaleErrSigma: 0,          // unitless 1-sigma [A]
    scaleAsymSigma: 0,         // unitless 1-sigma [A]
    tempCoeffDpsPerK: 0,       // deg/s per K [A]
    tempRiseK: 0,              // K [A]
    tempTauS: 600,             // s [A]
    gSensDpsPerG: 0,           // deg/s per g [A]
    calibDurationS: 0,         // s [A]
    shockSigmaDegPerMps: 0,    // deg per (m/s) impact [A]
    vibrationNoiseMult: 1,     // x noise with flywheel on [A]
  },
  // goBILDA Pinpoint (default). ICM-class MEMS gyro assumed.
  pinpoint: {
    internalHz: 1000,          // Hz [A]
    latencyS: 0.002,           // s [A]
    fullScaleDps: 2000,        // deg/s [R/A]
    rateLsbDps: 0.07,          // deg/s per LSB (70 mdps at +-2000 dps) [R]
    headingLsbDeg: 0,          // deg, float output [A]
    noiseDensityDpsRtHz: 0.005, // deg/s/sqrt(Hz) [S1 proxy]
    biasInitSigmaDps: 0.004,   // deg/s residual after startup cal; 3-sigma 0.72 deg/min < 1 deg/min [A, cf. S3]
    gmSigmaDph: 3,             // deg/h bias stability [S1]
    gmTauS: 100,               // s Gauss-Markov correlation time [A]
    biasRwDpsPerRtS: 1e-5,     // deg/s/sqrt(s) [A]
    scaleErrSigma: 0.002,      // residual after yaw-scalar calibration, cf. S4 0.001 step [A]
    scaleAsymSigma: 0.0005,    // [A]
    tempCoeffDpsPerK: 0.005,   // deg/s per K [S1]
    tempRiseK: 3,              // K warm-up rise [A]
    tempTauS: 600,             // s warm-up time constant [A]
    gSensDpsPerG: 0.1,         // deg/s per g [A]
    calibDurationS: 0.25,      // s [A]
    shockSigmaDegPerMps: 0.3,  // deg per m/s impact [A]
    vibrationNoiseMult: 2,     // x noise with flywheel on [S7/A]
  },
  // REV Control/Expansion Hub built-in BHI260AP.
  revBhi260: {
    internalHz: 400,           // Hz [A]
    latencyS: 0.010,           // s, hub I2C + fusion [A]
    fullScaleDps: 2000,        // deg/s [R/A]
    rateLsbDps: 0.07,          // deg/s per LSB [R]
    headingLsbDeg: 0,          // deg [A]
    noiseDensityDpsRtHz: 0.008, // deg/s/sqrt(Hz) [A]
    biasInitSigmaDps: 0.0083,  // deg/s = 0.5 deg/min, S1 moving drift as proxy [S1]
    gmSigmaDph: 3,             // deg/h [S1]
    gmTauS: 100,               // s [A]
    biasRwDpsPerRtS: 1e-5,     // deg/s/sqrt(s) [A]
    scaleErrSigma: 0.002,      // [A]
    scaleAsymSigma: 0.0005,    // [A]
    tempCoeffDpsPerK: 0.005,   // deg/s per K [S1]
    tempRiseK: 3,              // K [A]
    tempTauS: 600,             // s [A]
    gSensDpsPerG: 0.1,         // deg/s per g [A]
    calibDurationS: 0.05,      // s, BHI260AP init ~50 ms [S5]
    shockSigmaDegPerMps: 0.3,  // [A]
    vibrationNoiseMult: 2,     // [S7/A]
  },
  // REV Hub legacy BNO055 (Euler output, 1/16 deg LSB).
  revBno055: {
    internalHz: 100,           // Hz [R]
    latencyS: 0.015,           // s [A]
    fullScaleDps: 2000,        // deg/s [R/A]
    rateLsbDps: 0.07,          // deg/s per LSB [R]
    headingLsbDeg: 0.0625,     // deg, 16 LSB/deg Euler [R]
    noiseDensityDpsRtHz: 0.014, // deg/s/sqrt(Hz) [R]
    biasInitSigmaDps: 0.015,   // deg/s ~0.9 deg/min [A]
    gmSigmaDph: 3,             // deg/h [S1]
    gmTauS: 100,               // s [A]
    biasRwDpsPerRtS: 1e-5,     // deg/s/sqrt(s) [A]
    scaleErrSigma: 0.004,      // [A]
    scaleAsymSigma: 0.0005,    // [A]
    tempCoeffDpsPerK: 0.005,   // deg/s per K [S1]
    tempRiseK: 3,              // K [A]
    tempTauS: 600,             // s [A]
    gSensDpsPerG: 0.1,         // deg/s per g [A]
    calibDurationS: 0.1,       // s, BNO055 init ~100 ms [S5]
    shockSigmaDegPerMps: 0.3,  // [A]
    vibrationNoiseMult: 2,     // [S7/A]
  },
  // Kauai Labs navX2-MXP (ISM330DHCX).
  navx2: {
    internalHz: 416,           // Hz fusion rate [S1]
    latencyS: 0.005,           // s [A]
    fullScaleDps: 2000,        // deg/s [R/A]
    rateLsbDps: 0.07,          // deg/s per LSB [R]
    headingLsbDeg: 0,          // deg [A]
    noiseDensityDpsRtHz: 0.005, // deg/s/sqrt(Hz) [S1]
    biasInitSigmaDps: 0.0083,  // deg/s = 0.5 deg/min moving yaw drift [S1]
    gmSigmaDph: 3,             // deg/h bias stability [S1]
    gmTauS: 100,               // s [A]
    biasRwDpsPerRtS: 1e-5,     // deg/s/sqrt(s) [A]
    scaleErrSigma: 0.002,      // [A]
    scaleAsymSigma: 0.0005,    // [A]
    tempCoeffDpsPerK: 0.005,   // deg/s per K [S1]
    tempRiseK: 3,              // K [A]
    tempTauS: 600,             // s [A]
    gSensDpsPerG: 0.1,         // deg/s per g [A]
    calibDurationS: 5,         // s startup calibration, held still [S1]
    shockSigmaDegPerMps: 0.3,  // [A]
    vibrationNoiseMult: 2,     // [S7/A]
  },
  // SparkFun OTOS optical tracking sensor (IMU part used for heading).
  otos: {
    internalHz: 433,           // Hz [A]
    latencyS: 0.003,           // s [A]
    fullScaleDps: 2000,        // deg/s [R/A]
    rateLsbDps: 0.07,          // deg/s per LSB [R]
    headingLsbDeg: 0,          // deg [A]
    noiseDensityDpsRtHz: 0.005, // deg/s/sqrt(Hz) [S1 proxy]
    biasInitSigmaDps: 0.0056,  // deg/s ~1/3 deg/min [S4b]
    gmSigmaDph: 3,             // deg/h [S1 proxy]
    gmTauS: 100,               // s [A]
    biasRwDpsPerRtS: 1e-5,     // deg/s/sqrt(s) [A]
    scaleErrSigma: 0.001,      // angular-scalar step 0.001 [S4]
    scaleAsymSigma: 0.0005,    // [A]
    tempCoeffDpsPerK: 0.005,   // deg/s per K [S1 proxy]
    tempRiseK: 3,              // K [A]
    tempTauS: 600,             // s [A]
    gSensDpsPerG: 0.1,         // deg/s per g [A]
    calibDurationS: 0.612,     // s, 255 samples x 2.4 ms [S4]
    shockSigmaDegPerMps: 0.3,  // [A]
    vibrationNoiseMult: 2,     // [S7/A]
  },
};

/** Odometry (two dead-wheel pods). */
export const ODOMETRY_DEFAULTS = {
  podScaleSigma: 0.005,        // unitless 1-sigma wheel-diameter/scale error [A]
  podNoiseInPerRtIn: 0.003,    // in per sqrt(in travelled) [A]
  podSlipSigmaIn: 0.5,         // in 1-sigma slip per collision per pod [A]
  ticksPerIn: 505.3,           // ticks per inch, goBILDA 4-bar 19.89 ticks/mm [R]
};

/** Synthetic AprilTag vision. */
export const VISION_DEFAULTS = {
  enabled: false,              // [A]
  rateHz: 10,                  // Hz [A]
  latencyS: 0.05,              // s [A]
  posSigmaIn: 1.0,             // in 1-sigma [A]
  headingSigmaDeg: 1.5,        // deg 1-sigma [A]
  maxRangeIn: 120,             // in max visible range [A]
};

export const ESTIMATOR_DEFAULTS = {
  mode: 'deadReckon',          // 'deadReckon' | 'truth' (pass-through) [A]
  historyS: 1.0,               // s of pose/gyro history kept for latency-compensated vision [A]
  useKF: true,                 // [A]
};

export const SYSTEM_DEFAULTS = {
  preset: 'pinpoint',
  seed: 1,
  hostHz: 100,                 // Hz, typical FTC loop [A]
  enabled: true,
};

export const MC_DEFAULTS = {
  runs: 200,
  seed: 12345,
  dt: 0.01,                    // s
};

// ---- source tags -------------------------------------------------------------------------------

// Tags for the pinpoint preset; other presets inherit them unless listed in PRESET_TAG_OVERRIDES.
const BASE_TAGS = {
  internalHz: 'A', latencyS: 'A', fullScaleDps: 'R/A', rateLsbDps: 'R', headingLsbDeg: 'A',
  noiseDensityDpsRtHz: 'S1', biasInitSigmaDps: 'A', gmSigmaDph: 'S1', gmTauS: 'A',
  biasRwDpsPerRtS: 'A', scaleErrSigma: 'A', scaleAsymSigma: 'A', tempCoeffDpsPerK: 'S1',
  tempRiseK: 'A', tempTauS: 'A', gSensDpsPerG: 'A', calibDurationS: 'A',
  shockSigmaDegPerMps: 'A', vibrationNoiseMult: 'S7/A',
};
const PRESET_TAG_OVERRIDES = {
  ideal: Object.fromEntries(Object.keys(BASE_TAGS).map((k) => [k, 'A'])),
  pinpoint: {},
  revBhi260: { biasInitSigmaDps: 'S1', noiseDensityDpsRtHz: 'A', calibDurationS: 'S5' },
  revBno055: { internalHz: 'R', noiseDensityDpsRtHz: 'R', headingLsbDeg: 'R', calibDurationS: 'S5' },
  navx2: { internalHz: 'S1', noiseDensityDpsRtHz: 'S1', biasInitSigmaDps: 'S1', calibDurationS: 'S1' },
  otos: { biasInitSigmaDps: 'S4b', scaleErrSigma: 'S4', calibDurationS: 'S4' },
};

/** Map 'preset.field' (and 'odometry.*', 'vision.*', 'estimator.*', 'system.*') -> source tag. */
export const PARAM_SOURCES = (() => {
  const m = {};
  for (const name of Object.keys(IMU_PRESETS)) {
    for (const f of Object.keys(IMU_PRESETS[name])) {
      m[`${name}.${f}`] = PRESET_TAG_OVERRIDES[name][f] ?? BASE_TAGS[f];
    }
  }
  const odo = { podScaleSigma: 'A', podNoiseInPerRtIn: 'A', podSlipSigmaIn: 'A', ticksPerIn: 'R' };
  for (const k of Object.keys(odo)) m[`odometry.${k}`] = odo[k];
  for (const k of Object.keys(VISION_DEFAULTS)) m[`vision.${k}`] = 'A';
  for (const k of Object.keys(ESTIMATOR_DEFAULTS)) m[`estimator.${k}`] = 'A';
  for (const k of Object.keys(SYSTEM_DEFAULTS)) m[`system.${k}`] = 'A';
  return Object.freeze(m);
})();

// ---- resolvers ---------------------------------------------------------------------------------

function checkKeys(overrides, allowed, what) {
  for (const k of Object.keys(overrides || {})) {
    if (!Object.prototype.hasOwnProperty.call(allowed, k)) {
      throw new Error(`Unknown ${what} override key: ${k}`);
    }
  }
}

/** Resolve a preset (+ human-unit overrides) into a frozen internal-unit config. */
export function resolveImuConfig(presetName = 'pinpoint', overrides = {}) {
  if (!Object.prototype.hasOwnProperty.call(IMU_PRESETS, presetName)) {
    throw new Error(`Unknown IMU preset: ${presetName}`);
  }
  const base = IMU_PRESETS[presetName];
  checkKeys(overrides, base, 'IMU');
  const p = { ...base, ...(overrides || {}) };
  return Object.freeze({
    name: presetName,
    internalHz: p.internalHz,
    latencyS: p.latencyS,
    fullScaleRad: p.fullScaleDps * DEG,              // rad/s (Infinity stays Infinity)
    rateLsbRad: p.rateLsbDps * DEG,                  // rad/s
    headingLsbRad: p.headingLsbDeg * DEG,            // rad
    noiseDensityRad: p.noiseDensityDpsRtHz * DEG,    // rad/s/sqrt(Hz)
    biasInitSigmaRad: p.biasInitSigmaDps * DEG,      // rad/s
    gmSigmaRad: (p.gmSigmaDph / 3600) * DEG,         // rad/s
    gmTauS: p.gmTauS,
    biasRwRad: p.biasRwDpsPerRtS * DEG,              // rad/s/sqrt(s)
    scaleErrSigma: p.scaleErrSigma,
    scaleAsymSigma: p.scaleAsymSigma,
    tempCoeffRad: p.tempCoeffDpsPerK * DEG,          // rad/s per K
    tempRiseK: p.tempRiseK,
    tempTauS: p.tempTauS,
    gSensRad: p.gSensDpsPerG * DEG,                  // rad/s per g
    calibDurationS: p.calibDurationS,
    shockSigmaRad: p.shockSigmaDegPerMps * DEG,      // rad per m/s
    vibrationNoiseMult: p.vibrationNoiseMult,
  });
}

export function resolveOdometryConfig(overrides = {}) {
  checkKeys(overrides, ODOMETRY_DEFAULTS, 'odometry');
  return Object.freeze({ ...ODOMETRY_DEFAULTS, ...(overrides || {}) });
}

export function resolveVisionConfig(overrides = {}) {
  checkKeys(overrides, VISION_DEFAULTS, 'vision');
  const v = { ...VISION_DEFAULTS, ...(overrides || {}) };
  return Object.freeze({
    enabled: v.enabled,
    rateHz: v.rateHz,
    latencyS: v.latencyS,
    posSigmaIn: v.posSigmaIn,
    headingSigmaRad: v.headingSigmaDeg * DEG,
    maxRangeIn: v.maxRangeIn,
  });
}

export function resolveEstimatorConfig(overrides = {}) {
  checkKeys(overrides, ESTIMATOR_DEFAULTS, 'estimator');
  const e = { ...ESTIMATOR_DEFAULTS, ...(overrides || {}) };
  if (e.mode !== 'deadReckon' && e.mode !== 'truth') {
    throw new Error(`Unknown estimator mode: ${e.mode}`);
  }
  return Object.freeze(e);
}
