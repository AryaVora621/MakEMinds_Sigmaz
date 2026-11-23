// params.js - single source of defaults for the Pinpoint odometry sim. Pure ES module, no DOM.
// Units: inches, radians, seconds unless the name says otherwise (Mm, Hz, Us).
// Provenance tags: [SRC url] sourced, [DERIVED ...] computed, [RECALLED-UNVERIFIED] from memory, [ASSUMPTION] tunable guess.

export const MM_PER_IN = 25.4; // [DERIVED] exact definition of the inch

// Pod hardware. ticksPerMm = cpr / (pi * wheelDiameterMm).
export const POD_TYPES = {
  goBILDA_4_BAR_POD: {
    ticksPerMm: 19.89436789,   // [DERIVED 2000/(pi*32)]
    wheelDiameterMm: 32,       // [SRC https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/]
    cpr: 2000,                 // [SRC https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/] counts/rev
  },
  goBILDA_SWINGARM_POD: {
    ticksPerMm: 13.26291192,   // [DERIVED 2000/(pi*48); RECALLED-UNVERIFIED that the driver uses this value]
    wheelDiameterMm: 48,       // [SRC goBILDA Pinpoint product page, related products]
    cpr: 2000,                 // [ASSUMPTION] same 2000 CPR as the 4-Bar pod
  },
};

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}

function isPlain(v) {
  return v !== null && typeof v === 'object' && !Array.isArray(v);
}

function deepClone(v) {
  if (Array.isArray(v)) return v.map(deepClone);
  if (isPlain(v)) {
    const o = {};
    for (const k of Object.keys(v)) o[k] = deepClone(v[k]);
    return o;
  }
  return v;
}

/** Returns a new object: plain objects merge recursively, arrays and primitives replace. Inputs are not mutated. */
export function deepMerge(base, over) {
  const out = deepClone(base);
  if (!isPlain(over)) return out;
  for (const k of Object.keys(over)) {
    const ov = over[k];
    if (isPlain(ov) && isPlain(out[k])) out[k] = deepMerge(out[k], ov);
    else out[k] = deepClone(ov);
  }
  return out;
}

export const DEFAULTS = deepFreeze({
  podType: 'goBILDA_4_BAR_POD', // [ASSUMPTION] the robot uses 4-Bar pods

  // True physical pod mounts, robot frame (in), +x forward, +y left.
  // Values [SRC repo robot_spec.json odometry.pods, prov vault and placeholder]. The mapping xPod->forward pod,
  // yPod->strafe pod is an [ASSUMPTION] because the spec "axis" field is ambiguous.
  truth: {
    forwardPod: { x: 0.96, y: -0.63, angleRad: 0, diameterScale: 1, encoderSign: 1 }, // angle/scale/sign: [ASSUMPTION] perfect pod
    strafePod: { x: -1.2, y: 0.9, angleRad: 0, diameterScale: 1, encoderSign: 1 },    // angle/scale/sign: [ASSUMPTION] perfect pod
  },

  // What the Pinpoint is told (setOffsets etc.).
  config: {
    forwardPodYIn: -0.63, // in; convention (X pod sideways position, left +) [RECALLED-UNVERIFIED]; value [SRC robot_spec.json]
    strafePodXIn: -1.2,   // in; convention (Y pod forward position, fwd +) [RECALLED-UNVERIFIED]; value [SRC robot_spec.json]
    forwardDir: 1,        // [ASSUMPTION] encoder direction sign
    strafeDir: 1,         // [ASSUMPTION] encoder direction sign
    ticksPerMm: null,     // null = POD_TYPES[podType].ticksPerMm [DERIVED]
    yawScalar: 1,         // [ASSUMPTION] dimensionless firmware yaw scale
  },

  firmware: {
    rateHz: 1500,          // Hz; [RECALLED-UNVERIFIED] the task states about 1.5 kHz
    rateJitterFrac: 0,     // [ASSUMPTION] fractional period jitter
    velWindowSamples: 15,  // samples; [ASSUMPTION] velocity window
    historyLen: 512,       // snapshots; [ASSUMPTION] about 0.34 s at 1.5 kHz
    resetDurationS: 0.25,  // s; [ASSUMPTION]
  },

  counter: { quantize: true }, // [DERIVED] real quadrature counters are integer

  imu: {
    scaleError: 0,                 // dimensionless; [ASSUMPTION]
    biasRadS: 0,                   // rad/s; [ASSUMPTION]
    biasWalkRadSPerSqrtS: 1e-5,    // rad/s per sqrt(s); [ASSUMPTION]
    noiseRadPerSqrtS: 1e-4,        // rad per sqrt(s) (angle random walk); [ASSUMPTION]
    quantRad: 0,                   // rad; [ASSUMPTION] 0 = no quantization
  },

  host: {
    dataAgeS: 0.002,        // s; [ASSUMPTION] I2C transfer plus sample-and-hold
    dataAgeJitterS: 0.0003, // s; [ASSUMPTION]
    readCostS: 0.002,       // s; [ASSUMPTION] diagnostic loop-time cost
    loopPeriodS: 0,         // s; [ASSUMPTION] 0 = localizer updates every physics step
  },

  slip: { // all [ASSUMPTION]: researchers found no sources
    enabled: true,
    accelThreshInS2: 250,         // in/s^2 axis acceleration above which slip starts
    accelGain: 0.0005,            // slip fraction per in/s^2 over threshold
    scrubThreshRadS: 3.0,         // rad/s yaw rate above which scrub starts
    scrubGain: 0.01,              // slip fraction per rad/s over threshold
    maxSlip: 0.9,                 // fraction
    liftoffSeverityMin: 0.5,      // collision severity 0..1
    liftoffMaxS: 0.04,            // s, at severity 1
    coastTauS: 0.05,              // s, wheel spin decay while lifted
    collisionSlipFrac: 0.3,       // fraction at severity 1
    collisionSlipDurationS: 0.06, // s
    pushingSlipFrac: 0.02,        // fraction
  },

  noise: { travelNoiseInPerSqrtIn: 0 }, // in per sqrt(in); [ASSUMPTION] Kelly 2004 style random walk, off by default

  seed: 12345, // [ASSUMPTION] arbitrary
});

export const PRESETS = deepFreeze({
  // Reproduces the current behaviour: belief equals truth up to pose-exp vs Euler error. [ASSUMPTION]
  ideal: {
    firmware: { rateHz: 0, velWindowSamples: 1 },
    counter: { quantize: false },
    imu: { scaleError: 0, biasRadS: 0, biasWalkRadSPerSqrtS: 0, noiseRadPerSqrtS: 0, quantRad: 0 },
    host: { dataAgeS: 0, dataAgeJitterS: 0 },
    slip: { enabled: false },
  },
  nominal: {},
  // RobotSettings PARALLEL/PERPENDICULAR_POD_OFFSET_IN are 0 PLACEHOLDERs, so configured offsets are zero. [SRC repo RobotSettings.java]
  asConfigured: {
    config: { forwardPodYIn: 0, strafePodXIn: 0 },
  },
  // [ASSUMPTION] pessimistic error mix for stress testing.
  harsh: {
    imu: { scaleError: 0.005 },
    truth: { forwardPod: { diameterScale: 1.01 }, strafePod: { angleRad: 0.02 } },
    config: { forwardPodYIn: -0.53, strafePodXIn: -1.1 },
    slip: { accelGain: 0.0015, collisionSlipFrac: 0.5 },
    noise: { travelNoiseInPerSqrtIn: 0.003 },
    host: { dataAgeJitterS: 0.001 },
  },
});

/** Build frozen, resolved params. Throws Error on unknown preset or podType. */
export function makeOdoParams(preset = 'nominal', overrides = {}) {
  if (!Object.prototype.hasOwnProperty.call(PRESETS, preset)) {
    throw new Error(`Unknown odometry preset: ${preset}`);
  }
  let p = deepMerge(DEFAULTS, PRESETS[preset]);
  p = deepMerge(p, overrides || {});
  if (!Object.prototype.hasOwnProperty.call(POD_TYPES, p.podType)) {
    throw new Error(`Unknown podType: ${p.podType}`);
  }
  const truthTicks = POD_TYPES[p.podType].ticksPerMm;
  if (p.config.ticksPerMm === null || p.config.ticksPerMm === undefined) p.config.ticksPerMm = truthTicks;
  p.ticksPerMmTruth = truthTicks;
  return deepFreeze(p);
}
