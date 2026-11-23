// Validation-tuning parameters: the ONLY file with tunable constants for this area.
// Units: lengths in PEDRO inches, time in s, angles in rad, mass in kg (see UNITS).
// Tags: SOURCED(url) = taken from a cited source; PRIOR = lead's prior knowledge, not verified this
// session; ASSUMED = a project choice.

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}

export const UNITS = deepFreeze({ length: 'in', time: 's', angle: 'rad', mass: 'kg' }); // ASSUMED
export const M_PER_IN = 0.0254; // m per in, exact by definition (SOURCED: international inch)
export const IN_PER_M = 1 / 0.0254; // in per m

// Golden-file tolerances. Check is numpy assert_allclose: |a-d| <= atol + rtol*|d|.
// SOURCED numpy.org/doc/stable/reference/generated/numpy.testing.assert_allclose.html
// atol nonzero because relative tests collapse near zero (Dawson 2012,
// randomascii.wordpress.com/2012/02/25/). Values themselves ASSUMED.
export const GOLDEN = deepFreeze({
  atolPos: 1e-3, // in, ASSUMED
  rtolPos: 1e-6, // ASSUMED
  atolVel: 1e-2, // in/s, ASSUMED
  rtolVel: 1e-6, // ASSUMED
  atolAng: 1e-5, // rad, ASSUMED
  rtolAng: 1e-6, // ASSUMED
});

// Real-data acceptance targets (ASSUMED). rpeDeltaS default SOURCED
// cvg.cit.tum.de/data/datasets/rgbd-dataset/tools
export const REAL = deepFreeze({
  ateRmseMaxIn: 2.0, // in, ASSUMED
  frechetMaxIn: 3.0, // in, ASSUMED
  rpeTransMaxIn: 1.0, // in, ASSUMED
  rpeRotMaxRad: 0.05, // rad, ASSUMED
  rpeDeltaS: 1.0, // s, SOURCED TUM RGB-D tools
});

// Invariant limits (ASSUMED except overlapBallIn, which matches the hive-stack test in sim_physics.test.mjs)
export const INVARIANT = deepFreeze({
  penFloorIn: 0.1, // in, ASSUMED
  penWallIn: 0.1, // in, ASSUMED
  overlapBallIn: 0.15, // in, matches sim_physics.test.mjs hive-stack test
  robotInsideSlackIn: 0.0, // in, ASSUMED
  speedSlackInS: 1e-6, // in/s, ASSUMED
  energyRelStep: 0.02, // fraction of E0 per step, ASSUMED
  energyRelTotal: 0.05, // fraction of E0 total, ASSUMED
  energyAbs: 1.0, // kg*in^2/s^2, ASSUMED
});

// Property testing. numRuns SOURCED fast-check Parameters default
// (fast-check.dev/docs/api/interfaces/Parameters/); the rest ASSUMED.
export const PROP = deepFreeze({
  numRuns: 100, // SOURCED fast-check
  numRunsCI: 20, // ASSUMED, keeps tests fast
  maxShrinkSteps: 50, // ASSUMED
  seedEnv: 'PROP_SEED', // ASSUMED
  runsEnv: 'PROP_RUNS', // ASSUMED
});

// Benchmark. warmupSteps/runCount SOURCED Box2D benchmark/main.c (one untimed step, 4 runs, min over repeats).
export const BENCH = deepFreeze({
  dt: 1 / 60, // s, ASSUMED
  warmupSteps: 1, // SOURCED Box2D benchmark/main.c
  runCount: 4, // SOURCED Box2D benchmark/main.c
  steps: 300, // ASSUMED
  regressFactor: 1.5, // ASSUMED
  gateEnv: 'BENCH_GATE', // ASSUMED
  scenes: { rain: { balls: 200 }, pile: { balls: 100 }, push: { balls: 60 }, drop: { balls: 12 } }, // ASSUMED
});

export const SENS = deepFreeze({ relStep: 0.1 }); // relative step for central differences, ASSUMED

// Fitting. rampRatePerS and rampMaxPower SOURCED rr.brott.dev/docs/v1-0/tuning/ ; rest ASSUMED.
export const FIT = deepFreeze({
  minVelInS: 1.0, // in/s, ASSUMED
  trimStartS: 0.05, // s, ASSUMED
  smoothWindow: 5, // samples, ASSUMED
  rampRatePerS: 0.1, // power/s, SOURCED rr.brott.dev/docs/v1-0/tuning/
  rampMaxPower: 0.9, // power, SOURCED rr.brott.dev/docs/v1-0/tuning/
  accelMatch: 't90', // ASSUMED
  rhoAirKgM3: 1.2, // kg/m^3, ASSUMED (typical sea-level air)
  dragDt: 1e-3, // s, ASSUMED
  decelMinVelInS: 2.0, // in/s, ASSUMED
});

// CMA-ES. sigma0Frac SOURCED Hansen arXiv:1604.00772 App. A; tolerances ASSUMED (Hansen's not found).
export const CMAES = deepFreeze({
  sigma0Frac: 0.3, // SOURCED Hansen arXiv:1604.00772 App. A (sigma0 = 0.3(b-a))
  maxEvalsFactor: 1000, // maxEvals = factor*n^2, ASSUMED
  tolFun: 1e-12, // ASSUMED
  tolX: 1e-11, // ASSUMED
});

// Statistics. z values are standard normal quantiles (standard tables); alpha/power ASSUMED.
export const STAT = deepFreeze({
  alpha: 0.05, // ASSUMED
  power: 0.8, // ASSUMED
  z95: 1.959963984540054, // z_{0.975}, standard table
  zPower80: 0.8416212335729143, // z_{0.8}, standard table
});

// Shot model (ASSUMED)
export const SHOT = deepFreeze({
  flightTimeS: 0.6, // s
  speedSigmaFrac: 0.02, // fraction of speed
  angleSigmaRad: 0.0087, // rad (about 0.5 deg)
  nTrials: 200,
  settleS: 3.0, // s
  standoffIn: 48, // in
  launchZIn: 14, // in
});

// Debug layer bit flags. SOURCED modelled on Rapier DebugRenderMode
// (github.com/dimforge/rapier .../debug_render_pipeline.rs).
export const LAYERS = deepFreeze({
  COLLIDERS: 1,
  BODY_AXES: 2,
  CONTACTS: 4,
  CONTACT_NORMALS: 8,
  VELOCITY: 16,
  IMPULSES: 32,
  AABBS: 64,
  ENERGY: 128,
  ROBOT: 256,
  TRAILS: 512,
  DEFAULT: 1 | 4 | 8 | 16 | 256,
});

// Overlay style. Colours are RGB 0..1. Bullet btIDebugDraw DefaultColors SOURCED for contact/aabb/active/sleeping;
// circleSubdiv SOURCED Rapier DebugRenderStyle subdivisions=20; rest ASSUMED.
export const OVERLAY = deepFreeze({
  contactSlopIn: 0.05, // in, ASSUMED
  contactNormalLenIn: 3.0, // in, ASSUMED (Rapier uses 0.3 m in its own scale)
  velScaleS: 0.02, // s: arrow length in = |v| * 0.02, ASSUMED
  impulseScaleS: 0.02, // s, ASSUMED
  circleSubdiv: 20, // SOURCED Rapier DebugRenderStyle
  trailLen: 120, // samples, ASSUMED
  hudEveryFrames: 10, // frames, ASSUMED
  stepWindow: 120, // steps, ASSUMED
  keyOverlay: 'F8', // ASSUMED
  keyPanel: 'F9', // ASSUMED
  colors: {
    contact: [1, 1, 0], // SOURCED Bullet btIDebugDraw DefaultColors
    aabb: [1, 0, 0], // SOURCED Bullet
    active: [1, 1, 1], // SOURCED Bullet
    sleeping: [0, 1, 0], // SOURCED Bullet
    collider: [0.6, 0.6, 0.6], // ASSUMED
    normal: [0, 1, 1], // ASSUMED
    velocity: [0.2, 0.6, 1], // ASSUMED
    impulse: [1, 0.3, 0.3], // ASSUMED
    robot: [1, 0.5, 0], // ASSUMED
  },
});

// FTC Dashboard field origin = mat centre, inches, radians. SOURCED
// acmerobotics.github.io/ftc-dashboard/fieldview. The Dashboard<->Pedro rotation is ASSUMED 0; user must check.
export const DASH = deepFreeze({ offsetIn: [72, 72], rotRad: 0 });

export const WPILOG = deepFreeze({ magic: 'WPILOG', version: 0x0100 }); // PRIOR (datalog.adoc not fetched)

export const PRESET = deepFreeze({ version: 1, storageKey: 'vt.presets.v1' }); // ASSUMED

// Tunable sim parameters exposed to the tuning panel. Ranges are ASSUMED project choices.
const E = (key, min, max, step, unit) =>
  Object.freeze({ key, group: key.split('.')[0], field: key.split('.')[1], min, max, step, unit, tag: 'ASSUMED' });
export const PARAM_SCHEMA = Object.freeze([
  E('ball.restitutionFloor', 0.05, 0.95, 0.01, ''),
  E('ball.restitutionWall', 0.05, 0.95, 0.01, ''),
  E('ball.restitutionBall', 0.05, 0.95, 0.01, ''),
  E('ball.restitutionRobot', 0, 0.95, 0.01, ''),
  E('ball.restitutionHive', 0, 0.95, 0.01, ''),
  E('ball.dragK', 0, 0.003, 0.00005, '1/in'),
  E('ball.rollFriction', 0, 0.3, 0.005, ''),
  E('ball.slideFriction', 0, 1.5, 0.05, ''),
  E('ball.restSpeed', 0, 20, 0.5, 'in/s'),
  E('drive.maxSpeed', 10, 120, 1, 'in/s'),
  E('drive.maxAccel', 20, 600, 5, 'in/s^2'),
  E('drive.maxDecel', 20, 900, 5, 'in/s^2'),
  E('drive.maxTurn', 0.5, 8, 0.1, 'rad/s'),
  E('drive.turnAccel', 1, 60, 0.5, 'rad/s^2'),
  E('drive.turnDecel', 1, 90, 0.5, 'rad/s^2'),
  E('drive.strafeFactor', 0.5, 1, 0.01, ''),
]);

export function schemaEntry(key) {
  for (const e of PARAM_SCHEMA) if (e.key === key) return e;
  return null;
}
