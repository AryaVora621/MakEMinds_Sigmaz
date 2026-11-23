// params.js - the only home for rigid-body-solver tunables. SI units unless a name says "In".
// Source tags: "source:" = taken from a named engine/library; "PLACEHOLDER" = assumed, must be measured.
// Researcher findings were medium confidence (model-summarised, not read line by line).

function deepFreeze(o) {
  for (const k of Object.keys(o)) {
    const x = o[k];
    if (x && typeof x === 'object') deepFreeze(x);
  }
  return Object.freeze(o);
}

function deepClone(o) {
  if (o && typeof o === 'object') {
    const r = {};
    for (const k of Object.keys(o)) r[k] = deepClone(o[k]);
    return r;
  }
  return o;
}

const DEFAULTS = {
  units: {
    IN_TO_M: 0.0254, // m per inch, exact by definition
  },
  step: {
    maxSubstepDt: 0.004, // s; source: maple-sim SIMULATION_SUB_TICKS_IN_1_PERIOD = 5 per 20 ms
    maxSubsteps: 16, // count; PLACEHOLDER guard
    maxFrameDt: 0.1, // s; PLACEHOLDER, clamps tab-switch frames
  },
  solver: {
    velocityIterations: 8, // count; source: planck.js World DEFAULTS velocityIterations
    relaxIterations: 3, // count; PLACEHOLDER (Box2D v3 relax stage = solve with useBias=false)
    positionIterations: 3, // count; source: planck.js World DEFAULTS positionIterations (NGS mode only)
    restitutionIterations: 8, // count; assumed (Box2D v3 uses 1 pass; 8 passes converge a 2-point face, see contact_solver.js)
    restitutionTolerance: 1e-4, // m/s; assumed early-exit tolerance for the restitution passes
    positionMode: 'soft', // 'soft' (Box2D v3 soft step) | 'baumgarte' (box2d-lite) | 'ngs' (planck / Box2D v2.4)
    warmStarting: true, // source: Catto 2005 contact caching, box2d-lite
    warmStartFactor: 1.0, // dimensionless; source: Rapier warmstart_coefficient = 1.0 (Bullet uses 0.85)
  },
  contact: {
    linearSlop: 0.005, // m; source: planck Settings.ts, Box2D v3 B2_LINEAR_SLOP
    speculativeDistance: 0.02, // m = 4*linearSlop; source: Box2D v3 constants.h B2_SPECULATIVE_DISTANCE
    baumgarte: 0.2, // dimensionless; source: planck Settings.baumgarte, box2d-lite k_biasFactor
    allowedPenetration: 0.01, // m; source: box2d-lite k_allowedPenetration (baumgarte mode)
    maxLinearCorrection: 0.2, // m; source: planck Settings
    ngsAcceptFactor: 3, // multiples of linearSlop; source: planck Solver.ts contactsOkay (minSep >= -3*slop)
    contactHertz: 30, // Hz; source: Box2D v3 b2DefaultWorldDef, Rapier contact_defaults
    contactDampingRatio: 10, // dimensionless; source: Box2D v3 b2DefaultWorldDef, Rapier
    maxContactPushSpeed: 3.0, // m/s; source: Box2D v3 contactSpeed / maxContactPushSpeed
    restitutionThreshold: 1.0, // m/s; source: Box2D v3.1.0 types.c, planck velocityThreshold
  },
  collide: {
    faceRelTol: 0.95, // dimensionless; source: box2d-lite Collide.cpp relativeTol
    faceAbsTolFrac: 0.01, // fraction of shapeA.boundRadius; source: box2d-lite Collide.cpp absoluteTol
  },
  motion: {
    maxTranslation: 2.0, // m per substep; source: planck Settings
    maxRotation: 0.5 * Math.PI, // rad per substep; source: planck Settings
    linearDamping: 0.1, // 1/s; source: maple-sim chassis Body damping
    angularDamping: 0.1, // 1/s; source: maple-sim chassis Body damping
    gravity: 9.81, // m/s^2; standard gravity, used only for the wheel normal force
  },
  materials: {
    robot: { friction: 0.65, restitution: 0.08 }, // source: maple-sim BUMPER_COEFFICIENT_OF_FRICTION/RESTITUTION
    obstacle: { friction: 0.6, restitution: 0.3 }, // source: maple-sim FieldMap.createObstacle
    wall: { friction: 0.6, restitution: 0.3 }, // source: maple-sim FieldMap.createObstacle
  },
  robot: {
    massKg: 15, // kg; task brief, tunable (FTC mass limit not verified)
    halfXIn: 9, // in; source: collision.js ROBOT
    halfYIn: 9, // in; source: collision.js ROBOT
    inertiaScale: 1.0, // dimensionless; I = scale*m*(L^2+W^2)/12, Choreo "Estimating MOI" (likely an underestimate)
  },
  drive: {
    muWheel: 0.7, // dimensionless; PLACEHOLDER
    strafeEfficiency: 0.75, // dimensionless; PLACEHOLDER (mecanum strafe traction fraction)
    turnLeverArm: 0.19, // m; PLACEHOLDER (about (lx+ly)/2 for an 18 in base)
    motorForceMax: 180, // N; PLACEHOLDER (about 4 * stall torque / wheel radius)
  },
  field: {
    zCapIn: 18, // in; source: collision.js Z_ROBOT
    wallThickness: 0.5, // m; assumed (thick enough to prevent tunnelling)
  },
  filter: {
    CAT_STATIC: 0x0001, // bit mask; design choice
    CAT_ROBOT: 0x0002, // bit mask; design choice
    MASK_ALL: 0xffff, // bit mask; design choice
  },
};

export const PARAMS = deepFreeze(deepClone(DEFAULTS));

function merge(target, src) {
  for (const k of Object.keys(src)) {
    const sv = src[k];
    if (sv && typeof sv === 'object' && !Array.isArray(sv) && target[k] && typeof target[k] === 'object') {
      merge(target[k], sv);
    } else if (sv && typeof sv === 'object') {
      target[k] = deepClone(sv);
    } else if (sv !== undefined) {
      target[k] = sv;
    }
  }
  return target;
}

// Returns a NEW unfrozen deep-merged plain object; PARAMS is never mutated.
export function makeParams(overrides = {}) {
  return merge(deepClone(DEFAULTS), overrides || {});
}
