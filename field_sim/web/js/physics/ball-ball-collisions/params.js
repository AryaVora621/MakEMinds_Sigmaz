// params.js - tunable constants for ball-ball collisions v2.
// Units: length in, time s, mass kg, velocity in/s, angular velocity rad/s, inertia kg*in^2.
// Every entry is tagged SOURCED(url) / DERIVED(how) / ASSUMED. No imports (must not import balls.js).

export const M_TO_IN = 39.3701;   // DERIVED: 1 m = 39.3701 in (1/0.0254)
export const G_IN = 386.0886;     // DERIVED: 9.80665 m/s^2 (standard gravity) * 39.3701 in/m

const RAW = {
  massKg: { pollen: 0.025, nectar: 0.041 }, // kg. pollen: balls.js BALL_KINDS (medium confidence); nectar: ASSUMED
  defaultMassKg: 0.025,   // kg. ASSUMED: unknown kind falls back to pollen mass
  inertiaK: { pollen: 0.4, nectar: 0.4 }, // dimensionless, I = k*m*r^2. ASSUMED: solid foam sphere 2/5 (hollow would be 2/3)

  mu: 0.5,        // dimensionless ball-ball Coulomb friction. ASSUMED (old slideFriction; maple-sim 0.8 is piece-field, noted only)
  muRoll: 0.04,   // dimensionless rolling-resistance coeff. ASSUMED (matches BALL.rollFriction; DEM constant-torque model A)
  muSpin: 0.02,   // dimensionless spin (drilling) friction coeff. ASSUMED

  e0: 0.7,          // restitution at eRefSpeed. ASSUMED (keeps BALL.restitutionBall)
  eRefSpeed: 39.37, // in/s (1 m/s). DERIVED: reference speed for e0
  eSpeedExp: 0.2,   // dimensionless. UNVERIFIED recollection of Kuwabara-Kono/Brilliantov 1996 viscoelastic 1-e ~ v^(1/5); 0 gives constant e
  eMin: 0.2,        // ASSUMED lower clamp on restitution
  eMax: 0.85,       // ASSUMED upper clamp on restitution
  restitutionThreshold: 7.874, // in/s = 0.2 m/s. SOURCED(https://github.com/bulletphysics/bullet3 src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h m_restitutionVelocityThreshold)

  velocityIterations: 6, // SOURCED(https://github.com/dyn4j/dyn4j Settings DEFAULT_VELOCITY_CONSTRAINT_SOLVER_ITERATIONS=6); Jolt/Bullet use 10
  positionIterations: 2, // SOURCED(https://github.com/dyn4j/dyn4j Settings DEFAULT_POSITION_CONSTRAINT_SOLVER_ITERATIONS=2); Jolt mNumPositionSteps=2
  baumgarte: 0.2,        // SOURCED(https://github.com/dyn4j/dyn4j Settings DEFAULT_BAUMGARTE; same 0.2 in Jolt mBaumgarte and Bullet m_erp2)
  linearSlop: 0.02,      // in. DERIVED: Box2D B2_LINEAR_SLOP / dyn4j linearTolerance 0.005 m scaled by ball size ~0.1 = 0.0005 m = 0.0197 in
  maxLinearCorrection: 0.25, // in per position iteration. ASSUMED (dyn4j 0.2 m scaled by 0.1 = 0.79 in is too close to maxStepTravel 0.8 in)
  speculativeDistance: 0.08, // in. DERIVED: Box2D B2_SPECULATIVE_DISTANCE = 4*linearSlop
  touchDistance: 0.04,   // in. ASSUMED = 2*linearSlop; s below this counts as touching for islands and wake

  warmStart: true,       // SOURCED(https://github.com/dyn4j/dyn4j warmStartingEnabled default true); Box2D also on
  warmStartFactor: 1.0,  // dimensionless. SOURCED(dyn4j default full warm start)

  cellSize: 4.0,    // in. DERIVED: >= max diameter 3.599 in + margins (Ericson/Teschner cell ~ max object diameter rule, UNVERIFIED)
  fastMargin: 1.0,  // in. ASSUMED: balls with fat margin above this are tested brute-force

  sleepLinear: 1.969, // in/s = 0.05 m/s, compared with pointSpeed. SOURCED(https://github.com/erincatto/box2d src/types.c b2DefaultBodyDef sleepThreshold)
  timeToSleep: 0.5,   // s. SOURCED(https://github.com/erincatto/box2d B2_TIME_TO_SLEEP; Jolt mTimeBeforeSleep, dyn4j minimumAtRestTime all 0.5 s)

  rollSyncRate: 30,     // 1/s. ASSUMED relaxation rate of spin toward rolling-without-slip on a surface
  angularDamping: 0.5,  // 1/s. ASSUMED
  maxAngularSpeed: 400, // rad/s. ASSUMED clamp

  sortByHeight: true, // ASSUMED: solve bottom-up as cheap shock-propagation stand-in (Guendelman 2003, unverified)
};

function deepFreeze(o) {
  for (const k of Object.keys(o)) {
    const v = o[k];
    if (v && typeof v === 'object' && !Object.isFrozen(v)) deepFreeze(v);
  }
  return Object.freeze(o);
}

export const BB_PARAMS = deepFreeze(RAW);

// New frozen params: shallow merge of BB_PARAMS and overrides, nested massKg/inertiaK merged one level.
export function makeParams(overrides = {}) {
  const out = Object.assign({}, BB_PARAMS, overrides);
  out.massKg = Object.assign({}, BB_PARAMS.massKg, overrides.massKg || {});
  out.inertiaK = Object.assign({}, BB_PARAMS.inertiaK, overrides.inertiaK || {});
  return deepFreeze(out);
}
