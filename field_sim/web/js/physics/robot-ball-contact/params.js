// params.js - single source of all robot-ball-contact tunables.
// Units: length in, time s, velocity in/s, angle rad, mass kg, impulse kg*in/s,
// "sim force" = kg*in/s^2 (1 N = 39.3700787 sim force). Stiffness in N/m = kg/s^2 (length-unit independent).
// Every numeric/boolean leaf has an entry in RBC_SOURCES keyed by its dotted path:
//   SOURCED(url) / COMPUTED / ASSUMED.

const BOX2D = 'https://github.com/erincatto/box2d/blob/main/src/';
const BULLET = 'https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h';

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}

function ballKind(mass, r, k, sq) {
  return { massKg: mass, radiusIn: r, inertiaFactor: 2 / 3, restitution: 0.35, friction: 0.5, stiffnessNpm: k, maxSquishFrac: sq };
}

const DEFAULTS = {
  units: { N_TO_SIM: 39.3700787, LB_TO_KG: 0.45359237, IN_PER_M: 39.3700787 },
  robot: {
    massKg: 15, restitution: 0.2, friction: 0.5, reactionEnabled: true, legacyInfiniteMass: false,
    maxReactionDvPerStep: 5, maxReactionDomPerStep: 0.5,
  },
  ball: {
    pollen: ballKind(0.025, 1.391, 2000, 0.25),
    nectar: ballKind(0.041, 1.7995, 1500, 0.3),
  },
  contact: { restitutionThresholdInS: 7.874, maxContactsPerBall: 2, positionSlopIn: 0.0 },
  shape: { cheekWidthIn: 0.5, pocketDepthIn: 3.0, pocketHeightIn: 4.0, sides: ['front', 'rear'] },
  roller: {
    diameterIn: 2.8346, freeRpm: 1150, axleOutboardIn: 0.5, axleHeightIn: 4.0, stiffnessNpm: 3000,
    maxDeflectIn: 0.6, friction: 0.8, restitution: 0.3, dampingRatio: 0.3, stallForceN: 25, fullBehavior: 'block',
  },
  capture: {
    captureDepthIn: 1.0, maxBallSpeedInS: 220, maxBottomZIn: 3.2, failProbMax: 0,
    failSpeedLoInS: 60, failSpeedHiInS: 160,
  },
  hold: { limit: 4 },
  pinch: { cosOpposed: 0.7071, popSpeedInS: 60, overSquishEject: 1.5 },
  trap: { chassisClearanceIn: 0.375 },
};

/** Provenance for every leaf. Keys are dotted paths into RBC_PARAMS. */
const SRC = {
  'units.N_TO_SIM': 'COMPUTED 1 N = 39.3700787 in/m * kg*m/s^2 -> kg*in/s^2 (exact unit conversion)',
  'units.LB_TO_KG': 'COMPUTED exact definition of the avoirdupois pound, 0.45359237 kg',
  'units.IN_PER_M': 'COMPUTED 1/0.0254 m per in',
  'robot.massKg': 'ASSUMED 15 kg typical FTC robot; integrator may override with robotspec mass_lb*LB_TO_KG',
  'robot.restitution': 'ASSUMED aluminium/polycarbonate frame vs plastic ball',
  'robot.friction': 'ASSUMED frame surface vs plastic ball',
  'robot.reactionEnabled': 'ASSUMED design switch (not a physical value)',
  'robot.legacyInfiniteMass': 'ASSUMED design switch; true reproduces legacy infinite-mass robot',
  'robot.maxReactionDvPerStep': 'ASSUMED safety clamp, in/s per substep',
  'robot.maxReactionDomPerStep': 'ASSUMED safety clamp, rad/s per substep',
  'ball.pollen.massKg': 'ASSUMED pollen ball mass (matches balls.js)',
  'ball.pollen.radiusIn': 'ASSUMED BIOBUZZ pollen radius in inches (matches balls.js)',
  'ball.pollen.inertiaFactor': 'ASSUMED hollow thin shell I = 2/3 m r^2',
  'ball.pollen.restitution': 'ASSUMED equals legacy BALL.restitutionRobot so the max() mix keeps legacy behaviour',
  'ball.pollen.friction': 'ASSUMED equals legacy slideFriction',
  'ball.pollen.stiffnessNpm': 'ASSUMED soft-shell contact stiffness, N/m',
  'ball.pollen.maxSquishFrac': 'ASSUMED max compression as fraction of diameter',
  'ball.nectar.massKg': 'ASSUMED nectar ball mass (matches balls.js)',
  'ball.nectar.radiusIn': 'ASSUMED BIOBUZZ nectar radius in inches (matches balls.js)',
  'ball.nectar.inertiaFactor': 'ASSUMED hollow thin shell I = 2/3 m r^2',
  'ball.nectar.restitution': 'ASSUMED equals legacy BALL.restitutionRobot',
  'ball.nectar.friction': 'ASSUMED equals legacy slideFriction',
  'ball.nectar.stiffnessNpm': 'ASSUMED soft-shell contact stiffness, N/m',
  'ball.nectar.maxSquishFrac': 'ASSUMED max compression as fraction of diameter',
  'contact.restitutionThresholdInS': 'SOURCED(' + BULLET + ') m_restitutionVelocityThreshold = 0.2 m/s -> 7.874 in/s (Box2D uses 1 m/s = 39.37 in/s, ' + BOX2D + 'types.c)',
  'contact.maxContactsPerBall': 'ASSUMED cap on simultaneous box contacts per ball',
  'contact.positionSlopIn': 'SOURCED(' + BULLET + ') m_linearSlop default 0',
  'shape.cheekWidthIn': 'ASSUMED intake side plate width',
  'shape.pocketDepthIn': 'ASSUMED open mouth pocket depth from face to gate/body',
  'shape.pocketHeightIn': 'ASSUMED pocket opening height (> nectar diameter 3.6 in)',
  'shape.sides': 'ASSUMED intake faces present on the robot',
  'roller.diameterIn': 'ASSUMED hardware: 72 mm roller = 2.8346 in',
  'roller.freeRpm': 'ASSUMED Yellow-Jacket-class motor free speed (surface speed 170.7 in/s is COMPUTED = rpm*pi*D/60)',
  'roller.axleOutboardIn': 'ASSUMED axle offset beyond the body face, in',
  'roller.axleHeightIn': 'ASSUMED axle height above floor, in (pollen compression 0.199 in, nectar 1.016 in COMPUTED)',
  'roller.stiffnessNpm': 'ASSUMED compliant roller stiffness, N/m',
  'roller.maxDeflectIn': 'ASSUMED roller max deflection, in',
  'roller.friction': 'ASSUMED compliant wheel on plastic; not found in research',
  'roller.restitution': 'ASSUMED',
  'roller.dampingRatio': 'ASSUMED damping ratio zeta (dimensionless)',
  'roller.stallForceN': 'ASSUMED stall force at roller surface, N',
  'roller.fullBehavior': 'ASSUMED policy when hold full: block | spit (maple-sim style refuse-when-full)',
  'capture.captureDepthIn': 'ASSUMED ball centre depth inside face plane that counts as loaded, in',
  'capture.maxBallSpeedInS': 'ASSUMED legacy gate for balls in flight, in/s',
  'capture.maxBottomZIn': 'ASSUMED legacy maxLift, in',
  'capture.failProbMax': 'ASSUMED default 0 = deterministic capture',
  'capture.failSpeedLoInS': 'ASSUMED lower edge of miss-probability speed ramp, in/s',
  'capture.failSpeedHiInS': 'ASSUMED upper edge of miss-probability speed ramp, in/s',
  'hold.limit': 'ASSUMED game rule G407 hold limit 4 (matches balls.js MAX_HELD / matchrules RULES.holdLimit)',
  'pinch.cosOpposed': 'ASSUMED normals opposed within 45 deg (cos 45 = 0.7071)',
  'pinch.popSpeedInS': 'ASSUMED ejection vertical speed, in/s',
  'pinch.overSquishEject': 'ASSUMED eject when compression > factor * allowed (dimensionless)',
  'trap.chassisClearanceIn': 'ASSUMED chassis ground clearance, in',
};

export const RBC_SOURCES = deepFreeze(SRC);
export const RBC_PARAMS = deepFreeze(DEFAULTS);

function isPlain(o) { return o !== null && typeof o === 'object' && !Array.isArray(o); }
function clone(v) {
  if (Array.isArray(v)) return v.map(clone);
  if (isPlain(v)) { const o = {}; for (const k of Object.keys(v)) o[k] = clone(v[k]); return o; }
  return v;
}
function merge(base, over) {
  const out = clone(base);
  if (!isPlain(over)) return out;
  for (const k of Object.keys(over)) {
    if (isPlain(over[k]) && isPlain(out[k])) out[k] = merge(out[k], over[k]);
    else out[k] = clone(over[k]);
  }
  return out;
}

/** Deep-merge overrides onto the defaults. Returns a new, unfrozen object; never mutates RBC_PARAMS or overrides. */
export function makeParams(overrides = {}) { return merge(DEFAULTS, overrides); }
