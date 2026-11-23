// params.js - the ONLY place for hive-physics tunables. NO imports (avoids a cycle
// balls.js -> hivemesh.js -> params.js). Units in key names / comments. Every value is tagged in
// HIVE_PHYS.source as SOURCED (url) / DERIVED (formula) / ASSUMED (reason).
// Frame: inches (Pedro x right, y far, z up), SI inside the torque model.

export const HIVE_PHYS = {
  IN_M: 0.0254,        // m per inch (exact definition)
  G_MPS2: 9.80665,     // m/s^2 standard gravity
  ballMassKg: { pollen: 0.02495, nectar: 0.04128 },   // kg
  ballRadiusIn: { pollen: 1.391, nectar: 1.7995 },    // inch (diameter 2.8 / 3.6 in, rounded as in balls.js)
  tipModel: 'rule',        // 'rule' | 'torque'
  motionModel: 'dynamic',  // 'dynamic' | 'ease'
  calibration: { holdPollen: 7, tipPollen: 8, includeStartingNectar: true, calState: 'audience_up', calCell: 'audience' },
  nominalTipTimeS: 1.0,        // s
  hiveInertiaKgM2: null,       // kg m^2; null = calibrate to nominalTipTimeS
  inertiaSearchKgM2: [1e-3, 5], // kg m^2 bisection bounds
  viscousNmsPerRad: 0.02,      // N m s/rad
  coulombNm: 0.02,             // N m
  staticNm: 0.03,              // N m
  stopRestitution: 0,          // dimensionless
  restOmegaRadS: 0.05,         // rad/s
  maxSubstepS: 0.001,          // s
  maxStepS: 5,                 // s
  drainPastCentreDeg: 9.79,    // deg = asin(0.17)
  captureImpulse: true,
  includeBallInertia: true,
  upSign: { audience: -1, far: 1 },
  stack: { rowNestFactor: 1.0, layerRise: 1.0 },   // dimensionless
  mesh: { activeEdges: true, activeEdgeCos: 0.996195, weldTolIn: 1e-4, mergeNormalCos: 0.996195, sweepEps: 1e-9 },
  source: {
    IN_M: 'DERIVED: 1 in = 0.0254 m (SI definition)',
    G_MPS2: 'SOURCED: standard gravity, CGPM 1901 (9.80665 m/s^2)',
    ballMassKg: 'ASSUMED (low confidence): pollen 0.055 lb = 0.02495 kg (AndyMark/Plex listing snippet); nectar 0.091 lb = 0.04128 kg (pairing assumed). No manual mass found.',
    ballRadiusIn: 'SOURCED: FTC manual Sec 9.8 https://ftc-resources.firstinspires.org/ftc/game/manual-09 (pollen 2.8 in, nectar 3.6 in diameter; polyethylene Gopher ResisDent); repo radii 1.391/1.7995 in kept',
    tipModel: 'DERIVED: owner fact (8 pollen tip, 7 do not; hivetip_model.js) stays authoritative, so default is the rule',
    motionModel: 'ASSUMED: dynamic 1-DOF motion replaces the legacy 0.6 s smoothstep; no measured motion profile',
    calibration: 'DERIVED: owner fact 7 pollen hold / 8 pollen tip; starting nectar 3 per manual 10.3.1.B; calibration state/cell are the audience cell at audience_up',
    nominalTipTimeS: 'ASSUMED: no measured tip time exists; replace from video',
    hiveInertiaKgM2: 'ASSUMED: null means derived by bisection so a rule-assisted 8-pollen tip lasts nominalTipTimeS',
    inertiaSearchKgM2: 'ASSUMED: wide bracket around plausible FTC-hive inertia',
    viscousNmsPerRad: 'ASSUMED: no bearing/damper data',
    coulombNm: 'ASSUMED: no bearing friction data',
    staticNm: 'ASSUMED: no bearing friction data',
    stopRestitution: 'ASSUMED: damper absorbs the hit; 0 keeps motion monotone (existing tests require it)',
    restOmegaRadS: 'ASSUMED: numerical latch threshold',
    maxSubstepS: 'ASSUMED: numerical, 1 ms semi-implicit Euler',
    maxStepS: 'ASSUMED: numerical clamp on frame dt',
    drainPastCentreDeg: 'DERIVED: asin(0.17) = 9.79 deg from balls.js DROP_SLOPE = 0.17, so the model agrees with when balls.js releases balls',
    captureImpulse: 'ASSUMED: modelling choice, ball linear momentum about the pivot transfers on capture',
    includeBallInertia: 'ASSUMED: modelling choice, balls as point masses add to hive inertia',
    upSign: 'DERIVED: sign of the stable angle of the state in which that cell is up (audience_up = -30 deg, far_up = +30 deg)',
    stack: 'ASSUMED: square packing (nest 1.0) guarantees no overlap; no packing data from the real cell',
    mesh: 'SOURCED: activeEdgeCos = cos 5 deg = 0.996195 from Jolt MeshShape.h mActiveEdgeCosThresholdAngle (medium confidence); Box2D ghost-collision face-normal rule https://box2d.org/posts/2020/06/ghost-collisions/; Bullet btInternalEdgeUtility. weldTolIn 1e-4 in, mergeNormalCos = activeEdgeCos, sweepEps 1e-9 are ASSUMED numerical tolerances.',
  },
};

function isPlain(v) { return v !== null && typeof v === 'object' && !Array.isArray(v); }

function clone(v) {
  if (Array.isArray(v)) return v.map(clone);
  if (isPlain(v)) { const o = {}; for (const k of Object.keys(v)) o[k] = clone(v[k]); return o; }
  return v;
}

function merge(base, over) {
  for (const k of Object.keys(over)) {
    const ov = over[k];
    if (isPlain(ov) && isPlain(base[k])) merge(base[k], ov);
    else base[k] = clone(ov);
  }
  return base;
}

/** Fresh deep-merged copy of HIVE_PHYS. Plain objects merge recursively, arrays and scalars replace. Never mutates HIVE_PHYS. */
export function hivePhysParams(overrides = {}) {
  const out = clone(HIVE_PHYS);
  if (isPlain(overrides)) merge(out, overrides);
  return out;
}
