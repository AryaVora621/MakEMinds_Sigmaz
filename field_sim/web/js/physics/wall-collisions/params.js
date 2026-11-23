// params.js - single tunables file for the wall-collisions area (robot vs fixed field geometry).
// No DOM. Units: inches (in), seconds (s), radians, mass in pounds-mass (lb).
// Velocity in/s, impulse lb*in/s, angular impulse lb*in^2/s.
// Every value is tagged SOURCED (with origin) or ASSUMPTION. Edit here only; redeploy to apply.

// SI conversions, used only for the force/torque report. Exact: 1 lb = 0.45359237 kg, 1 in = 0.0254 m.
export const UNITS = Object.freeze({
  IN_PER_M: 39.37007874,          // in/m, sourced: 1/0.0254 (exact definition of the inch)
  N_PER_LB_IN_S2: 0.0115212462,   // N per (lb*in/s^2) = 0.45359237*0.0254, sourced: exact unit definitions
  NM_PER_LB_IN2_S2: 2.92639653e-4 // N*m per (lb*in^2/s^2) = 0.45359237*0.0254^2, sourced: exact unit definitions
});

function deepFreeze(o) {
  for (const k of Object.keys(o)) {
    const v = o[k];
    if (v && typeof v === 'object' && !Object.isFrozen(v)) deepFreeze(v);
  }
  return Object.freeze(o);
}

export const WALL_PARAMS = deepFreeze({
  robot: {
    halfX: 9,            // in; sourced: legacy collision.js ROBOT; FTC 18 in envelope (rule not verified)
    halfY: 9,            // in; sourced: legacy collision.js ROBOT; FTC 18 in envelope (rule not verified)
    chamfer: 0,          // in; ASSUMPTION: 0 keeps parity with legacy square corners
    bumper: 0,           // in; ASSUMPTION: added to both half extents, none fitted
    massLb: 31.4,        // lb; sourced: web/data/robot_spec.json mass_lb
    inertiaScale: 1.0    // dimensionless; ASSUMPTION: uniform plate, I = scale*m*((2hx)^2+(2hy)^2)/12
  },
  solver: {
    substeps: 4,                 // count; sourced (recollection, unverified): Box2D v3 default subStepCount
    relaxIterations: 1,          // count; sourced (medium confidence): Box2D v3 solver.c RELAX_ITERATIONS
    restitutionIterations: 2,    // count; sourced (medium): Box2D v3 types.c restitutionIterations
    contactHertz: 30,            // Hz; sourced (medium): Box2D v3 types.c, Rapier contact_softness.natural_frequency
    dampingRatio: 10,            // dimensionless; sourced (medium): Box2D v3 contactDampingRatio, Rapier damping_ratio
    maxPushSpeed: 118.11,        // in/s (3 m/s); sourced (medium): Box2D v3 contactSpeed, Rapier normalized_max_corrective_velocity
    linearSlop: 0.19685,         // in (0.005 m); sourced (medium): Box2D B2_LINEAR_SLOP, dyn4j linearTolerance
    speculativeDistance: 0.7874, // in (0.02 m = 4*slop); sourced (medium): Box2D B2_SPECULATIVE_DISTANCE, Rapier prediction distance
    refFaceTol: 0.019685,        // in (0.1*linearSlop); ASSUMPTION: Box2D 0.1*linearSlop tolerance used as ref-face hysteresis
    hertzSubstepCap: 0.25,       // dimensionless; sourced (recollection, unverified): effective hertz = min(hertz, cap/hSub), Box2D v3 world.c
    warmStart: true,             // boolean; sourced: standard sequential-impulse warm starting
    maxChunkTravel: 2.0,         // in; ASSUMPTION: max swept travel of any robot vertex per chunk (legacy used 1.5 in step)
    maxChunks: 32,               // count; ASSUMPTION: cap on per-frame chunks
    hardPenetration: 0.5         // in; ASSUMPTION: above this after the step, fall back to legacy kinematic projection
  },
  restitution: {
    threshold: 10                // in/s (0.254 m/s); ASSUMPTION: between Bullet 0.2 m/s and Box2D/dyn4j 1.0 m/s
  },
  materials: {
    // mu = sqrt(0.65*0.6): maple-sim bumper friction 0.65 x obstacle friction 0.6 (attribution unverified), geometric-mean mixing
    // (Box2D convention, ASSUMPTION). e = maple-sim BUMPER_COEFFICIENT_OF_RESTITUTION 0.08. No FTC-measured values exist: hardware tests advised.
    // Optional per-material overrides: hertz (Hz), zeta (dimensionless) to model compliance such as perimeter flex.
    default:    { mu: 0.62, e: 0.08 },
    bounds:     { mu: 0.62, e: 0.08 },
    hive_frame: { mu: 0.62, e: 0.08 },
    flower:     { mu: 0.62, e: 0.08 }
  }
});

function clone(o) {
  if (Array.isArray(o)) return o.map(clone);
  if (o && typeof o === 'object') {
    const r = {};
    for (const k of Object.keys(o)) r[k] = clone(o[k]);
    return r;
  }
  return o;
}

function isObj(v) { return v !== null && typeof v === 'object' && !Array.isArray(v); }

// Fresh mutable validated copy. One level of merge per group (robot, solver, restitution, materials);
// within materials each kind is merged field-wise so {materials:{bounds:{mu:0.3}}} keeps bounds.e.
export function makeParams(overrides) {
  const P = clone(WALL_PARAMS);
  if (overrides != null) {
    if (!isObj(overrides)) throw new RangeError('overrides must be an object');
    for (const g of Object.keys(overrides)) {
      const src = overrides[g];
      if (!isObj(src)) { P[g] = src; continue; }
      if (!isObj(P[g])) P[g] = {};
      if (g === 'materials') {
        for (const kind of Object.keys(src)) {
          P.materials[kind] = isObj(src[kind])
            ? Object.assign({}, P.materials[kind] || {}, src[kind])
            : src[kind];
        }
      } else {
        Object.assign(P[g], src);
      }
    }
  }
  validate(P);
  return P;
}

function need(cond, name, v) {
  if (!cond) throw new RangeError(`${name} invalid: ${v}`);
}
const fin = (v) => typeof v === 'number' && Number.isFinite(v);

function validate(P) {
  const r = P.robot, s = P.solver;
  need(Number.isInteger(s.substeps) && s.substeps >= 1, 'substeps', s.substeps);
  need(fin(r.chamfer) && r.chamfer >= 0, 'chamfer', r.chamfer);
  need(fin(r.halfX) && r.halfX > r.chamfer, 'halfX', r.halfX);
  need(fin(r.halfY) && r.halfY > r.chamfer, 'halfY', r.halfY);
  need(fin(r.bumper), 'bumper', r.bumper);
  need(fin(r.massLb) && r.massLb > 0, 'massLb', r.massLb);
  need(fin(r.inertiaScale) && r.inertiaScale > 0, 'inertiaScale', r.inertiaScale);
  need(fin(s.contactHertz) && s.contactHertz > 0, 'contactHertz', s.contactHertz);
  need(fin(s.dampingRatio) && s.dampingRatio >= 0, 'dampingRatio', s.dampingRatio);
  need(fin(s.hertzSubstepCap) && s.hertzSubstepCap > 0, 'hertzSubstepCap', s.hertzSubstepCap);
  for (const k of ['relaxIterations', 'restitutionIterations']) {
    need(Number.isInteger(s[k]) && s[k] >= 0, k, s[k]);
  }
  need(Number.isInteger(s.maxChunks) && s.maxChunks >= 1, 'maxChunks', s.maxChunks);
  for (const k of ['maxPushSpeed', 'linearSlop', 'speculativeDistance', 'maxChunkTravel']) {
    need(fin(s[k]) && s[k] > 0, k, s[k]);
  }
  for (const k of ['refFaceTol', 'hardPenetration']) need(fin(s[k]) && s[k] >= 0, k, s[k]);
  need(fin(P.restitution.threshold) && P.restitution.threshold >= 0, 'threshold', P.restitution.threshold);
  need(isObj(P.materials.default), 'materials.default', P.materials.default);
  for (const kind of Object.keys(P.materials)) {
    const m = P.materials[kind];
    need(isObj(m), `materials.${kind}`, m);
    if (m.mu !== undefined) need(fin(m.mu) && m.mu >= 0, `materials.${kind}.mu`, m.mu);
    if (m.e !== undefined) need(fin(m.e) && m.e >= 0 && m.e <= 1, `materials.${kind}.e`, m.e);
    if (m.hertz !== undefined) need(fin(m.hertz) && m.hertz > 0, `materials.${kind}.hertz`, m.hertz);
    if (m.zeta !== undefined) need(fin(m.zeta) && m.zeta >= 0, `materials.${kind}.zeta`, m.zeta);
  }
  need(fin(P.materials.default.mu) && fin(P.materials.default.e), 'materials.default', 'mu/e missing');
}

// Material for an obstacle kind; unknown kinds use default. hertz/zeta fall back to solver contactHertz/dampingRatio.
export function materialFor(P, kind) {
  const d = P.materials.default;
  const m = (kind != null && P.materials[kind]) || d;
  const pick = (k, fb) => (m[k] !== undefined ? m[k] : (d[k] !== undefined ? d[k] : fb));
  return {
    mu: pick('mu', 0),
    e: pick('e', 0),
    hertz: pick('hertz', P.solver.contactHertz),
    zeta: pick('zeta', P.solver.dampingRatio)
  };
}
