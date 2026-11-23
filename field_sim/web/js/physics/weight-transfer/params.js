// params.js - ALL tunable constants for the weight-transfer physics area.
// Units: length in, mass lbm, time s. Force unit lbm*in/s^2 (lbf = that / G).
// Provenance tags: vault/spec = robot_spec.json, src = sourced (URL), derived = derived here, placeholder = guessed.
// Robot frame: +x forward, +y left, z up, origin at robot centre on the floor. Wheel order FL, FR, BL, BR.

export const G = 386.0886; // in/s^2. src: standard gravity 9.80665 m/s^2 / 0.0254 m/in
export const WHEEL_ORDER = ['FL', 'FR', 'BL', 'BR'];
export const SX = [1, 1, -1, -1]; // +1 front wheels. derived
export const SY = [1, -1, 1, -1]; // +1 left wheels. derived

export const DEFAULT_PARAMS = {
  mass: {
    totalLb: 31.4, // lbm. vault/spec chassis.mass_lb; item masses are rescaled to sum to this
    // item positions in robot frame (in); turret items are relative to the turret pivot; box = self-inertia box (in). All placeholder unless noted.
    items: [
      { name: 'frame_drive', massLb: 13.0, x: 0, y: 0, z: 2.2, frame: 'chassis', box: { sx: 15, sy: 14.76, sz: 3.5 } }, // placeholder
      { name: 'battery', massLb: 1.3, x: -3.4, y: 4.1, z: 6.9, frame: 'chassis', box: { sx: 5.9, sy: 1.6, sz: 2.3 } }, // position vault/spec battery, mass placeholder
      { name: 'electronics', massLb: 1.5, x: -2.0, y: -3.0, z: 5.0, frame: 'chassis', box: { sx: 4, sy: 4, sz: 1.5 } }, // placeholder
      { name: 'transfer', massLb: 2.6, x: 0, y: 0, z: 5.2, frame: 'chassis', box: { sx: 6, sy: 3.3, sz: 3 } }, // placeholder
      { name: 'deck', massLb: 2.0, x: 0, y: 0, z: 8.82, frame: 'chassis', box: { sx: 15, sy: 14.76, sz: 0.25 } }, // z vault/spec chassis.deckZ, mass placeholder
      { name: 'intake', massLb: 3.0, x: 6.5, y: 0, z: 4.5, frame: 'intake', box: { sx: 4.2, sy: 14.9, sz: 4 } }, // placeholder; shifts +x by intakeDeployed*intakeTravelIn
      { name: 'turret_ring', massLb: 2.5, x: 0, y: 0, z: 9.5, frame: 'chassis', box: { sx: 5.9, sy: 5.9, sz: 1 } }, // z vault/spec shooter.pivot, mass placeholder
      { name: 'shooter_head', massLb: 4.0, x: 1.5, y: 0, z: 3.0, frame: 'turret', box: { sx: 5, sy: 4, sz: 4 } }, // placeholder; position relative to turret pivot
      { name: 'flower_arm', massLb: 1.5, x: -4.0, y: 0, z: 10.0, frame: 'chassis', box: { sx: 2, sy: 6, sz: 4 } }, // placeholder
    ],
    turretPivot: { x: 0, y: 0, z: 9.5 }, // in. vault/spec shooter.pivot
    intakeTravelIn: 2.7, // in. vault/spec intake protrusionDeployedIn(4.2) - protrusionFoldedIn(1.5)
    ballMassLb: 0.165, // lbm per ball, ADDED on top of totalLb. placeholder
    ballSlots: [ // in. x,y placeholder; z = vault/spec chassis.laneCenterZ (placeholder in spec), capacity 4
      { x: 4.5, y: 0, z: 5.2 },
      { x: 1.5, y: 0, z: 5.2 },
      { x: -1.5, y: 0, z: 5.2 },
      { x: -4.5, y: 0, z: 5.2 },
    ],
  },
  geometry: {
    wheels: [ // in, vault/spec wheels[] (y is placeholder in spec)
      { name: 'FL', x: 5.04, y: 6.35 },
      { name: 'FR', x: 5.04, y: -6.35 },
      { name: 'BL', x: -5.04, y: 6.35 },
      { name: 'BR', x: -5.04, y: -6.35 },
    ],
  },
  traction: {
    mu: 1.0, // placeholder; order of magnitude from https://rsms.me/etc/car-physics/ (street tyres mu=1.0); measure on tiles
    muScale: 1.0, // dimensionless; integrator may set 1 - 0.7*drive.slip. placeholder
    rollerAngleDeg: 45, // deg. vault/spec mecanum rollers; forceFactor = 1/cos = sqrt(2)
  },
  compliance: {
    kWheelLbfPerIn: 50, // lbf/in per wheel. placeholder (gives ~0.7 deg pitch at 140 in/s^2 braking, ~0.16 in static sink)
    zeta: 0.3, // damping ratio, dimensionless. placeholder; form from https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/Vehicles.html
    maxSubstepS: 0.002, // s. numerical
  },
  tip: {
    restitutionWall: 0.2, // dimensionless. placeholder
    contactZIn: 2.0, // in, bumper contact height (spec bumperHeight is 3). placeholder
    minRockOmega: 0.05, // rad/s. placeholder
    settleOmega: 0.3, // rad/s. placeholder
    substepS: 0.001, // s. numerical
    fallEaseRate: 8, // 1/s. placeholder
    antiTipMarginIn: 0.5, // in, used only when antiTip true. placeholder
    antiTip: false, // enable anti-tip limiting in traction. design choice
  },
  impact: {
    contactTimeS: 0.015, // s, impact dv -> peak accel for HUD only. placeholder
  },
  accel: {
    lpfTauS: 0.02, // s, low-pass on differenced accel. placeholder
    clampInS2: 2 * G, // in/s^2. placeholder
  },
  limits: {
    searchMaxInS2: 2 * G, // in/s^2 upper bound for limit search. placeholder
    bisectIters: 24, // count. numerical
  },
};

function clone(v) { return JSON.parse(JSON.stringify(v)); }
function isObj(v) { return v !== null && typeof v === 'object' && !Array.isArray(v); }
function deepMerge(dst, src) {
  for (const k of Object.keys(src || {})) {
    const sv = src[k];
    if (isObj(sv) && isObj(dst[k])) deepMerge(dst[k], sv);
    else dst[k] = (Array.isArray(sv) || isObj(sv)) ? clone(sv) : sv; // arrays replaced, not merged
  }
  return dst;
}
const num = (v) => typeof v === 'number' && Number.isFinite(v);

function applySpec(p, spec) {
  const ch = spec.chassis;
  if (ch) {
    if (num(ch.mass_lb)) p.mass.totalLb = ch.mass_lb;
    if (num(ch.laneCenterZ)) for (const s of p.mass.ballSlots) s.z = ch.laneCenterZ;
    if (num(ch.deckZ)) { const d = p.mass.items.find((i) => i.name === 'deck'); if (d) d.z = ch.deckZ; }
  }
  if (Array.isArray(spec.wheels)) {
    for (const w of p.geometry.wheels) {
      const sw = spec.wheels.find((q) => q.name === w.name);
      if (sw && num(sw.x) && num(sw.y)) { w.x = sw.x; w.y = sw.y; }
    }
  }
  const pv = spec.shooter && spec.shooter.pivot;
  if (pv) {
    if (num(pv.x)) p.mass.turretPivot.x = pv.x;
    if (num(pv.y)) p.mass.turretPivot.y = pv.y;
    if (num(pv.z)) p.mass.turretPivot.z = pv.z;
  }
  const it = spec.intake;
  if (it && num(it.protrusionDeployedIn) && num(it.protrusionFoldedIn)) {
    p.mass.intakeTravelIn = it.protrusionDeployedIn - it.protrusionFoldedIn;
  }
  const bt = spec.battery;
  if (bt) {
    const b = p.mass.items.find((i) => i.name === 'battery');
    if (b) { if (num(bt.x)) b.x = bt.x; if (num(bt.y)) b.y = bt.y; if (num(bt.z)) b.z = bt.z; }
  }
}

export function makeParams(overrides = {}, spec = null) {
  const p = clone(DEFAULT_PARAMS);
  if (spec) applySpec(p, spec);
  deepMerge(p, overrides || {});
  const sum = p.mass.items.reduce((s, i) => s + i.massLb, 0);
  if (sum > 0 && p.mass.totalLb > 0) {
    const f = p.mass.totalLb / sum;
    for (const i of p.mass.items) i.massLb *= f;
  }
  return p;
}

export function wheelGeometry(params) {
  const ws = params.geometry.wheels;
  const n = ws.length;
  let xc = 0, yc = 0;
  for (const w of ws) { xc += w.x; yc += w.y; }
  xc /= n; yc /= n;
  let a = 0, b = 0;
  for (const w of ws) { a += Math.abs(w.x - xc); b += Math.abs(w.y - yc); }
  a /= n; b /= n;
  return {
    xc, yc, a, b, k: a + b,
    wheels: ws.map((w, i) => ({ name: w.name, x: w.x, y: w.y, sx: SX[i], sy: SY[i] })),
  };
}

export function forceFactor(params) {
  return 1 / Math.cos((params.traction.rollerAngleDeg * Math.PI) / 180);
}
