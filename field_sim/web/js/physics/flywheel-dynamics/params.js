// params.js - single tunable-constants file for the flywheel-dynamics model.
// Pure ES module, no DOM, no dependencies. Internal units are SI (rad/s, N*m, kg*m^2, s, V, A, m, m/s).
// rpm and inches appear only at API edges.
//
// RESEARCH STATUS (honest): the web-search budget ran out; NO measured FTC/FRC dip %, recovery ms or spin-up
// trace was found. Every constant carries a tag in PARAM_SOURCES:
//   SOURCED (URL)            - taken from a cited page
//   DATASHEET-FROM-MEMORY    - recalled from a datasheet, VERIFY before trusting
//   DERIVED                  - first-principles algebra (see the file that uses it)
//   ASSUMPTION               - a guess; calibrate with fit.js on real logs

/** rpm -> rad/s multiplier. */
export const RPM_TO_RADS = (2 * Math.PI) / 60;
/** rad/s -> rpm multiplier. */
export const RADS_TO_RPM = 60 / (2 * Math.PI);
/** inches -> metres multiplier. */
export const IN_TO_M = 0.0254;

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}

function isPlain(o) {
  return o !== null && typeof o === 'object' && !Array.isArray(o);
}

function clone(o) {
  if (Array.isArray(o)) return o.map(clone);
  if (isPlain(o)) {
    const r = {};
    for (const k of Object.keys(o)) r[k] = clone(o[k]);
    return r;
  }
  return o;
}

/**
 * Default flywheel parameters (deep-frozen). Units are in the key names.
 * @type {Readonly<object>}
 */
export const FLYWHEEL_PARAMS = deepFreeze({
  motor: {
    name: 'goBILDA 5203 Yellow Jacket 1:1 6000rpm',
    nominalV: 12.0,          // V
    stallTorqueNm: 0.144,    // N*m per motor
    stallCurrentA: 11.2,     // A per motor
    freeCurrentA: 0.25,      // A per motor
    freeSpeedRpm: 6000,      // rpm at nominalV
    rotorInertiaKgM2: 2.0e-6, // kg*m^2 per motor rotor
    count: 2,
  },
  drive: { gearRatio: 1.0, efficiency: 1.0 }, // motor rev per wheel rev (>1 = reduction); dimensionless
  battery: { voltageV: 12.0, internalResOhm: 0.0 },
  wheel: { radiusM: 1.89 * IN_TO_M }, // 0.048006 m
  inertia: {
    components: [
      { name: '96 mm compliant/traction wheel', kind: 'annulus', massKg: 0.060, rOuterM: 0.048, rInnerM: 0.020, source: 'ASSUMPTION' },
      { name: 'goBILDA steel flywheel 82 mm 3628-0032-0082', kind: 'given', jKgM2: 1.651e-4, massKg: 0.152, source: 'SOURCED gobilda.com' },
      { name: '8 mm shaft + hubs', kind: 'disk', massKg: 0.040, rOuterM: 0.008, source: 'ASSUMPTION' },
    ],
    extraKgM2: 0,
  },
  losses: {
    motorFrictionFromFreeCurrent: true,
    bearingCoulombNm: 0.002,
    viscousNmsPerRad: 2.0e-6,
    windage: { enabled: true, airDensityKgM3: 1.2, kinematicViscosityM2s: 1.5e-5, cmLaminarCoeff: 3.87 },
    zeroPowerBehavior: 'float', // 'float' | 'brake'
  },
  controller: {
    periodS: 0.02,
    kPVoltsPerRadS: 0.10,
    kIVoltsPerRad: 0.0,
    kFVoltsPerRadS: null, // null = derive
    kSVolts: null,        // null = derive
    allowReverse: false,
    stopMode: 'coast',    // 'coast' | 'active'
    measFilterTauS: 0.02,
  },
  shot: {
    exitEta: 0.45,
    spinRatio: 1.0,
    exitSpeedRef: 'mid', // 'mid' | 'end' | 'start'
    contactPathM: 0.08,
    minContactS: 0.004,
    maxContactS: 0.05,
    defaultBall: 'pollen',
    balls: {
      pollen: { massKg: 0.025, radiusM: 1.391 * IN_TO_M, shellK: 2 / 3, etaScale: 1 },
      nectar: { massKg: 0.041, radiusM: 1.7995 * IN_TO_M, shellK: 2 / 3, etaScale: Math.sqrt(0.025 / 0.041) },
    },
  },
  integrator: { maxSubstepS: 0.0005 },
  limits: { maxRpm: 6000 },
});

const DS = 'DATASHEET-FROM-MEMORY (goBILDA 5203-2402-0001 page, verify): ';

/**
 * Source tag for every leaf, keyed by dotted path.
 * @type {Readonly<Record<string,string>>}
 */
export const PARAM_SOURCES = deepFreeze({
  'motor.name': 'ASSUMPTION: motor choice; user hardware is a 2-motor flywheel',
  'motor.nominalV': DS + '12 V nominal',
  'motor.stallTorqueNm': DS + '1.47 kg*cm = 0.144 N*m',
  'motor.stallCurrentA': DS + '11.2 A',
  'motor.freeCurrentA': DS + '0.25 A',
  'motor.freeSpeedRpm': DS + '6000 rpm at 1:1',
  'motor.rotorInertiaKgM2': 'ASSUMPTION: RS-555 class rotor, 2e-6 kg*m^2',
  'motor.count': 'SOURCED (user hardware note): flywheel has 2 motors',
  'drive.gearRatio': 'ASSUMPTION: direct drive (1.0)',
  'drive.efficiency': 'ASSUMPTION: lossless drive (1.0)',
  'battery.voltageV': 'ASSUMPTION: 12.0 V keeps datasheet free speed exact',
  'battery.internalResOhm': 'ASSUMPTION: 0 (ideal); about 0.02 ohm to study sag',
  'wheel.radiusM': 'SOURCED (robot_spec.json shooter.flywheelRadius, prov vault): 1.89 in * 0.0254',
  'inertia.components': 'MIXED, see per-component source field. 82 mm flywheel SOURCED https://www.gobilda.com/steel-flywheel-82mm-diameter-152g-1651-g-cm/ (152 g, 1651 g*cm^2); 60 mm variant https://www.gobilda.com/flywheels/ (115 g, 551 g*cm^2); other parts ASSUMPTION. robotmodel.js mentions 208 g weights: alternative to confirm with user.',
  'inertia.extraKgM2': 'ASSUMPTION: no extra inertia',
  'losses.motorFrictionFromFreeCurrent': 'DERIVED: motor friction torque = Kt*I_free (makes loaded free speed exact)',
  'losses.bearingCoulombNm': 'ASSUMPTION: 0.002 N*m bearing drag',
  'losses.viscousNmsPerRad': 'ASSUMPTION: 2e-6 N*m*s/rad',
  'losses.windage.enabled': 'ASSUMPTION: windage on',
  'losses.windage.airDensityKgM3': 'ASSUMPTION: standard air 1.2 kg/m^3',
  'losses.windage.kinematicViscosityM2s': 'ASSUMPTION: air at about 20 C, 1.5e-5 m^2/s',
  'losses.windage.cmLaminarCoeff': 'UNVERIFIED (from memory): von Karman laminar disk torque coefficient, both faces, 3.87',
  'losses.zeroPowerBehavior': 'ASSUMPTION: FTC FLOAT behaviour on zero power',
  'controller.periodS': 'ASSUMPTION: 20 ms FTC loop',
  'controller.kPVoltsPerRadS': 'ASSUMPTION: P gain 0.10 V per rad/s',
  'controller.kIVoltsPerRad': 'ASSUMPTION: no integral term',
  'controller.kFVoltsPerRadS': 'DERIVED: null means kF = G*Ke/eta_d from the motor model',
  'controller.kSVolts': 'DERIVED: null means kS = T_c*R/(count*Kt*G*eta_d); V = kS*sgn(v)+kV*v+kA*a per WPILib SysId https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html',
  'controller.allowReverse': 'ASSUMPTION: flywheel never driven backwards',
  'controller.stopMode': 'ASSUMPTION: target 0 means coast',
  'controller.measFilterTauS': 'ASSUMPTION: encoder velocity smoothing 20 ms',
  'shot.exitEta': 'SOURCED (app.js SHOOTER.eff and robot_spec efficiency, prov repo): 0.45',
  'shot.spinRatio': 'DERIVED: ball rolls on the hood, w_b*r_b = v',
  'shot.exitSpeedRef': 'ASSUMPTION: mid-contact wheel speed sets exit speed',
  'shot.contactPathM': 'ASSUMPTION: 0.08 m of wheel/hood contact path',
  'shot.minContactS': 'ASSUMPTION: contact-time clamp',
  'shot.maxContactS': 'ASSUMPTION: contact-time clamp',
  'shot.defaultBall': 'ASSUMPTION: pollen',
  'shot.balls': 'Copied from web/js/balls.js BALL_KINDS (pollen 0.025 kg r=1.391 in; nectar 0.041 kg ASSUMED there, r=1.7995 in). shellK 2/3 thin hollow shell ASSUMPTION (solid sphere 0.4). etaScale = sqrt(m_ref/m) copies balls.js launchScaleFor.',
  'integrator.maxSubstepS': 'ASSUMPTION: 0.5 ms keeps semi-implicit Euler energy error under 0.5%',
  'limits.maxRpm': 'SOURCED (repo robot_spec): 6000; user keeps the flywheel at or under 5000 rpm',
});

/**
 * Deep-merge overrides onto a deep clone of the defaults. Plain objects merge; arrays and
 * individual entries of shot.balls are replaced whole. Returns an unfrozen object and is
 * idempotent when given a full params object.
 * @param {object} [overrides]
 * @returns {object}
 */
export function makeFlywheelParams(overrides) {
  const out = clone(FLYWHEEL_PARAMS);
  const merge = (dst, src, path) => {
    if (!isPlain(src)) return;
    for (const k of Object.keys(src)) {
      const v = src[k];
      if (v === undefined) continue;
      const p = path ? path + '.' + k : k;
      if (p === 'shot.balls' && isPlain(v)) {
        if (!isPlain(dst[k])) dst[k] = {};
        for (const b of Object.keys(v)) dst[k][b] = clone(v[b]);
      } else if (isPlain(v) && isPlain(dst[k])) {
        merge(dst[k], v, p);
      } else {
        dst[k] = clone(v);
      }
    }
  };
  merge(out, overrides, '');
  return out;
}

const posFin = (x) => typeof x === 'number' && Number.isFinite(x) && x > 0;
const nonNeg = (x) => typeof x === 'number' && Number.isFinite(x) && x >= 0;

// Local copy of the inertia formulas so validation needs no import cycle with inertia.js.
function componentJ(c) {
  if (!c || typeof c !== 'object') return NaN;
  const m = c.massKg, ro = c.rOuterM, ri = c.rInnerM;
  switch (c.kind) {
    case 'disk': return 0.5 * m * ro * ro;
    case 'ring': return m * ro * ro;
    case 'annulus': return 0.5 * m * (ro * ro + ri * ri);
    case 'given': return c.jKgM2;
    default: return NaN;
  }
}

/**
 * Validate parameters.
 * @param {object} p
 * @returns {string[]} human-readable errors (empty when valid)
 */
export function validateFlywheelParams(p) {
  const e = [];
  if (!isPlain(p)) return ['params must be an object'];
  const m = p.motor || {}, d = p.drive || {}, w = p.wheel || {}, c = p.controller || {};
  const inr = p.inertia || {}, sh = p.shot || {}, ig = p.integrator || {}, l = p.losses || {}, b = p.battery || {};
  for (const k of ['nominalV', 'stallTorqueNm', 'stallCurrentA', 'freeSpeedRpm', 'rotorInertiaKgM2']) {
    if (!posFin(m[k])) e.push(`motor.${k} must be finite and > 0`);
  }
  if (!nonNeg(m.freeCurrentA)) e.push('motor.freeCurrentA must be finite and >= 0');
  if (!Number.isInteger(m.count) || m.count < 1) e.push('motor.count must be an integer >= 1');
  if (posFin(m.nominalV) && posFin(m.stallCurrentA) && nonNeg(m.freeCurrentA)) {
    if (m.freeCurrentA * (m.nominalV / m.stallCurrentA) >= m.nominalV) e.push('motor.freeCurrentA*R must be < nominalV');
  }
  if (!posFin(d.gearRatio)) e.push('drive.gearRatio must be finite and > 0');
  if (!posFin(d.efficiency) || d.efficiency > 1) e.push('drive.efficiency must be in (0, 1]');
  if (!posFin(b.voltageV)) e.push('battery.voltageV must be finite and > 0');
  if (!nonNeg(b.internalResOhm)) e.push('battery.internalResOhm must be finite and >= 0');
  if (!posFin(w.radiusM)) e.push('wheel.radiusM must be finite and > 0');
  if (!Array.isArray(inr.components)) e.push('inertia.components must be an array');
  else {
    inr.components.forEach((cc, i) => {
      if (!posFin(componentJ(cc))) e.push(`inertia.components[${i}] (${cc && cc.name}) must give finite positive J`);
    });
  }
  if (!nonNeg(inr.extraKgM2)) e.push('inertia.extraKgM2 must be finite and >= 0');
  if (!nonNeg(l.bearingCoulombNm)) e.push('losses.bearingCoulombNm must be finite and >= 0');
  if (!nonNeg(l.viscousNmsPerRad)) e.push('losses.viscousNmsPerRad must be finite and >= 0');
  if (l.zeroPowerBehavior !== 'float' && l.zeroPowerBehavior !== 'brake') e.push("losses.zeroPowerBehavior must be 'float' or 'brake'");
  if (!posFin(c.periodS)) e.push('controller.periodS must be finite and > 0');
  if (c.stopMode !== 'coast' && c.stopMode !== 'active') e.push("controller.stopMode must be 'coast' or 'active'");
  if (!nonNeg(c.measFilterTauS)) e.push('controller.measFilterTauS must be finite and >= 0');
  if (!posFin(ig.maxSubstepS)) e.push('integrator.maxSubstepS must be finite and > 0');
  if (!['mid', 'end', 'start'].includes(sh.exitSpeedRef)) e.push("shot.exitSpeedRef must be 'mid', 'end' or 'start'");
  if (!posFin(sh.exitEta)) e.push('shot.exitEta must be finite and > 0');
  if (!isPlain(sh.balls) || !sh.balls[sh.defaultBall]) e.push('shot.defaultBall must name an entry of shot.balls');
  if (!posFin((p.limits || {}).maxRpm)) e.push('limits.maxRpm must be finite and > 0');
  return e;
}

/**
 * Build makeFlywheelParams overrides from a robot_spec.json shooter object
 * (flywheelRadius in inches, efficiency, maxRpm) or the app.js SHOOTER shape (wheelR in inches, eff, maxRpm).
 * Missing or non-finite fields are ignored.
 * @param {object|null} sh
 * @returns {object}
 */
export function paramsFromRobotSpec(sh) {
  if (!sh || typeof sh !== 'object') return {};
  const num = (x) => (typeof x === 'number' && Number.isFinite(x) ? x : null);
  const r = num(sh.flywheelRadius) ?? num(sh.wheelR);
  const eff = num(sh.efficiency) ?? num(sh.eff);
  const rpm = num(sh.maxRpm);
  const o = {};
  if (r !== null && r > 0) o.wheel = { radiusM: r * IN_TO_M };
  if (eff !== null && eff > 0) o.shot = { exitEta: eff };
  if (rpm !== null && rpm > 0) o.limits = { maxRpm: rpm };
  return o;
}
