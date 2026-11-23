// Shooter-exit parameters: the single home for every tunable constant of the shooter-exit physics area.
// Units are in the key names or the comments. Every leaf has an entry in PARAM_SOURCES (same dotted path) saying where
// it came from, whether it is an ASSUMPTION, and how to measure it on the robot.
// Sources actually found: recalc ballShot.ts (energy-transfer maths), goBILDA 2000-0025-0002 servo page, hardware memo.

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}
const clone = (o) => (o && typeof o === 'object' ? Object.fromEntries(Object.entries(o).map(([k, v]) => [k, clone(v)])) : o);

export const PARAMS = deepFreeze({
  model: {
    DEFAULT_EXIT_MODEL: 'legacy',   // 'legacy' | 'physical'
    LEGACY_EFF: 0.45,               // unitless wheel-surface -> ball speed efficiency (app.js value)
    CALIBRATE_TO_LEGACY: true,
    CALIBRATION_KIND: 'pollen',
    ETA_SCALE_MANUAL: 1.0,          // unitless, used when CALIBRATE_TO_LEGACY is false
  },
  wheel: {
    CONFIG: 'hooded',               // 'hooded' | 'dual'
    R_IN: 1.89,                     // in, wheel 1 radius
    R2_IN: 1.89,                    // in, wheel 2 radius (dual only)
    SPEED_RATIO_2: 1.0,             // omega2 / omega1 (dual only)
    MOI_KGM2: 3.46e-4,              // kg*m^2, wheel 1 + flywheel rotating inertia
    MOI2_KGM2: 3.46e-4,             // kg*m^2, wheel 2 (dual only)
    RPM_SCALE: 1.0,                 // unitless, sim rpm / physical rpm
  },
  ball: {
    MOI_FACTOR: 0.4,                // unitless, solid sphere iB = 0.4*m*r^2
    DIAM_SIGMA_IN: 0.02,            // in, 1-sigma ball diameter variation
  },
  compression: {
    GAP_IN: { pollen: 2.2256, nectar: 2.8792 },   // in, gap between wheel and hood at the ball (c = 0.20 each)
    C0: 0.08,                       // unitless compression scale of the grip curve
    C_OPT: 0.25,                    // unitless compression above which the efficiency falls again
    K_OVER: 2.0,                    // unitless over-compression penalty
  },
  slip: {
    V_SLIP_INS: Infinity,           // in/s wheel-surface speed where slip efficiency is 1/sqrt(2); Infinity = no slip loss
  },
  hood: {
    ELEV_OFFSET_DEG: 0,             // deg
    ELEV_GAIN: 1,                   // deg elevation per deg hood
    THROWDOWN_DEG_PER_UNIT_C: 0,    // deg of elevation lost per unit compression
    WRAP_DEG: 90,                   // deg of wheel arc in contact with the ball
    LINK_RATIO: 1,                  // servo deg per hood deg
  },
  geom: {
    ENABLED_PHYSICAL: true,
    AXLE_FWD_IN: 0,                 // in, wheel axle forward of the turret pivot
    AXLE_UP_IN: null,               // in; null = auto so that exit height offset is 0 at 45 deg for pollen
    Z_REF_IN: 0,                    // in, height reference above sh.exitHeight
  },
  timing: {
    FEED_DELAY_S: 0.06,             // s, fire request -> ball released
    CMD_LATENCY_S: 0.02,            // s, command -> servo reacts
  },
  dispersion: {
    SIGMA_SPEED_FRAC: 0.015,        // unitless 1-sigma exit-speed fraction
    SIGMA_SPEED_FRAC_LOWCOMP: 0.03, // unitless extra speed sigma at zero compression
    SIGMA_ELEV_DEG: 0.5,            // deg
    SIGMA_AZ_DEG: 0.5,              // deg
    SIGMA_FEED_S: 0.005,            // s release-time jitter
    MC_SAMPLES: 400,                // count
    SEED: 12345,                    // integer rng seed
  },
  turretServo: {
    MAX_RATE_DEG_S: 450,
    MAX_ACCEL_DEG_S2: 3000,
    DEADBAND_DEG: 0.5,
    BACKLASH_DEG: 0.5,
    LATENCY_S: 0.02,
    QUANT_DEG: 0,
    SETTLE_TOL_DEG: 1.0,
    SETTLE_VEL_DEG_S: 20,
    SETTLE_TIME_S: 0.05,
  },
  hoodServo: {
    NOLOAD_RATE_DEG_S: 300,         // deg/s at 6 V
    LOAD_FACTOR: 0.7,               // unitless
    MAX_ACCEL_DEG_S2: 5000,
    DEADBAND_US: 4,                 // us
    DEG_PER_US: 0.15,               // deg/us (300 deg over 2000 us)
    LATENCY_S: 0.02,
    BACKLASH_DEG: 0.3,
    SETTLE_TOL_DEG: 0.75,
    SETTLE_VEL_DEG_S: 10,
    SETTLE_TIME_S: 0.05,
  },
  flywheel: {
    TAU_S: 0.35,                    // s, closed-loop first-order time constant
    RECOVER_TAU_S: 0.04,            // s, time constant right after a shot
    RPM_DIP_FRAC_LEGACY: 0.06,      // unitless rpm drop per shot, legacy mode
    MAX_RPM_HW: 5000,               // rpm, hardware memo (informational)
  },
});

const RECALC = 'tervay/recalc app/lib/math/ballShot.ts https://raw.githubusercontent.com/tervay/recalc/main/app/lib/math/ballShot.ts (medium confidence)';
const GOBILDA = 'goBILDA 2000-0025-0002 page https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque/';
const MEMO = 'user hardware memo 2026-11-21';
const A = (why) => ({ source: 'ASSUMPTION: ' + why, assumed: true });
const S = (source) => ({ source, assumed: false });
const E = (o, measure) => ({ ...o, measure });

export const PARAM_SOURCES = {
  'model.DEFAULT_EXIT_MODEL': E(S('design decision: legacy stays default so old tests stay bit-identical'), 'n/a (switch)'),
  'model.LEGACY_EFF': E(S('current app.js value'), 'n/a (legacy compatibility)'),
  'model.CALIBRATE_TO_LEGACY': E(S('design decision: pollen physical == legacy'), 'n/a (switch)'),
  'model.CALIBRATION_KIND': E(S('design decision'), 'n/a (switch)'),
  'model.ETA_SCALE_MANUAL': E(A('only used with CALIBRATE_TO_LEGACY=false'), 'fit so measured exit speed = model at one rpm'),
  'wheel.CONFIG': E(S(MEMO + ': single hooded flywheel'), 'inspect the robot'),
  'wheel.R_IN': E(A('goBILDA-class 96 mm wheel (1.89 in)'), 'caliper the wheel diameter'),
  'wheel.R2_IN': E(A('same as wheel 1; dual only'), 'caliper'),
  'wheel.SPEED_RATIO_2': E(A('dual only'), 'tach both wheels'),
  'wheel.MOI_KGM2': E(A('0.5*0.30 kg*(0.048 m)^2, flywheel plus wheels'), 'weigh the rotating parts, or spin-down test with known torque'),
  'wheel.MOI2_KGM2': E(A('same as wheel 1; dual only'), 'weigh / spin-down test'),
  'wheel.RPM_SCALE': E(A('sim scaling, 1 = real rpm'), 'compare encoder rpm to a tachometer'),
  'ball.MOI_FACTOR': E(S(RECALC + ': iB = 0.4*m*r^2'), 'n/a (hollow ball would be ~0.667)'),
  'ball.DIAM_SIGMA_IN': E(A('ball diameter tolerance'), 'caliper 20 balls and take the std dev'),
  'compression.GAP_IN.pollen': E(A('c = 0.20'), 'measure wheel-to-hood gap with a pollen in place'),
  'compression.GAP_IN.nectar': E(A('c = 0.20'), 'measure wheel-to-hood gap with a nectar in place'),
  'compression.C0': E(A('grip curve scale'), 'fit exit speed vs gap on a chronograph or video'),
  'compression.C_OPT': E(A('best compression'), 'sweep the gap and find the peak exit speed'),
  'compression.K_OVER': E(A('over-compression penalty'), 'sweep the gap past the peak'),
  'slip.V_SLIP_INS': E(A('Infinity = no slip loss'), 'exit speed vs rpm at high rpm; fit the roll-off'),
  'hood.ELEV_OFFSET_DEG': E(A('identity hood map'), 'measure launch angle at two hood positions with a protractor/video'),
  'hood.ELEV_GAIN': E(A('identity hood map'), 'same'),
  'hood.THROWDOWN_DEG_PER_UNIT_C': E(A('no compression throw-down'), 'compare launch angle for pollen vs nectar at the same hood'),
  'hood.WRAP_DEG': E(A('arc of contact'), 'high-speed video of the ball on the wheel'),
  'hood.LINK_RATIO': E(A('direct link'), 'measure servo deg per hood deg'),
  'geom.ENABLED_PHYSICAL': E(S('design decision'), 'n/a (switch)'),
  'geom.AXLE_FWD_IN': E(A('axle above the pivot'), 'read from CAD'),
  'geom.AXLE_UP_IN': E(A('null = auto, exit offset 0 at 45 deg'), 'read from CAD'),
  'geom.Z_REF_IN': E(A('reference height = sh.exitHeight'), 'read from CAD'),
  'timing.FEED_DELAY_S': E(A('feeder delay'), 'high-speed video or encoder timestamps'),
  'timing.CMD_LATENCY_S': E(A('one 20 ms servo PWM frame plus loop'), 'high-speed video of command to motion'),
  'dispersion.SIGMA_SPEED_FRAC': E(A('shot-to-shot exit speed spread'), 'chronograph 30 shots, std dev / mean'),
  'dispersion.SIGMA_SPEED_FRAC_LOWCOMP': E(A('extra speed spread at low compression'), 'repeat at a wider gap'),
  'dispersion.SIGMA_ELEV_DEG': E(A('elevation spread'), 'video of 30 shots'),
  'dispersion.SIGMA_AZ_DEG': E(A('azimuth spread'), 'mark landing points of 30 shots'),
  'dispersion.SIGMA_FEED_S': E(A('release-time jitter'), 'video timing of feed'),
  'dispersion.MC_SAMPLES': E(S('design decision'), 'n/a'),
  'dispersion.SEED': E(S('design decision'), 'n/a'),
  'turretServo.MAX_RATE_DEG_S': E(A('Axon specs not found'), 'step-response video or telemetry'),
  'turretServo.MAX_ACCEL_DEG_S2': E(A('Axon specs not found'), 'step-response'),
  'turretServo.DEADBAND_DEG': E(A('Axon specs not found'), 'creep command by 0.1 deg and watch for motion'),
  'turretServo.BACKLASH_DEG': E(A('gear train slop'), 'dial indicator at the turret edge, reverse direction'),
  'turretServo.LATENCY_S': E(A('one 20 ms PWM frame plus loop'), 'video'),
  'turretServo.QUANT_DEG': E(A('0 = off'), 'smallest commandable step'),
  'turretServo.SETTLE_TOL_DEG': E(A('settled criterion'), 'n/a'),
  'turretServo.SETTLE_VEL_DEG_S': E(A('settled criterion'), 'n/a'),
  'turretServo.SETTLE_TIME_S': E(A('settled criterion'), 'n/a'),
  'hoodServo.NOLOAD_RATE_DEG_S': E(S(GOBILDA + ': 0.20 s/60 deg at 6 V'), 'time a full sweep unloaded'),
  'hoodServo.LOAD_FACTOR': E(A('loaded rate is 0.7x no-load'), 'time a sweep with the carriage attached'),
  'hoodServo.MAX_ACCEL_DEG_S2': E(A('servo acceleration'), 'step-response video'),
  'hoodServo.DEADBAND_US': E(S(GOBILDA + ': 4 us'), 'n/a'),
  'hoodServo.DEG_PER_US': E(S(GOBILDA + ': 500-2500 us over 300 deg'), 'n/a'),
  'hoodServo.LATENCY_S': E(A('one 20 ms PWM frame'), 'video'),
  'hoodServo.BACKLASH_DEG': E(A('linkage slop'), 'dial indicator'),
  'hoodServo.SETTLE_TOL_DEG': E(A('settled criterion'), 'n/a'),
  'hoodServo.SETTLE_VEL_DEG_S': E(A('settled criterion'), 'n/a'),
  'hoodServo.SETTLE_TIME_S': E(A('settled criterion'), 'n/a'),
  'flywheel.TAU_S': E(A('legacy value'), 'log rpm after a setpoint step; fit 63% time'),
  'flywheel.RECOVER_TAU_S': E(A('legacy value'), 'log rpm recovery after a shot'),
  'flywheel.RPM_DIP_FRAC_LEGACY': E(A('legacy value'), 'log rpm just before and after a shot'),
  'flywheel.MAX_RPM_HW': E(S(MEMO + ': flywheel kept at or under 5000 rpm'), 'n/a'),
};
deepFreeze(PARAM_SOURCES);

function merge(base, over) {
  const out = clone(base);
  if (over && typeof over === 'object') {
    for (const k of Object.keys(over)) {
      const v = over[k];
      if (v && typeof v === 'object' && !Array.isArray(v) && out[k] && typeof out[k] === 'object') out[k] = merge(out[k], v);
      else out[k] = v;
    }
  }
  return out;
}

/** New deep-frozen copy of PARAMS with the nested `overrides` applied. */
export function mergeParams(overrides) {
  return deepFreeze(merge(PARAMS, overrides || {}));
}
