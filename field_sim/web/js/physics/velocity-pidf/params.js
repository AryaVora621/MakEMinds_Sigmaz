// params.js - every constant of the velocity-PIDF flywheel module. Pure ES module, no imports.
// Units are in each key's comment. PARAM_SOURCES tags every key path:
//   [REPO] read from this repo   [SRC] cited from a source   [DERIVED] computed from other values
//   [ASSUMED] engineering guess  [UNVERIFIED] recalled, not checked against a datasheet or firmware source
// Primary mode 'sdk' reproduces the OpMode-side FlywheelPIDF loop (RUN_WITHOUT_ENCODER). Mode 'hub' models an ASSUMED
// RUN_USING_ENCODER hub-side PIDF.

const DEFAULTS = {
  motor: {
    freeRpm: 6000,            // rpm, goBILDA 5203 1:1 free speed
    stallTorqueNm: 0.144,     // N*m (1.47 kg*cm)
    stallCurrentA: 9.2,       // A
    freeCurrentA: 0.25,       // A
    nominalV: 12,             // V
    cpr: 28,                  // counts per motor-shaft rev
    coulombFrac: 0.5,         // share of no-load friction that is Coulomb (rest viscous)
    coulombSmoothRadS: 1.0,   // rad/s, tanh smoothing
  },
  flywheel: {
    motorCount: 2,
    gearRatio: 1,             // motor rev per wheel rev
    inertiaKgM2: 5e-4,        // kg*m^2 total reflected to the wheel
    wheelRadiusM: 0.048,      // m
    exitSpeedFrac: 0.5,       // ball speed / wheel surface speed
    energyTransferEff: 0.6,
    ballSpinFactor: 0.4,
    ballContactS: 0.03,       // s
    fallbackBallMassKg: 0.025, // kg
  },
  battery: {
    openCircuitV: 12.8,       // V
    internalOhm: 0.05,        // ohm
    minV: 6,                  // V
  },
  settings: {
    ticksPerRev: 28,          // ticks/rev
    minRpm: 2500,             // rpm
    maxRpm: 5000,             // rpm
    atTargetTolTicks: 50,     // ticks/s
    velocityFromBoth: false,
    zeroPowerFloat: true,
    minDtS: 1e-3,             // s
  },
  sdkPidf: {
    kS: 0.10,                 // V
    kV: 0.0043,               // V/(tick/s)
    kA: 0.0005,               // V/(tick/s^2)
    kP: 0.005,                // V/(tick/s)
    kI: 0,
    kD: 0,
    iMaxVolts: 2.0,           // V
    nominalV: 12.0,           // V
  },
  hubPidf: {
    periodS: 0.05,            // s
    kP: 10,
    kI: 3,
    kD: 0,
    kF: 0,
    outputScale: 32767,
  },
  encoder: {
    velWindowS: 0.05,         // s
    historyS: 0.25,           // s
    int16Wrap: true,
    direction: 1,
  },
  timing: {
    physicsDtS: 0.0005,       // s
    maxFrameDtS: 0.25,        // s
    loopPeriodMeanS: 0.020,   // s
    loopJitterSdS: 0.004,     // s
    loopPeriodMinS: 0.008,    // s
    loopPeriodMaxS: 0.060,    // s
    bulkReadS: 0.003,         // s
    cmdLatencyS: 0.0015,      // s per setPower command
    powerLevels: 32767,
    seed: 12345,
  },
  ready: {
    mode: 'ticks',            // 'ticks' | 'frac'
    tolTicks: 50,             // ticks/s
    fracTol: 0.04,
    settleLoops: 1,
  },
  mode: 'sdk',                // 'sdk' | 'hub'
};

function deepCopy(o) {
  if (Array.isArray(o)) return o.map(deepCopy);
  if (o && typeof o === 'object') {
    const r = {};
    for (const k of Object.keys(o)) r[k] = deepCopy(o[k]);
    return r;
  }
  return o;
}

/** Fresh deep copy of the default parameters. */
export function defaultParams() { return deepCopy(DEFAULTS); }

/** defaultParams() with each group shallow-merged from overrides[group]; top-level scalars (mode) replace. Never mutates defaults. */
export function mergeParams(overrides = {}) {
  const p = defaultParams();
  if (!overrides || typeof overrides !== 'object') return p;
  for (const k of Object.keys(overrides)) {
    const v = overrides[k];
    if (v === undefined) continue;
    if (p[k] && typeof p[k] === 'object' && v && typeof v === 'object') p[k] = { ...p[k], ...deepCopy(v) };
    else p[k] = deepCopy(v);
  }
  return p;
}

const GM0 = 'https://gm0.org/en/latest/docs/software/concepts/control-loops.html';
const UNV5203 = '[UNVERIFIED] goBILDA 5203 1:1 6000rpm product-page recollection, datasheet not found';
const FLY_J = 'TeamCode/src/main/java/robot/RobotSettings.java FlywheelPIDF (lines 304-328)';
const sources = {
  'motor.freeRpm': UNV5203,
  'motor.stallTorqueNm': UNV5203 + ' (1.47 kg*cm)',
  'motor.stallCurrentA': UNV5203,
  'motor.freeCurrentA': UNV5203,
  'motor.nominalV': '[UNVERIFIED] nominal 12 V rating of the 5203 motor',
  'motor.cpr': '[REPO] RobotSettings.Flywheel.TICKS_PER_REV=28 (motor shaft, gearing included)',
  'motor.coulombFrac': '[ASSUMED] half of no-load friction is Coulomb',
  'motor.coulombSmoothRadS': '[ASSUMED] tanh smoothing radius',
  'flywheel.motorCount': '[REPO] Flywheel.java drives 2 motors',
  'flywheel.gearRatio': '[REPO] TICKS_PER_REV=28 already includes any gearing, so 1',
  'flywheel.inertiaKgM2': '[ASSUMED] PLACEHOLDER, gives open-loop mechanical tau of about 1.1 s',
  'flywheel.wheelRadiusM': '[ASSUMED] 96 mm wheel',
  'flywheel.exitSpeedFrac': '[ASSUMED] ball speed / wheel surface speed; used only when caller gives no exit speed',
  'flywheel.energyTransferEff': '[ASSUMED] wheel-to-ball energy transfer efficiency',
  'flywheel.ballSpinFactor': '[ASSUMED] 2/5 solid sphere with v = omega*r',
  'flywheel.ballContactS': '[ASSUMED] ball contact duration',
  'flywheel.fallbackBallMassKg': '[REPO] web/js/balls.js BALL_KINDS.pollen.massKg=0.025',
  'battery.openCircuitV': '[ASSUMED] fresh 12 V battery open-circuit voltage',
  'battery.internalOhm': '[ASSUMED] battery plus wiring internal resistance',
  'battery.minV': '[ASSUMED] terminal voltage floor (brown-out region)',
  'settings.ticksPerRev': '[REPO] RobotSettings.Flywheel.TICKS_PER_REV',
  'settings.minRpm': '[REPO] RobotSettings.Flywheel.MIN_RPM',
  'settings.maxRpm': '[REPO] Flywheel.HARD_MAX_RPM',
  'settings.atTargetTolTicks': '[REPO] RobotSettings.Flywheel.AT_TARGET_TOLERANCE',
  'settings.velocityFromBoth': '[REPO] RobotSettings.Flywheel velocity-from-both flag (false)',
  'settings.zeroPowerFloat': '[REPO] RobotSettings.Flywheel zero-power behaviour FLOAT',
  'settings.minDtS': '[REPO] Flywheel.update Math.max(timer.seconds(), 1e-3)',
  'sdkPidf.kS': '[REPO] ' + FLY_J,
  'sdkPidf.kV': '[REPO] ' + FLY_J + '; [DERIVED] sanity: Ke*2pi/28 = 0.00417 V/(tick/s)',
  'sdkPidf.kA': '[REPO] ' + FLY_J,
  'sdkPidf.kP': '[REPO] ' + FLY_J,
  'sdkPidf.kI': '[REPO] ' + FLY_J,
  'sdkPidf.kD': '[REPO] ' + FLY_J,
  'sdkPidf.iMaxVolts': '[REPO] ' + FLY_J,
  'sdkPidf.nominalV': '[REPO] ' + FLY_J,
  'hubPidf.periodS': '[SRC] ' + GM0 + ' "built-in PID locked at 20 Hz"',
  'hubPidf.kP': '[UNVERIFIED] SDK default recollection',
  'hubPidf.kI': '[UNVERIFIED] SDK default recollection',
  'hubPidf.kD': '[UNVERIFIED] SDK default recollection',
  'hubPidf.kF': '[UNVERIFIED] SDK default recollection; helper rule kF=32767/maxTicksPerS',
  'hubPidf.outputScale': '[ASSUMED] int16 power scale',
  'encoder.velWindowS': '[ASSUMED] hub velocity window not found; gives 20 ticks/s quantum',
  'encoder.historyS': '[ASSUMED] count-history retention',
  'encoder.int16Wrap': '[SRC] FTCLib Encoder.getCorrectedVelocity unwraps by CPS_STEP=0x10000, implying signed 16-bit ticks/s',
  'encoder.direction': '[ASSUMED] +1 encoder direction',
  'timing.physicsDtS': '[ASSUMED] integration step',
  'timing.maxFrameDtS': '[ASSUMED] frame dt clamp',
  'timing.loopPeriodMeanS': '[ASSUMED] mean OpMode loop period; [SRC] ' + GM0 + ' best teams reach 80 Hz = 12.5 ms',
  'timing.loopJitterSdS': '[ASSUMED] loop period jitter standard deviation',
  'timing.loopPeriodMinS': '[ASSUMED] loop period lower clamp',
  'timing.loopPeriodMaxS': '[ASSUMED] loop period upper clamp',
  'timing.bulkReadS': '[ASSUMED] bulk-read transaction time',
  'timing.cmdLatencyS': '[ASSUMED] per setPower LynxCommand, serial',
  'timing.powerLevels': '[ASSUMED] power quantisation levels',
  'timing.seed': '[ASSUMED] deterministic RNG seed',
  'ready.mode': '[REPO] ticks = Java atTarget(); frac = web/js/app.js line 684',
  'ready.tolTicks': '[REPO] RobotSettings.Flywheel.AT_TARGET_TOLERANCE',
  'ready.fracTol': '[REPO] web/js/app.js line 684',
  'ready.settleLoops': '[ASSUMED] 1 = Java atTarget() single-loop check',
  'mode': '[ASSUMED] sdk = Flywheel.java RUN_WITHOUT_ENCODER loop [REPO]; hub = assumed RUN_USING_ENCODER',
};
export const PARAM_SOURCES = Object.freeze(sources);

/** rpm -> encoder ticks per second. */
export function rpmToTicksPerS(rpm, ticksPerRev) { return rpm * ticksPerRev / 60; }
/** encoder ticks per second -> rpm. */
export function ticksPerSToRpm(ticksPerS, ticksPerRev) { return ticksPerS * 60 / ticksPerRev; }

