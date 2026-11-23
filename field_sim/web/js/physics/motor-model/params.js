// params.js - single source of every tunable constant in the motor-model area.
// Internal units are SI (m, kg, s, rad, N*m, A, V, degC). Each value is tagged:
//   [SOURCED url] = taken from a published datasheet/page, [ASSUMPTION] = engineering estimate, not found in any source.
// All objects are deep-frozen. Edit values here only.

function deepFreeze(o) {
  if (o && typeof o === 'object' && !Object.isFrozen(o)) {
    Object.freeze(o);
    for (const k of Object.keys(o)) deepFreeze(o[k]);
  }
  return o;
}

// ---- unit constants ----
export const IN_PER_M = 39.37007874;   // inch per metre [exact conversion]
export const G_ACCEL = 9.80665;        // m/s^2 [standard gravity, exact]
export const KGCM_TO_NM = 0.0980665;   // N*m per kg-cm [exact: 1 kgf = 9.80665 N, 1 cm = 0.01 m]

// ---- bare motors ----
export const MOTORS = deepFreeze({
  gobilda5203: {
    id: 'gobilda5203',
    nominalV: 12,                  // V [SOURCED goBILDA 5203 product pages]
    stallTorqueNm: 0.144158,       // N*m = 1.47 kg-cm * KGCM_TO_NM [SOURCED goBILDA 5203 product page]
    stallCurrentA: 9.2,            // A [SOURCED goBILDA 5203 product page]
    freeCurrentA: 0.25,            // A [SOURCED goBILDA 5203 product page]
    freeSpeedRpm: 6000,            // rpm [SOURCED goBILDA 5203 product page]
    rotorJ: 5.0e-6,                // kg*m^2 [ASSUMPTION: solid cylinder, ~60 g armature, r ~12.5 mm; no source]
    cprMotor: 28,                  // counts/rev at the motor [SOURCED goBILDA 5203 product page]
    source: 'goBILDA 5203 RS-555 12 V bare motor, goBILDA product pages; rotorJ is an assumption',
  },
  revHdHex: {
    id: 'revHdHex',
    nominalV: 12,                  // V [SOURCED docs.revrobotics.com hd-hex-motor]
    stallTorqueNm: 0.105,          // N*m [SOURCED docs.revrobotics.com hd-hex-motor]
    stallCurrentA: 8.5,            // A [SOURCED docs.revrobotics.com hd-hex-motor]
    freeCurrentA: 0.4,             // A [SOURCED docs.revrobotics.com hd-hex-motor]
    freeSpeedRpm: 6000,            // rpm [SOURCED docs.revrobotics.com hd-hex-motor]
    rotorJ: 5.0e-6,                // kg*m^2 [ASSUMPTION: same as 5203]
    cprMotor: 28,                  // counts/rev [SOURCED docs.revrobotics.com hd-hex-motor]
    source: 'REV HD Hex bare motor, published values (docs.revrobotics.com hd-hex-motor); rotorJ assumed',
  },
  revHdHexDyno: {
    id: 'revHdHexDyno',
    nominalV: 12,                  // V
    stallTorqueNm: 0.18,           // N*m [SOURCED dyno data: reca.lc/motors, gm0]
    stallCurrentA: 11,             // A [SOURCED dyno data: reca.lc/motors, gm0]
    freeCurrentA: 0.4,             // A [SOURCED dyno data: reca.lc/motors, gm0]
    freeSpeedRpm: 5960,            // rpm [SOURCED dyno data: reca.lc/motors, gm0]
    rotorJ: 5.0e-6,                // kg*m^2 [ASSUMPTION]
    cprMotor: 28,                  // counts/rev [SOURCED REV]
    source: 'REV HD Hex bare motor, dyno alternative (reca.lc/motors, gm0); rotorJ assumed',
  },
  revCoreHex: {
    id: 'revCoreHex',
    nominalV: 12,                  // V
    stallTorqueNm: 0.044444,       // N*m = 3.2 N*m output / 72:1 [SOURCED REV Core Hex pages; output torque already includes gearbox losses, so eta = 1 on coreHex72]
    stallCurrentA: 4.4,            // A [SOURCED REV Core Hex pages]
    freeCurrentA: 0.2,             // A [SOURCED gm0 (third-party)]
    freeSpeedRpm: 9000,            // rpm at motor = 125 rpm output * 72 [derived from REV output 125 rpm, 72:1]
    rotorJ: 2.0e-6,                // kg*m^2 [ASSUMPTION: smaller than the 555 can]
    cprMotor: 4,                   // counts/rev at the motor [SOURCED REV Core Hex pages]
    source: 'REV Core Hex modelled as bare motor behind 72:1 (coreHex72, eta 1); free current from gm0',
  },
});

// ---- gearboxes ----
export const GEARBOXES = deepFreeze({
  // goBILDA exact ratios from (1+46/17), (1+46/11); reproduce published PPR 145.1/384.5/537.7 to 0.1 [SOURCED goBILDA]
  none:   { ratio: 1, stages: 0 },
  gb5_2:  { ratio: 57 / 11, stages: 1 },                 // 5.1818 [SOURCED goBILDA formula]
  gb13_7: { ratio: (63 / 17) ** 2, stages: 2 },          // 13.7336 [SOURCED goBILDA formula]
  gb19_2: { ratio: (63 / 17) * (57 / 11), stages: 2 },   // 19.2032 [SOURCED goBILDA formula]
  // REV UltraPlanetary actual ratios [SOURCED docs.revrobotics.com ultraplanetary]
  ultra3:  { ratio: 84 / 29, stages: 1 },
  ultra4:  { ratio: 76 / 21, stages: 1 },
  ultra5:  { ratio: 68 / 13, stages: 1 },
  ultra20: { ratio: (76 / 21) * (68 / 13), stages: 2 },
  // REV Core Hex 72:1; efficiency 1 because the published output torque already includes losses [SOURCED REV]
  coreHex72: { ratio: 72, stages: 0, efficiency: 1 },
});

// Per-stage gearbox efficiency; eta = 0.93^stages.
// [ASSUMPTION: no manufacturer value. Consistent with goBILDA 19.2:1 page (0.861 vs 0.865).]
export const GEARBOX_STAGE_EFF = 0.93;

// ---- REV hub / FTC SDK ----
export const HUB = deepFreeze({
  pwmCounts: 32767,          // counts for |power|=1; trunc(p*32767) [SOURCED FTC SDK / REV expansion-hub-basics]
  portContinuousA: 10,       // A [SOURCED REV expansion-hub-basics]
  portAbsMaxA: 20,           // A [SOURCED REV expansion-hub-basics]
  batteryFuseA: 20,          // A [SOURCED REV expansion-hub-basics]
  defaultZeroPower: 'BRAKE', // [SOURCED FTC SDK default for DcMotor]
});

export const BATTERY = deepFreeze({
  openCircuitV: 12.0,          // V [ASSUMPTION: matches datasheet nominal]
  internalResistanceOhm: 0.0,  // ohm [ASSUMPTION: 0 = sag off; 0.10 suggested for a realistic preset, unsourced]
});

export const THERMAL = deepFreeze({
  ambientC: 25,              // degC [ASSUMPTION]
  refC: 25,                  // degC reference for R, Ke, Kt [ASSUMPTION]
  rthWindingHousing: 2.5,    // K/W [ASSUMPTION: interpolated between Maxon A-max 26 (3.2) and RE40 (1.93/4.65); no goBILDA/REV data]
  cthWinding: 10,            // J/K [ASSUMPTION: tau 25 s with Rwh]
  rthHousingAmbient: 8.0,    // K/W [ASSUMPTION]
  cthHousing: 75,            // J/K [ASSUMPTION: tau 600 s]
  copperAlpha: 0.00393,      // 1/K [SOURCED annealed copper; motioncontroltips quotes 0.0040]
  magnetAlpha: -0.0020,      // 1/K [ASSUMPTION: ceramic/ferrite magnets assumed for RS-555]
  windingMaxC: 130,          // degC [ASSUMPTION: class B; Maxon RE is 155]
  diodeDropV: 0.7,           // V [ASSUMPTION: for the FLOAT clamp]
});

export const SIM = deepFreeze({
  substepS: 0.001,        // s [design choice]
  maxFrameDtS: 0.1,       // s [design choice]
  coulombEpsMps: 0.005,   // m/s Coulomb friction regularisation width for drive [design choice]
});

// ---- flywheel ----
export const FLYWHEEL = deepFreeze({
  motor: 'gobilda5203',      // [hardware notes: 2 motors]
  gearbox: 'none',           // 1:1 [hardware notes]
  numMotors: 2,
  extraRatio: 1,
  wheelDiameterM: 0.096,     // m [SOURCED robotmodel 96 mm]
  wheelMassKg: 0.10,         // kg [ASSUMPTION: solid disc]
  wheelJ: 1.152e-4,          // kg*m^2 = 0.5*0.10*0.048^2 [ASSUMPTION, from wheelMassKg]
  extraJ: 0,                 // kg*m^2 [ASSUMPTION]
  maxRpm: 5000,              // rpm [hardware notes: runs at or under 5000]
  zeroPower: 'FLOAT',
  shotEff: 0.70,             // [ASSUMPTION] fraction of wheel energy transferred to the ball
  ballRotFactor: 2 / 3,      // [ASSUMPTION: hollow sphere rotational energy factor]
  defaultBallMassKg: 0.025,  // kg [ASSUMPTION: docs/overnight/app2.md 0.055 lb, unverified]
  pidf: {
    kP: 1.0e-3,              // power per tick/s [ASSUMPTION]
    kI: 5.0e-4,              // power per tick [ASSUMPTION]
    kD: 0,
    kF: null,                // null -> 1/freeTicksPerSec computed from the gearmotor
    iClamp: 0.3,             // power [ASSUMPTION]
    periodS: 0.02,           // s control period [ASSUMPTION]
    voltageCompensation: true,
  },
  readyTolFrac: 0.04,        // [ASSUMPTION]
});

// ---- turret (legacy DC option; the real turret is now 2 Axon servos) ----
export const TURRET = deepFreeze({
  motor: 'gobilda5203',      // [ASSUMPTION]
  gearbox: 'gb5_2',          // [ASSUMPTION]
  numMotors: 1,
  extraRatio: 176 / 64,      // ring 176T / pinion 64T = 2.75 [ASSUMPTION from spec]
  J: 0.02,                   // kg*m^2 ring + turret [ASSUMPTION]
  kpPowerPerDeg: 0.05,       // power per degree [ASSUMPTION]
  minDeg: -180,              // deg [spec default]
  maxDeg: 180,               // deg [spec default]
  zeroPower: 'BRAKE',
});

// ---- drive ----
export const DRIVE = deepFreeze({
  massKg: 13.6,           // kg [ASSUMPTION: 30 lb]
  frameXM: 0.381,         // m [ASSUMPTION]
  frameYM: 0.375,         // m [ASSUMPTION]
  yawInertia: 0.3238,     // kg*m^2 = m*(0.381^2+0.375^2)/12 [ASSUMPTION, derived from the above]
  wheelRadiusM: 0.052,    // m, 104 mm mecanum [SOURCED goBILDA mecanum wheel]
  lxM: 0.150,             // m half wheelbase [ASSUMPTION]
  lyM: 0.160,             // m half track [ASSUMPTION]
  motor: 'gobilda5203',
  gearbox: 'gb13_7',
  extraRatio: 2.0,        // belt 24T:48T [ASSUMPTION: confirm direction and variant with team]; free surface speed 46.8 in/s
  numMotorsPerWheel: 1,
  wheelJ: 2.7e-4,         // kg*m^2 [ASSUMPTION: 0.2 kg disc]
  mu: 0.7,                // traction coefficient [ASSUMPTION]
  lateralEff: 0.85,       // [taken from existing driver.js strafeFactor]
  zeroPower: 'BRAKE',
});
