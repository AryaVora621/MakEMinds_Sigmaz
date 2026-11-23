// params.js - single tunable file for the battery-sag electrical model. Pure ES module, no DOM.
// Units are SI inside the electrical code (V, A, ohm, s, rad/s, N*m, kg*m^2) unless a name says otherwise (In = inches).
// Legend: [S] sourced figure (source named). [A] assumption/calibration, needs a bench measurement.

export const BATTERY = {
  nCells: 10,                 // [S] REV-31-1302 product page: "10-cell, 12V 3000mAh"
  capacityAh: 3.0,            // Ah [S] REV-31-1302; also TETRIX W39057 (Pitsco/Studica)
  peukertN: 1.0,              // [A] NiMH shows little Peukert effect (verlab Roomba table); real-cell range is 1.1-1.3. Needs bench measurement.
  peukertRefA: 3.0,           // A, 1C reference current
  // V/cell vs SoC (0..1). [A] shape, needs a bench measurement; endpoints: 1.39 V full-charge cell (MathWorks NiMH example),
  // 0.90 V/cell = REV 9.0 V minimum no-load pack voltage, 1.18-1.2 V nominal.
  ocvTable: [[0, 0.90], [0.05, 1.10], [0.10, 1.17], [0.20, 1.21], [0.40, 1.24], [0.60, 1.27], [0.80, 1.30], [0.90, 1.33], [1.00, 1.40]],
  // ohm per cell. [A] gives 90 mOhm per pack; the researcher's extrapolation from 12-15 mOhm sub-C cells was about 120 mOhm.
  // Calibrated so the total path R (about 0.105 ohm) reproduces the FTC-community sag of 13.5 V -> 9.7 V with 8 motors at about 30 A
  // (ftc-community.firstinspires.org/t/voltage-drops-during-chassis-run/983). Needs bench measurement.
  r0CellOhm: 0.009,
  r1CellOhm: 0.0025,          // ohm per cell [A] polarisation resistance, 25 mOhm per pack. Needs bench measurement.
  tau1S: 15,                  // s [S-assumed] COMSOL NiMH ECM RC time constant of 15 s
  rSocTable: [[0, 1.8], [0.2, 1.35], [0.5, 1.15], [1.0, 1.0]], // [A] R0 multiplier vs SoC (resistance rises at low SoC, per Battery University qualitative). Needs bench measurement.
  vMinNoLoad: 9.0,            // V [S] REV accessories page
};

export const WIRING = {
  fuseColdOhm: 0.00338,       // ohm [S] Littelfuse ATOF 20 A datasheet, cold resistance 3.38 mOhm
  xt30PairOhm: 0.00085,       // ohm [S] XT30 contact <= 0.85 mOhm (Cirkit Designer)
  nXt30Pairs: 2,
  switchOhm: 0.002,           // ohm [A] REV-31-1387 switch resistance not found. Needs bench measurement.
  wireOhmPerM: 0.0132,        // ohm/m [S] 16 AWG copper at 20 C (HyperPhysics)
  wireOneWayM: 0.3,           // m [A] the round trip is 2x this. Needs measurement.
};

export const FUSE = {
  ratedA: 20,                 // A [S] REV-31-1302 in-line 20 A ATM/ATO fuse
  enabled: true,
  noTripFrac: 1.10,           // [S] Littelfuse ATOF: at 110% the minimum is 360000 s, so it never opens
  // [I/I_rated, typical open time s] = geometric mean of the datasheet min/max (0.75/600, 0.25/50, 0.15/5, 0.08/0.5, <=0.1) [S-derived]
  curve: [[1.35, 21.2], [1.60, 3.54], [2.0, 0.866], [3.5, 0.2], [6.0, 0.05]],
  i2tA2s: 520,                // A^2*s [S] Littelfuse ATOF 20 A I^2t (typical); used as an adiabatic cap
  coolTauS: 10,               // s [A] damage decay below noTripFrac. Needs bench measurement.
};

// [S] goBILDA 5203 Yellow Jacket listing (1:1 ratio, 9.2 A stall, 0.25 A no-load at 12 V)
export const MOTOR_5203 = { vRated: 12, stallA: 9.2, freeA: 0.25, freeRpm: 6000 };

export const DRIVE_ELEC = {
  gearRatio: 19.2,            // [S] goBILDA 312 rpm 19.2:1 (FTC thread drive motors)
  wheelRadiusIn: 2.0472,      // in [A] 104 mm goBILDA mecanum
  gearEff: 0.9,               // [A]
  massKg: 15,                 // kg [A] robot mass
  halfTrackIn: 7.0, halfBaseIn: 6.5, // in [A] placeholders, should match robot_spec
  nominalV: 12,               // V at which DRIVE.maxSpeed 60 in/s is assumed to be tuned
  compensation: 'none',       // 'none' | 'nominal12'
};

export const FLYWHEEL = {
  nMotors: 2, gearRatio: 1,   // [A] 5203 1:1 motors (maxRpm 6000 matches the free speed)
  inertiaKgM2: 1.85e-4,       // kg*m^2 [A] calibrated: tau_m = J*R/(n*kT*kE) = 0.35 s at 12 V, matching the existing flywheelTauS. Needs bench measurement.
  viscousNmPerRadS: 0,
  kP: 0.01,                   // duty per rad/s of error; closed-loop pole about 21 /s (tau about 0.047 s), matches the existing recover tau of 0.04 [A]
  compensation: 'none',       // 'none' (kF = 1/omegaFree12) | 'nominal12' (kF *= 12/V)
  rpmDipFrac: 0.06,           // same as the existing PLANT value [A]
  substepMaxS: 0.002,         // s
};

export const INTAKE = { nMotors: 1, gearRatio: 1, inertiaKgM2: 2e-5, loadNm: 0.03 }; // [A] kg*m^2, N*m. Needs bench measurement.

export const HUB = {
  lowBatteryV: 7.0,           // V [S] REV LED blink codes: Blinking Orange below 7 V, clears above 7 V
  vinMinV: 8.0,               // V [S] Expansion Hub V_IN minimum 8 V (informational flag only)
  resetV: 5.5,                // V [A] the REV reset threshold was not found. Needs bench measurement.
  resetDwellS: 0.002,         // s [A]
  recoverV: 7.0,              // V [A], uses the 7 V fault line
  rebootS: 0.5,               // s [A]
  servoRailDropV: 6.0,        // V [A] 5 V buck dropout
  motorPortContA: 10, motorPortAbsA: 20, // A [S] Expansion Hub basics
  resetClearsEncoders: true,  // [A]
  resetClearsPinpoint: true,  // [S-partial] goBILDA Pinpoint loses its configuration on a power cycle
  electronicsW: 4.2,          // W [A] hub plus radio, about 0.35 A at 12 V
  pinpointW: 0.59,            // W [S] Pinpoint 100 mA at 5 V, divided by railEff
  railV: 5.0,                 // V [S] servo supply 5 V typ
  railEff: 0.85,              // [A]
  servoIdleA: 0.05, servoMovingA: 0.8, // A per servo, on the 5 V rail [A]
};

export const SENSOR = { quantV: 0.01, noiseSigmaV: 0, lpTauS: 0, seed: 1 }; // V [A] Lynx ADC resolution not found

export const PRESETS = {
  fresh: { soc: 1.0, ageFactor: 1.0 },
  nominal: { soc: 0.7, ageFactor: 1.0 },
  tired: { soc: 0.3, ageFactor: 1.4 },
  dead: { soc: 0.1, ageFactor: 1.8 },
}; // [A]

export const BUS = { iters: 4, vFloor: 0, powerVMin: 1.0 };

// Piecewise-linear over [[x,y],...] with ascending x, clamped at the ends.
export function interp(table, x) {
  const n = table.length;
  if (x <= table[0][0]) return table[0][1];
  if (x >= table[n - 1][0]) return table[n - 1][1];
  for (let i = 1; i < n; i++) {
    if (x <= table[i][0]) {
      const [x0, y0] = table[i - 1];
      const [x1, y1] = table[i];
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }
  }
  return table[n - 1][1];
}
