// params.js - single tunable-constants file for ball aerodynamics (ESM, no DOM, no imports).
// Units: inches, seconds, rad/s, in/s^2 for sim-facing values; SI (K, Pa, kg, m) for the air/consts blocks.
// AERO is a plain mutable object so tests and the app can override fields. Every constant is tagged in SOURCES as
// 'SOURCED: <url>' or 'ASSUMED: <reason>'. Nothing aerodynamic was verified in the research run, so it is ASSUMED.
export const AERO = {
  model: 'legacy',          // 'legacy' = current balls.js inline drag, bit for bit | 'physical' = air-density-based drag/Magnus
  integrator: 'rk4',        // used in physical mode: 'legacy' | 'rk4'
  legacyDragK: 0.0004,      // 1/in, quadratic-drag coefficient of the existing sim (terminal speed ~982 in/s)
  g: 386.0886,              // in/s^2
  airborneEps: 0.05,        // in, ball is airborne when z > r + eps
  maxSpeed: 1800,           // in/s, speed clamp
  maxSubstep: 0.004,        // s, largest substep
  maxStepTravel: 0.8,       // in, largest distance per substep
  air: { tempC: 20, pressurePa: 101325, relHumidity: 0.5, altitudeM: null }, // deg C, Pa, 0..1, m (null = use pressurePa)
  consts: { Rd: 287.05, Rv: 461.5, mu0: 1.716e-5, T0: 273.15, Suth: 110.4, IN_TO_M: 0.0254 }, // J/(kg K), J/(kg K), Pa s, K, K, m/in
  cd: { subcritical: 0.47, supercritical: 0.2, reCritLo: 2e5, reCritHi: 3e5, dragCrisis: false }, // Cd dimensionless, Re dimensionless
  lift: { enabled: false, a: 2.022, b: 0.981, clMax: 0.35, spinSlip: 0.5, spinDecayTau: 2.0, contactSpinKeep: 0.5, inverseMagnus: false }, // CL = S/(a*S+b); tau in s
  kinds: {
    pollen: { radiusIn: 1.391, massKg: 0.025 },   // in, kg
    nectar: { radiusIn: 1.7995, massKg: 0.041 },  // in, kg
  },
  solver: { h: 0.002, tMax: 3.0, tol: 0.05, maxIter: 60, vMin: 50, vMax: 1800, elevMinDeg: 5, elevMaxDeg: 85, elevStepDeg: 1 }, // s, s, in, -, in/s, in/s, deg
};

export const SOURCES = {
  model: 'ASSUMED: default stays legacy so existing behaviour and tests are unchanged until the integrator switches it on',
  integrator: 'ASSUMED: RK4 chosen for accuracy (order 4); order facts from standard numerical analysis (gafferongames.com/post/integration_basics, low confidence)',
  legacyDragK: 'ASSUMED: existing sim value, tuned to existing tests; no measurement',
  g: 'SOURCED (unit conversion): standard gravity 9.80665 m/s^2 / 0.0254 m/in = 386.0886 in/s^2',
  airborneEps: 'ASSUMED: matches the existing balls.js airborne threshold, not measured',
  maxSpeed: 'ASSUMED: existing sim clamp, not measured',
  maxSubstep: 'ASSUMED: stability/accuracy choice, not measured',
  maxStepTravel: 'ASSUMED: keeps per-substep travel well below ball radius, not measured',
  air: 'ASSUMED: typical indoor 20 C, 1 atm, RH 50%; set to the actual venue conditions. Not measured',
  consts: 'ASSUMED: Rd/Rv, Sutherland (mu0, T0, S=110.4 K) and Alduchov-Eskridge/ISA forms from memory, unverified in the research run; IN_TO_M = 0.0254 is exact by definition of the inch',
  cd: 'ASSUMED: Cd_sub 0.47 is the textbook smooth-sphere value (unverified); supercritical 0.2 and crisis Re 2e5..3e5 unverified; ball Re is ~6e4 so crisis is off',
  lift: 'ASSUMED: CL = S/(2.022*S+0.981) is the Sawicki/Hubbard/Stronge 2003 form (unverified here); clMax, spinSlip 0.5, spinDecayTau 2.0 s, contactSpinKeep 0.5 are placeholders; inverse Magnus (Kim et al. 2014, doi:10.1017/jfm.2014.428) has no sourced Re/S range so it is OFF',
  kinds: 'SOURCED (radius, pollen mass): mirrors balls.js BALL_KINDS (FTC BIOBUZZ manual TU06 Sec 9.8, via ftc.uprobotics.tech/questions/78); ASSUMED: nectar mass 0.091 lb (~41 g) inferred from an ambiguous snippet',
  solver: 'ASSUMED: numerical-solver defaults (h = 2 ms matches shotrules sampling), not physical constants',
};
