// air.js - air properties, Reynolds number, aero factors and the per-kind model builder.
// SI inside this file, converted to sim units (inches) at the boundary. Pure functions, no DOM.
// All coefficient values are ASSUMED (see SOURCES in params.js).
import { AERO } from './params.js';

const C = AERO.consts;
const IN = C.IN_TO_M; // m per inch, exact

/** Saturation vapour pressure [Pa] at tempC [deg C]. Alduchov-Eskridge form (ASSUMED, unverified). */
export function saturationVaporPressurePa(tempC) {
  return 610.94 * Math.exp((17.625 * tempC) / (tempC + 243.04));
}

/** ISA static pressure [Pa] at altitudeM [m]. */
export function pressureAtAltitude(altitudeM) {
  return 101325 * Math.pow(1 - 2.25577e-5 * altitudeM, 5.25588);
}

/** Moist-air density [kg/m^3]. */
export function airDensity({ tempC, pressurePa, relHumidity }) {
  const T = tempC + C.T0;
  const pv = relHumidity * saturationVaporPressurePa(tempC);
  return (pressurePa - pv) / (C.Rd * T) + pv / (C.Rv * T);
}

/** Dynamic viscosity [Pa s], Sutherland's law. */
export function dynamicViscosity(tempC) {
  const T = tempC + C.T0;
  return C.mu0 * Math.pow(T / C.T0, 1.5) * (C.T0 + C.Suth) / (T + C.Suth);
}

/** Resolve an air block {tempC, pressurePa, relHumidity, altitudeM} to {rho [kg/m^3], nu [m^2/s], mu [Pa s], ...}. */
export function resolveEnv(airParams = AERO.air) {
  const tempC = airParams.tempC;
  const relHumidity = airParams.relHumidity;
  const pressurePa = airParams.altitudeM != null ? pressureAtAltitude(airParams.altitudeM) : airParams.pressurePa;
  const rho = airDensity({ tempC, pressurePa, relHumidity });
  const mu = dynamicViscosity(tempC);
  return { rho, nu: mu / rho, mu, tempC, pressurePa, relHumidity };
}

/** Reynolds number from speed [in/s], ball radius [in], kinematic viscosity nu [m^2/s]. */
export function reynolds(speedInPerS, radiusIn, nu) {
  return (speedInPerS * IN) * (2 * radiusIn * IN) / nu;
}

/** Base aero factor 0.5*rho*A/m, in 1/in (rho kg/m^3, radiusIn in, massKg kg). */
export function aeroBasePerIn(rho, radiusIn, massKg) {
  const A = Math.PI * (radiusIn * IN) * (radiusIn * IN); // m^2
  return (0.5 * rho * A / massKg) * IN;
}

/** Quadratic drag coefficient k = base*Cd, in 1/in. */
export function dragFactorPerIn(cd, rho, radiusIn, massKg) {
  return aeroBasePerIn(rho, radiusIn, massKg) * cd;
}

/**
 * Build the aero model for one ball kind. overrides is shallow-merged into a copy of params first.
 * Legacy mode: kDrag = legacyDragK, lift forced off, integrator forced 'legacy'.
 */
export function buildAeroModel(params = AERO, kindSpec = AERO.kinds.pollen, overrides = {}) {
  const p = Object.assign({}, params, overrides);
  const env = resolveEnv(p.air);
  const { radiusIn, massKg } = kindSpec;
  const base = aeroBasePerIn(env.rho, radiusIn, massKg);
  const legacy = p.model !== 'physical';
  const cd = p.cd;
  const lift = p.lift;
  return {
    mode: legacy ? 'legacy' : 'physical',
    g: p.g,
    radiusIn,
    massKg,
    rho: env.rho,
    nu: env.nu,
    base,
    kDrag: legacy ? p.legacyDragK : base * cd.subcritical,
    cd: {
      subcritical: cd.subcritical, supercritical: cd.supercritical,
      reCritLo: cd.reCritLo, reCritHi: cd.reCritHi, dragCrisis: cd.dragCrisis,
    },
    lift: {
      enabled: legacy ? false : !!lift.enabled,
      a: lift.a, b: lift.b, clMax: lift.clMax, inverseMagnus: !!lift.inverseMagnus,
    },
    spinDecayTau: lift.spinDecayTau,
    contactSpinKeep: lift.contactSpinKeep,
    maxSpeed: p.maxSpeed,
    airborneEps: p.airborneEps,
    maxSubstep: p.maxSubstep,
    maxStepTravel: p.maxStepTravel,
    integrator: legacy ? 'legacy' : p.integrator,
  };
}
