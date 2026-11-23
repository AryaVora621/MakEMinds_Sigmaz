// params.js - the ONLY place ball-bounce-rolling tunables live.
// Units: inches, seconds, radians, in/s, rad/s. Mass is unitless 1.0 per ball (impulses are per unit mass).
// Every constant is tagged SOURCED (with URL) or ASSUMED (with TODO measure).
// Imports nothing (must not import balls.js). Nothing is frozen so tests can override fields temporarily.

export const BBR = {
  // SOURCED: 9.80665 m/s^2 * 39.37007874 in/m = 386.0886 in/s^2 (unit conversion of standard gravity).
  g: 386.0886,                // in/s^2
  // ASSUMED (kept from the old sim). TODO measure: slowest impact speed that visibly bounces.
  restSpeed: 5,               // in/s
  // ASSUMED (kept). Compared against the Box2D metric |v|+|w|*r, https://raw.githubusercontent.com/erincatto/box2d/main/src/solver.c
  sleepSpeed: 1.5,            // in/s
  // ASSUMED (kept). Box2D uses 0.5 s (constants.h); this sim keeps 0.25 s. TODO measure.
  sleepTime: 0.25,            // s
  // ASSUMED. Height tolerance for "off the floor". TODO measure.
  airEps: 0.05,               // in
  // ASSUMED: LDPE hollow ball shell density. TODO measure shell thickness/mass of a real ball.
  rhoShell: 0.95,             // g/cm^3
  // Debug override for the inertia factor k; null = compute from shell model.
  kOverride: null,            // dimensionless or null
  // Rolling-spin initialisation for balls created touching the floor with nonzero velocity.
  initRollingSpin: true,      // bool
  // ASSUMED: flywheel backspin is not measured. TODO measure with high-speed video. >0 = backspin.
  launchSpinRatio: 0,         // dimensionless (w*r/|v|)
  // Must equal BALL.maxSpeed in balls.js.
  maxSpeed: 1800,             // in/s
};

const PB = 0.197; // SOURCED: pooltool min_bounce_height 0.005 m = 0.197 in,
// https://raw.githubusercontent.com/ekiefl/pooltool/main/pooltool/physics/evolve/__init__.py

// Fields: eRef (restitution at vRef), vRef (in/s), ePow (dimensionless, e ~ v^-ePow), eMin, eMax,
// muSlide (Coulomb, dimensionless), et (tangential restitution), crr (rolling resistance coeff),
// cv (1/s, velocity-proportional rolling term), spinLen (in, torsional friction length after MuJoCo
// https://mujoco.readthedocs.io/en/stable/modeling.html), minBounceH (in).
export const SURFACES = {
  floor: {
    eRef: 0.45,   // ASSUMED (kept from old sim; TODO measure: foam drop COR = sqrt(h_rebound/h_drop) at 2, 10, 40 in)
    vRef: 172,    // in/s: speed of a drop from 40 in centre height, sqrt(2*386.09*38.6); makes e(172)=0.45 exact
    ePow: 0.1,    // ASSUMED, TODO measure (same drop test at 3 heights)
    eMin: 0.2,    // ASSUMED, TODO measure
    eMax: 0.75,   // ASSUMED, TODO measure
    muSlide: 0.5, // ASSUMED (kept), TODO measure
    et: 0,        // SOURCED: pooltool no-slip branch, https://raw.githubusercontent.com/ekiefl/pooltool/main/pooltool/physics/resolve/ball_table/frictional_inelastic/__init__.py
    crr: 0.04,    // ASSUMED (kept; 15.4 in/s^2 decel), TODO measure roll-out distance
    cv: 0,        // ASSUMED off (unsourced form), 1/s
    spinLen: 0.02,// ASSUMED, TODO measure spin decay
    minBounceH: PB,
  },
  wall: {
    eRef: 0.55,   // weak source: hard plastic on concrete; TODO measure
    vRef: 172, ePow: 0, eMin: 0, eMax: 1,
    muSlide: 0.5, // ASSUMED, TODO measure
    et: 0,        // pooltool no-slip, see above
    crr: 0.04,    // ASSUMED, TODO measure
    cv: 0,        // ASSUMED off
    spinLen: 0.02,// ASSUMED, TODO measure
    minBounceH: PB,
  },
  robot: {
    eRef: 0.35,   // ASSUMED, TODO measure
    vRef: 172, ePow: 0, eMin: 0, eMax: 1,   // ASSUMED
    muSlide: 0.5, et: 0, crr: 0.04, cv: 0, spinLen: 0.02, // all ASSUMED, TODO measure
    minBounceH: PB,
  },
  hive: {
    eRef: 0.3,    // ASSUMED, TODO measure
    vRef: 172, ePow: 0, eMin: 0, eMax: 1,   // ASSUMED
    muSlide: 0.5, et: 0, crr: 0.04, cv: 0, spinLen: 0.02, // all ASSUMED, TODO measure
    minBounceH: PB,
  },
  ball: {
    eRef: 0.7,    // ASSUMED, TODO measure
    vRef: 172, ePow: 0, eMin: 0, eMax: 1,   // ASSUMED
    muSlide: 0.5, et: 0, crr: 0, cv: 0, spinLen: 0,       // all ASSUMED, TODO measure
    minBounceH: PB,
  },
};

// Inertia factor k = I/(m r^2) of a hollow shell from radius (in) and mass (kg).
// SOURCED: thick shell I = (2/5) m (R^5-r^5)/(R^3-r^3), https://en.wikipedia.org/wiki/List_of_moments_of_inertia
export function inertiaFactor(radiusIn, massKg) {
  if (typeof BBR.kOverride === 'number') return BBR.kOverride;
  const Rcm = radiusIn * 2.54;
  const tcm = (massKg * 1000) / (BBR.rhoShell * 4 * Math.PI * Rcm * Rcm);
  const x = Math.max(0, 1 - tcm / Rcm);
  let k;
  if (Math.abs(1 - x) < 1e-9) k = 2 / 3;
  else {
    const x3 = x * x * x;
    k = 0.4 * (1 - x3 * x * x) / (1 - x3);
  }
  return Math.min(2 / 3, Math.max(0.4, k));
}

// Restitution as a function of impact speed vin = -vn (in/s).
export function restitutionAt(surf, vin) {
  const v = vin < 1 ? 1 : vin;
  const e = surf.eRef * Math.pow(v / surf.vRef, -surf.ePow);
  return Math.min(surf.eMax, Math.max(surf.eMin, e));
}

// True when the impact should rebound (otherwise it becomes a resting contact).
// nz = z component of the contact normal (1 = up-facing surface).
export function bounceArmed(surf, vin, e, nz) {
  if (!(vin > BBR.restSpeed)) return false;
  if (nz <= 0.2) return true;
  return e * vin > Math.sqrt(2 * BBR.g * nz * surf.minBounceH);
}

export function surfaceParams(name) {
  return SURFACES[name] || SURFACES.wall;
}
