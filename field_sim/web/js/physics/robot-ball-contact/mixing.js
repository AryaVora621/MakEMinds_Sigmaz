// mixing.js - pure material-mixing and unit helpers for robot-ball contact.
// Units: length in, time s, velocity in/s, mass kg, impulse kg*in/s, sim force = kg*in/s^2.
import { RBC_PARAMS } from './params.js';

/** Friction mix, dimensionless: sqrt(a*b) (Box2D v3 contact.h). */
export function mixFriction(a, b) { return Math.sqrt(a * b); }

/** Restitution mix, dimensionless: max(a,b) (Box2D v3 contact.h). */
export function mixRestitution(a, b) { return Math.max(a, b); }

/** Restitution actually used: e if closing speed vnAbs (in/s) exceeds thresholdInS (in/s), else 0. */
export function effectiveRestitution(e, vnAbs, thresholdInS) { return vnAbs > thresholdInS ? e : 0; }

/** Ball properties for 'pollen' | 'nectar' (unknown kind falls back to pollen). Units: kg, in, N/m. */
export function ballProps(kind, params = RBC_PARAMS) {
  const b = params.ball[kind] || params.ball.pollen;
  return {
    massKg: b.massKg, radiusIn: b.radiusIn, inertiaFactor: b.inertiaFactor, restitution: b.restitution,
    friction: b.friction, stiffnessNpm: b.stiffnessNpm, maxSquishFrac: b.maxSquishFrac,
  };
}

/** Yaw inertia of a uniform plate, kg*in^2 = m*((2hx)^2+(2hy)^2)/12; halfX, halfY in inches. */
export function robotInertia(massKg, halfX, halfY) {
  const w = 2 * halfX, l = 2 * halfY;
  return massKg * (w * w + l * l) / 12;
}

/** Roller surface speed, in/s = rpm*PI*diameterIn/60. */
export function rollerSurfaceSpeed(rpm, diameterIn) { return rpm * Math.PI * diameterIn / 60; }

const N_TO_SIM = RBC_PARAMS.units.N_TO_SIM;
/** Newtons -> sim force (kg*in/s^2). */
export function newtonsToSim(fN) { return fN * N_TO_SIM; }
/** Sim force (kg*in/s^2) -> Newtons. */
export function simToNewtons(f) { return f / N_TO_SIM; }
/** Stiffness N/m (= kg/s^2) is length-unit independent, so F[sim] = k * delta[in]; identity. */
export function stiffnessToSim(kNpm) { return kNpm; }
