// inertia.js - moment of inertia of the flywheel shaft assembly. SI units (kg*m^2). DERIVED: standard formulas.
// disk 0.5*m*r^2, ring m*r^2, annulus 0.5*m*(ro^2+ri^2), given = tabulated J.

/**
 * Inertia of one component about the shaft axis.
 * @param {{kind:string, massKg?:number, rOuterM?:number, rInnerM?:number, jKgM2?:number}} c
 * @returns {number} kg*m^2, NaN if invalid
 */
export function componentInertia(c) {
  if (!c || typeof c !== 'object') return NaN;
  const fp = (x) => typeof x === 'number' && Number.isFinite(x) && x > 0;
  switch (c.kind) {
    case 'disk':
      if (!fp(c.massKg) || !fp(c.rOuterM)) return NaN;
      return 0.5 * c.massKg * c.rOuterM * c.rOuterM;
    case 'ring':
      if (!fp(c.massKg) || !fp(c.rOuterM)) return NaN;
      return c.massKg * c.rOuterM * c.rOuterM;
    case 'annulus':
      if (!fp(c.massKg) || !fp(c.rOuterM) || !(typeof c.rInnerM === 'number' && Number.isFinite(c.rInnerM) && c.rInnerM >= 0)) return NaN;
      return 0.5 * c.massKg * (c.rOuterM * c.rOuterM + c.rInnerM * c.rInnerM);
    case 'given':
      return fp(c.jKgM2) ? c.jKgM2 : NaN;
    default:
      return NaN;
  }
}

/**
 * Wheel-side inertia: sum of components plus extraKgM2.
 * @param {{components:object[], extraKgM2?:number}} inertiaParams
 * @returns {number} kg*m^2 (NaN if any component is invalid)
 */
export function wheelSideInertia(inertiaParams) {
  const comps = (inertiaParams && inertiaParams.components) || [];
  let s = Number.isFinite(inertiaParams && inertiaParams.extraKgM2) ? inertiaParams.extraKgM2 : 0;
  for (const c of comps) s += componentInertia(c);
  return s;
}

/**
 * Motor rotor inertia reflected to the wheel: count * J_rotor * G^2.
 * @param {{count:number, rotorInertiaKgM2:number}} motorParams
 * @param {number} gearRatio motor rev per wheel rev
 * @returns {number} kg*m^2
 */
export function reflectedMotorInertia(motorParams, gearRatio) {
  return motorParams.count * motorParams.rotorInertiaKgM2 * gearRatio * gearRatio;
}

/**
 * Total inertia at the wheel shaft with breakdown.
 * @param {object} params full flywheel params
 * @returns {{jWheelKgM2:number, jMotorReflectedKgM2:number, jTotalKgM2:number, breakdown:Array<{name:string,jKgM2:number}>}}
 */
export function totalInertia(params) {
  const breakdown = (params.inertia.components || []).map((c) => ({ name: c.name, jKgM2: componentInertia(c) }));
  const extra = params.inertia.extraKgM2 || 0;
  if (extra) breakdown.push({ name: 'extra', jKgM2: extra });
  const jw = wheelSideInertia(params.inertia);
  const jm = reflectedMotorInertia(params.motor, params.drive.gearRatio);
  breakdown.push({ name: 'reflected motor rotors', jKgM2: jm });
  return { jWheelKgM2: jw, jMotorReflectedKgM2: jm, jTotalKgM2: jw + jm, breakdown };
}

/**
 * Rotational kinetic energy 0.5*J*w^2.
 * @param {number} jKgM2 inertia, kg*m^2
 * @param {number} omegaRadS speed, rad/s
 * @returns {number} joules
 */
export function storedEnergyJ(jKgM2, omegaRadS) {
  return 0.5 * jKgM2 * omegaRadS * omegaRadS;
}
