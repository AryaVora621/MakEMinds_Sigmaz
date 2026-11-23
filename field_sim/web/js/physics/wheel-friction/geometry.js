// geometry.js - builds the SI DrivetrainConfig (spec section 2) from a robot_spec.json-shaped object.
// Pure ES module. Body frame: +x forward, +y left, omega > 0 CCW. Wheel order FL, FR, BL, BR.
import { makeParams } from './params.js';

export const WHEEL_ORDER = ['FL', 'FR', 'BL', 'BR'];

/** Rotate a 2-vector CCW by angRad. @returns {number[]} [x,y] */
export function rot2(v, angRad) {
  const c = Math.cos(angRad), s = Math.sin(angRad);
  return [c * v[0] - s * v[1], s * v[0] + c * v[1]];
}

/**
 * @typedef {Object} WheelConfig
 * @property {string} name  FL|FR|BL|BR
 * @property {string} type  mecanum|omni|traction
 * @property {number[]} pos [x,y] m, body frame
 * @property {number} radiusM
 * @property {number} inertiaKgM2 wheel+rotor inertia
 * @property {number[]} driveDir unit drive direction t
 * @property {{dir:number[],coupling:number,muFactor:number}[]} axes constrained axes (1 roller, 2 traction)
 * @property {number[]|null} freeDir free roller direction p (null for traction)
 * @property {number} muStatic @property {number} muKinetic @property {number} crrFree @property {number} crrDrive
 *
 * @typedef {Object} DrivetrainConfig
 * @property {number} massKg @property {number} weightN @property {number} inertiaKgM2
 * @property {number[]} cgM [x,y] m @property {number} cgHeightM
 * @property {WheelConfig[]} wheels length 4, order FL,FR,BL,BR
 * @property {{freeSpeedRadS:number,stallTorqueNm:number,coulombTorqueNm:number,viscousNmPerRadS:number,voltageScale:number,zeroPowerBehavior:string}} motor stallTorqueNm includes efficiency
 * @property {{stribeckVelocityMS:number,stribeckExponent:number,stickVelocityMS:number,slipRatioVMinMS:number,gripScale:number,slipKnobGripLoss:number}} friction
 * @property {{maxSubstepS:number,iterations:number,maxDtS:number,maxSubsteps:number,accelFilterTauS:number}} solver
 * @property {{turnInputScale:number,rhoM:number}} driver
 * @property {Object} params the frozen params used
 */

function quadrantName(x, y) {
  if (x > 0) return y > 0 ? 'FL' : 'FR';
  return y > 0 ? 'BL' : 'BR';
}

function orderWheels(list) {
  if (list.length !== 4) throw new Error(`wheel-friction: expected exactly 4 wheels, got ${list.length}`);
  const allNamed = list.every(w => WHEEL_ORDER.includes(w.name));
  const keyed = list.map(w => ({ w, key: allNamed ? w.name : quadrantName(w.x, w.y) }));
  const out = WHEEL_ORDER.map(n => keyed.find(k => k.key === n));
  if (out.some(o => !o) || new Set(keyed.map(k => k.key)).size !== 4) {
    throw new Error('wheel-friction: wheels must map to distinct FL, FR, BL, BR positions');
  }
  return out.map((o, i) => ({ ...o.w, name: WHEEL_ORDER[i] }));
}

/**
 * Build the SI drivetrain config.
 * @param {Object|null} spec robot_spec.json-shaped {chassis:{mass_lb,footprint,cgOffsetIn,cgHeightIn}, wheels:[...]} or null for defaults
 * @param {Object} params frozen result of makeParams()
 * @returns {DrivetrainConfig}
 */
export function buildDrivetrainConfig(spec = null, params = makeParams()) {
  const { UNITS, WHEEL_TYPES, FRICTION, MOTOR, CHASSIS_DEFAULTS, SOLVER, DRIVER } = params;
  const IN = UNITS.IN_TO_M;
  const ch = (spec && spec.chassis) || {};

  const massLb = ch.mass_lb ?? CHASSIS_DEFAULTS.massLb;
  let fp = CHASSIS_DEFAULTS.footprintIn;
  if (Array.isArray(ch.footprint) && ch.footprint.length) {
    const xs = ch.footprint.map(p => p[0]), ys = ch.footprint.map(p => p[1]);
    fp = [Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)];
  }
  const cgOffsetIn = ch.cgOffsetIn ?? CHASSIS_DEFAULTS.cgOffsetIn;
  const cgHeightIn = ch.cgHeightIn ?? CHASSIS_DEFAULTS.cgHeightIn;

  const massKg = massLb * UNITS.LB_TO_KG;
  const weightN = massKg * UNITS.G;
  const Lx = fp[0] * IN, Ly = fp[1] * IN;
  const inertiaKgM2 = CHASSIS_DEFAULTS.inertiaScale * massKg * (Lx * Lx + Ly * Ly) / 12;

  const rawWheels = orderWheels((spec && Array.isArray(spec.wheels)) ? spec.wheels : CHASSIS_DEFAULTS.wheels);

  const wheels = rawWheels.map(w => {
    const type = w.type ?? 'mecanum';
    const wt = WHEEL_TYPES[type];
    if (!wt) throw new Error(`wheel-friction: unknown wheel type '${type}'`);
    const pos = [w.x * IN, w.y * IN];
    const t = rot2([1, 0], (w.mountDeg ?? 0) * Math.PI / 180);
    let axes, freeDir;
    if (wt.axisAngleDeg === null) {
      axes = [
        { dir: t, coupling: 1, muFactor: 1 },
        { dir: rot2(t, Math.PI / 2), coupling: 0, muFactor: wt.muLatRatio },
      ];
      freeDir = null;
    } else {
      const s = type === 'mecanum' ? (w.x * w.y > 0 ? -1 : 1) : 1;
      const angDeg = (w.rollerAxisDeg !== undefined && w.rollerAxisDeg !== null) ? w.rollerAxisDeg : s * wt.axisAngleDeg;
      const d = rot2(t, angDeg * Math.PI / 180);
      const c = t[0] * d[0] + t[1] * d[1];
      axes = [{ dir: d, coupling: c, muFactor: 1 }];
      freeDir = rot2(d, Math.PI / 2);
    }
    return {
      name: w.name, type, pos, radiusM: w.radius * IN, inertiaKgM2: MOTOR.wheelInertiaKgM2,
      driveDir: t, axes, freeDir,
      muStatic: wt.muStatic, muKinetic: wt.muKinetic, crrFree: wt.crrFree, crrDrive: wt.crrDrive,
    };
  });

  // rhoM: mean |r x d| / c over each wheel's first axis with coupling != 0
  let sum = 0, n = 0;
  for (const w of wheels) {
    const ax = w.axes.find(a => Math.abs(a.coupling) > 1e-9);
    if (!ax) continue;
    sum += Math.abs(w.pos[0] * ax.dir[1] - w.pos[1] * ax.dir[0]) / Math.abs(ax.coupling);
    n++;
  }
  const rhoM = n ? sum / n : 1;

  return {
    massKg, weightN, inertiaKgM2,
    cgM: [cgOffsetIn[0] * IN, cgOffsetIn[1] * IN],
    cgHeightM: cgHeightIn * IN,
    wheels,
    motor: {
      freeSpeedRadS: MOTOR.freeSpeedRpm * 2 * Math.PI / 60,
      stallTorqueNm: MOTOR.stallTorqueNm * MOTOR.drivetrainEfficiency,
      coulombTorqueNm: MOTOR.coulombTorqueNm,
      viscousNmPerRadS: MOTOR.viscousNmPerRadS,
      voltageScale: MOTOR.voltageScale,
      zeroPowerBehavior: MOTOR.zeroPowerBehavior,
    },
    friction: {
      stribeckVelocityMS: FRICTION.stribeckVelocityMS,
      stribeckExponent: FRICTION.stribeckExponent,
      stickVelocityMS: FRICTION.stickVelocityMS,
      slipRatioVMinMS: FRICTION.slipRatioVMinMS,
      gripScale: FRICTION.fieldGripScale,
      slipKnobGripLoss: FRICTION.slipKnobGripLoss,
    },
    solver: { ...SOLVER },
    driver: { turnInputScale: DRIVER.turnInputScale, rhoM },
    params,
  };
}
