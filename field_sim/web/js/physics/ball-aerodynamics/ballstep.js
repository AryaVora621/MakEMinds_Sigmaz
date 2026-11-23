// ballstep.js - drop-in adapter between balls.js ball objects {x,y,z,vx,vy,vz,r,type} and the aero integrators.
// Wiring (done later by the integrator): in BallWorld._substep replace the inline drag/gravity block with
//   airStep(b, h, reg.modelFor(b.type));  where reg = createAeroRegistry(AERO).
import { AERO } from './params.js';
import { buildAeroModel } from './air.js';
import { stepFlight, clampSpeed } from './integrator.js';
import { launchSpin } from './spin.js';

function isObj(v) { return v !== null && typeof v === 'object' && !Array.isArray(v); }

function deepMerge(dst, patch) {
  for (const k of Object.keys(patch)) {
    if (isObj(patch[k]) && isObj(dst[k])) deepMerge(dst[k], patch[k]);
    else dst[k] = isObj(patch[k]) ? deepMerge({}, patch[k]) : patch[k];
  }
  return dst;
}

// Registry: one cached model per ball kind. Works on a private deep copy of params so AERO is never mutated.
export function createAeroRegistry(params = AERO) {
  const reg = {
    params: deepMerge({}, params),
    _cache: new Map(),
    modelFor(kind) {
      const key = reg.params.kinds[kind] ? kind : 'pollen'; // unknown kind falls back to pollen
      let m = reg._cache.get(key);
      if (!m) {
        m = buildAeroModel(reg.params, reg.params.kinds[key]);
        reg._cache.set(key, m);
      }
      return m;
    },
    setParams(patch) {
      deepMerge(reg.params, patch);
      reg._cache.clear();
      return reg;
    },
    mode() { return reg.params.model; },
  };
  return reg;
}

// One substep of free flight for ball b: drag (+Magnus in physical mode), gravity, spin decay, then speed clamp.
export function airStep(b, h, model) {
  const airborne = b.z > b.r + model.airborneEps;
  stepFlight(b, h, model, airborne);
  clampSpeed(b, model.maxSpeed);
  return b;
}

// Call on fire: sets b.wx,b.wy,b.wz (rad/s) from launch velocity when lift is enabled, else zeros.
export function initBallAero(b, model, slip = AERO.lift.spinSlip) {
  if (model.lift && model.lift.enabled) {
    const w = launchSpin(b.vx, b.vy, b.vz, model.radiusIn, slip);
    b.wx = w[0]; b.wy = w[1]; b.wz = w[2];
  } else {
    b.wx = 0; b.wy = 0; b.wz = 0;
  }
  return b;
}
