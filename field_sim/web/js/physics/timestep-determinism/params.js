// params.js - all tunable constants for the fixed-timestep / determinism layer.
// Tags: [sourced] = taken from a cited reference; [assumed] = modelling choice, tunable.
// Units are in the comments. Angles rad, lengths in, time s.

const BASE = {
  PHYSICS_HZ: 240,            // [Hz] [assumed] matches BallWorld.stepFixed FIXED_HZ in balls.js
  PHYSICS_DT: 1 / 240,        // [s] derived = 1/PHYSICS_HZ
  CONTROL_DIVIDER: 4,         // [ticks] derived = PHYSICS_HZ/CONTROL_HZ
  CONTROL_HZ: 60,             // [Hz] [assumed] matches autorunner.js FIXED_DT = 1/60; real FTC loop rate unverified
  CONTROL_DT: 1 / 60,         // [s] derived = 1/CONTROL_HZ
  MAX_FRAME_DT: 0.25,         // [s] [sourced] Fiedler clamp, https://gafferongames.com/post/fix_your_timestep/
  MAX_STEPS_PER_FRAME: 32,    // [ticks] [assumed] 133 ms sim/frame; Bullet-style clamp-and-drop (btDiscreteDynamicsWorld.cpp)
  BACKLOG_POLICY: 'drop',     // ['drop'|'carry'] [assumed] 'carry' for headless runs
  HASH_EVERY_TICKS: 240,      // [ticks] [assumed] 1 s of sim between state hashes
  HASH_QUANT: 1e-6,           // [model units: in, rad, in/s] [assumed] quantised-hash resolution
  AXIS_QUANT: 32767,          // [counts] [assumed] analog axis quantisation, int16 range
  REPLAY_VERSION: 1,          // [-] replay log format version
  DEFAULT_SEED: 0x5EED1234,   // [uint32] [assumed] default PRNG seed
  LPF_REF_HZ: 60,             // [Hz] [assumed] frame rate the legacy per-frame smoothing factors were tuned at
  LINKED_VEL_LPF_TAU: 0.046727, // [s] [derived] -1/(60*ln(0.7)) from legacy k=0.3 per 60 Hz frame
  EXP_MAP_EPS: 1e-9,          // [rad] [assumed] small-angle switch in the SE(2) exponential
  MOTOR_TAU_DEFAULT: 0.030,   // [s] [assumed] placeholder, 20-50 ms band from motor-lag research
};

export const TS = Object.freeze({ ...BASE });

// Build a new frozen params object. PHYSICS_DT, CONTROL_DT and CONTROL_DIVIDER are
// always derived from PHYSICS_HZ / CONTROL_HZ.
export function makeParams(overrides = {}) {
  const p = { ...BASE, ...overrides };
  if (!(p.PHYSICS_HZ > 0) || !(p.CONTROL_HZ > 0)) throw new Error('PHYSICS_HZ and CONTROL_HZ must be > 0');
  if (!Number.isInteger(p.PHYSICS_HZ) || !Number.isInteger(p.CONTROL_HZ)) {
    throw new Error('PHYSICS_HZ and CONTROL_HZ must be integers');
  }
  if (p.PHYSICS_HZ % p.CONTROL_HZ !== 0) {
    throw new Error(`PHYSICS_HZ (${p.PHYSICS_HZ}) must be a multiple of CONTROL_HZ (${p.CONTROL_HZ})`);
  }
  p.PHYSICS_DT = 1 / p.PHYSICS_HZ;
  p.CONTROL_DT = 1 / p.CONTROL_HZ;
  p.CONTROL_DIVIDER = p.PHYSICS_HZ / p.CONTROL_HZ;
  return Object.freeze(p);
}
