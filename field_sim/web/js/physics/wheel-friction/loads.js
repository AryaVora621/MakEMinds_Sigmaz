// loads.js - per-wheel normal load with d'Alembert load transfer (spec section 3).
// Pure numeric, no imports. SI units: m, s, N, m/s^2. Body frame +x forward, +y left.
// Wheel order FL, FR, BL, BR.

const G = 9.80665; // m/s^2, standard gravity (SOURCED: SI definition)
const PIVOT_EPS = 1e-9; // singular-matrix threshold (dimensionless, ASSUMED numeric guard)

// Scratch storage so computeNormalLoads does not allocate.
const _A = new Float64Array(16); // 4x4 row-major
const _b = new Float64Array(4);
const _x = new Float64Array(4);

// Solve the 4x4 system in place (Gaussian elimination, partial pivoting).
// Returns false if singular.
function gaussSolve4(A, b, x) {
  for (let c = 0; c < 4; c++) {
    let p = c;
    let best = Math.abs(A[c * 4 + c]);
    for (let r = c + 1; r < 4; r++) {
      const v = Math.abs(A[r * 4 + c]);
      if (v > best) { best = v; p = r; }
    }
    if (best < PIVOT_EPS) return false;
    if (p !== c) {
      for (let k = 0; k < 4; k++) {
        const t = A[c * 4 + k]; A[c * 4 + k] = A[p * 4 + k]; A[p * 4 + k] = t;
      }
      const tb = b[c]; b[c] = b[p]; b[p] = tb;
    }
    for (let r = c + 1; r < 4; r++) {
      const f = A[r * 4 + c] / A[c * 4 + c];
      if (f === 0) continue;
      for (let k = c; k < 4; k++) A[r * 4 + k] -= f * A[c * 4 + k];
      b[r] -= f * b[c];
    }
  }
  for (let r = 3; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < 4; k++) s -= A[r * 4 + k] * x[k];
    x[r] = s / A[r * 4 + r];
  }
  return true;
}

function solveInto(wheelPos, weightN, cgX, cgY, out) {
  // rows: sum N = W; sum N x = W cgX; sum N y = W cgY; N_FL - N_FR - N_BL + N_BR = 0
  for (let i = 0; i < 4; i++) {
    _A[i] = 1;
    _A[4 + i] = wheelPos[i][0];
    _A[8 + i] = wheelPos[i][1];
  }
  _A[12] = 1; _A[13] = -1; _A[14] = -1; _A[15] = 1;
  _b[0] = weightN; _b[1] = weightN * cgX; _b[2] = weightN * cgY; _b[3] = 0;
  if (!gaussSolve4(_A, _b, _x)) {
    for (let i = 0; i < 4; i++) out[i] = weightN / 4;
    return out;
  }
  for (let i = 0; i < 4; i++) out[i] = _x[i];
  return out;
}

/**
 * Static load solve. May return negative entries (no clamping).
 * @param {number[][]} wheelPos 4 x [x,y] in m, order FL,FR,BL,BR
 * @param {number} weightN total weight, N
 * @param {number} cgX effective CG x, m
 * @param {number} cgY effective CG y, m
 * @returns {Float64Array} loads, N
 */
export function solveStaticLoads(wheelPos, weightN, cgX, cgY) {
  return solveInto(wheelPos, weightN, cgX, cgY, new Float64Array(4));
}

/**
 * Normal loads including longitudinal/lateral load transfer.
 * @param {object} cfg DrivetrainConfig (reads wheels[i].pos, weightN, cgM, cgHeightM)
 * @param {number[]} accelBody [ax, ay] filtered body acceleration, m/s^2
 * @param {Float64Array|null} out optional Float64Array(4) to write into
 * @returns {{loads: Float64Array, tipping: boolean}} loads sum to cfg.weightN, all >= 0
 */
export function computeNormalLoads(cfg, accelBody, out = null) {
  const loads = out || new Float64Array(4);
  const W = cfg.weightN;
  const h = cfg.cgHeightM;
  const ax = accelBody ? accelBody[0] : 0;
  const ay = accelBody ? accelBody[1] : 0;
  const xe = cfg.cgM[0] - (ax * h) / G; // m
  const ye = cfg.cgM[1] - (ay * h) / G; // m
  const w = cfg.wheels;
  _pos4[0] = w[0].pos; _pos4[1] = w[1].pos; _pos4[2] = w[2].pos; _pos4[3] = w[3].pos;
  solveInto(_pos4, W, xe, ye, loads);

  let tipping = false;
  let posSum = 0;
  for (let i = 0; i < 4; i++) {
    if (!(loads[i] >= 0)) { tipping = true; loads[i] = 0; }
    posSum += loads[i];
  }
  if (tipping) {
    if (posSum > 0) {
      const s = W / posSum;
      for (let i = 0; i < 4; i++) loads[i] *= s;
    } else {
      for (let i = 0; i < 4; i++) loads[i] = W / 4;
    }
  }
  return { loads, tipping };
}

const _pos4 = [null, null, null, null];
