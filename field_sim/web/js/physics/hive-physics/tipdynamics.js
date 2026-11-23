// tipdynamics.js - hive bistable tip dynamics (pure ES module, no DOM).
// Spec: HIVE-PHYSICS SPEC v1 Sec 2 (torque model), 3 (calibration), 5 (tip body).
//
// Units: inches in the records (rest = [ly, lz] body-frame offsets from the pivot, inch),
// SI inside (kg, m, N*m, rad, s). Torque about +x, positive pushes theta toward far_up.
//
// SOURCED: gravity torque of a point mass about a horizontal axis (basic statics).
// DERIVED: mgh and pollenStepNm come from the owner's observation "8 pollen tip, 7 do not"
//   (hivetip_model.js) via calibrateHive(). Hive inertia is bisected so the assisted tip
//   takes params.nominalTipTimeS.
// ASSUMED (see params.js .source): viscous / coulomb / static friction, stop restitution,
//   nominal tip time, point-mass balls, assist rule.
//
// Only state: the calibration cache Map below.

import { HIVE_PHYS } from './params.js';
import { stackPositions, ballMassKg, ballRadiusIn, restLocal } from './cellstack.js';

export const IN_M = 0.0254; // m per inch (exact, SI definition)
const DEG = Math.PI / 180;
const UP_STATE = { audience: 'audience_up', far: 'far_up' };

// ---------------------------------------------------------------- loads

/**
 * Gravity torque (N*m about +x) and inertia (kg m^2 about pivot) of a set of ball records.
 * Records in a cell that is draining (theta*upSign[cell] < -drainPastCentreRad) contribute 0.
 */
export function loadTorqueNm(records, thetaRad, opts = {}) {
  const g = opts.gMps2 ?? HIVE_PHYS.G_MPS2;
  const drain = opts.drainPastCentreRad ?? HIVE_PHYS.drainPastCentreDeg * DEG;
  const upSign = opts.upSign ?? HIVE_PHYS.upSign;
  const withI = opts.includeBallInertia ?? HIVE_PHYS.includeBallInertia;
  const c = Math.cos(thetaRad), s = Math.sin(thetaRad);
  let tau = 0, inertia = 0, n = 0;
  for (let i = 0; i < records.length; i++) {
    const r = records[i];
    const us = upSign[r.cell];
    if (us !== undefined && thetaRad * us < -drain) continue; // draining
    const ly = r.rest[0], lz = r.rest[1];
    const yw = ly * c - lz * s; // world y offset from pivot, inch
    tau += -r.massKg * g * yw * IN_M;
    if (withI) inertia += r.massKg * (ly * ly + lz * lz) * IN_M * IN_M;
    n++;
  }
  return { tauNm: tau, inertiaKgM2: inertia, nActive: n };
}

/** Hive self torque mgh*sin(theta) in N*m (mghNm = m*g*h in N*m). */
export function hiveTorqueNm(thetaRad, mghNm) {
  return mghNm * Math.sin(thetaRad);
}

/**
 * Rule-assist torque (N*m) in direction dir (+1/-1): fills whatever is missing so the
 * drive beyond coulomb friction is at least assistMinNm for the whole motion.
 */
export function assistTorqueNm(dir, tauLoadNm, thetaRad, mghNm, coulombNm, assistMinNm) {
  const have = dir * (tauLoadNm + mghNm * Math.sin(thetaRad));
  return dir * Math.max(0, assistMinNm + coulombNm - have);
}

// ---------------------------------------------------------------- tip body

export class TipBody {
  /** p: { mghNm, viscousNmsPerRad, coulombNm, staticNm, stopRad, stopRestitution, restOmegaRadS } */
  constructor(p) {
    this.p = p;
    this.theta = -p.stopRad;
    this.omega = 0;
    this.latched = 0;
  }

  latchAt(side) {
    this.theta = side * this.p.stopRad;
    this.omega = 0;
    this.latched = side;
  }

  release() { this.latched = 0; }

  /** Torque trying to push the hive off its stop (N*m, >0 leaves). 0 when not latched. */
  leaveTorqueNm(tauExtNm) {
    if (!this.latched) return 0;
    return (-this.latched) * (tauExtNm + this.p.mghNm * Math.sin(this.theta));
  }

  /** One semi-implicit Euler step. Returns the side latched this step (-1/+1) or 0. */
  substep(h, tauExtNm, inertiaKgM2) {
    if (this.latched) return this.latched;
    const p = this.p;
    const I = inertiaKgM2 > 1e-9 ? inertiaKgM2 : 1e-9;
    const tauDrive = tauExtNm + p.mghNm * Math.sin(this.theta);
    const w0 = this.omega;
    let tau;
    if (Math.abs(w0) > 1e-6) {
      tau = tauDrive - p.viscousNmsPerRad * w0 - p.coulombNm * Math.sign(w0);
    } else if (Math.abs(tauDrive) <= p.staticNm) {
      tau = 0;
      this.omega = 0;
    } else {
      tau = tauDrive - p.coulombNm * Math.sign(tauDrive);
    }
    this.omega += (tau / I) * h;
    // friction may only stop motion, never reverse it (avoids sign chatter)
    if (w0 !== 0 && this.omega * w0 < 0 && Math.abs(tauDrive) <= p.staticNm) this.omega = 0;
    this.theta += this.omega * h;

    const e = p.stopRestitution;
    if (this.theta <= -p.stopRad) {
      this.theta = -p.stopRad;
      if (this.omega < 0) this.omega = -e * this.omega;
      if (Math.abs(this.omega) <= p.restOmegaRadS && (-1) * tauDrive >= -p.staticNm) {
        this.latched = -1; this.omega = 0; return -1;
      }
    } else if (this.theta >= p.stopRad) {
      this.theta = p.stopRad;
      if (this.omega > 0) this.omega = -e * this.omega;
      if (Math.abs(this.omega) <= p.restOmegaRadS && (+1) * tauDrive >= -p.staticNm) {
        this.latched = 1; this.omega = 0; return 1;
      }
    }
    return 0;
  }

  /** Angular impulse L (N*m*s) about the pivot. A stop absorbs impulses into it. */
  applyImpulse(L, I) {
    const Ie = I > 1e-9 ? I : 1e-9;
    this.omega += L / Ie;
    if (this.latched) {
      if (Math.sign(this.omega) === -this.latched) this.latched = 0; // points away from stop
      else this.omega = 0; // absorbed by the stop
    }
  }
}

// ---------------------------------------------------------------- shared substep

function loadOptsFor(params) {
  return {
    gMps2: params.G_MPS2,
    drainPastCentreRad: params.drainPastCentreDeg * DEG,
    upSign: params.upSign,
    includeBallInertia: params.includeBallInertia,
  };
}

/**
 * THE shared substep (HiveDynamics and calibration both use it).
 * ctx: { params, mghNm, hiveInertiaKgM2, assist: null | { dir, minNm } }
 */
export function tipSubstep(body, records, h, ctx) {
  const lo = ctx._loadOpts || (ctx._loadOpts = loadOptsFor(ctx.params));
  const load = loadTorqueNm(records, body.theta, lo);
  let tauExt = load.tauNm;
  if (ctx.assist) {
    tauExt += assistTorqueNm(ctx.assist.dir, load.tauNm, body.theta, ctx.mghNm,
      ctx.params.coulombNm, ctx.assist.minNm);
  }
  return body.substep(h, tauExt, ctx.hiveInertiaKgM2 + load.inertiaKgM2);
}

// ---------------------------------------------------------------- records

/** Stacked resting ball records of one cell, in the cell's up-state frame. */
export function buildCellRecords(cellsByState, cellName, types, pivot, stableDeg, params = HIVE_PHYS) {
  const upState = UP_STATE[cellName];
  if (!upState) throw new Error(`buildCellRecords: unknown cell '${cellName}'`);
  const stateObj = cellsByState && cellsByState[upState];
  const cell = stateObj && stateObj.cells && stateObj.cells[cellName];
  if (!cell) throw new Error(`buildCellRecords: no cell '${cellName}' in state '${upState}'`);
  const radii = types.map((t) => ballRadiusIn(t, params));
  const pos = stackPositions(cell, radii, params.stack);
  const ang = stableDeg[upState] * DEG;
  return types.map((type, i) => ({
    cell: cellName,
    type,
    massKg: ballMassKg(type, params),
    rest: restLocal(pos[i].pos, pivot, ang),
    overflow: pos[i].overflow,
  }));
}

// ---------------------------------------------------------------- simulation

function bodyParams(params, mghNm, stopRad) {
  return {
    mghNm,
    viscousNmsPerRad: params.viscousNmsPerRad,
    coulombNm: params.coulombNm,
    staticNm: params.staticNm,
    stopRad,
    stopRestitution: params.stopRestitution,
    restOmegaRadS: params.restOmegaRadS,
  };
}

/**
 * Run tipSubstep from a latched start side (released) until latched or maxTimeS.
 * assistMinNm null = no assist. stopRad is an optional extension (default 30 deg).
 */
export function simulateTip({ records, params = HIVE_PHYS, mghNm, hiveInertiaKgM2, startSide,
  assistMinNm = null, maxTimeS = 10, stopRad = 30 * DEG }) {
  const body = new TipBody(bodyParams(params, mghNm, stopRad));
  body.latchAt(startSide);
  body.release();
  const ctx = {
    params, mghNm, hiveInertiaKgM2,
    assist: assistMinNm === null || assistMinNm === undefined
      ? null : { dir: -startSide, minNm: assistMinNm },
  };
  const h = params.maxSubstepS;
  const nMax = Math.ceil(maxTimeS / h);
  let t = 0, finalSide = 0, maxOmega = 0;
  for (let i = 0; i < nMax; i++) {
    const side = tipSubstep(body, records, h, ctx);
    t += h;
    const aw = Math.abs(body.omega);
    if (aw > maxOmega) maxOmega = aw;
    if (side !== 0) { finalSide = side; break; }
  }
  return { timeS: t, finalSide, maxOmegaRadS: maxOmega };
}

// ---------------------------------------------------------------- calibration

const calCache = new Map();

/**
 * Calibrate mgh from "calibration.holdPollen hold, tipPollen tip", then bisect inertia so the
 * assisted tip takes nominalTipTimeS. Cached by JSON key.
 */
export function calibrateHive({ cellsByState, stableDeg, pivot, startingNectar, params = HIVE_PHYS }) {
  const cal = params.calibration;
  const calCell = cal.calCell, calState = cal.calState;
  const cellObj = cellsByState && cellsByState[calState] && cellsByState[calState].cells
    && cellsByState[calState].cells[calCell];
  const key = JSON.stringify({
    pivot, stableDeg, startingNectar, cellObj,
    p: [params.G_MPS2, params.ballMassKg, params.ballRadiusIn, cal, params.nominalTipTimeS,
      params.hiveInertiaKgM2, params.inertiaSearchKgM2,
      params.viscousNmsPerRad, params.coulombNm, params.staticNm, params.stopRestitution,
      params.restOmegaRadS, params.maxSubstepS, params.drainPastCentreDeg, params.includeBallInertia,
      params.upSign, params.stack],
  });
  const hit = calCache.get(key);
  if (hit) return { ...hit };

  const stopRad = Math.abs(stableDeg[calState]) * DEG;
  const sinStop = Math.sin(stopRad);
  if (!(sinStop > 0)) throw new Error('calibrateHive: stop angle must be > 0');
  const side = Math.sign(stableDeg[calState]) || -1;
  const dir = -side; // leave direction (+1 for audience_up at -30 deg)

  const recsFor = (nPollen) => {
    const types = [];
    if (cal.includeStartingNectar) for (let i = 0; i < startingNectar; i++) types.push('nectar');
    for (let i = 0; i < nPollen; i++) types.push('pollen');
    return buildCellRecords(cellsByState, calCell, types, pivot, stableDeg, params);
  };
  const lo = loadOptsFor(params);
  const tauAt = (recs) => dir * loadTorqueNm(recs, side * stopRad, lo).tauNm;
  const rec7 = recsFor(cal.holdPollen), rec8 = recsFor(cal.tipPollen);
  const tau7 = tauAt(rec7), tau8 = tauAt(rec8);
  if (!(tau8 > tau7)) {
    throw new Error(`calibrateHive: tau(${cal.tipPollen} pollen)=${tau8} <= tau(${cal.holdPollen} pollen)=${tau7}; stack not monotone`);
  }
  const tauCal = (tau7 + tau8) / 2;
  const mghNm = (tauCal - params.staticNm) / sinStop;
  const pollenStepNm = tau8 - tau7;
  const assistMinNm = pollenStepNm / 2;

  let I, tipTimeS, atBound = false;
  const [iLo, iHi] = params.inertiaSearchKgM2;
  const sim = (Ih) => simulateTip({
    records: rec8, params, mghNm, hiveInertiaKgM2: Ih, startSide: side, assistMinNm, stopRad,
  });
  const tipTime = (Ih) => {
    const r = sim(Ih);
    return r.finalSide === -side ? r.timeS : Infinity;
  };
  if (typeof params.hiveInertiaKgM2 === 'number') {
    I = params.hiveInertiaKgM2;
    tipTimeS = sim(I).timeS;
  } else {
    // monotone: time increases with I; a run that does not reach the far stop counts as too slow
    let a = Math.log(iLo), b = Math.log(iHi);
    for (let k = 0; k < 50; k++) {
      const m = 0.5 * (a + b);
      if (tipTime(Math.exp(m)) > params.nominalTipTimeS) b = m; else a = m;
    }
    I = Math.exp(0.5 * (a + b));
    tipTimeS = sim(I).timeS;
    atBound = Math.abs(I - iLo) <= 0.01 * iLo || Math.abs(I - iHi) <= 0.01 * iHi;
  }

  const out = {
    mghNm, tau7Nm: tau7, tau8Nm: tau8, pollenStepNm, assistMinNm,
    holdTorqueNm: mghNm * sinStop, hiveInertiaKgM2: I, tipTimeS, inertiaAtBound: atBound,
  };
  calCache.set(key, out);
  return { ...out };
}
