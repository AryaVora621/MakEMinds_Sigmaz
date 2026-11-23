// hivecycle.js - bistable hive: the tip is an explicit, editable RULE plus a physical 1-DOF motion.
// Pure ES module: no DOM, no three.js. Units: inches, seconds, degrees at the API,
// radians inside. Pedro frame (x right, y toward far wall, z up).
//
// ANGLE CONVENTION (identical to app.js): angleDeg is the "shown" hive angle, the same
// numbers as stable_angles_deg: audience_up = -30, far_up = +30. app.js applies
// group.rotation.x = rad(angleDeg - stable_angles_deg[cad_pose_state]).
// State 'audience_up' means the AUDIENCE cell is the up-facing (loading) cell;
// 'far_up' means the FAR cell is up.
//
// FROM THE OFFICIAL MANUAL (Competition Manual TU08; section numbers per research R4):
//  - 8 / 9.6: the hive is bi-stable and holds its position until enough pollen or nectar
//    are LAUNCHED into the up-facing cell; then it tips.
//  - 10.3.1.B: each hive STARTS with 3 NECTAR already in its up cell.
//  - 10.5.1: a TIP = the hive moves from one stable state to the other (down cell becomes
//    up cell) and the damper that was not touching the frame now touches it.
//  - G417 / 10.5.1: launching into the up cell is the only legal way to tip.
//  - Table 10-2: each tip scores 20 (auto or teleop); balls left in the up cell at the end
//    score 2 each (scoring itself is NOT done here).
//  - CAD: stable angles are +-30 deg.
//  - ASSUMED (manual is silent, docs/overnight/hive-physics.md): balls in the cell that becomes the down cell drop out
//    through its opening (balls.js _releaseDrained does it physically; releaseOnTip mirrors it in this model).
//
// TIP RULE (SOURCED: project owner's observation of the real hive; NOT in the manual):
//  8 POLLEN tip a hive. The manual only says "enough" scoring elements. The old
//  ballsToTip = 4 was a wrong guess and is replaced by HIVE_TIP.threshold (= 8).
//  Source of the numbers and the nectar weight: web/js/hivetip_model.js (see its ledger).
//
// ASSUMPTIONS (NOT in the manual):
//  - mode 'weight' is the default. A tip happens when the summed weight of counted balls in the
//    up cell >= tipWeightThreshold (= HIVE_TIP.threshold = 8 pollen-equivalent units).
//    pollenWeight = 1 (unit scale, ASSUMED). nectarWeight = HIVE_TIP.nectarWeight
//    (1.75, INFERRED from team 14343 Chief Delphi tip table; AndyMark mass ratio 1.65 is too low).
//    So 8 pollen tip, and mixed loads tip when pollen + 1.75*nectar >= 8.
//  - mode 'count' still exists: tip when the number of counted balls in the up cell >= ballsToTip.
//    setRule({ballsToTip, mode: 'count'}) keeps the old count behaviour for callers that want it.
//  - DECIDED (ASSUMED): the 3 starting nectar are stored in the up cell with counts=false. They
//    sit in the cell but do NOT count toward the tip, so an empty up cell still needs 8 pollen
//    (the user's rule). Starting nectar is still stored, so it is drawn and it rotates.
//  - Only balls entering the CURRENTLY UP cell count. A ball into the down cell (or into a
//    hive that is mid-motion) is stored but never counts.
//  - After a tip the new up cell starts empty and needs a fresh threshold to tip back.
//  - releaseOnTip (default true): when true, balls in the cell that just became the down
//    cell are removed at the end of a tip. Does not affect counting.
//  - The tip motion (motionModel 'dynamic', default) is a 1-DOF rigid rotation about the pivot,
//    semi-implicit Euler at <= maxSubstepS (physics/hive-physics/tipdynamics.js). motionModel 'ease'
//    keeps the legacy animationDuration smoothstep (cosmetic, deterministic).
//  - setState() does not count as a tip and does not call onTip; it empties the up cell count.
//
// PHYSICS (all tunables live in physics/hive-physics/params.js, each tagged SOURCED / DERIVED / ASSUMED):
//  PHYSICAL now: gravitational bistable torque (ball weight torque from stacked resting positions, hive
//  self torque mgh*sin(theta)) with mgh CALIBRATED so 3 starting nectar + 7 pollen hold and 8 pollen tip
//  (the owner's observation, not a guessed hive mass); rigid rotation with viscous + Coulomb friction,
//  damper stops, drain past centre (asin(0.17) from balls.js DROP_SLOPE), optional capture impulse
//  (ball angular momentum about the pivot, torque mode only).
//  STILL ASSUMED (no source found): hive inertia (chosen so a tip takes nominalTipTimeS = 1.0 s), friction,
//  damper restitution, nectar mass pairing, stop rest speed. See params.js `source`.
//  tipModel 'rule' (default): the owner's 8-pollen rule decides WHEN the hive tips (authoritative);
//  the motion is physical with an assist torque that guarantees completion. tipModel 'torque': the hive
//  tips only if leave torque > static friction; a weak load can fail and fall back (no count clear, no onTip).
//
// STORAGE: a stored ball is {cell, type, counts, local:[y,z], massKg, rest:[ly,lz]}. local is its centre
// relative to the pivot in the hive body frame (frame at angleDeg = 0), so balls rotate with the hive.
// rest is the stacked resting position (cellstack.js, same body frame), restacked per cell on every change.

import { HIVE_TIP, shouldTip as modelShouldTip, tipWeight } from './hivetip_model.js';
import { hivePhysParams } from './physics/hive-physics/params.js';
import { stackPositions, ballRadiusIn, ballMassKg, restLocal } from './physics/hive-physics/cellstack.js';
import { TipBody, tipSubstep, loadTorqueNm, hiveTorqueNm, calibrateHive, IN_M } from './physics/hive-physics/tipdynamics.js';

export const HIVE_PARAMS = {
  ballsToTip: HIVE_TIP.threshold / HIVE_TIP.pollenWeight, // =8; SOURCED user: 8 pollen tip a hive; manual silent; the old 4 was a wrong guess
  mode: 'weight',         // 'count' | 'weight'. Weight: mixed nectar+pollen; pure pollen still needs 8
  pollenWeight: HIVE_TIP.pollenWeight,
  nectarWeight: HIVE_TIP.nectarWeight, // 1.75 (INFERRED, see hivetip_model.js)
  tipWeightThreshold: HIVE_TIP.threshold, // 8 pollen-equivalent units (SOURCED user fact)
  startingNectar: 3,      // manual 10.3.1.B: nectar stored in the up cell at reset (NOT counted, see header)
  releaseOnTip: true,     // ASSUMED (manual silent): balls in the cell that became the DOWN cell drop out; balls.js drops them physically
  animationDuration: 0.6, // s, smooth ease of a tip or an animated setState (motionModel 'ease' only)
  maxAngleDeg: 30,        // CAD stable angles
  ballRadius: 1.391,      // kept for callers; unused
  tipModel: 'rule',       // 'rule' (owner rule decides when) | 'torque' (leave torque > static friction decides)
  motionModel: 'dynamic', // 'dynamic' (rigid-body rotation) | 'ease' (legacy smoothstep)
  phys: hivePhysParams(), // physical constants + sources: physics/hive-physics/params.js
};

const DEG = Math.PI / 180;
const TIP_MODELS = ['rule', 'torque'];
const MOTION_MODELS = ['dynamic', 'ease'];
const STUCK_LIMIT_S = 2; // s; an unassisted motion that sits still this long is forced to the nearest stop (guard)

function centroid(pts) {
  const n = pts.length;
  let x = 0, y = 0, z = 0;
  for (const p of pts) { x += p[0]; y += p[1]; z += p[2]; }
  return [x / n, y / n, z / n];
}

export class HiveDynamics {
  constructor(hiveGroup, cellsByState, params = HIVE_PARAMS) {
    this.group = hiveGroup;
    this.cellsByState = cellsByState;
    this.p = { ...HIVE_PARAMS, ...params };
    this.p.phys = hivePhysParams(params && params.phys ? params.phys : {});
    this.pivot = hiveGroup.pivot;
    this.stable = hiveGroup.stable_angles_deg;
    this.onTip = null;
    this.reset();
  }

  /** Reset to the CAD pose state, with the starting nectar in the up cell, no stats. */
  reset() {
    const s = this.group.cad_pose_state || 'audience_up';
    this.state = s;
    this.theta = this.stable[s] * DEG;
    this.moving = false;
    this.scored = [];
    this._tips = 0;
    this._autoTips = null;
    this._scoredTotal = 0;
    this._anim = null; // legacy ease: {from, to, t, state, natural}
    this._mot = null;  // dynamic motion: {natural, assist:{dir,minNm}|null, stuckS}
    const ph = this.p.phys;
    this._stopRad = Math.abs(this.stable.audience_up) * DEG;
    // The physical model needs full cell geometry (opening/floor/back-wall corners). Synthetic or partial
    // cells (floor_corners only) cannot be stacked: then physics is off and motion falls back to 'ease'.
    this._physOk = true;
    try {
      this._calibration = calibrateHive({
        cellsByState: this.cellsByState,
        stableDeg: this.stable,
        pivot: this.pivot,
        startingNectar: this.p.startingNectar,
        params: ph,
      });
      this.body = new TipBody({
        mghNm: this._calibration.mghNm,
        viscousNmsPerRad: ph.viscousNmsPerRad,
        coulombNm: ph.coulombNm,
        staticNm: ph.staticNm,
        stopRad: this._stopRad,
        stopRestitution: ph.stopRestitution,
        restOmegaRadS: ph.restOmegaRadS,
      });
      this.body.latchAt(this._sideOf(s));
    } catch (err) {
      this._physOk = false;
      this._calibration = null;
      this.body = null;
    }
    const up = this.upCell;
    // ASSUMED decision: starting nectar sits in the up cell but does not count toward a tip.
    for (let i = 0; i < this.p.startingNectar; i++) this._store(up, null, 'nectar', false);
  }

  get angleDeg() {
    return this.moving ? this.theta / DEG : this.stable[this.state];
  }

  /** Name of the currently up-facing cell ('audience' or 'far'). */
  get upCell() { return this.state === 'far_up' ? 'far' : 'audience'; }

  /** Angular speed in deg/s (0 at rest). */
  get omegaDegS() { return this.moving && this._mot && this.body ? this.body.omega / DEG : 0; }

  /** Effective motion model: 'ease' when the cell geometry cannot support the physical model. */
  get _motionModel() { return this._physOk ? this.p.motionModel : 'ease'; }

  /** Calibration of the physical model (mgh, inertia, tip time, ...). */
  get calibration() { return this._calibration; }

  _sideOf(stateName) { return this.stable[stateName] < 0 ? -1 : 1; }
  _stateOfSide(side) { return side < 0 ? 'audience_up' : 'far_up'; }
  _upStateOfCell(cellName) { return cellName === 'far' ? 'far_up' : 'audience_up'; }

  _nearestState(theta = this.theta) { return theta < 0 ? 'audience_up' : 'far_up'; }

  _floorCentre(cellName) {
    const st = this._nearestState();
    const c = this.cellsByState[st].cells[cellName];
    return { world: centroid(c.floor_corners), angle: this.stable[st] * DEG };
  }

  _physOpts() {
    const ph = this.p.phys;
    return {
      gMps2: ph.G_MPS2,
      drainPastCentreRad: ph.drainPastCentreDeg * DEG,
      upSign: ph.upSign,
      includeBallInertia: ph.includeBallInertia,
    };
  }

  /** Recompute the stacked resting position of every record of one cell (insertion order). */
  _restack(cellName) {
    const recs = [];
    for (const b of this.scored) if (b.cell === cellName) recs.push(b);
    if (!recs.length || !this._physOk) return;
    const upState = this._upStateOfCell(cellName);
    const cell = this.cellsByState[upState].cells[cellName];
    const ph = this.p.phys;
    const radii = recs.map((b) => ballRadiusIn(b.type, ph));
    const pos = stackPositions(cell, radii, ph.stack);
    const ang = this.stable[upState] * DEG;
    for (let i = 0; i < recs.length; i++) recs[i].rest = restLocal(pos[i].pos, this.pivot, ang);
  }

  _store(cellName, worldPos, type, counts) {
    let w, ang;
    if (worldPos) { w = worldPos; ang = this.theta; }
    else { const f = this._floorCentre(cellName); w = f.world; ang = f.angle; }
    const dy = w[1] - this.pivot[1], dz = w[2] - this.pivot[2];
    const c = Math.cos(-ang), s = Math.sin(-ang);
    const rec = {
      cell: cellName, type, counts,
      local: [dy * c - dz * s, dy * s + dz * c],
      massKg: ballMassKg(type, this.p.phys),
      rest: [0, 0],
    };
    this.scored.push(rec);
    this._restack(cellName);
  }

  _weightOf(type) { return type === 'nectar' ? this.p.nectarWeight : this.p.pollenWeight; }

  /** Weight at which the hive tips in 'weight' mode. */
  get weightThreshold() {
    const p = this.p;
    if (typeof p.tipWeightThreshold === 'number') return p.tipWeightThreshold;
    return HIVE_TIP.threshold; // default: 8 pollen-equivalent units (SOURCED user fact)
  }

  _upCount() { let n = 0; for (const b of this.scored) if (b.counts) n++; return n; }
  _upWeight() { let w = 0; for (const b of this.scored) if (b.counts) w += this._weightOf(b.type); return w; }

  /** Counted balls in the up cell by type: 'nectar' is nectar, anything else is pollen. */
  _upCounts() {
    let pollen = 0, nectar = 0;
    for (const b of this.scored) {
      if (!b.counts) continue;
      if (b.type === 'nectar') nectar++; else pollen++;
    }
    return { pollen, nectar };
  }

  _shouldTip() {
    if (this.p.mode === 'weight') {
      return modelShouldTip(this._upCounts(), {
        ...HIVE_TIP,
        pollenWeight: this.p.pollenWeight,
        nectarWeight: this.p.nectarWeight,
        threshold: this.weightThreshold,
      });
    }
    return this._upCount() >= this.p.ballsToTip;
  }

  /** Per-hive tip progress in [0, 1]: up-cell weight / threshold (weight), or count / ballsToTip. */
  get tipProgress() {
    if (this.p.mode === 'weight') {
      const t = this.weightThreshold;
      return t > 0 ? Math.min(1, this._upWeight() / t) : 1;
    }
    const n = this.p.ballsToTip;
    return n > 0 ? Math.min(1, this._upCount() / n) : 1;
  }

  /** Change the tip rule at run time. Takes effect on the next ball launched. */
  setRule({ ballsToTip, mode, nectarWeight, pollenWeight, tipWeightThreshold, tipModel, motionModel } = {}) {
    if (ballsToTip !== undefined) this.p.ballsToTip = Math.max(1, Math.round(ballsToTip));
    if (mode !== undefined) { if (mode !== 'count' && mode !== 'weight') throw new Error('mode must be count or weight'); this.p.mode = mode; }
    if (nectarWeight !== undefined) this.p.nectarWeight = nectarWeight;
    if (pollenWeight !== undefined) this.p.pollenWeight = pollenWeight;
    if (tipWeightThreshold !== undefined) this.p.tipWeightThreshold = tipWeightThreshold;
    if (tipModel !== undefined) {
      if (!TIP_MODELS.includes(tipModel)) throw new Error('tipModel must be rule or torque');
      this.p.tipModel = tipModel;
    }
    if (motionModel !== undefined) {
      if (!MOTION_MODELS.includes(motionModel)) throw new Error('motionModel must be dynamic or ease');
      this.p.motionModel = motionModel;
    }
  }

  _ctx(assist) {
    return {
      params: this.p.phys,
      mghNm: this._calibration.mghNm,
      hiveInertiaKgM2: this._calibration.hiveInertiaKgM2,
      assist,
    };
  }

  /** Leave torque (N*m, positive = hive wants to leave its stop) of the latched state, minus nothing. */
  _leaveTorqueNm() {
    if (!this._physOk) return 0;
    const side = this._sideOf(this.state);
    const th = side * this._stopRad;
    const tau = loadTorqueNm(this.scored, th, this._physOpts()).tauNm + hiveTorqueNm(th, this._calibration.mghNm);
    return -side * tau;
  }

  /** A launched ball entered a cell. Only the currently up cell (hive at rest) counts. */
  addScoredBall(cellName, worldPos = null, type = 'pollen', opts = {}) {
    const wasMoving = this.moving;
    const counts = !this.moving && cellName === this.upCell;
    // Undefined, null or any unknown type is pollen. 'nectar' passes through unchanged.
    const kind = type === 'nectar' ? 'nectar' : 'pollen';
    this._store(cellName, worldPos, kind, counts);
    this._scoredTotal++;
    const dynamic = this._motionModel === 'dynamic';
    if (this.p.tipModel === 'torque' && dynamic) {
      if (wasMoving || cellName !== this.upCell) return;
      const ph = this.p.phys;
      const rec = this.scored[this.scored.length - 1];
      const v = opts && opts.velocity;
      if (ph.captureImpulse && v && worldPos) {
        const dy = worldPos[1] - this.pivot[1], dz = worldPos[2] - this.pivot[2];
        const L = rec.massKg * (dy * v[2] - dz * v[1]) * IN_M * IN_M;
        const load = loadTorqueNm(this.scored, this.theta, this._physOpts());
        const I = this._calibration.hiveInertiaKgM2 + load.inertiaKgM2;
        if (Number.isFinite(L) && I > 0) this.body.applyImpulse(L, I);
      }
      if (this.body.latched === 0 || this._leaveTorqueNm() > ph.staticNm) {
        this._startMotion(this.state === 'far_up' ? 'audience_up' : 'far_up', true, false);
      }
      return;
    }
    if (counts && this._shouldTip()) {
      const to = this.state === 'far_up' ? 'audience_up' : 'far_up';
      if (dynamic) this._startMotion(to, true, true);
      else this._startAnim(to, true);
    }
  }

  /** Ball weight torque about +x (positive = toward far_up); informational only. */
  torqueBalls(theta = this.theta) {
    const c = Math.cos(theta), s = Math.sin(theta);
    let t = 0;
    for (const b of this.scored) t += -386.09 * this._weightOf(b.type) * (b.local[0] * c - b.local[1] * s);
    return t;
  }

  /** Net torque the rule model applies: zero while latched (the hive holds its position). */
  torque() { return 0; }

  /** Physical net torque (N*m about +x, + = toward far_up): ball weight on stacked rest positions + hive mgh sin(theta). The stop reaction is not included. */
  netTorqueNm(theta = this.theta) {
    if (!this._physOk) return 0;
    return loadTorqueNm(this.scored, theta, this._physOpts()).tauNm + hiveTorqueNm(theta, this._calibration.mghNm);
  }

  _startAnim(to, natural) {
    this._anim = { from: this.theta, to: this.stable[to] * DEG, t: 0, state: to, natural };
    this.moving = true;
  }

  /** Start the dynamic (rigid-body) motion toward state `to`. assist: add the guaranteed-completion torque. */
  _startMotion(to, natural, assist) {
    const dir = this._sideOf(to);
    this.body.theta = this.theta;
    this.body.release();
    this._mot = {
      natural,
      assist: assist ? { dir, minNm: this._calibration.assistMinNm } : null,
      stuckS: 0,
    };
    this.moving = true;
  }

  _settleAt(stateName, natural) {
    const prev = this.state;
    this.theta = this.stable[stateName] * DEG;
    this.moving = false;
    this._anim = null;
    this._mot = null;
    this.state = stateName;
    if (this.body) this.body.latchAt(this._sideOf(stateName));
    for (const b of this.scored) b.counts = false; // the new up cell starts empty
    if (natural && prev !== stateName) {
      this._tips++;
      if (this.p.releaseOnTip) {
        const lowered = stateName === 'far_up' ? 'audience' : 'far';
        this.scored = this.scored.filter((b) => b.cell !== lowered);
      }
      this._restack('audience');
      this._restack('far');
      if (typeof this.onTip === 'function') this.onTip(stateName, this.scored.length);
    } else {
      this._restack('audience');
      this._restack('far');
    }
  }

  /** Dynamic motion reached a stop at `side`. */
  _settleDynamic(side) {
    const m = this._mot;
    const to = this._stateOfSide(side);
    if (m && m.natural && to === this.state) {
      // Failed physical tip: the hive fell back. Counts stay, no tip, no onTip.
      this.theta = this.stable[to] * DEG;
      this.moving = false;
      this._mot = null;
      this.body.latchAt(side);
      return;
    }
    this._settleAt(to, !!(m && m.natural));
  }

  step(dt) {
    if (!(dt > 0) || !Number.isFinite(dt)) return;
    if (this._motionModel === 'ease') {
      if (!this._anim) return;
      const a = this._anim;
      const dur = Math.max(1e-6, this.p.animationDuration);
      a.t = Math.min(dur, a.t + dt);
      const u = a.t / dur;
      if (u >= 1) { this._settleAt(a.state, a.natural); return; }
      const e = u * u * (3 - 2 * u); // smoothstep
      this.theta = a.from + (a.to - a.from) * e;
      return;
    }
    // dynamic
    if (this._anim) { // an ease started before motionModel was switched: finish it the legacy way
      const a = this._anim;
      const dur = Math.max(1e-6, this.p.animationDuration);
      a.t = Math.min(dur, a.t + dt);
      if (a.t >= dur) this._settleAt(a.state, a.natural);
      else { const u = a.t / dur; this.theta = a.from + (a.to - a.from) * u * u * (3 - 2 * u); }
      return;
    }
    dt = Math.min(dt, this.p.phys.maxStepS);
    if (!this.moving && this.p.tipModel === 'torque') {
      // forgetBall (or a drop-out) may have changed the load: re-check the leave torque.
      if (this._leaveTorqueNm() > this.p.phys.staticNm) {
        this._startMotion(this.state === 'far_up' ? 'audience_up' : 'far_up', true, false);
      }
    }
    if (!this.moving || !this._mot) return;
    const n = Math.max(1, Math.ceil(dt / this.p.phys.maxSubstepS));
    const h = dt / n;
    const ctx = this._ctx(this._mot.assist);
    for (let i = 0; i < n; i++) {
      const side = tipSubstep(this.body, this.scored, h, ctx);
      if (side !== 0) { this._settleDynamic(side); return; }
    }
    this.theta = this.body.theta;
    // Guard: an unassisted motion stuck in friction at an unstable point never ends by itself.
    if (!this._mot.assist) {
      if (Math.abs(this.body.omega) < 1e-6) this._mot.stuckS += dt; else this._mot.stuckS = 0;
      if (this._mot.stuckS > STUCK_LIMIT_S) this._settleDynamic(this.body.theta < 0 ? -1 : 1);
    }
  }

  /** Force a state. animate=false: instant. animate=true: moving; no onTip, no tip count. */
  setState(name, animate = false) {
    if (!(name in this.stable)) throw new Error('unknown hive state ' + name);
    if (!animate) { this._settleAt(name, false); return; }
    if (this.state === name && !this.moving) return;
    if (this._motionModel === 'dynamic') this._startMotion(name, false, true);
    else this._startAnim(name, false);
  }

  /** Forget up to n stored records of a cell (a ball left it physically: drop-out or recycle). Returns how many were removed. */
  forgetBall(cellName, n = 1) {
    let removed = 0;
    for (let i = this.scored.length - 1; i >= 0 && removed < n; i--) {
      if (this.scored[i].cell === cellName) { this.scored.splice(i, 1); removed++; }
    }
    if (removed) this._restack(cellName);
    return removed;
  }

  /** Mark the end of auto: tips so far are reported as tipsByAuto. */
  markAutoEnd() { this._autoTips = this._tips; }

  cycleStats() {
    const scoredInCell = { audience: 0, far: 0 };
    for (const b of this.scored) scoredInCell[b.cell] = (scoredInCell[b.cell] || 0) + 1;
    const up = this._upCounts();
    return {
      tips: this._tips,
      upCellCount: this._upCount(),
      upCellWeight: this._upWeight(),
      ballsToTip: this.p.ballsToTip,
      tipProgress: this.tipProgress,
      upPollen: up.pollen,
      upNectar: up.nectar,
      scoredTotal: this._scoredTotal,
      scoredInCell,
      tipsByAuto: this._autoTips,
      tipModel: this.p.tipModel,
      motionModel: this.p.motionModel,
      omegaDegS: this.omegaDegS,
      leaveTorqueNm: this._physOk ? this._leaveTorqueNm() - this.p.phys.staticNm : 0, // > 0 means the hive would leave its stop
      holdTorqueNm: this._physOk ? this._calibration.mghNm * Math.sin(this._stopRad) : 0,
    };
  }
}
