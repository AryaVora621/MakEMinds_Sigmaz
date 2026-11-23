// servo_plant.js - slew-limited servo plant for turret/hood plus flywheel (spec section 4). Pure ES module, no DOM.
// Units: degrees, deg/s, deg/s^2, seconds, rpm.
// Sourced: goBILDA 2000-0025-0002 (0.20 s/60 deg at 6 V = 300 deg/s no-load, 4 us deadband, 0.15 deg/us)
//   https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque/
// Everything else (accel, latency, backlash, turret rate, loaded-rate factor) is ASSUMED, see params.js PARAM_SOURCES.
import { PARAMS } from './params.js';
import { rpmAfterShot } from './exit_model.js';

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export class SlewServo {
  constructor(spec) {
    this.spec = {
      deadbandDeg: 0, backlashDeg: 0, latencyS: 0, quantDeg: 0,
      settleTolDeg: 1, settleVelDegS: 20, settleTimeS: 0.05, initDeg: 0, ...spec,
    };
    this.reset(this.spec.initDeg);
  }
  reset(posDeg) {
    const s = this.spec, p = clamp(posDeg, s.minDeg, s.maxDeg);
    this._inner = p; this._out = p; this._vel = 0; this._held = p;
    this._t = 0; this._fifo = []; this._cmdLatest = p;
    this._settleT = 0; this._settled = false; this._atLimit = null;
  }
  step(dt, cmdDeg) {
    if (!(dt >= 0)) dt = 0;
    const s = this.spec;
    this._t += dt;
    if (cmdDeg != null && Number.isFinite(cmdDeg)) {
      let c = cmdDeg;
      if (s.quantDeg > 0) c = Math.round(c / s.quantDeg) * s.quantDeg;   // 1 quantize
      c = clamp(c, s.minDeg, s.maxDeg);                                  // 2 clamp
      this._cmdLatest = c;
      this._fifo.push([this._t, c]);                                     // 3 latency FIFO
    }
    let eff = null;
    const tEdge = this._t - s.latencyS;
    let k = -1;
    for (let i = 0; i < this._fifo.length; i++) { if (this._fifo[i][0] <= tEdge + 1e-12) k = i; else break; }
    if (k >= 0) { eff = this._fifo[k][1]; if (k > 0) this._fifo.splice(0, k); }
    if (eff != null && Math.abs(eff - this._held) > s.deadbandDeg / 2) this._held = eff;   // 4 deadband
    // 5 trapezoid (rate + accel limited, never overshoots the held target)
    const e = this._held - this._inner;
    if (!(e === 0 && this._vel === 0)) {
      const sg = Math.sign(e);
      // spec: vDes = sign(e)*min(maxRate, sqrt(2*a*|e|)). The continuous sqrt profile is too steep for a fixed step near the
      // target (the velocity cannot follow it and is snapped to 0 at ~a*dt*many deg/s), so use its discrete-time form:
      // the largest v that n decel steps of a*dt can still stop within |e|; it tends to sqrt(2*a*|e|) as dt -> 0.
      const adt = s.maxAccelDegS2 * dt;
      const vStop = adt > 0
        ? adt * (Math.sqrt(1 + 8 * Math.abs(e) / (dt * adt)) - 1) / 2
        : Math.sqrt(2 * s.maxAccelDegS2 * Math.abs(e));
      const vDes = sg * Math.min(s.maxRateDegS, vStop);
      const dv = clamp(vDes - this._vel, -s.maxAccelDegS2 * dt, s.maxAccelDegS2 * dt);
      this._vel += dv;
      this._inner += this._vel * dt;
      const e2 = this._held - this._inner;
      if (e2 === 0 || (e !== 0 && Math.sign(e2) !== sg)) { this._inner = this._held; this._vel = 0; }
    }
    // 6 backlash
    const b = s.backlashDeg;
    if (this._inner - this._out > b / 2) this._out = this._inner - b / 2;
    if (this._out - this._inner > b / 2) this._out = this._inner + b / 2;
    // 7 clamp
    this._out = clamp(this._out, s.minDeg, s.maxDeg);
    this._atLimit = this._out <= s.minDeg ? 'min' : this._out >= s.maxDeg ? 'max' : null;
    // 8 settled timer
    if (Math.abs(this._out - this._cmdLatest) <= s.settleTolDeg && Math.abs(this._vel) <= s.settleVelDegS) {
      this._settleT += dt;
    } else this._settleT = 0;
    this._settled = this._settleT >= s.settleTimeS;
    return { pos: this._out, vel: this._vel, settled: this._settled, atLimit: this._atLimit };
  }
  get pos() { return this._out; }
  get vel() { return this._vel; }
  get settled() { return this._settled; }
}

/** Turret servo spec (deg). Range from the shooter spec; dynamics ASSUMED (Axon specs not found). */
export function turretServoSpec(sh, params = PARAMS) {
  const t = params.turretServo;
  return {
    minDeg: sh.turretMinDeg, maxDeg: sh.turretMaxDeg,
    maxRateDegS: t.MAX_RATE_DEG_S, maxAccelDegS2: t.MAX_ACCEL_DEG_S2,
    deadbandDeg: t.DEADBAND_DEG, backlashDeg: t.BACKLASH_DEG, latencyS: t.LATENCY_S, quantDeg: t.QUANT_DEG,
    settleTolDeg: t.SETTLE_TOL_DEG, settleVelDegS: t.SETTLE_VEL_DEG_S, settleTimeS: t.SETTLE_TIME_S,
    initDeg: 0,
  };
}

/** Hood servo spec in HOOD degrees (servo degrees divided by LINK_RATIO). */
export function hoodServoSpec(sh, params = PARAMS) {
  const h = params.hoodServo, link = params.hood.LINK_RATIO;
  return {
    minDeg: sh.hoodMin, maxDeg: sh.hoodMax,
    maxRateDegS: h.NOLOAD_RATE_DEG_S * h.LOAD_FACTOR / link,
    maxAccelDegS2: h.MAX_ACCEL_DEG_S2 / link,
    deadbandDeg: h.DEADBAND_US * h.DEG_PER_US / link,
    backlashDeg: h.BACKLASH_DEG / link, latencyS: h.LATENCY_S, quantDeg: h.DEG_PER_US / link,
    settleTolDeg: h.SETTLE_TOL_DEG, settleVelDegS: h.SETTLE_VEL_DEG_S, settleTimeS: h.SETTLE_TIME_S,
    initDeg: sh.hoodMin,
  };
}

const DEFAULT_PLANT_V2 = {
  turretMinDeg: -180, turretMaxDeg: 180, hoodMinDeg: 25, hoodMaxDeg: 65, maxRpm: 6000,
  turretCableWrap: false,
};

export class ShooterPlantV2 {
  constructor(spec = {}) {
    this.spec = {
      ...DEFAULT_PLANT_V2,
      flywheelTauS: PARAMS.flywheel.TAU_S, flywheelRecoverTauS: PARAMS.flywheel.RECOVER_TAU_S,
      rpmDipFrac: PARAMS.flywheel.RPM_DIP_FRAC_LEGACY, kind: 'pollen',
      ...spec,
    };
    const s = this.spec;
    this.turret = new SlewServo(s.turretServo || turretServoSpec({ turretMinDeg: s.turretMinDeg, turretMaxDeg: s.turretMaxDeg }));
    this.hood = new SlewServo(s.hoodServo || hoodServoSpec({ hoodMin: s.hoodMinDeg, hoodMax: s.hoodMaxDeg }));
    this.state = { turretDeg: this.turret.pos, hoodDeg: this.hood.pos, rpm: 0, turretRateDegS: 0, hoodRateDegS: 0 };
    this.atLimit = null;
    this._dipT = 0;
    this._dipFrac = s.rpmDipFrac;
  }
  step(dt, cmd = {}) {
    if (!(dt >= 0)) dt = 0;
    const s = this.spec, st = this.state;
    const fin = (v) => (v != null && Number.isFinite(v) ? v : null);
    const tr = this.turret.step(dt, fin(cmd.turretDeg));
    st.turretDeg = tr.pos; st.turretRateDegS = tr.vel; this.atLimit = tr.atLimit;
    const hd = this.hood.step(dt, fin(cmd.hoodDeg));
    st.hoodDeg = hd.pos; st.hoodRateDegS = hd.vel;
    if (fin(cmd.rpm) != null) {
      const tgt = clamp(cmd.rpm, 0, s.maxRpm);
      // same dipped-recovery logic as aimcontrol.ShooterPlant; dip fraction follows the last kick
      const dipped = this._dipT > 0 && st.rpm >= tgt * (1 - 2 * this._dipFrac - 0.02);
      const a = 1 - Math.exp(-dt / Math.max(1e-6, dipped ? s.flywheelRecoverTauS : s.flywheelTauS));
      if (this._dipT > 0) this._dipT = Math.max(0, this._dipT - dt);
      st.rpm += (tgt - st.rpm) * a;
    }
    st.rpm = clamp(st.rpm, 0, s.maxRpm);
    return { atTurretLimit: tr.atLimit, atHoodLimit: hd.atLimit };
  }
  kick(kind) {
    const s = this.spec, st = this.state, before = st.rpm;
    if (s.exitModel === 'physical') {
      st.rpm = rpmAfterShot(st.rpm, kind || s.kind, { model: 'physical', ...(s.exitOpts || {}) });
      this._dipFrac = before > 0 ? Math.max(0, 1 - st.rpm / before) : s.rpmDipFrac;
    } else {
      st.rpm *= (1 - s.rpmDipFrac);
      this._dipFrac = s.rpmDipFrac;
    }
    this._dipT = 0.6;
  }
  atSetpoint(cmd, tol = {}) {
    const t = { turretDeg: 1.5, hoodDeg: 1, rpm: 50, ...tol };
    const st = this.state;
    const ok = (c, v, tl) => c == null || Math.abs(c - v) <= tl;
    return {
      turret: ok(cmd.turretDeg, st.turretDeg, t.turretDeg),
      hood: ok(cmd.hoodDeg, st.hoodDeg, t.hoodDeg),
      rpm: ok(cmd.rpm, st.rpm, t.rpm),
    };
  }
  settled() { return { turret: this.turret.settled, hood: this.hood.settled }; }
}
