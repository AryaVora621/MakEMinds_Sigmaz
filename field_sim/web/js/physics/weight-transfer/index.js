// Weight-transfer facade: composes params, massmodel, loads, traction, compliance, tipover.
// Units: inches, lbm, seconds, radians. Robot frame +x fwd, +y left, z up. Wheels FL,FR,BL,BR.
// Provenance of every constant lives in params.js; this file holds no tunables.
import { G, makeParams, wheelGeometry } from './params.js';
import { computeCG } from './massmodel.js';
import { bodyAccel, wheelLoads } from './loads.js';
import { limitBodyVelocity, loadAwareLimits } from './traction.js';
import { ChassisCompliance } from './compliance.js';
import { TipState } from './tipover.js';

export { G, makeParams } from './params.js';

const clampAbs = (v, m) => (v > m ? m : v < -m ? -m : v);

// column-major 4x4 helpers (Float64Array(16))
function ident() { const m = new Float64Array(16); m[0] = m[5] = m[10] = m[15] = 1; return m; }
function mul(A, B) {
  const o = new Float64Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[k * 4 + r] * B[c * 4 + k];
      o[c * 4 + r] = s;
    }
  }
  return o;
}
function trans(x, y, z) { const m = ident(); m[12] = x; m[13] = y; m[14] = z; return m; }
function rotX(t) { const m = ident(), c = Math.cos(t), s = Math.sin(t); m[5] = c; m[9] = -s; m[6] = s; m[10] = c; return m; }
function rotY(t) { const m = ident(), c = Math.cos(t), s = Math.sin(t); m[0] = c; m[8] = s; m[2] = -s; m[10] = c; return m; }

export class WeightTransferSim {
  constructor(spec = null, overrides = {}) {
    this._params = makeParams(overrides, spec);
    this._geom = wheelGeometry(this._params);
    this._mech = { turretRad: 0, ballsHeld: 0, intakeDeployed: 0 };
    this._cg = computeCG(this._params, this._mech);
    this.compliance = new ChassisCompliance(this._params, this._cg);
    this.tip = new TipState(this._params);
    this.reset();
  }

  reset() {
    this.compliance.reset();
    this.tip.reset();
    this._prevV = [0, 0, 0];
    this._acc = { ax: 0, ay: 0, alpha: 0 };
    this._scale = 1;
    this._slip = [false, false, false, false];
    this._impactPeakLbf = 0;
    this._loads = wheelLoads(this._params, this._cg, 0, 0);
    this._snap = this._makeSnapshot(this._loads);
  }

  setMechState(m = {}) {
    if (m.turretRad !== undefined) this._mech.turretRad = m.turretRad;
    if (m.ballsHeld !== undefined) this._mech.ballsHeld = m.ballsHeld;
    if (m.intakeDeployed !== undefined) this._mech.intakeDeployed = m.intakeDeployed;
    this._cg = computeCG(this._params, this._mech);
    this.compliance.setCG(this._cg);
  }

  get cg() { return this._cg; }
  get params() { return this._params; }
  get snapshot() { return this._snap; }

  limitDrive(prevBodyV, wantBodyV, dt, heading) {
    const r = limitBodyVelocity(this._params, this._cg, prevBodyV, wantBodyV, dt, {
      lifted: this.tip.snapshot.lifted,
      antiTip: this._params.tip.antiTip ?? false,
    });
    const c = Math.cos(heading || 0), s = Math.sin(heading || 0);
    const [vf, vl, om] = r.bodyV;
    this._scale = r.scale;
    this._slip = r.slip;
    return {
      bodyV: r.bodyV, vx: c * vf - s * vl, vy: s * vf + c * vl, om,
      scale: r.scale, sliding: r.sliding, slip: r.slip,
    };
  }

  onCollision({ normals = [], vBefore, h = 0 }) {
    let vx = 0, vy = 0;
    for (const n of normals) {
      const d = vBefore.vx * n[0] + vBefore.vy * n[1];
      if (d < 0) { vx += d * n[0]; vy += d * n[1]; }
    }
    const c = Math.cos(h), s = Math.sin(h);
    const dvBody = { x: c * vx + s * vy, y: -s * vx + c * vy };
    const p = this._params, cg = this._cg;
    const r = this.tip.applyImpact(dvBody, cg);
    const lever = cg.z - (1 + p.tip.restitutionWall) * p.tip.contactZIn;
    this.compliance.kick(
      (cg.massLb * dvBody.x * lever) / cg.Iyy,
      (-cg.massLb * dvBody.y * lever) / cg.Ixx,
    );
    const ct = p.impact.contactTimeS;
    const l = wheelLoads(p, cg, -dvBody.x / ct, -dvBody.y / ct);
    this._impactPeakLbf = Math.max(...l.lbf);
    return { dvBody, impactPeakLbf: this._impactPeakLbf, tipStarted: !!(r && r.started) };
  }

  step(dt, bodyV) {
    const p = this._params, cg = this._cg;
    const raw = bodyAccel(this._prevV, bodyV, dt);
    const cl = p.accel.clampInS2;
    const af = dt > 0 ? dt / (p.accel.lpfTauS + dt) : 1;
    const t = this._acc;
    t.ax += af * (clampAbs(raw.ax, cl) - t.ax);
    t.ay += af * (clampAbs(raw.ay, cl) - t.ay);
    t.alpha += af * (clampAbs(raw.alpha, cl) - t.alpha);
    const loads = wheelLoads(p, cg, t.ax, t.ay);
    this._loads = loads;
    this.tip.step(dt, { cg, loads, ax: t.ax, ay: t.ay });
    this.compliance.step(dt, { ax: t.ax, ay: t.ay });
    this._prevV = [bodyV[0], bodyV[1], bodyV[2]];
    this._snap = this._makeSnapshot(loads);
    return this._snap;
  }

  _makeSnapshot(loads) {
    const cs = this.compliance.state, tip = this.tip.snapshot;
    return {
      cg: this._cg,
      accel: { ax: this._acc.ax, ay: this._acc.ay, alpha: this._acc.alpha },
      loads, scale: this._scale, slip: this._slip,
      pitch: cs.pitch, roll: cs.roll, heave: cs.heave,
      tip, disabled: tip.disabled, impactPeakLbf: this._impactPeakLbf,
    };
  }

  visualMatrix() {
    const cs = this.compliance.state, tip = this.tip.snapshot, g = this._geom;
    let m = trans(0, 0, cs.heave);
    if (tip.pivot && tip.angle) {
      const pv = tip.pivot, ang = tip.angle;
      let R;
      switch (tip.edge) {
        case 'front': R = rotY(ang); break;
        case 'rear': R = rotY(-ang); break;
        case 'left': R = rotX(-ang); break;
        default: R = rotX(ang); break; // right
      }
      m = mul(m, mul(trans(pv.x, pv.y, 0), mul(R, trans(-pv.x, -pv.y, 0))));
    }
    const tilt = mul(rotY(cs.pitch), rotX(cs.roll));
    m = mul(m, mul(trans(g.xc, g.yc, 0), mul(tilt, trans(-g.xc, -g.yc, 0))));
    return m;
  }

  pathLimits() { return loadAwareLimits(this._params, this._cg); }
}
