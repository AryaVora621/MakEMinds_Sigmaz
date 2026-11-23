// FrictionMecanumDrive: drop-in adapter with the same step() contract as driver.js MecanumDrive.
// Converts sim units (in, in/s, Pedro world frame) to/from the SI drivetrain model.
import { driverForwardOffset } from '../../driver.js';
import { makeParams, UNITS } from './params.js';
import { buildDrivetrainConfig } from './geometry.js';
import { mixWheelPowers } from './kinematics.js';
import { WheelFrictionDrivetrain } from './drivetrain.js';

const IN_TO_M = UNITS.IN_TO_M; // 0.0254 m per inch
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const BODYV_TOL = 1e-9; // in/s, tolerance for detecting external edits of state.bodyV

export class FrictionMecanumDrive {
  /** @param {{spec?:object|null, params?:object|null, alliance?:string|null, slip?:number}} opts */
  constructor(opts = {}) {
    const params = makeParams(opts.params || {});
    this.params = params;
    this.cfg = buildDrivetrainConfig(opts.spec ?? null, params);
    this.drivetrain = new WheelFrictionDrivetrain(this.cfg);
    this.alliance = opts.alliance ?? null;
    this._slip = 0;
    this._last = null;        // last bodyV written [in/s, in/s, rad/s]
    this._powers = new Float64Array(4);
    this.slip = opts.slip ?? 0;
  }

  get slip() { return this._slip; }
  set slip(v) {
    this._slip = clamp(+v || 0, 0, 1);
    const f = this.cfg.friction;
    this.drivetrain.setMuScale(f.gripScale * (1 - f.slipKnobGripLoss * this._slip));
  }

  step(cmd, dt, state, { fieldCentric = false, headingOffset: extraOffset = 0, alliance = this.alliance, extForceWorldN = null, extTorqueNm = 0 } = {}) {
    const dtn = this.drivetrain;
    // 1. honour external edits of state.bodyV (e.g. wall-collision zeroing)
    const bv = state.bodyV;
    if (bv) {
      const l = this._last;
      if (!l || Math.abs(bv[0] - l[0]) > BODYV_TOL || Math.abs(bv[1] - l[1]) > BODYV_TOL || Math.abs(bv[2] - l[2]) > BODYV_TOL) {
        dtn.setBodyVelocity(bv[0] * IN_TO_M, bv[1] * IN_TO_M, bv[2]);
      }
    }
    // 2. stick mapping (identical to MecanumDrive)
    const headingOffset = (alliance ? driverForwardOffset(alliance) : 0) + extraOffset;
    const fwd = clamp(cmd.fwd || 0, -1, 1), str = clamp(cmd.strafe || 0, -1, 1), trn = clamp(cmd.turn || 0, -1, 1);
    const c = Math.cos(state.h), s = Math.sin(state.h);
    let f, l;
    if (fieldCentric) {
      const dx = -str, dy = fwd;
      const co = Math.cos(headingOffset), so = Math.sin(headingOffset);
      const wx = dx * co - dy * so, wy = dx * so + dy * co;
      f = wx * c + wy * s;
      l = -wx * s + wy * c;
    } else { f = fwd; l = str; }
    mixWheelPowers(f, l, trn, this.cfg, this._powers);
    // 3. external force into body frame (N)
    let ext = null;
    if (extForceWorldN) {
      ext = [c * extForceWorldN[0] + s * extForceWorldN[1], -s * extForceWorldN[0] + c * extForceWorldN[1]];
    }
    // 4. step
    const r = dtn.step(this._powers, dt, { extForceBodyN: ext, extTorqueNm });
    // 5. outputs
    const hEnd = state.h + r.dHeading;
    const ce = Math.cos(hEnd), se = Math.sin(hEnd);
    const vxW = (ce * r.vx - se * r.vy) / IN_TO_M;
    const vyW = (se * r.vx + ce * r.vy) / IN_TO_M;
    const out = [r.vx / IN_TO_M, r.vy / IN_TO_M, r.om];
    state.bodyV = out;
    this._last = [out[0], out[1], out[2]];
    return { vx: vxW, vy: vyW, om: r.om };
  }

  telemetry() { return this.drivetrain.getTelemetry(); }

  reset() {
    this.drivetrain.reset();
    this._last = null;
  }
}

export { buildDrivetrainConfig } from './geometry.js';
export { makeParams, PROVENANCE } from './params.js';
export { WheelFrictionDrivetrain } from './drivetrain.js';
