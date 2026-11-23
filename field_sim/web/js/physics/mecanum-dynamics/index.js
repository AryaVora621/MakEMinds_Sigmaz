// index.js - inch-unit facade over the SI mecanum rigid-body model. Only place that converts inches.
// Frame: body x forward, y LEFT, omega CCW (+). state.bodyV = [vf, vl, om] in in/s, in/s, rad/s.
// World velocity = (c*vx - s*vy, s*vx + c*vy), c=cos h, s=sin h. This class never moves the pose.
import { resolveParams, UNITS } from './params.js';
import { commandToWheelPowers } from './kinematics.js';
import { buildMassMatrix, stepTwist, applyImpulse } from './dynamics.js';

const IN = UNITS.IN_TO_M;

export class MecanumDynamics {
  constructor({ spec = null, overrides = {} } = {}) {
    this.params = { ...resolveParams(spec, overrides) }; // mutable shallow copy (gripScale)
    this.M = buildMassMatrix(this.params);
  }

  static fromSpec(spec, overrides = {}) {
    return new MecanumDynamics({ spec, overrides });
  }

  setGripScale(k) {
    const v = Number.isFinite(k) ? k : 1;
    this.params.gripScale = Math.max(0, Math.min(1, v));
  }

  commandToPowers(cmd) {
    const p = this.params;
    return commandToWheelPowers(p.wheels, cmd, {
      lateralMultiplier: p.lateralMultiplier,
      turnScale: p.turnScale,
      mode: p.desaturateMode,
      kMax: p.kMax,
    });
  }

  step(state, cmd, dt) {
    return this.stepPowers(state, this.commandToPowers(cmd).powers, dt);
  }

  stepPowers(state, powers, dt) {
    if (!state.bodyV) state.bodyV = [0, 0, 0];
    const b = state.bodyV;
    const tw = { vx: b[0] * IN, vy: b[1] * IN, omega: b[2] };
    const r = stepTwist(this.params, this.M, tw, powers, dt);
    const t = r.twist;
    b[0] = t.vx / IN;
    b[1] = t.vy / IN;
    b[2] = t.omega;
    const h = state.h || 0;
    const c = Math.cos(h), s = Math.sin(h);
    return {
      vx: c * b[0] - s * b[1],
      vy: s * b[0] + c * b[1],
      om: b[2],
      powers: powers.slice(),
      telemetry: r.telemetry,
    };
  }

  applyWorldImpulse(state, jxNs, jyNs, angularNms = 0) {
    if (!state.bodyV) state.bodyV = [0, 0, 0];
    const h = state.h || 0;
    const c = Math.cos(h), s = Math.sin(h);
    // rotate world -> body by -h
    const jx = c * jxNs + s * jyNs;
    const jy = -s * jxNs + c * jyNs;
    const b = state.bodyV;
    const tw = applyImpulse(this.params, this.M, { vx: b[0] * IN, vy: b[1] * IN, omega: b[2] },
      { jx, jy, jz: angularNms });
    b[0] = tw.vx / IN;
    b[1] = tw.vy / IN;
    b[2] = tw.omega;
    return state;
  }

  steadyState(cmd, seconds = 3, dt = 0.01) {
    const st = { h: 0, bodyV: [0, 0, 0] };
    const n = Math.max(1, Math.round(seconds / dt));
    for (let i = 0; i < n; i++) this.step(st, cmd, dt);
    return { vfInS: st.bodyV[0], vlInS: st.bodyV[1], omRadS: st.bodyV[2] };
  }
}

export { resolveParams, DEFAULTS, UNITS } from './params.js';
