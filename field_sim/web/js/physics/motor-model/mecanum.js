// mecanum.js - motor-based mecanum drive dynamics, drop-in for driver.js MecanumDrive.step (spec section 7).
// Internal units SI (m, kg, s, rad, N*m, A, V). Public API (state.bodyV, return value) in in/s and rad/s.
// Four MotorUnit wheels on the mecanum Jacobian, traction cap, regularised Coulomb friction, reflected inertia,
// explicit Euler at SIM.substepS. Bus voltage and current accounting come from a PowerSystem (shared or private).
//
// Modelling note (deviation from the literal spec, flagged to the integrator): the spec applies lateralEff only to
// the lateral FORCE. With no drag that leaves strafe top speed equal to forward top speed. To make the lateral
// path power-consistent AND give strafe top speed ~ lateralEff*forward (the spec's expected result), the lateral
// kinematic ratio is also divided by lateralEff: u_i = J_i0*vf + J_i1*vl/lateralEff + J_i2*w. Virtual work holds
// (Fy = lateralEff*sum(J_i1*F_i)), so nothing is created from nothing.
import { DRIVE, SIM, IN_PER_M, G_ACCEL } from './params.js';
import { gearmotorFromConfig } from './derive.js';
import { MotorUnit } from './dcmotor.js';
import { PowerSystem, powerToDuty } from './hub.js';
import { driverForwardOffset } from '../../driver.js';

export const WHEEL_ORDER = ['FL', 'FR', 'BL', 'BR'];

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const sign = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);

/** 4x3 Jacobian rows FL,FR,BL,BR mapping [vf, vl, w] (m/s, m/s, rad/s) to wheel surface speed (m/s). k = lx+ly. */
export function mecanumJacobian(lxM, lyM) {
  const k = lxM + lyM;
  return [
    [1, -1, -k],  // FL
    [1, 1, k],    // FR
    [1, 1, -k],   // BL
    [1, -1, k],   // BR
  ];
}

const POWER_SIGNS = mecanumJacobian(1, 1); // only the signs matter for wheelPowers

/** FTC standard normalised wheel powers [FL,FR,BL,BR] from forward F, left L, ccw turn T fractions. */
export function wheelPowers(F, L, T) {
  const p = POWER_SIGNS.map((j) => F * j[0] + L * j[1] + T * sign(j[2]));
  const d = Math.max(1, Math.abs(F) + Math.abs(L) + Math.abs(T));
  return p.map((x) => x / d);
}

export class MotorMecanumDrive {
  /** opts: { params (DRIVE overrides, shallow), power (shared PowerSystem|null), alliance, slip, zeroPower } */
  constructor(opts = {}) {
    this.p = { ...DRIVE, ...(opts.params || {}) };
    this.alliance = opts.alliance ?? null;
    this.slip = opts.slip ?? 0;                       // 0..1, scales traction cap by (1 - 0.7*slip)
    this.zeroPower = opts.zeroPower ?? this.p.zeroPower ?? 'BRAKE';
    this.gm = gearmotorFromConfig(this.p);
    this._ownPower = !opts.power;
    this.power = opts.power || new PowerSystem();
    this.names = WHEEL_ORDER.map((w) => 'drive_' + w);
    for (const nm of this.names) this.power.register(nm, { zeroPower: this.zeroPower });
    this._motors = WHEEL_ORDER.map(() => new MotorUnit(this.gm));
    this.J = mecanumJacobian(this.p.lxM, this.p.lyM);
    this._tel = { wheelDuty: [0, 0, 0, 0], wheelStatorA: [0, 0, 0, 0], tractionLimited: [false, false, false, false] };
    this._busV = 12;
  }

  get motors() { return this._motors; }

  freeSpeedInS() { return this.gm.freeSpeedOutRadS * this.p.wheelRadiusM * IN_PER_M; }

  /** Driver command -> body-frame fractions F (forward), L (left), turn. Copy of driver.js MecanumDrive logic. */
  _frame(cmd, state, fieldCentric, headingOffset, alliance) {
    const off = (alliance ? driverForwardOffset(alliance) : 0) + headingOffset;
    const fwd = clamp(cmd.fwd || 0, -1, 1), str = clamp(cmd.strafe || 0, -1, 1), trn = clamp(cmd.turn || 0, -1, 1);
    if (!fieldCentric) return [fwd, str, trn];
    const c = Math.cos(state.h), s = Math.sin(state.h);
    const dx = -str, dy = fwd;
    const co = Math.cos(off), so = Math.sin(off);
    const wx = dx * co - dy * so, wy = dx * so + dy * co;
    return [wx * c + wy * s, -wx * s + wy * c, trn];
  }

  step(cmd, dtS, state, { fieldCentric = false, headingOffset = 0, alliance = this.alliance } = {}) {
    if (!state.bodyV) state.bodyV = [0, 0, 0];
    const [F, L, Tn] = this._frame(cmd || {}, state, fieldCentric, headingOffset, alliance);
    const b = state.bodyV;
    const c = Math.cos(state.h), s = Math.sin(state.h);
    if (!(dtS > 0)) return { vx: c * b[0] - s * b[1], vy: s * b[0] + c * b[1], om: b[2] };

    const { wheelRadiusM: r, massKg: m, mu, lateralEff: eff } = this.p;
    const k = this.p.lxM + this.p.lyM;
    const gm = this.gm;
    const Jw = this.p.wheelJ + gm.rotorJReflected;
    const Mlin = m + 4 * Jw / (r * r);
    const Mrot = this.p.yawInertia + 4 * Jw * k * k / (r * r);
    const slipK = 1 - 0.7 * clamp(this.slip, 0, 1);
    const Fcap = mu * m * G_ACCEL / 4 * slipK;           // N per wheel, ASSUMED mu
    const uEps = SIM.coulombEpsMps;

    const p = wheelPowers(F, L, Tn);
    const duty = p.map(powerToDuty);
    for (let i = 0; i < 4; i++) this.power.setPower(this.names[i], p[i]);

    let vf = b[0] / IN_PER_M, vl = b[1] / IN_PER_M, w = b[2];
    const n = Math.ceil(Math.min(dtS, SIM.maxFrameDtS) / SIM.substepS);
    const h = Math.min(dtS, SIM.maxFrameDtS) / n;
    const Vb = this.power.busVoltage();
    this._busV = Vb;
    const limited = [false, false, false, false];

    for (let t = 0; t < n; t++) {
      let Fx = 0, Fy = 0, Mz = 0;
      const res = new Array(4);
      for (let i = 0; i < 4; i++) {
        const Ji = this.J[i];
        const u = Ji[0] * vf + Ji[1] * vl / eff + Ji[2] * w;     // wheel surface speed, m/s
        const ci = this._motors[i].compute(u / r, Vb, duty[i], this.zeroPower);
        res[i] = ci;
        let Fi = ci.torqueOut / r;
        if (Math.abs(Fi) > Fcap) { Fi = sign(Fi) * Fcap; limited[i] = true; } else limited[i] = false;
        const fi = -(ci.frictionOut / r) * clamp(u / uEps, -1, 1);
        const Ft = Fi + fi;
        Fx += Ft;
        Fy += eff * Ji[1] * Ft;
        Mz += Ji[2] * Ft;
      }
      vf += h * Fx / Mlin;
      vl += h * Fy / Mlin;
      w += h * Mz / Mrot;
      for (let i = 0; i < 4; i++) {
        this._motors[i].commit(res[i], h);
        this.power.record(this.names[i], res[i].statorA, res[i].supplyA, gm.n, h);
      }
      for (let i = 0; i < 4; i++) this._tel.wheelStatorA[i] = res[i].statorA;
    }
    this._tel.wheelDuty = duty;
    this._tel.tractionLimited = limited;
    if (this._ownPower) this.power.endFrame(dtS);

    b[0] = vf * IN_PER_M; b[1] = vl * IN_PER_M; b[2] = w;
    return { vx: c * b[0] - s * b[1], vy: s * b[0] + c * b[1], om: w };
  }

  /** Write post-collision world velocities (in/s, rad/s) back into state.bodyV. */
  syncBodyVelocity(state, vx, vy, om) {
    if (!state.bodyV) state.bodyV = [0, 0, 0];
    const c = Math.cos(state.h), s = Math.sin(state.h);
    state.bodyV[0] = vx * c + vy * s;
    state.bodyV[1] = -vx * s + vy * c;
    state.bodyV[2] = om;
  }

  telemetry() {
    return {
      wheelDuty: this._tel.wheelDuty.slice(),
      wheelStatorA: this._tel.wheelStatorA.slice(),
      wheelWindingC: this._motors.map((mu) => mu.windingC),
      busV: this._busV,
      tractionLimited: this._tel.tractionLimited.slice(),
    };
  }

  reset() {
    for (const mu of this._motors) mu.reset();
    this.power.reset();
    this._tel = { wheelDuty: [0, 0, 0, 0], wheelStatorA: [0, 0, 0, 0], tractionLimited: [false, false, false, false] };
  }
}
