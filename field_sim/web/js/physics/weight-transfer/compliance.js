// Chassis compliance (for looks): pitch / roll / heave spring-dampers on the four wheels.
// Units: in, lbm, s, rad. Robot frame +x forward, +y left, z up.
// Sign conventions: pitch + = NOSE DOWN (right-hand about +y), roll + = LEFT SIDE UP (right-hand about +x), heave + = up.
// Model (spec section 7). Spring-damper form from the PhysX vehicle tuning guide
// (https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/Vehicles.html): stiffness = wn^2*m,
// damping = zeta*2*sqrt(stiffness*mass), applied rotationally here as C = 2*zeta*sqrt(K*I).
// Equal-stiffness four-wheel layout and the constants kWheelLbfPerIn, zeta are ASSUMED placeholders (params.js).
import { G, wheelGeometry } from './params.js';

export class ChassisCompliance {
  constructor(params, cg) {
    this.params = params;
    this._geom = wheelGeometry(params);
    this.setCG(cg);
    this.reset();
  }

  /** Recompute stiffness/damping for a new mass/inertia; the dynamic state is kept. */
  setCG(cg) {
    const c = this.params.compliance;
    const { a, b } = this._geom;
    const kG = c.kWheelLbfPerIn * G; // lbm/s^2 per in (lbf = lbm*G in lbm*in/s^2 units)
    this.cg = cg;
    this.M = cg.massLb;
    this.h = cg.z;
    this.Kp = 4 * kG * a * a; // lbm*in^2/s^2 per rad
    this.Kr = 4 * kG * b * b;
    this.Kz = 4 * kG; // lbm/s^2
    this.Cp = 2 * c.zeta * Math.sqrt(this.Kp * cg.Iyy);
    this.Cr = 2 * c.zeta * Math.sqrt(this.Kr * cg.Ixx);
    this.Cz = 2 * c.zeta * Math.sqrt(this.Kz * cg.massLb);
    this.zSink = (cg.massLb * G) / this.Kz; // in, static sink = W/(4k)
  }

  reset() {
    this.theta = 0; this.thetaRate = 0;
    this.phi = 0; this.phiRate = 0;
    this.z = -this.zSink; this.zRate = 0;
  }

  /** Add angular velocity (rad/s), e.g. from a wall impact. */
  kick(dThetaDot, dPhiDot) {
    this.thetaRate += dThetaDot || 0;
    this.phiRate += dPhiDot || 0;
  }

  step(dt, { ax = 0, ay = 0 } = {}) {
    if (dt > 0) {
      const n = Math.max(1, Math.ceil(dt / this.params.compliance.maxSubstepS));
      const h = dt / n;
      const { M, Kp, Kr, Kz, Cp, Cr, Cz, zSink } = this;
      const { Iyy, Ixx } = this.cg;
      const tauP = -M * ax * this.h; // lbm*in^2/s^2
      const tauR = M * ay * this.h;
      for (let i = 0; i < n; i++) {
        this.thetaRate += ((tauP - Cp * this.thetaRate - Kp * this.theta) / Iyy) * h;
        this.theta += this.thetaRate * h;
        this.phiRate += ((tauR - Cr * this.phiRate - Kr * this.phi) / Ixx) * h;
        this.phi += this.phiRate * h;
        this.zRate += ((-Cz * this.zRate - Kz * (this.z + zSink)) / M) * h;
        this.z += this.zRate * h;
      }
    }
    return { pitch: this.theta, roll: this.phi, heave: this.z };
  }

  get state() {
    return {
      pitch: this.theta, pitchRate: this.thetaRate,
      roll: this.phi, rollRate: this.phiRate,
      heave: this.z, heaveRate: this.zRate, zSink: this.zSink,
    };
  }

  /** Undamped natural frequencies in Hz. */
  naturalFreqHz() {
    const tw = 2 * Math.PI;
    return {
      pitch: Math.sqrt(this.Kp / this.cg.Iyy) / tw,
      roll: Math.sqrt(this.Kr / this.cg.Ixx) / tw,
      heave: Math.sqrt(this.Kz / this.M) / tw,
    };
  }
}
