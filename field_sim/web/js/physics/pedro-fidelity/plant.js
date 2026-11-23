// plant.js - physical drivetrain plant for the Pedro closed loop (spec section 3).
// State: field pose (x, y in inches; h in rad, continuous/unwrapped) and field velocity (vx, vy in in/s; w in rad/s, +CCW).
// Wheel powers are held (zero-order hold) between setPowers() calls, so one loop of actuation lag is automatic.
//
// Model per axis j in {x,y,w} in the ROBOT frame (u forward, s left):
//   FLOAT (all four powers exactly 0): a = -F_j*sign(v), never reversing v. (Pedro ZPA tuner behaviour.)
//   POWERED: drive = (kV*c_j*vFree_j - v)/tau_j ; a = drive - F_j*sign(v); stiction if |v|<1e-3 and |drive|<=F_j.
//   vFree_j = Vss_j + F_j*tau_j, so c=1, kV=1 gives steady state Vss_j exactly.
// Traction: friction circle n = |(ax, ay, aw*halfSumIn)| clamped to muTraction*gIn (sets slipping).
// Wheel vectors use the same formula as ftc Mecanum.java:72-77 (order LF, LR, RF, RR); computed locally, P2 is not imported.
// Voltage: kV = V/plantNominalV, V from an injected voltageSource() or batteryVoc - Rint*I (I from the previous step).

const TAU = 2 * Math.PI;
const norm2pi = (a) => { const r = a % TAU; return r < 0 ? r + TAU : r; };
const sgn = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);

export class MecanumPlant {
  constructor(params, pose = { x: 0, y: 0, h: 0 }, { voltageSource = null } = {}) {
    this.params = params;
    this._vs = typeof voltageSource === 'function' ? voltageSource : null;
    const thF = norm2pi(Math.atan2(-params.mecanum.yVelocity, params.mecanum.xVelocity));
    const a = [Math.cos(thF), Math.sin(thF)], b = [Math.cos(TAU - thF), Math.sin(TAU - thF)];
    this._vec = [a, b, b, a].map(([x, y]) => ({ x, y })); // LF, LR, RF, RR
    const cxD = 4 * Math.abs(Math.cos(thF)), cyD = 4 * Math.abs(Math.sin(thF));
    this._cxD = cxD > 1e-12 ? cxD : Infinity;
    this._cyD = cyD > 1e-12 ? cyD : Infinity;
    this._p = [0, 0, 0, 0];
    this._V = params.plant.batteryVoc;
    this._slip = false;
    this.x = pose.x; this.y = pose.y; this.h = pose.h;
    this.vx = 0; this.vy = 0; this.w = 0;
  }

  /** Sets wheel powers [LF, LR, RF, RR], each clamped to [-1, 1]. */
  setPowers(p) {
    for (let i = 0; i < 4; i++) { const v = +p[i] || 0; this._p[i] = v > 1 ? 1 : v < -1 ? -1 : v; }
  }

  get pose() { return { x: this.x, y: this.y, h: this.h }; }
  get velocity() { return { vx: this.vx, vy: this.vy, w: this.w }; }
  get robotVelocity() {
    const c = Math.cos(this.h), s = Math.sin(this.h);
    return { u: this.vx * c + this.vy * s, s: -this.vx * s + this.vy * c, w: this.w };
  }
  get slipping() { return this._slip; }

  /** Axis efforts (dimensionless, 1 = Pedro full-power teleop) from the current wheel powers. */
  efforts() {
    const p = this._p, v = this._vec;
    let fx = 0, fy = 0;
    for (let i = 0; i < 4; i++) { fx += p[i] * v[i].x; fy += p[i] * v[i].y; }
    return { cx: fx / this._cxD, cy: fy / this._cyD, cw: ((p[2] + p[3]) - (p[0] + p[1])) / 4 };
  }

  /** Battery voltage in V used for kV. */
  voltage() { return this._vs ? this._vs() : this._V; }

  /** Overwrites pose and (optionally) field velocity, e.g. after a collision or external push. */
  syncTo(pose, vel) {
    this.x = pose.x; this.y = pose.y; this.h = pose.h;
    if (vel) { this.vx = vel.vx; this.vy = vel.vy; this.w = vel.w; }
  }

  _updateVoltage(kV) {
    const pl = this.params.plant;
    if (this._vs || !(pl.batteryRintOhm > 0)) { this._V = pl.batteryVoc; return; }
    // inverse mecanum kinematics: normalised wheel speed of each wheel at the current robot velocity
    const rv = this.robotVelocity;
    const ex = rv.u / pl.vFreeX, ey = rv.s / pl.vFreeY, ew = rv.w / pl.vFreeW;
    let I = 0;
    for (let i = 0; i < 4; i++) {
      const xi = this._vec[i].x >= 0 ? 1 : -1;
      const yi = this._vec[i].y >= 0 ? 1 : -1;
      const wi = i >= 2 ? 1 : -1;
      I += Math.abs(this._p[i] * kV - (ex * xi + ey * yi + ew * wi));
    }
    I *= pl.stallCurrentA;
    this._V = Math.max(0, pl.batteryVoc - pl.batteryRintOhm * I);
  }

  step(dtS) {
    if (!(dtS > 0)) return;
    const pl = this.params.plant;
    const n = Math.max(1, Math.ceil(dtS / pl.physicsDt - 1e-9));
    const h = dtS / n;
    const kV = this.voltage() / pl.plantNominalV;
    const { cx, cy, cw } = this.efforts();
    const float = this._p[0] === 0 && this._p[1] === 0 && this._p[2] === 0 && this._p[3] === 0;
    const Vf = [pl.vFreeX, pl.vFreeY, pl.vFreeW];
    const F = [pl.coulombX, pl.coulombY, pl.coulombW];
    const tau = [pl.tauX, pl.tauY, pl.tauW];
    const c = [cx, cy, cw];
    const aMax = pl.aTractionMax, hs = pl.halfSumIn;
    let slip = false;
    for (let k = 0; k < n; k++) {
      const hm = this.h + this.w * h * 0.5;
      const co = Math.cos(hm), si = Math.sin(hm);
      const v = [this.vx * co + this.vy * si, -this.vx * si + this.vy * co, this.w];
      const a = [0, 0, 0], stop = [false, false, false];
      for (let j = 0; j < 3; j++) {
        if (float) {
          if (Math.abs(v[j]) <= F[j] * h) stop[j] = true;
          else a[j] = -F[j] * sgn(v[j]);
        } else {
          const drive = (kV * c[j] * Vf[j] - v[j]) / tau[j];
          if (Math.abs(v[j]) < 1e-3 && Math.abs(drive) <= F[j]) stop[j] = true;
          else a[j] = drive - F[j] * sgn(v[j]);
        }
      }
      const nrm = Math.hypot(a[0], a[1], a[2] * hs);
      if (nrm > aMax) { const s = aMax / nrm; a[0] *= s; a[1] *= s; a[2] *= s; slip = true; }
      // integrate in the robot frame at the midpoint heading, then rotate back to the field frame
      let u = v[0] + a[0] * h, s = v[1] + a[1] * h, w = v[2] + a[2] * h;
      if (stop[0]) u = 0;
      if (stop[1]) s = 0;
      if (stop[2]) w = 0;
      this.vx = u * co - s * si; this.vy = u * si + s * co; this.w = w;
      this.x += this.vx * h; this.y += this.vy * h; this.h += w * h;
    }
    this._slip = slip;
    this._updateVoltage(kV);
  }
}
