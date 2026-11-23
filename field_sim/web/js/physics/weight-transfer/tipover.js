// Tip-over rocking: Housner-style rigid rocking about a wheel edge.
// Units: in, lbm, s, rad. Robot frame +x forward, +y left, z up. Wheel order FL, FR, BL, BR.
// Housner relations (restitution r = 1 - 1.5*sin^2(alpha), rocking ODE) are RECALLED, not verified against a source.
// Impact omega0 and critical speed are DERIVED here (angular momentum about the edge). Constants (restitutionWall,
// contactZIn, minRockOmega, settleOmega, substepS, fallEaseRate) are ASSUMED placeholders in params.js.
import { G, wheelGeometry } from './params.js';

const UNIT = { front: { x: 1, y: 0 }, rear: { x: -1, y: 0 }, left: { x: 0, y: 1 }, right: { x: 0, y: -1 } };
const OPPOSITE = { front: 'rear', rear: 'front', left: 'right', right: 'left' };
// lifted mask (FL, FR, BL, BR) = the two wheels NOT on the edge
const LIFTED = {
  front: [false, false, true, true],
  rear: [true, true, false, false],
  left: [false, true, false, true],
  right: [true, false, true, false],
};

/** Pure geometry of rocking about an edge. */
export function edgeGeometry(params, cg, edge) {
  const dir = UNIT[edge];
  if (!dir) throw new Error('edgeGeometry: bad edge ' + edge);
  const { xc, yc, a, b } = wheelGeometry(params);
  let d, pivot, Iaxis;
  if (edge === 'front') { d = xc + a - cg.x; pivot = { x: xc + a, y: yc, z: 0 }; Iaxis = cg.Iyy; }
  else if (edge === 'rear') { d = cg.x - (xc - a); pivot = { x: xc - a, y: yc, z: 0 }; Iaxis = cg.Iyy; }
  else if (edge === 'left') { d = yc + b - cg.y; pivot = { x: xc, y: yc + b, z: 0 }; Iaxis = cg.Ixx; }
  else { d = cg.y - (yc - b); pivot = { x: xc, y: yc - b, z: 0 }; Iaxis = cg.Ixx; }
  const h = cg.z;
  const R = Math.sqrt(d * d + h * h);
  return { d, h, R, alpha: Math.atan2(d, h), Ie: Iaxis + cg.massLb * R * R, pivot, dir: { x: dir.x, y: dir.y } };
}

/** Critical head-on impact speed (in/s) to tip over `edge`; Infinity if it cannot. */
export function impactTipSpeed(params, cg, edge) {
  const g = edgeGeometry(params, cg, edge);
  const e = params.tip.restitutionWall, zc = params.tip.contactZIn;
  const den = cg.massLb * (g.h - (1 + e) * zc);
  if (!(den > 0)) return Infinity;
  const rise = g.R - g.h;
  if (!(rise > 0)) return 0;
  const wc = Math.sqrt((2 * cg.massLb * G * rise) / g.Ie);
  return (g.Ie * wc) / den;
}

export class TipState {
  constructor(params) {
    this.params = params;
    this.reset();
  }

  reset() {
    this.mode = 'grounded';
    this.edge = null;
    this.angle = 0;
    this.omega = 0;
    this.disabled = false;
  }

  /** dvBody {x,y} in/s toward the obstacle in the body frame. */
  applyImpact(dvBody, cg) {
    if (this.mode === 'fallen') return { edge: this.edge, omega0: 0, started: false };
    const adx = Math.abs(dvBody.x), ady = Math.abs(dvBody.y);
    let edge, dv;
    if (adx >= ady) { edge = dvBody.x >= 0 ? 'front' : 'rear'; dv = adx; }
    else { edge = dvBody.y >= 0 ? 'left' : 'right'; dv = ady; }
    if (!(dv > 0)) return { edge: null, omega0: 0, started: false };
    const e = this.params.tip.restitutionWall, zc = this.params.tip.contactZIn;
    let g = edgeGeometry(this.params, cg, edge);
    let omega0 = (cg.massLb * dv * (g.h - (1 + e) * zc)) / g.Ie;
    if (omega0 < 0) {
      // wall impulse is above the CG height line: it pitches the robot the other way
      edge = OPPOSITE[edge];
      g = edgeGeometry(this.params, cg, edge);
      omega0 = (cg.massLb * dv * ((1 + e) * zc - g.h)) / g.Ie;
    }
    if (!(omega0 > this.params.tip.minRockOmega)) return { edge, omega0, started: false };
    if (this.mode === 'rocking' && this.edge === edge) {
      this.omega += omega0;
    } else {
      this.mode = 'rocking';
      this.edge = edge;
      this.angle = 0;
      this.omega = omega0;
    }
    return { edge, omega0, started: true };
  }

  step(dt, { cg, loads, ax = 0, ay = 0 }) {
    const tp = this.params.tip;
    if (this.mode === 'fallen') {
      if (dt > 0) this.angle += (Math.PI / 2 - this.angle) * (1 - Math.exp(-tp.fallEaseRate * dt));
      return this.snapshot;
    }
    if (this.mode === 'grounded' && loads && loads.mode === 'tipping' && loads.axis) {
      this.mode = 'rocking';
      this.edge = loads.axis;
      this.angle = 0;
      this.omega = 0;
    }
    if (this.mode === 'rocking' && dt > 0) {
      const g = edgeGeometry(this.params, cg, this.edge);
      const aE = -(ax * g.dir.x + ay * g.dir.y); // inertial pseudo-force toward the edge (in/s^2)
      const M = cg.massLb;
      const r = 1 - 1.5 * Math.sin(g.alpha) ** 2; // Housner restitution (unverified)
      const n = Math.max(1, Math.ceil(dt / tp.substepS));
      const h = dt / n;
      for (let i = 0; i < n; i++) {
        const acc = (M * g.R * (aE * Math.cos(g.alpha - this.angle) - G * Math.sin(g.alpha - this.angle))) / g.Ie;
        this.omega += acc * h;
        this.angle += this.omega * h;
        if (this.angle >= g.alpha) {
          this.mode = 'fallen';
          this.disabled = true;
          break;
        }
        if (this.angle <= 0) {
          this.angle = 0;
          if (this.omega < 0) {
            this.omega = -r * this.omega; // touchdown
            if (this.omega < tp.settleOmega) this._ground();
          } else if (acc <= 0 && this.omega < tp.settleOmega) {
            this._ground(); // resting, no drive toward the edge
          }
          if (this.mode === 'grounded') break;
        }
      }
    }
    return this.snapshot;
  }

  _ground() {
    this.mode = 'grounded';
    this.edge = null;
    this.angle = 0;
    this.omega = 0;
  }

  get snapshot() {
    const active = this.mode !== 'grounded' && this.edge;
    return {
      mode: this.mode,
      edge: this.edge,
      angle: this.angle,
      omega: this.omega,
      pivot: active ? this._pivot() : null,
      lifted: active ? LIFTED[this.edge].slice() : [false, false, false, false],
      disabled: this.disabled,
    };
  }

  _pivot() {
    const { xc, yc, a, b } = wheelGeometry(this.params);
    switch (this.edge) {
      case 'front': return { x: xc + a, y: yc, z: 0 };
      case 'rear': return { x: xc - a, y: yc, z: 0 };
      case 'left': return { x: xc, y: yc + b, z: 0 };
      default: return { x: xc, y: yc - b, z: 0 };
    }
  }
}
