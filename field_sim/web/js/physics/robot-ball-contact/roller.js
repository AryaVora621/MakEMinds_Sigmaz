// roller.js - intake roller contact, implicit spring-damper compression, friction grip, stall model, capture test.
// Pure ES module, no DOM. Units: length in, time s, velocity in/s, angle rad, mass kg, impulse kg*in/s,
// force kg*in/s^2 ("sim force", 1 N = 39.3700787 sim force). Stiffness N/m == kg/s^2 (unit-free w.r.t. length).
// All numeric tunables come from params.js (roller.*, capture.*, shape.*), each tagged SOURCED/COMPUTED/ASSUMED there.
import { RBC_PARAMS } from './params.js';
import { mixFriction, mixRestitution, effectiveRestitution, ballProps, rollerSurfaceSpeed, newtonsToSim } from './mixing.js';
import { pocketTest, toLocal, toWorldDir } from './robot_geometry.js';
import { robotPointVelocity } from './solver.js';

const kindOf = (b) => b.kind ?? b.type;

/**
 * Contact between a ball and the roller of side sd (+1 front, -1 rear). Roller axle is a line segment
 * A = (sd*(halfX+axleOutboardIn), y in [-yL,yL], z = axleHeightIn) in robot-local in, yL = halfY - cheekWidthIn.
 * @returns {null|{sd,nx,ny,nz,tx,ty,tz,delta,px,py,pz,rx,ry,overSquish:boolean}} n: roller -> ball (world, unit);
 *   t: surface direction for positive rpm (world, unit, or zero if degenerate); delta: compression in; (rx,ry) lever in.
 */
export function rollerContact(ball, robot, dims, sd, params = RBC_PARAMS) {
  const ro = params.roller, sp = params.shape;
  const r = ball.r, rr = ro.diameterIn / 2;
  const yL = Math.max(0, dims.halfY - sp.cheekWidthIn);
  const [lx, ly] = toLocal(robot, ball.x, ball.y);
  const ax = sd * (dims.halfX + ro.axleOutboardIn);
  const lyc = Math.min(Math.max(ly, -yL), yL);
  const dx = lx - ax, dy = ly - lyc, dz = ball.z - ro.axleHeightIn;
  const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
  const delta = r + rr - dist;
  if (delta <= 0 || dist < 1e-9) return null;
  const nlx = dx / dist, nly = dy / dist, nlz = dz / dist;
  // surface direction: t = sd * (yhat x n) = sd*(nz, 0, -nx); bottom of roller gives (-sd,0,0) = inward for side sd.
  let tlx = sd * nlz, tlz = -sd * nlx;
  const tl = Math.hypot(tlx, tlz);
  if (tl > 1e-9) { tlx /= tl; tlz /= tl; } else { tlx = 0; tlz = 0; }
  const [nx, ny] = toWorldDir(robot, nlx, nly);
  const [tx, ty] = toWorldDir(robot, tlx, 0);
  const px = ball.x - r * nx, py = ball.y - r * ny, pz = ball.z - r * nlz;
  const bp = ballProps(kindOf(ball), params);
  const deltaMax = bp.maxSquishFrac * 2 * r + ro.maxDeflectIn;
  return {
    sd, nx, ny, nz: nlz, tx, ty, tz: tlz, delta, px, py, pz,
    rx: px - robot.x, ry: py - robot.y, overSquish: delta > deltaMax,
  };
}

/**
 * Apply compression + friction grip of one roller contact to the ball (mutates ball.v*, position if over-squished,
 * spin w* if tracked). Compression is an implicit spring-damper (stable for any h). Robot is not touched.
 * @param {number} surfaceSpeedInS roller surface speed along rc.t, in/s (already scaled by stall load factor)
 * @param {number} h substep, s
 * @returns {{jx,jy,jz,jn,jt,rx,ry,slip}} impulse ON the ball, kg*in/s; slip = residual slip speed in/s
 */
export function applyRollerGrip(ball, robot, rc, surfaceSpeedInS, h, params = RBC_PARAMS) {
  const ro = params.roller;
  const bp = ballProps(kindOf(ball), params);
  const mb = ball.massKg ?? bp.massKg;
  const kappa = bp.inertiaFactor;
  const r = ball.r;
  const nx = rc.nx, ny = rc.ny, nz = rc.nz;
  const [vrx, vry] = robotPointVelocity(robot, rc.rx, rc.ry);

  // --- compression (implicit spring-damper): k kg/s^2, delta in, force kg*in/s^2
  const kb = bp.stiffnessNpm, kr = ro.stiffnessNpm;
  const k = 1 / (1 / kb + 1 / kr);
  const c = 2 * ro.dampingRatio * Math.sqrt(k * mb);
  let vn = (ball.vx - vrx) * nx + (ball.vy - vry) * ny + ball.vz * nz;
  const v1 = (mb * vn + h * k * rc.delta) / (mb + h * c + h * h * k);
  let Jn = Math.max(0, mb * (v1 - vn));
  ball.vx += (Jn / mb) * nx; ball.vy += (Jn / mb) * ny; ball.vz += (Jn / mb) * nz;
  vn += Jn / mb;

  // --- hard limit
  const deltaMax = bp.maxSquishFrac * 2 * r + ro.maxDeflectIn;
  if (rc.delta > deltaMax) {
    const over = rc.delta - deltaMax;
    ball.x += over * nx; ball.y += over * ny; ball.z += over * nz;
    if (vn < 0) {
      const e = effectiveRestitution(mixRestitution(bp.restitution, ro.restitution), -vn, params.contact.restitutionThresholdInS);
      const Jh = -(1 + e) * vn * mb;
      ball.vx += (Jh / mb) * nx; ball.vy += (Jh / mb) * ny; ball.vz += (Jh / mb) * nz;
      Jn += Jh;
    }
  }

  // --- friction grip toward the roller surface velocity
  const spin = typeof ball.wx === 'number';
  const wx = spin ? ball.wx : 0, wy = spin ? (ball.wy ?? 0) : 0, wz = spin ? (ball.wz ?? 0) : 0;
  // w x (-r n)
  const sx = -r * (wy * nz - wz * ny), sy = -r * (wz * nx - wx * nz), sz = -r * (wx * ny - wy * nx);
  const bcx = ball.vx + sx, bcy = ball.vy + sy, bcz = ball.vz + sz;
  const ux = vrx + surfaceSpeedInS * rc.tx, uy = vry + surfaceSpeedInS * rc.ty, uz = surfaceSpeedInS * rc.tz;
  let sl = [ux - bcx, uy - bcy, uz - bcz];
  const sn = sl[0] * nx + sl[1] * ny + sl[2] * nz;
  sl = [sl[0] - sn * nx, sl[1] - sn * ny, sl[2] - sn * nz];
  const slipMag = Math.hypot(sl[0], sl[1], sl[2]);
  let jtx = 0, jty = 0, jtz = 0, Jt = 0;
  if (slipMag > 1e-9) {
    const muRoll = mixFriction(bp.friction, ro.friction);
    const Kt = (1 + 1 / kappa) / mb;
    Jt = Math.min(slipMag / Kt, muRoll * Jn);
    jtx = (Jt * sl[0]) / slipMag; jty = (Jt * sl[1]) / slipMag; jtz = (Jt * sl[2]) / slipMag;
    ball.vx += jtx / mb; ball.vy += jty / mb; ball.vz += jtz / mb;
    if (spin) {
      const Ib = kappa * mb * r * r;
      // tau = r_b x Jt, r_b = -r n
      ball.wx += (-r * (ny * jtz - nz * jty)) / Ib;
      ball.wy += (-r * (nz * jtx - nx * jtz)) / Ib;
      ball.wz += (-r * (nx * jty - ny * jtx)) / Ib;
    }
  }
  return {
    jx: Jn * nx + jtx, jy: Jn * ny + jty, jz: Jn * nz + jtz,
    jn: Jn, jt: Jt, rx: rc.rx, ry: rc.ry, slip: slipMag,
  };
}

export class IntakeController {
  /** @param {object} params RBC_PARAMS-shaped  @param {null|(()=>number)} rng returns [0,1) */
  constructor(params = RBC_PARAMS, rng = null) {
    this.params = params;
    this.rng = rng;
    this.enabled = true;
    const f = params.roller.freeRpm;
    this.rpm = { front: f, rear: f };           // rpm, negative = spit
    this._loadPrev = { 1: 0, '-1': 0 };          // sim force from the previous step (one-substep lag)
    this._stalled = { front: false, rear: false };
    this._rolled = new Map();                    // ball key -> missed? (miss roll made once per mouth entry)
  }

  setRpm(side, rpm) { this.rpm[side] = rpm; }
  setEnabled(on) { this.enabled = !!on; }
  get stalled() { return { front: this._stalled.front, rear: this._stalled.rear }; }

  reset() {
    this._loadPrev = { 1: 0, '-1': 0 };
    this._stalled = { front: false, rear: false };
    this._rolled.clear();
  }

  /**
   * One substep of intake physics over all balls.
   * @returns {{captured:Array<[object,string]>, jammed:object[], touching:object[]}}
   */
  step(balls, robot, dims, shape, hold, reaction, h) {
    const P = this.params, ro = P.roller, cp = P.capture;
    const fStall = newtonsToSim(ro.stallForceN);
    const load = { 1: 0, '-1': 0 };
    const captured = [], jammed = [], touching = [];
    const seenInPocket = new Set();
    const mark = (arr, b) => { if (!arr.includes(b)) arr.push(b); };

    for (const b of balls) {
      if (b.state !== 'free' || b.shot || b._ghost) continue;
      // roller grip per side
      for (const sd of shape.sides) {
        const rc = rollerContact(b, robot, dims, sd, P);
        if (!rc) continue;
        const name = sd > 0 ? 'front' : 'rear';
        mark(touching, b);
        let rpm = this.enabled ? this.rpm[name] : 0;
        if (hold.full) {
          mark(jammed, b);
          if (ro.fullBehavior === 'spit') rpm = -Math.abs(ro.freeRpm);
        }
        const loadFactor = Math.max(0, 1 - this._loadPrev[sd] / fStall);
        const vs = rollerSurfaceSpeed(rpm, ro.diameterIn) * loadFactor;
        const g = applyRollerGrip(b, robot, rc, vs, h, P);
        if (reaction) reaction.addImpulse(-g.jx, -g.jy, g.rx, g.ry);
        load[sd] += g.jt / h;
      }
      // capture test
      const pt = pocketTest(b, robot, shape, P);
      if (!pt.side) continue;
      const inPocket = pt.inMouth && pt.u <= 0;
      const key = b.id ?? b;
      if (inPocket) seenInPocket.add(key);
      if (!(inPocket && pt.u <= -cp.captureDepthIn)) continue;
      if (b.z - b.r > cp.maxBottomZIn) continue;
      if (Math.hypot(b.vx, b.vy) > cp.maxBallSpeedInS) continue;
      if (!hold.canAccept()) { mark(jammed, b); continue; }
      if (cp.failProbMax > 0 && this.rng) {
        if (!this._rolled.has(key)) {
          const vApp = Math.hypot(b.vx - robot.vx, b.vy - robot.vy);
          const span = cp.failSpeedHiInS - cp.failSpeedLoInS;
          const f = span > 0 ? Math.min(1, Math.max(0, (vApp - cp.failSpeedLoInS) / span)) : (vApp > cp.failSpeedLoInS ? 1 : 0);
          this._rolled.set(key, this.rng() < cp.failProbMax * f);
        }
        if (this._rolled.get(key)) continue; // missed this mouth entry
      }
      if (hold.push(b)) {
        b.state = 'intaken';
        captured.push([b, pt.side]);
        this._rolled.delete(key);
      }
    }
    // clear mouth-entry memory for balls that left the pocket
    for (const k of [...this._rolled.keys()]) if (!seenInPocket.has(k)) this._rolled.delete(k);

    this._loadPrev = load;
    this._stalled = { front: load[1] >= 0.95 * fStall, rear: load[-1] >= 0.95 * fStall };
    return { captured, jammed, touching };
  }
}
