// Flower arm (matchloader-style) kinematics + ball feeding. Pure, deterministic, no timers.
// Robot frame +x fwd, +y left, z up. angleDeg 0 = forward, 90 = up (stowed, upright at the front arch), negative = dropped
// down in front of the robot (deployed). The paddle always hangs straight down from the arm end; the tip is its bottom edge. cfg.side 'rear' mirrors the arm to the back of the robot.
const RAD = Math.PI / 180;
const ease = p => { const t = p < 0 ? 0 : p > 1 ? 1 : p; return t * t * (3 - 2 * t); };
export { ease };

/** Full fallback config (mirror of data/flower_arm.json) so a failed fetch of the json cannot crash the app (new FlowerArm(undefined) used to throw). */
export const DEFAULT_FLOWER_CFG = {
  side: 'front',
  hinge: { x: 8.0, y: 0, z: 9.2 },
  arm: { length: 8.2, width: 9, linkSpacing: 10.8, linkThickness: 0.5, linkDepth: 1.2 },
  angles: { stowedDeg: 90, deployedDeg: -35 },
  timing: { deployS: 0.45, stowS: 0.45 },
  paddle: { width: 8, height: 3.2, thickness: 0.35, rubberThickness: 0.25 },
  reach: { minTipZ: 1, maxTipZ: 19 },
  feed: { feedIntervalS: 0.25, maxBallsPerDeploy: 4, alignTolDeg: 20, alignTolIn: 4, minProgress: 0.9, flightS: 0.3, volumeMarginIn: 1.5, pendingTimeoutS: 1.5,
    intakeMouth: { x: 10.5, y: 0, z: 3, halfX: 4, halfY: 8.5 } },
};
const merge = (d, o) => { if (!o || typeof o !== 'object') return (d && typeof d === 'object' && !Array.isArray(d)) ? structuredClone(d) : d; const r = { ...d }; for (const k of Object.keys(d)) if (d[k] && typeof d[k] === 'object') r[k] = structuredClone(d[k]); for (const k of Object.keys(o)) r[k] =(d[k] && typeof d[k] === 'object' && !Array.isArray(d[k])) ? merge(d[k], o[k]) : o[k]; return r; };

export class FlowerArm {
  constructor(cfg) {
    this.cfg = merge(DEFAULT_FLOWER_CFG, cfg);
    this._cbs = [];
    this.reset();
  }
  reset() {
    this.state = 'stowed'; this.progress = 0; this.fedCount = 0; this.feedT = 0; this._pending = [];
    this.angleDeg = this.cfg.angles.stowedDeg;
  }
  onState(cb) { this._cbs.push(cb); }
  _set(s) { if (s !== this.state) { const old = this.state; this.state = s; for (const cb of this._cbs) cb(s, old); } }
  _angle() {
    const a = this.cfg.angles;
    return a.stowedDeg + (a.deployedDeg - a.stowedDeg) * ease(this.progress);
  }
  deploy() {
    if (this.state === 'deployed' || this.state === 'deploying') return;
    this.fedCount = 0; this.feedT = 0;
    this._set('deploying');
  }
  stow() {
    if (this.state === 'stowed' || this.state === 'stowing') return;
    this._set('stowing');
  }
  toggle() {
    if (this.state === 'stowed' || this.state === 'stowing') this.deploy(); else this.stow();
  }
  update(dt) {
    dt = dt > 0 ? dt : 0; // negative/NaN dt must never run the arm backwards
    const t = this.cfg.timing;
    if (this.state === 'deploying') {
      this.progress = Math.min(1, this.progress + dt / t.deployS);
      if (this.progress >= 1) this._set('deployed');
    } else if (this.state === 'stowing') {
      this.progress = Math.max(0, this.progress - dt / t.stowS);
      if (this.progress <= 0) this._set('stowed');
    }
    this.angleDeg = this._angle();
  }
  get sign() { return this.cfg.side === 'rear' ? -1 : 1; }
  /** 0 = stowed .. 1 = deployed (eased), what the 3D model and servos follow */
  get deployedFrac() { return ease(this.progress); }
  // tip (centre of the paddle's bottom edge) in the robot frame
  tipRobot() {
    const h = this.cfg.hinge, L = this.cfg.arm.length, a = this.angleDeg * RAD;
    const P = this.cfg.paddle, drop = P ? P.height + P.rubberThickness - 0.4 : 0;
    return [this.sign * (h.x + L * Math.cos(a)), h.y, h.z + L * Math.sin(a) - drop];
  }
  tipWorld(pose) {
    const [lx, ly, lz] = this.tipRobot(), c = Math.cos(pose.h || 0), s = Math.sin(pose.h || 0);
    return [pose.x + c * lx - s * ly, pose.y + s * lx + c * ly, lz];
  }
}

const angDiff = (a, b) => { let d = a - b; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return d; };

export function findFlowerContact(arm, pose, flowers, opts) {
  const f = arm.cfg.feed, o = opts || {};
  const tolIn = o.alignTolIn ?? f.alignTolIn, tolDeg = o.alignTolDeg ?? f.alignTolDeg;
  if (arm.progress < (o.minProgress ?? f.minProgress)) return null;
  const tip = arm.tipWorld(pose), r = arm.cfg.reach;
  if (tip[2] < r.minTipZ || tip[2] > r.maxTipZ) return null;
  let best = null;
  for (const fl of flowers || []) {
    const oc = fl.opening_center;
    const dist = Math.hypot(tip[0] - oc[0], tip[1] - oc[1]);
    if (dist > tolIn) continue;
    const bearing = Math.atan2(oc[1] - pose.y, oc[0] - pose.x);
    const al = Math.abs(angDiff(bearing, (pose.h || 0) + (arm.sign < 0 ? Math.PI : 0))) / RAD;
    if (al > tolDeg) continue;
    if (!best || dist < best.distance) best = { flower: fl, distance: dist, alignmentDeg: al };
  }
  return best;
}

function inVolume(b, fl, margin) {
  const bb = fl.scoring_volume_bbox, zr = fl.scoring_volume_z;
  return b.x >= bb[0][0] - margin && b.x <= bb[1][0] + margin && b.y >= bb[0][1] - margin && b.y <= bb[1][1] + margin &&
    b.z >= 0 && b.z <= zr[1] + b.r;
}

// Paddle sweeps down the stack, so the top ball is pushed out first.
export function feedFromFlower(arm, world, contact, dt, hopper, pose) {
  const f = arm.cfg.feed, out = { released: 0 };
  arm._pending = arm._pending.filter(p => { p.age += dt; return p.age < f.pendingTimeoutS && p.ball.state === 'free' && world.balls.includes(p.ball); });
  if (!contact || arm.progress < f.minProgress || (arm.state !== 'deployed' && arm.state !== 'deploying')) { arm.feedT = 0; return out; }
  arm.feedT += dt;
  while (arm.feedT >= f.feedIntervalS - 1e-9) {
    if (arm.fedCount >= f.maxBallsPerDeploy || hopper.count + arm._pending.length >= hopper.cap) { arm.feedT = f.feedIntervalS; break; }
    let top = null;
    for (const b of world.balls) {
      if (b.state === 'scored' || arm._pending.some(p => p.ball === b) || !/pollen/.test(b.type || 'pollen') || !inVolume(b, contact.flower, f.volumeMarginIn)) continue;
      if (!top || b.z > top.z) top = b;
    }
    if (!top) { arm.feedT = f.feedIntervalS; break; }
    arm.feedT -= f.feedIntervalS;
    const m = f.intakeMouth, c = Math.cos(pose.h || 0), s = Math.sin(pose.h || 0);
    const mx = m.x * arm.sign, tx = pose.x + c * mx - s * m.y, ty = pose.y + s * mx + c * m.y, T = f.flightS, g = 386.0886;
    const vel = [(tx - top.x) / T, (ty - top.y) / T, (m.z - top.z + 0.5 * g * T * T) / T];
    if (top.state === 'held' && world.release) world.release(top, vel);
    else { top.state = 'free'; top.sleeping = false; top.sleepT = 0; top.vx = vel[0]; top.vy = vel[1]; top.vz = vel[2]; }
    arm._pending.push({ ball: top, age: 0 });
    arm.fedCount++; out.released++;
  }
  return out;
}
