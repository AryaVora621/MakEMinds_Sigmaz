// localizer.js - equivalent of Pedro's PinpointLocalizer. Converts firmware mm to inches.
// No DOM; imports only from this folder.
import { wrapAngle } from './kinematics.js';

const MM_PER_IN_ = 25.4; // [SRC definition of the inch]

export class PinpointLocalizerSim {
  constructor(device, host) {
    this._dev = device;
    this._host = host;
    this._pose = { x: 0, y: 0, h: 0 };
    this._vel = { x: 0, y: 0, h: 0 };
    this._total = 0;
    this._prevH = 0;
  }

  update(nowS) {
    const s = this._host.read(nowS);
    if (!s) return;
    const h = wrapAngle(s.hRad);
    this._total += wrapAngle(h - this._prevH);
    this._prevH = h;
    this._pose = { x: s.xMm / MM_PER_IN_, y: s.yMm / MM_PER_IN_, h };
    this._vel = { x: s.vxMm / MM_PER_IN_, y: s.vyMm / MM_PER_IN_, h: s.wRadS };
  }

  setStartPose(pose) { this.setPose(pose); }

  setPose(pose) {
    this._dev.setPosition({ xMm: pose.x * MM_PER_IN_, yMm: pose.y * MM_PER_IN_, hRad: pose.h });
    this._dev.clearHistory(); // stale snapshots would otherwise be served after the relocalize
    const h = wrapAngle(pose.h);
    this._pose = { x: pose.x, y: pose.y, h };
    this._vel = { x: 0, y: 0, h: 0 };
    this._total = pose.h;
    this._prevH = h;
  }

  getPose() { return { ...this._pose }; }
  getVelocity() { return { ...this._vel }; }
  getTotalHeading() { return this._total; }
  isNAN() {
    const p = this._pose, v = this._vel;
    return [p.x, p.y, p.h, v.x, v.y, v.h].some(Number.isNaN);
  }
  getForwardMultiplier() { return 1; }
  getLateralMultiplier() { return 1; }
  getTurningMultiplier() { return 1; }
}
