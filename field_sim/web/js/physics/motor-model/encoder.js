// encoder.js - quadrature encoder: integer ticks and a sampled velocity.
// ASSUMPTION: velocity = (ticks - prevTicks)/(t - prevT) at each sample, held in between
// (the real FTC getVelocity window was not found).
const TWO_PI = 2 * Math.PI;

export class QuadEncoder {
  constructor(ticksPerRev) {
    this.ticksPerRev = ticksPerRev;
    this.reset();
  }
  sample(tS, thetaRad) {
    const ticks = Math.floor(thetaRad * this.ticksPerRev / TWO_PI);
    if (this._prevT !== null && tS > this._prevT) {
      this._vel = (ticks - this._ticks) / (tS - this._prevT);
    }
    this._ticks = ticks;
    this._prevT = tS;
  }
  get position() { return this._ticks; }
  get velocity() { return this._vel; }
  reset() { this._ticks = 0; this._vel = 0; this._prevT = null; }
}

export function radSToTicksPerSec(omega, ticksPerRev) { return omega * ticksPerRev / TWO_PI; }
export function ticksPerSecToRadS(tps, ticksPerRev) { return tps * TWO_PI / ticksPerRev; }
