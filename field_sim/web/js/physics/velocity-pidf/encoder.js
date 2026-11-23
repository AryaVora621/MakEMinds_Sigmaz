// encoder.js - quadrature encoder counts and hub-style windowed velocity.
// Pure ES module, no DOM. Units: counts (integer), seconds, rad, ticks/s.
// Windowing/int16 wrap follow params.encoder ([ASSUMED] window; int16 per FTCLib [SRC]).

/** Wrap to signed 16-bit range [-32768, 32767]. */
export function wrapInt16(v) {
  return ((((Math.round(v) + 32768) % 65536) + 65536) % 65536) - 32768;
}

export class QuadratureEncoder {
  constructor(opts) {
    this.cpr = opts.cpr;                       // counts per motor-shaft rev
    this.velWindowS = opts.velWindowS;         // s
    this.historyS = opts.historyS;             // s
    this.int16Wrap = opts.int16Wrap !== false;
    this.direction = opts.direction === -1 ? -1 : 1;
    this.reset(0, 0);
  }

  _count(theta) {
    return Math.floor(this.direction * theta * this.cpr / (2 * Math.PI));
  }

  reset(t = 0, thetaMotorRad = 0) {
    this._ts = [t];
    this._cs = [this._count(thetaMotorRad)];
    this._head = 0; // index of first live entry (amortised trimming)
  }

  update(t, thetaMotorRad) {
    this._ts.push(t);
    this._cs.push(this._count(thetaMotorRad));
    const cutoff = t - this.historyS;
    // trim, but always keep at least 2 samples
    while (this._head < this._ts.length - 2 && this._ts[this._head] < cutoff) this._head++;
    if (this._head > 256 && this._head * 2 > this._ts.length) {
      this._ts = this._ts.slice(this._head);
      this._cs = this._cs.slice(this._head);
      this._head = 0;
    }
  }

  get position() { return this._cs[this._cs.length - 1]; }

  velocityAt(t) {
    const ts = this._ts, cs = this._cs;
    const last = ts.length - 1;
    const target = t - this.velWindowS + 1e-9;
    // binary search: newest index in [head,last] with ts <= target
    let lo = this._head, hi = last, idx = this._head;
    if (ts[lo] <= target) {
      while (lo <= hi) {
        const mid = (lo + hi) >> 1;
        if (ts[mid] <= target) { idx = mid; lo = mid + 1; } else hi = mid - 1;
      }
    } // else: oldest sample
    const v = Math.round((cs[last] - cs[idx]) / this.velWindowS);
    return this.int16Wrap ? wrapInt16(v) : v;
  }
}
