// localizer.js - Pinpoint-like localizer over a MecanumPlant.
// update() samples the plant truth at clock time, optionally adds seeded Gaussian noise and delays the output by latencyS
// through a time-stamped ring buffer. With default params (no noise, no latency) it is exactly the truth.
// pose() heading is in [0, 2PI) as Pedro's localizer reports; velocity() is the field-frame {x,y} in in/s.

const TAU = 2 * Math.PI;
const norm2pi = (a) => { const r = a % TAU; return r < 0 ? r + TAU : r; };

/** Seeded PRNG, uniform in [0,1). Deterministic across platforms. */
export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class SimLocalizer {
  constructor(plant, cfg = {}, clock = null) {
    this.plant = plant;
    this.cfg = { posNoiseIn: 0, headNoiseRad: 0, velNoiseInS: 0, latencyS: 0, seed: 1, ...cfg };
    this.clock = clock;
    this._rng = mulberry32(this.cfg.seed);
    this._spare = null;
    this._buf = [];
    this._out = null;
    this.update();
  }

  _now() { return this.clock ? this.clock.nowNs() / 1e9 : 0; }

  _gauss() {
    if (this._spare !== null) { const s = this._spare; this._spare = null; return s; }
    let u = this._rng(); if (u < 1e-12) u = 1e-12;
    const v = this._rng();
    const r = Math.sqrt(-2 * Math.log(u));
    this._spare = r * Math.sin(TAU * v);
    return r * Math.cos(TAU * v);
  }

  /** Samples the plant (truth + noise), stores it with a timestamp, publishes the sample latencyS old. */
  update() {
    const c = this.cfg, p = this.plant.pose, v = this.plant.velocity;
    const pn = c.posNoiseIn > 0, hn = c.headNoiseRad > 0, vn = c.velNoiseInS > 0;
    const s = {
      t: this._now(),
      x: p.x + (pn ? c.posNoiseIn * this._gauss() : 0),
      y: p.y + (pn ? c.posNoiseIn * this._gauss() : 0),
      h: p.h + (hn ? c.headNoiseRad * this._gauss() : 0),
      vx: v.vx + (vn ? c.velNoiseInS * this._gauss() : 0),
      vy: v.vy + (vn ? c.velNoiseInS * this._gauss() : 0),
    };
    if (c.latencyS > 0) {
      this._buf.push(s);
      const target = s.t - c.latencyS;
      let pick = this._buf[0], idx = 0;
      for (let i = 0; i < this._buf.length; i++) {
        if (this._buf[i].t <= target + 1e-12) { pick = this._buf[i]; idx = i; } else break;
      }
      if (idx > 0) this._buf.splice(0, idx); // drop samples older than the one published
      this._out = pick;
    } else {
      this._out = s;
    }
  }

  pose() { const o = this._out; return { x: o.x, y: o.y, h: norm2pi(o.h) }; }
  velocity() { const o = this._out; return { x: o.vx, y: o.vy }; }

  /** Teleports the plant to pose (keeps its velocity) and clears the latency buffer. */
  setPose(pose) {
    this.plant.syncTo({ x: pose.x, y: pose.y, h: pose.h }, this.plant.velocity);
    this._buf = [];
    this.update();
  }
}
