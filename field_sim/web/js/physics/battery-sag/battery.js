// battery.js - first-order Thevenin ECM of a 10-cell NiMH FTC battery (batterydesign.net ECM). Pure ES module, no DOM.
// SI units: V, A, ohm, s, Ah. I > 0 means discharge.
import { BATTERY, WIRING, PRESETS, interp } from './params.js';

export function ocvPerCell(soc, table = BATTERY.ocvTable) {
  return interp(table, soc);
}

// Series resistance of fuse + connectors + switch + wire round trip, ohm.
export function wiringOhm(w = WIRING) {
  return w.fuseColdOhm + w.nXt30Pairs * w.xt30PairOhm + w.switchOhm + 2 * w.wireOneWayM * w.wireOhmPerM;
}

export class Battery {
  constructor(opts = {}) {
    this.params = opts.params || BATTERY;
    this.wiring = opts.wiring || WIRING;
    this.ageFactor = opts.ageFactor ?? 1;
    this.tempFactor = opts.tempFactor ?? 1;
    this.state = { soc: opts.soc ?? 1, v1: 0, ahUsed: 0, iLast: 0, vTermLast: 0 };
    this.state.vTermLast = this.vth();
  }

  ocv() { return this.params.nCells * ocvPerCell(this.state.soc, this.params.ocvTable); }

  // Pack R0 including wiring (= Rs seen by the bus), ohm.
  r0() {
    const p = this.params;
    return p.nCells * p.r0CellOhm * interp(p.rSocTable, this.state.soc) * this.ageFactor * this.tempFactor + wiringOhm(this.wiring);
  }

  r1() { return this.params.nCells * this.params.r1CellOhm * this.ageFactor; }

  vth() { return this.ocv() - this.state.v1; }

  rs() { return this.r0(); }

  step(iA, dt) {
    const p = this.params, s = this.state;
    const vthBefore = this.vth();
    const rsBefore = this.rs();
    const a = Math.exp(-dt / p.tau1S);
    s.v1 = a * s.v1 + this.r1() * (1 - a) * iA;
    const peuk = iA > p.peukertRefA ? Math.pow(iA / p.peukertRefA, p.peukertN - 1) : 1;
    const dAh = iA * dt / 3600 * peuk;
    s.soc = Math.min(1, Math.max(0, s.soc - dAh / p.capacityAh));
    s.ahUsed += dAh;
    s.iLast = iA;
    s.vTermLast = vthBefore - iA * rsBefore;
  }

  applyPreset(name) {
    const pr = PRESETS[name];
    if (!pr) throw new Error('unknown battery preset: ' + name);
    this.state.soc = pr.soc;
    this.ageFactor = pr.ageFactor;
    this.state.v1 = 0;
    this.state.ahUsed = 0;
    this.state.iLast = 0;
    this.state.vTermLast = this.vth();
  }

  snapshot() {
    const s = this.state;
    return { soc: s.soc, v1: s.v1, ahUsed: s.ahUsed, ocv: this.ocv(), vth: this.vth(), rs: this.rs(), vTermLast: s.vTermLast };
  }
}
