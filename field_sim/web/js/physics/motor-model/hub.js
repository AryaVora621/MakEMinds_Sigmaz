// hub.js - REV Expansion Hub power command quantisation, battery and shared power bus.
// Units: SI (V, A, s). Constants come from params.js (each sourced/assumed there).
// Sources: FTC SDK / REV docs (docs.revrobotics.com expansion-hub-basics): power p in [-1,1] maps to
//   trunc(p*32767) counts, duty = counts/32767. Port 10 A continuous, 20 A absolute, 20 A battery fuse.
// ASSUMPTION: averaged-voltage model, applied V = duty*Vbus (slow decay).
import { BATTERY, HUB } from './params.js';

/** FTC power [-1,1] -> hub duty. NaN -> 0, clamp, truncate to 1/32767 counts toward zero. */
export function powerToDuty(p) {
  let x = Number(p);
  if (!Number.isFinite(x)) x = Number.isNaN(x) ? 0 : (x > 0 ? 1 : -1);
  if (x > 1) x = 1; else if (x < -1) x = -1;
  const d = Math.trunc(x * HUB.pwmCounts) / HUB.pwmCounts;
  return d === 0 ? 0 : d; // avoid -0
}

/** Battery: Vterm = max(0, Voc - Rint*I). Rint is an ASSUMPTION (default 0 = sag off). */
export class Battery {
  constructor({ openCircuitV = BATTERY.openCircuitV, internalResistanceOhm = BATTERY.internalResistanceOhm } = {}) {
    this.openCircuitV = openCircuitV;
    this.internalResistanceOhm = internalResistanceOhm;
  }
  voltage(totalSupplyA) {
    return Math.max(0, this.openCircuitV - this.internalResistanceOhm * Math.max(0, totalSupplyA || 0));
  }
}

export class PowerSystem {
  constructor({ battery = BATTERY, hub = HUB } = {}) {
    this.battery = (battery && typeof battery.voltage === 'function') ? battery : new Battery(battery);
    this.hub = hub;
    this.ch = new Map();
    this._vb = this.battery.voltage(0);
    this._totalA = 0;
    this._overFuse = false;
  }
  _c(name) {
    const c = this.ch.get(name);
    if (!c) throw new Error(`PowerSystem: unknown channel '${name}'`);
    return c;
  }
  register(name, { zeroPower = this.hub.defaultZeroPower ?? HUB.defaultZeroPower, alertA = null } = {}) {
    this.ch.set(name, {
      power: 0, zeroPower, alertA,
      accStator: 0, accSupply: 0, accT: 0,
      statorA: 0, supplyA: 0, overCurrent: false, overContinuous: false,
    });
  }
  setPower(name, p) { this._c(name).power = Number.isFinite(p) ? Math.max(-1, Math.min(1, p)) : 0; }
  getPower(name) { return this._c(name).power; }
  duty(name) { return powerToDuty(this._c(name).power); }
  setZeroPowerBehavior(name, z) {
    if (z !== 'BRAKE' && z !== 'FLOAT') throw new Error("zeroPower must be 'BRAKE' or 'FLOAT'");
    this._c(name).zeroPower = z;
  }
  zeroPower(name) { return this._c(name).zeroPower; }
  setCurrentAlert(name, amps) { this._c(name).alertA = amps; }
  busVoltage() { return this._vb; }

  /** Accumulate time-weighted currents. statorAPerMotor signed (stored as magnitude); supply per motor. */
  record(name, statorAPerMotor, supplyAPerMotor, nMotors, dtS) {
    const c = this._c(name);
    c.accStator += Math.abs(statorAPerMotor) * dtS;
    c.accSupply += Math.abs(supplyAPerMotor) * nMotors * dtS; // total supply of the channel
    c.accT += dtS;
  }

  endFrame(dtS) {
    let total = 0;
    const channels = {};
    for (const [name, c] of this.ch) {
      const T = dtS > 0 ? dtS : c.accT;
      c.statorA = T > 0 ? c.accStator / T : 0;
      c.supplyA = T > 0 ? c.accSupply / T : 0;
      c.overCurrent = c.alertA != null && c.statorA > c.alertA;
      c.overContinuous = c.statorA > this.hub.portContinuousA;
      total += c.supplyA;
      c.accStator = 0; c.accSupply = 0; c.accT = 0;
      channels[name] = { statorA: c.statorA, supplyA: c.supplyA, overCurrent: c.overCurrent, overContinuous: c.overContinuous };
    }
    this._totalA = total;
    this._overFuse = total > this.hub.batteryFuseA;
    this._vb = this.battery.voltage(total); // used by the NEXT frame (one-frame lag)
    return { busV: this._vb, totalSupplyA: total, overFuse: this._overFuse, channels };
  }

  getCurrent(name) { return this._c(name).statorA; }
  isOverCurrent(name) { return this._c(name).overCurrent; }
  overContinuous(name) { return this._c(name).overContinuous; }
  get overFuse() { return this._overFuse; }

  reset() {
    for (const c of this.ch.values()) {
      c.power = 0; c.accStator = c.accSupply = c.accT = 0;
      c.statorA = c.supplyA = 0; c.overCurrent = c.overContinuous = false;
    }
    this._totalA = 0; this._overFuse = false;
    this._vb = this.battery.voltage(0);
  }
}
