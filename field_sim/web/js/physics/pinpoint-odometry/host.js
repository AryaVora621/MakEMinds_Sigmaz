// host.js - Pinpoint host-side read model: I2C data age plus firmware sample-and-hold.
// Units: seconds. Returns firmware snapshots (mm, rad). No DOM; imports only from this folder.

export class PinpointHost {
  constructor(device, params, rng) {
    this._dev = device;
    this._p = params.host;     // dataAgeS, dataAgeJitterS, readCostS [ASSUMPTION, see params.js]
    this._rng = rng;
    this._lastAge = 0;
    this._cost = 0;
  }

  // Newest snapshot with tS <= nowS - age; oldest if none qualifies; null if history is empty.
  read(nowS) {
    const hist = this._dev.history();
    this._cost = this._p.readCostS;
    if (hist.length === 0) return null;
    const jit = this._p.dataAgeJitterS ? this._p.dataAgeJitterS * this._rng.gauss() : 0;
    const age = Math.max(0, this._p.dataAgeS + jit);
    const tMax = nowS - age;
    let pick = hist[0];
    for (let i = hist.length - 1; i >= 0; i--) {
      if (hist[i].tS <= tMax) { pick = hist[i]; break; }
    }
    this._lastAge = nowS - pick.tS;
    return pick;
  }

  get lastReadCostS() { return this._cost; }
  get lastDataAgeS() { return this._lastAge; }
}
