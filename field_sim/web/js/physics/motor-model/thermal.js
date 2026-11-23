// thermal.js - two-node (winding, housing) thermal model for a brushed DC motor.
// Units: degC, W, s, J/K, K/W. All constants come from params.js THERMAL (ASSUMED, see there).
//   Cw*dTw/dt = Pw - (Tw - Th)/Rwh
//   Ch*dTh/dt = Ph + (Tw - Th)/Rwh - (Th - Ta)/Rha
// Explicit Euler; the caller passes the substep (1 ms), far below tau_winding = Rwh*Cw = 25 s.
// overTemp is only a flag; performance effect comes via rScale()/kScale().
import { THERMAL } from './params.js';

export class ThermalTwoNode {
  constructor(params = THERMAL, ambientC = params.ambientC) {
    this.p = params;
    this.ambientC = ambientC;
    this.Tw = ambientC;
    this.Th = ambientC;
  }
  step(dtS, windingPowerW, housingPowerW = 0) {
    const p = this.p;
    const q = (this.Tw - this.Th) / p.rthWindingHousing; // W, winding -> housing
    const dTw = (windingPowerW - q) / p.cthWinding;
    const dTh = (housingPowerW + q - (this.Th - this.ambientC) / p.rthHousingAmbient) / p.cthHousing;
    this.Tw += dTw * dtS;
    this.Th += dTh * dtS;
  }
  get windingC() { return this.Tw; }
  get housingC() { return this.Th; }
  /** Copper resistance scale: 1 + alphaCu*(Tw - Tref). */
  rScale() { return 1 + this.p.copperAlpha * (this.Tw - this.p.refC); }
  /** Magnet flux scale (Kt and Ke): 1 + alphaMag*(Th - Tref). */
  kScale() { return 1 + this.p.magnetAlpha * (this.Th - this.p.refC); }
  get overTemp() { return this.Tw >= this.p.windingMaxC; }
  reset(ambientC = this.ambientC) {
    this.ambientC = ambientC;
    this.Tw = ambientC;
    this.Th = ambientC;
  }
}
