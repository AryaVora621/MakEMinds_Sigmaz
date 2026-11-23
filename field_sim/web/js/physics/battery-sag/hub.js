// hub.js - REV Expansion Hub brownout state machine, hub loads, port overcurrent flags.
// SI units: V, A, W, s. Constants come from params.js HUB (each tagged [S]/[A] there).
import { HUB } from './params.js';

export class HubBrownout {
  constructor(p = HUB) {
    this.p = p;
    this.events = [];
    this.reset();
  }

  reset() {
    this.state = {
      mode: 'ok', outputsEnabled: true, lowBattery: false, below8V: false,
      ledFault: 'none', servoRailOk: true, resetCount: 0,
      lowT: 0,      // s below resetV while in 'ok'
      rebootT: 0,   // s spent in 'rebooting'
    };
    this.events = [];
  }

  _enterReset() {
    const s = this.state, p = this.p;
    s.mode = 'reset';
    s.outputsEnabled = false;
    s.resetCount++;
    s.lowT = 0;
    s.rebootT = 0;
    this.events.push('hubReset');
    if (p.resetClearsEncoders) this.events.push('encodersCleared');
    if (p.resetClearsPinpoint) this.events.push('pinpointLost');
  }

  // vBus: hub input voltage (V). dt: s. Returns the state object.
  step(vBus, dt) {
    const s = this.state, p = this.p;

    // Low-battery LED fault: latched below lowBatteryV, clears above it.
    if (!s.lowBattery && vBus < p.lowBatteryV) {
      s.lowBattery = true; this.events.push('lowBattery');
    } else if (s.lowBattery && vBus > p.lowBatteryV) {
      s.lowBattery = false; this.events.push('lowBatteryCleared');
    }
    s.ledFault = s.lowBattery ? 'blinkOrange' : 'none';
    s.below8V = vBus < p.vinMinV;

    const dead = vBus < 1.0; // V, total power loss (e.g. blown fuse)
    if (dead) {
      if (s.mode === 'ok' || s.mode === 'rebooting') this._enterReset();
      s.mode = 'dead';
      s.rebootT = 0;
    } else if (s.mode === 'dead') {
      if (vBus >= p.recoverV) { s.mode = 'rebooting'; s.rebootT = 0; }
      else s.mode = 'reset';
    } else if (s.mode === 'ok') {
      if (vBus < p.resetV) {
        s.lowT += dt;
        if (s.lowT >= p.resetDwellS) this._enterReset();
      } else {
        s.lowT = 0;
      }
    } else if (s.mode === 'reset') {
      if (vBus >= p.recoverV) { s.mode = 'rebooting'; s.rebootT = 0; }
    } else if (s.mode === 'rebooting') {
      if (vBus < p.resetV) {
        s.mode = 'reset'; s.rebootT = 0;
      } else {
        s.rebootT += dt;
        if (s.rebootT >= p.rebootS) {
          s.mode = 'ok'; s.rebootT = 0; s.lowT = 0;
          this.events.push('recovered');
        }
      }
    }

    s.outputsEnabled = s.mode === 'ok';
    s.servoRailOk = vBus >= p.servoRailDropV && s.mode === 'ok';
    return s;
  }

  consumeEvents() {
    const e = this.events;
    this.events = [];
    return e;
  }
}

// Per-port flag: |I| above the continuous port rating (A). Flag only; no cutoff modelled [A].
export function portOvercurrent(currents, p = HUB) {
  return currents.map(i => Math.abs(i) > p.motorPortContA);
}

// Bus power loads (W) of hub electronics and the servo rail (servo current on 5 V, through the buck).
export function hubLoads({ servosMoving = 0, servosIdle = 0 } = {}, p = HUB) {
  const servoA = servosMoving * p.servoMovingA + servosIdle * p.servoIdleA; // A at railV
  return [
    { kind: 'power', watts: p.electronicsW + p.pinpointW },
    { kind: 'power', watts: servoA * p.railV / p.railEff },
  ];
}
