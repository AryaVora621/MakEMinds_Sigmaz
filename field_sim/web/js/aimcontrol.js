// aimcontrol.js - mechanism control side: shooter plant, manual aim, mode commands.
// Pure ES module, no DOM.
import { BALL_KINDS, LAUNCH_REF_KIND } from './balls.js';

export const AIM_MODES = ['auto', 'assist', 'manual', 'free'];

/** Physics radius (in) of the ball kind used for the opening rim clearance of an aim solution: a pollen (2.8 in) or a nectar
 *  (3.6 in) ball must keep its whole circle inside the opening, so the clearance is the kind radius, not one fixed value. */
export function rimClearanceIn(kind) {
  return (BALL_KINDS[kind] || BALL_KINDS[LAUNCH_REF_KIND]).radiusIn;
}

export const DEFAULT_PLANT = {
  turretRateDegS: 540, turretKp: 14, turretMinDeg: -180, turretMaxDeg: 180,
  hoodRateDegS: 120, hoodMinDeg: 25, hoodMaxDeg: 65,
  flywheelTauS: 0.35, flywheelRecoverTauS: 0.04, maxRpm: 6000, rpmDipFrac: 0.06, turretCableWrap: false,
};

const PRESET_CYCLE = ['low', 'mid', 'high'];
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export function pickEquivalentTurretAngle(targetDeg, currentDeg, minDeg, maxDeg) {
  if (!Number.isFinite(targetDeg) || !Number.isFinite(minDeg) || !Number.isFinite(maxDeg)) return null;
  const cur = Number.isFinite(currentDeg) ? currentDeg : 0;
  const kLo = Math.ceil((minDeg - targetDeg) / 360 - 1e-9);
  const kHi = Math.floor((maxDeg - targetDeg) / 360 + 1e-9);
  if (kLo > kHi) return null;
  let best = null, bestD = Infinity;
  const kc = Math.round((cur - targetDeg) / 360);
  for (let k = Math.max(kLo, kc - 1); k <= Math.min(kHi, kc + 1); k++) {
    const a = targetDeg + 360 * k, d = Math.abs(a - cur);
    if (d < bestD) { bestD = d; best = a; }
  }
  if (best === null) { // current far outside; fall back to nearest bound candidate
    for (const k of [kLo, kHi]) {
      const a = targetDeg + 360 * k, d = Math.abs(a - cur);
      if (d < bestD) { bestD = d; best = a; }
    }
  }
  return best;
}

export class ShooterPlant {
  constructor(spec = DEFAULT_PLANT) {
    this.spec = { ...DEFAULT_PLANT, ...spec };
    this.state = { turretDeg: 0, hoodDeg: this.spec.hoodMinDeg, rpm: 0 };
    this.atLimit = null;
  }
  step(dt, cmd = {}) {
    if (!(dt >= 0)) dt = 0;   // negative/NaN dt would move the plant away from its target
    const s = this.spec, st = this.state;
    this.atLimit = null;
    if (cmd.turretDeg != null && Number.isFinite(cmd.turretDeg)) {
      const tgt = clamp(cmd.turretDeg, s.turretMinDeg, s.turretMaxDeg);
      let v = clamp(s.turretKp * (tgt - st.turretDeg), -s.turretRateDegS, s.turretRateDegS);
      let move = v * dt;
      const err = tgt - st.turretDeg;
      if (Math.abs(move) > Math.abs(err)) move = err; // never overshoot
      st.turretDeg += move;
    }
    if (st.turretDeg <= s.turretMinDeg) { st.turretDeg = s.turretMinDeg; this.atLimit = 'min'; }
    else if (st.turretDeg >= s.turretMaxDeg) { st.turretDeg = s.turretMaxDeg; this.atLimit = 'max'; }
    let hoodLimit = null;
    if (cmd.hoodDeg != null && Number.isFinite(cmd.hoodDeg)) {
      const tgt = clamp(cmd.hoodDeg, s.hoodMinDeg, s.hoodMaxDeg);
      const err = tgt - st.hoodDeg, maxMove = s.hoodRateDegS * dt;
      st.hoodDeg += clamp(err, -maxMove, maxMove);
    }
    if (st.hoodDeg <= s.hoodMinDeg) { st.hoodDeg = s.hoodMinDeg; hoodLimit = 'min'; }
    else if (st.hoodDeg >= s.hoodMaxDeg) { st.hoodDeg = s.hoodMaxDeg; hoodLimit = 'max'; }
    if (cmd.rpm != null && Number.isFinite(cmd.rpm)) {
      const tgt = clamp(cmd.rpm, 0, s.maxRpm);
      // a shot dip is recovered by the flywheel's closed loop quickly (flywheelRecoverTauS); spin-up from rest uses flywheelTauS.
      // Without this the rpm sags to ~86 % when firing every 0.15 s and every shot after the first falls short.
      const dipped = this._dipT > 0 && st.rpm >= tgt * (1 - 2 * s.rpmDipFrac - 0.02);
      const a = 1 - Math.exp(-dt / Math.max(1e-6, dipped ? s.flywheelRecoverTauS : s.flywheelTauS));
      if (this._dipT > 0) this._dipT = Math.max(0, this._dipT - dt);
      st.rpm += (tgt - st.rpm) * a;
    }
    st.rpm = clamp(st.rpm, 0, s.maxRpm);
    return { atTurretLimit: this.atLimit, atHoodLimit: hoodLimit };
  }
  kick() { this.state.rpm *= (1 - this.spec.rpmDipFrac); this._dipT = 0.6; }
  atSetpoint(cmd, tol = {}) {
    const t = { turretDeg: 1.5, hoodDeg: 1, rpm: 50, ...tol };
    const st = this.state;
    const ok = (c, v, tl) => c == null || Math.abs(c - v) <= tl;
    return {
      turret: ok(cmd.turretDeg, st.turretDeg, t.turretDeg),
      hood: ok(cmd.hoodDeg, st.hoodDeg, t.hoodDeg),
      rpm: ok(cmd.rpm, st.rpm, t.rpm),
    };
  }
}

export class ManualAim {
  constructor(opts = {}) {
    this.opts = {
      turretRate: 120, hoodRate: 30, rpmRate: 1500, fineScale: 0.25,
      turretMinDeg: DEFAULT_PLANT.turretMinDeg, turretMaxDeg: DEFAULT_PLANT.turretMaxDeg,
      hoodMinDeg: DEFAULT_PLANT.hoodMinDeg, hoodMaxDeg: DEFAULT_PLANT.hoodMaxDeg,
      maxRpm: DEFAULT_PLANT.maxRpm,
      presets: { low: 2200, mid: 3200, high: 4200, stop: 0 },
      ...opts,
    };
    this.opts.presets = { low: 2200, mid: 3200, high: 4200, stop: 0, ...(opts.presets || {}) };
    this.state = { turretDeg: 0, hoodDeg: this.opts.hoodMinDeg, rpm: 0 };
    this._prevSnap = false;
  }
  update(dt, input = {}) {
    const o = this.opts, st = this.state;
    const f = input.fine ? o.fineScale : 1;
    st.turretDeg = clamp(st.turretDeg + (input.turretAxis || 0) * o.turretRate * f * dt, o.turretMinDeg, o.turretMaxDeg);
    st.hoodDeg = clamp(st.hoodDeg + (input.hoodAxis || 0) * o.hoodRate * f * dt, o.hoodMinDeg, o.hoodMaxDeg);
    st.rpm = clamp(st.rpm + (input.rpmAxis || 0) * o.rpmRate * f * dt, 0, o.maxRpm);
    const p = input.rpmPreset || null;
    if (p === 'cycle') {
      if (this._prevPreset !== 'cycle') {
        this._cycleIdx = ((this._cycleIdx ?? -1) + 1) % PRESET_CYCLE.length;
        st.rpm = clamp(o.presets[PRESET_CYCLE[this._cycleIdx]], 0, o.maxRpm);
      }
    } else if (p && o.presets[p] != null) {
      st.rpm = clamp(o.presets[p], 0, o.maxRpm);
    }
    this._prevPreset = p;
    const snap = !!input.snapToAuto;
    if (snap && !this._prevSnap && input.sol && input.sol.ok !== false) {
      const sol = input.sol;
      const t = pickEquivalentTurretAngle(sol.turretDeg ?? sol.yawDeg, st.turretDeg, o.turretMinDeg, o.turretMaxDeg);
      if (t != null) st.turretDeg = t;
      const el = sol.elevDeg ?? sol.hoodDeg;
      if (Number.isFinite(el)) st.hoodDeg = clamp(el, o.hoodMinDeg, o.hoodMaxDeg);
      if (Number.isFinite(sol.rpm)) st.rpm = clamp(sol.rpm, 0, o.maxRpm);
    }
    this._prevSnap = snap;
    return st;
  }
}

export const AIM_KEYS = {
  turretLeft: 'arrowleft', turretRight: 'arrowright', hoodUp: 'arrowup', hoodDown: 'arrowdown',
  rpmDown: '[', rpmUp: ']', fine: 'shift', stop: 'z', snap: 'x', cycle: 'c',
};

export function aimInputFromKeys(keysSet) {
  const lower = new Set();
  if (keysSet) for (const k of keysSet) lower.add(String(k).toLowerCase());
  const hasKey = (k) => lower.has(k);
  const axis = (pos, neg) => (hasKey(pos) ? 1 : 0) - (hasKey(neg) ? 1 : 0);
  let rpmPreset = null;
  if (hasKey(AIM_KEYS.stop)) rpmPreset = 'stop';
  else if (hasKey(AIM_KEYS.cycle)) rpmPreset = 'cycle';
  return {
    // turret: left (ccw, +) / right (cw, -)
    turretAxis: axis(AIM_KEYS.turretLeft, AIM_KEYS.turretRight),
    hoodAxis: axis(AIM_KEYS.hoodUp, AIM_KEYS.hoodDown),
    rpmAxis: axis(AIM_KEYS.rpmUp, AIM_KEYS.rpmDown),
    fine: hasKey(AIM_KEYS.fine),
    rpmPreset, // 'cycle' is edge-detected inside ManualAim
    snapToAuto: hasKey(AIM_KEYS.snap),
  };
}

export function nextPreset(current) {
  const i = PRESET_CYCLE.indexOf(current);
  return PRESET_CYCLE[(i + 1) % PRESET_CYCLE.length];
}

export function aimInputFromGamepad(pad) {
  const out = { turretAxis: 0, hoodAxis: 0, rpmAxis: 0, fine: false, rpmPreset: null, snapToAuto: false };
  if (!pad) return out;
  const dz = (v) => (Math.abs(v || 0) < 0.1 ? 0 : v);
  const ax = pad.axes || [], b = pad.buttons || [];
  const pressed = (i) => !!(b[i] && (b[i].pressed || b[i].value > 0.5));
  out.turretAxis = -dz(ax[2]) || 0; // stick right = clockwise = negative
  out.hoodAxis = -dz(ax[3]) || 0;   // stick up (negative y) = hood up
  out.rpmAxis = (pressed(12) ? 1 : 0) - (pressed(13) ? 1 : 0);
  out.snapToAuto = pressed(2);
  if (pressed(1)) out.rpmPreset = 'stop';
  out.fine = pressed(4);
  return out;
}

export function modeCommand(mode, sol, manual, plantLimits = DEFAULT_PLANT, prev = null) {
  const L = { ...DEFAULT_PLANT, ...plantLimits };
  const m = manual && manual.state ? manual.state : (manual || { turretDeg: 0, hoodDeg: L.hoodMinDeg, rpm: 0 });
  if (mode === 'auto') {
    const hold = prev || { turretDeg: null, hoodDeg: null, rpm: null };
    if (!sol || !sol.ok) return { turretDeg: hold.turretDeg, hoodDeg: hold.hoodDeg, rpm: hold.rpm, label: 'no solution' };
    const cur = prev && prev.turretDeg != null ? prev.turretDeg : 0;
    const t = pickEquivalentTurretAngle(sol.turretDeg ?? sol.yawDeg, cur, L.turretMinDeg, L.turretMaxDeg);
    const el = sol.elevDeg ?? sol.hoodDeg;
    return {
      turretDeg: t != null ? t : hold.turretDeg,
      hoodDeg: Number.isFinite(el) ? clamp(el, L.hoodMinDeg, L.hoodMaxDeg) : hold.hoodDeg,
      rpm: Number.isFinite(sol.rpm) ? clamp(sol.rpm, 0, L.maxRpm) : hold.rpm,   // non-finite rpm holds the last command (was NaN/0)
      label: t != null ? 'auto' : 'turret out of range',
    };
  }
  if (mode === 'free') {
    return { turretDeg: m.turretDeg, hoodDeg: m.hoodDeg, rpm: clamp(m.rpm, 0, L.maxRpm * 1.5), label: 'free' };
  }
  return {
    turretDeg: m.turretDeg, hoodDeg: m.hoodDeg, rpm: clamp(m.rpm, 0, L.maxRpm),
    label: mode === 'assist' ? 'assist' : 'manual',
  };
}

export function describeMode(mode) {
  const keys = [
    'Arrow Left/Right: turret', 'Arrow Up/Down: hood', '[ / ]: rpm down/up', 'Shift: fine',
    'Z: rpm stop', 'X: snap to auto solution', 'C: cycle rpm preset',
    'Gamepad: right stick turret/hood, dpad up/down rpm, X snap, B stop, LB fine',
  ];
  switch (mode) {
    case 'auto': return { label: 'Auto', summary: 'Turret, hood and flywheel follow the ballistic solver.', keys: [] };
    case 'assist': return { label: 'Assist', summary: 'You set turret, hood and rpm; shot rules still apply.', keys };
    case 'manual': return { label: 'Manual', summary: 'You control everything with no help.', keys };
    case 'free': return { label: 'Free', summary: 'Sandbox: rpm may exceed the real maximum (up to 1.5x).', keys };
    default: return { label: String(mode), summary: 'Unknown mode.', keys: [] };
  }
}
