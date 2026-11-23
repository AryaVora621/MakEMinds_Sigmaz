// Driver-practice pure logic (no DOM, no three.js; unit-testable in node).
// Frame = PEDRO inches: x right, y toward the far wall, z up. Audience wall is y=0.
//
// ALLIANCE SIDES (from web/data/scene.json zones kind 'alliance_area'):
//   red  alliance_area x in [-53.65, 0.35]    -> red is the LOW-x side (left)
//   blue alliance_area x in [143.65, 197.65]  -> blue is the HIGH-x side (right)
//   Both areas span y 23.6..120.4. Per the manual (9.5, 10.3.2) drivers stand in their own alliance area
//   OUTSIDE the field beside it, NOT behind the audience wall. Red on the left (low x), blue on the right.
//
// ASSUMPTIONS (not from the rules):
//   * 'endgame' = last 30 s is a sim label only. Match 30/8/120 and the 60 s flower window are official (see matchrules.js).
//   * Gamepad/keyboard mappings, deadzone 0.08, drive accel/decel numbers, camera heights and limits.

import { RULES } from './matchrules.js';
import { MecanumDynamics, resolveParams } from './physics/mecanum-dynamics/index.js';
export { MecanumDynamics, resolveParams };

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const wrap = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };
const DEG = Math.PI / 180;

// ---------------------------------------------------------------- input
// Sign convention (matches app.js driveStep): +fwd = forward, +strafe = LEFT, +turn = counter-clockwise (left).

export const DEADZONE = 0.08;

/** Radial deadzone on a 2D stick with rescale so the output ramps 0..1 from the edge of the deadzone. */
export function radialDeadzone(x, y, dz = DEADZONE) {
  const m = Math.hypot(x, y);
  if (m <= dz) return [0, 0];
  const s = Math.min(1, (m - dz) / (1 - dz)) / m;
  return [x * s, y * s];
}
export function scalarDeadzone(v, dz = DEADZONE) {
  const a = Math.abs(v);
  if (a <= dz) return 0;
  return Math.sign(v) * Math.min(1, (a - dz) / (1 - dz));
}
/** Response curve: cubic = 0 linear, 1 pure cubic (finer control near centre). Preserves direction and +-1. */
export function applyCurve(v, cubic = 0) { return (1 - cubic) * v + cubic * v * v * v; }

function emptyCmd() {
  return {
    connected: false, fwd: 0, strafe: 0, turn: 0, shoot: false, fieldCentricToggle: false,
    aux: { a: false, b: false, x: false, y: false, lb: false, rb: false, lt: false, rt: false,
           back: false, start: false, ls: false, rs: false,
           dpadUp: false, dpadDown: false, dpadLeft: false, dpadRight: false },
  };
}

/** pads = navigator.getGamepads() (may contain nulls). Standard mapping. opts {deadzone, cubic}.
 *  Left stick = translate, right stick x = turn. Shoot = right trigger (>0.5). Field-centric toggle = Back button. */
export function readGamepad(pads, which = 0, opts = {}) {
  const out = emptyCmd();
  const gp = pads && pads[which];
  if (!gp) return out;
  const dz = opts.deadzone ?? DEADZONE, cubic = opts.cubic ?? 0;
  const ax = i => (gp.axes && Number.isFinite(gp.axes[i]) ? gp.axes[i] : 0);
  const btn = i => { const b = gp.buttons && gp.buttons[i]; return b ? (typeof b === 'object' ? !!b.pressed || (b.value ?? 0) > 0.5 : !!b) : false; };
  const val = i => { const b = gp.buttons && gp.buttons[i]; return b ? (typeof b === 'object' ? (b.value ?? (b.pressed ? 1 : 0)) : (b ? 1 : 0)) : 0; };
  const [sx, sy] = radialDeadzone(ax(0), ax(1), dz);
  const mag = Math.hypot(sx, sy);
  const k = mag > 0 ? applyCurve(mag, cubic) / mag : 0;
  out.connected = true;
  out.strafe = (-sx * k) || 0;               // stick right -> strafe right (negative "left")
  out.fwd = (-sy * k) || 0;                  // stick up (axis -1) -> forward
  out.turn = -applyCurve(scalarDeadzone(ax(2), dz), cubic) || 0;   // stick right -> clockwise
  const a = out.aux;
  a.a = btn(0); a.b = btn(1); a.x = btn(2); a.y = btn(3); a.lb = btn(4); a.rb = btn(5);
  a.lt = val(6) > 0.5; a.rt = val(7) > 0.5; a.back = btn(8); a.start = btn(9); a.ls = btn(10); a.rs = btn(11);
  a.dpadUp = btn(12); a.dpadDown = btn(13); a.dpadLeft = btn(14); a.dpadRight = btn(15);
  out.shoot = a.rt;
  out.fieldCentricToggle = a.back;
  return out;
}

/** keysSet: Set of lowercase key names. W/S fwd/back, A/D strafe left/right, Q/E turn left/right,
 *  Space = shoot, F = field-centric toggle. Diagonals are normalised to magnitude 1. */
export function mapKeyboard(keysSet) {
  const out = emptyCmd();
  const k = n => keysSet.has(n);
  out.connected = true;
  let f = (k('w') ? 1 : 0) - (k('s') ? 1 : 0), l = (k('q') ? 1 : 0) - (k('e') ? 1 : 0);
  const m = Math.hypot(f, l);
  if (m > 1) { f /= m; l /= m; }
  out.fwd = f; out.strafe = l;
  out.turn = (k('a') ? 1 : 0) - (k('d') ? 1 : 0);
  out.shoot = k(' ') || k('space');
  out.fieldCentricToggle = k('f');
  out.aux.lb = k('shift');
  return out;
}

// ---------------------------------------------------------------- drive

/**
 * Field-centric heading offset (rad, ccw, applied to the base frame forward=+y) for a driver standing in
 * their alliance area beside the field. Red stands on the low-x (left) side looking across toward +x, so
 * stick-up = +x: rotate +y by -90 deg. Blue stands on the high-x side looking toward -x: rotate +y by +90 deg.
 * Stick-left then maps to +y for red (toward the far wall) and -y for blue, as seen by that driver.
 */
export function driverForwardOffset(alliance) { return alliance === 'blue' ? Math.PI / 2 : -Math.PI / 2; }

/**
 * Mecanum kinematic drive with separate accel/decel limits.
 * state = {x, y, h, bodyV:[vf, vl, om]} (mutated: bodyV only). Heading h in rad, 0 = +x, pi/2 = +y.
 * Field-centric (FTC style): the BASE driver frame has forward = +y, left = -x. The actual driver frame is that
 * base rotated ccw by (driverForwardOffset(alliance) + headingOffset): the alliance part is applied when
 * opts.alliance (constructor or step option) is given, headingOffset (rad, ccw) is an extra IMU-style reset.
 * Robot-centric: fwd/strafe are along the robot's own axes.
 *
 * Models: opts.model = 'kinematic' (default, the slew-rate model above, unchanged) or 'dynamic' (opt-in
 * rigid-body mecanum model in physics/mecanum-dynamics; opts.spec = robot spec JSON or null, opts.dynamics =
 * parameter overrides). In dynamic mode state.bodyV is the momentum state (in/s, in/s, rad/s), maxSpeed/
 * maxAccel/strafeFactor etc. are ignored, the stick is normalised to the unit disc, and slip scales grip
 * (1 - 0.7*slip). step() never moves the pose. Per-wheel telemetry is left in this.lastTelemetry.
 */
export class MecanumDrive {
  constructor(opts = {}) {
    this.maxSpeed = opts.maxSpeed ?? 60;
    this.maxAccel = opts.maxAccel ?? 140;
    this.maxDecel = opts.maxDecel ?? this.maxAccel * 1.6;   // braking is harder than accelerating
    this.maxTurn = opts.maxTurn ?? 3.2;
    this.turnAccel = opts.turnAccel ?? this.maxTurn * 6;
    this.turnDecel = opts.turnDecel ?? this.turnAccel * 1.6;
    this.strafeFactor = opts.strafeFactor ?? 0.85;
    this.slip = opts.slip ?? 0.0;                           // 0..1: reduces grip (accel/decel) when >0
    this.alliance = opts.alliance ?? null;                  // 'red'|'blue' -> field-centric forward points across the field
    this._model = opts.model ?? 'kinematic';                // 'kinematic' | 'dynamic'
    if (this._model !== 'kinematic' && this._model !== 'dynamic') throw new Error('unknown drive model ' + this._model);
    this.lastTelemetry = null;                              // per-wheel telemetry of the last dynamic step
    this._dyn = this._model === 'dynamic'
      ? new MecanumDynamics({ spec: opts.spec ?? null, overrides: opts.dynamics ?? {} })
      : null;
  }

  get model() { return this._model; }

  _approach(cur, want, accel, decel, dt) {
    const braking = Math.abs(want) < Math.abs(cur) || cur * want < 0;
    const grip = 1 - 0.7 * clamp(this.slip, 0, 1);
    const lim = (braking ? decel : accel) * grip * dt;
    return cur + clamp(want - cur, -lim, lim);
  }

  step(cmd, dt, state, { fieldCentric = false, headingOffset: extraOffset = 0, alliance = this.alliance } = {}) {
    const headingOffset = (alliance ? driverForwardOffset(alliance) : 0) + extraOffset;
    if (!state.bodyV) state.bodyV = [0, 0, 0];
    const fwd = clamp(cmd.fwd || 0, -1, 1), str = clamp(cmd.strafe || 0, -1, 1), trn = clamp(cmd.turn || 0, -1, 1);
    const c = Math.cos(state.h), s = Math.sin(state.h);
    if (this._dyn) {
      let f, l;
      if (fieldCentric) {
        const dx = -str, dy = fwd;
        const co = Math.cos(headingOffset), so = Math.sin(headingOffset);
        const wx = dx * co - dy * so, wy = dx * so + dy * co;
        f = wx * c + wy * s;
        l = -wx * s + wy * c;
      } else { f = fwd; l = str; }
      const m = Math.hypot(f, l);
      if (m > 1) { f /= m; l /= m; }
      this._dyn.setGripScale(1 - 0.7 * clamp(this.slip, 0, 1));
      const r = this._dyn.step(state, { fwd: f, strafe: l, turn: trn }, dt);
      this.lastTelemetry = r.telemetry;
      return { vx: r.vx, vy: r.vy, om: r.om };
    }
    let wantF, wantL;
    if (fieldCentric) {
      // driver frame -> world: driver forward = +y, driver left = -x, then rotate by headingOffset
      const dx = -str, dy = fwd;
      const co = Math.cos(headingOffset), so = Math.sin(headingOffset);
      const wx = dx * co - dy * so, wy = dx * so + dy * co;
      wantF = wx * c + wy * s;       // world -> robot body
      wantL = -wx * s + wy * c;
      wantF *= this.maxSpeed; wantL *= this.maxSpeed * this.strafeFactor;
    } else {
      wantF = fwd * this.maxSpeed; wantL = str * this.maxSpeed * this.strafeFactor;
    }
    // Keep the commanded body speed within the maxSpeed circle (ellipse with strafeFactor)
    const e = Math.hypot(wantF / this.maxSpeed, wantL / (this.maxSpeed * this.strafeFactor));
    if (e > 1) { wantF /= e; wantL /= e; }
    const wantW = trn * this.maxTurn;
    const b = state.bodyV;
    b[0] = this._approach(b[0], wantF, this.maxAccel, this.maxDecel, dt);
    b[1] = this._approach(b[1], wantL, this.maxAccel, this.maxDecel, dt);
    b[2] = this._approach(b[2], wantW, this.turnAccel, this.turnDecel, dt);
    return { vx: c * b[0] - s * b[1], vy: s * b[0] + c * b[1], om: b[2] };
  }
}

/** Integrates the pose in place (call after collision resolution has adjusted vx, vy). Returns state. */
export function integrate(state, vx, vy, om, dt) {
  state.x += vx * dt; state.y += vy * dt; state.h = wrap(state.h + om * dt);
  return state;
}

// ---------------------------------------------------------------- match clock

export class MatchClock {
  constructor(cfg = {}) {
    this.cfg = { auto: 30, transition: 8, teleop: 120, endgame: 30, ...cfg };
    this._cbs = [];
    this.reset();
  }
  get total() { return this.cfg.auto + this.cfg.transition + this.cfg.teleop; }
  get secondsLeft() { return Math.max(0, this.total - this.elapsed); }
  /** Flower scoring is legal only in the last 60 s of the match (G410). False while idle. */
  get flowerWindowOpen() {
    return this.phase !== 'idle' && this.secondsLeft <= RULES.timing.endgameFlowerUnlockRemaining + 1e-9;
  }
  onPhase(cb) { this._cbs.push(cb); return () => { this._cbs = this._cbs.filter(f => f !== cb); }; }
  reset() { this.elapsed = 0; this.running = false; this.phase = 'idle'; this.remaining = 0; this.label = '0:00'; }
  start() { if (this.phase === 'idle') { this.running = true; this._set(this._phaseAt(0)); this._refresh(); } else if (this.phase !== 'over') this.running = true; }
  pause() { this.running = false; }
  _phaseAt(t) {
    t += 1e-9;   // tolerate float accumulation at boundaries
    const { auto, transition, teleop, endgame } = this.cfg;
    if (t < auto) return 'auto';
    if (t < auto + transition) return 'transition';
    if (t < auto + transition + teleop - endgame) return 'teleop';
    if (t < this.total) return 'endgame';
    return 'over';
  }
  _phaseEnd(p) {
    const { auto, transition, teleop, endgame } = this.cfg;
    return { auto, transition: auto + transition, teleop: this.total - endgame, endgame: this.total, over: this.total }[p];
  }
  _set(p) {
    if (p === this.phase) return;
    const prev = this.phase; this.phase = p;
    for (const cb of this._cbs) cb(p, prev);
  }
  _refresh() {
    this.remaining = this.phase === 'idle' || this.phase === 'over' ? 0 : Math.max(0, this._phaseEnd(this.phase) - this.elapsed);
    const t = Math.ceil(this.remaining - 1e-9);
    this.label = `${Math.floor(t / 60)}:${String(t % 60).padStart(2, '0')}`;
  }
  update(dt) {
    if (!this.running) return;
    this.elapsed = Math.min(this.total, this.elapsed + dt);
    // step through every phase crossed so callbacks fire in order even for a huge dt
    const order = ['auto', 'transition', 'teleop', 'endgame', 'over'];
    const target = this._phaseAt(this.elapsed);
    for (let i = order.indexOf(this.phase) + 1; i <= order.indexOf(target); i++) this._set(order[i]);
    if (this.phase === 'over') this.running = false;
    this._refresh();
  }
}

// ---------------------------------------------------------------- cameras

export const DRIVER_EYE_Z = 64;          // standing adult eye height, in
export const DRIVER_STATION_Y = 40;      // along the alliance area (y 23.6..120.4), near the audience end like real drive stations (assumption)
// Middle of each alliance area in x (red -53.65..0.35, blue 143.65..197.65), outside the field wall.
export const DRIVER_STATION_X = { red: -26.65, blue: 170.65 };
export const DRIVER_STATION = {
  red: [DRIVER_STATION_X.red, DRIVER_STATION_Y, DRIVER_EYE_Z],
  blue: [DRIVER_STATION_X.blue, DRIVER_STATION_Y, DRIVER_EYE_Z],
};
export const LOOK_LIMITS = { yawDeg: 70, pitchMinDeg: -45, pitchMaxDeg: 25 };
const FIELD_CENTRE = [72, 72, 0];

function lookAt(pos, base, look, limits = LOOK_LIMITS) {
  const dx = base[0] - pos[0], dy = base[1] - pos[1], dz = base[2] - pos[2];
  const yaw0 = Math.atan2(dx, dy);                   // 0 = looking +y, + = toward +x
  const pitch0 = Math.atan2(dz, Math.hypot(dx, dy));
  const yaw = yaw0 + clamp((look?.yawDeg || 0), -limits.yawDeg, limits.yawDeg) * DEG;
  const pitch = clamp(pitch0 + (look?.pitchDeg || 0) * DEG, limits.pitchMinDeg * DEG, limits.pitchMaxDeg * DEG);
  const R = 200;
  return [pos[0] + R * Math.cos(pitch) * Math.sin(yaw), pos[1] + R * Math.cos(pitch) * Math.cos(yaw), pos[2] + R * Math.sin(pitch)];
}

/**
 * Camera for practice. alliance 'red'|'blue'; mode 'driver'|'driver_wide'|'robot_chase'|'coach_box';
 * S = robot {x,y,h}; look = {yawDeg, pitchDeg} head-look offsets (clamped; yaw +-70, pitch -45..+25 absolute).
 * Returns {pos, target, fov(vertical degrees)}.
 * The driver stands in their alliance area beside the field (red x=-26.65, blue x=170.65, y=40, eye 64 in)
 * and looks across at the field centre; the yaw/pitch look offsets are relative to that gaze.
 * Realism: from there the centre is ~33 deg below horizontal, so far robots foreshorten.
 */
export function driverCamera(alliance, mode, S, look = {}) {
  const side = alliance === 'blue' ? 'blue' : 'red';
  switch (mode) {
    case 'driver_wide': {
      const pos = [...DRIVER_STATION[side]];
      return { pos, target: lookAt(pos, FIELD_CENTRE, look), fov: 85 };
    }
    case 'robot_chase': {
      const h = S?.h ?? Math.PI / 2, d = 36;
      const pos = [(S?.x ?? 72) - Math.cos(h) * d, (S?.y ?? 24) - Math.sin(h) * d, 26];
      const base = [(S?.x ?? 72) + Math.cos(h) * 30, (S?.y ?? 24) + Math.sin(h) * 30, 10];
      return { pos, target: lookAt(pos, base, look), fov: 70 };
    }
    case 'coach_box': {
      // same alliance side, raised (80 in) and nearer the field wall, looking across at the field centre
      const pos = [side === 'red' ? -12 : 156, 40, 80];
      return { pos, target: lookAt(pos, FIELD_CENTRE, look), fov: 60 };
    }
    case 'driver':
    default: {
      const pos = [...DRIVER_STATION[side]];
      return { pos, target: lookAt(pos, FIELD_CENTRE, look), fov: 60 };
    }
  }
}

// ---------------------------------------------------------------- scoring

// Derived from RULES (BIOBUZZ manual TU08). hive_* kinds are legacy names mapped to 'ball in up cell' bookkeeping.
// penalty is an ASSUMPTION (not researched).
export const POINTS = {
  ball: RULES.points.ballInGarden,
  hive_audience: RULES.points.ballInUpCellAtEnd,
  hive_far: RULES.points.ballInUpCellAtEnd,
  ballInUpCell: RULES.points.ballInUpCellAtEnd,
  tip: RULES.points.tipTeleop,
  penalty: -5,
};

/** @deprecated The host should use MatchScorer from matchrules.js. Kept for compatibility. */
export class ScoreKeeper {
  constructor(points = POINTS) { this.points = { ...points }; this.reset(); }
  reset() { this.totals = { red: 0, blue: 0 }; this.history = []; }
  addPoints(kind, alliance, t = 0) {
    if (!(alliance in this.totals)) throw new Error('unknown alliance ' + alliance);
    const pts = this.points[kind];
    if (pts === undefined) throw new Error('unknown score kind ' + kind);
    this.totals[alliance] += pts;
    this.history.push({ t, kind, alliance, points: pts, total: this.totals[alliance] });
    return this.totals[alliance];
  }
  total(alliance) { return this.totals[alliance]; }
}

// ---------------------------------------------------------------- practice stats

export class PracticeStats {
  constructor() { this.reset(); }
  reset() { this.shots = 0; this.scored = 0; this.misses = 0; this.cycles = []; this.events = []; }
  record(kind, value = 0, t = 0) {
    if (kind === 'shot') this.shots++;
    else if (kind === 'scored') this.scored++;
    else if (kind === 'miss') this.misses++;
    else if (kind === 'cycle') this.cycles.push(value);
    else throw new Error('unknown stat ' + kind);
    this.events.push({ t, kind, value });
  }
  summary() {
    const n = this.cycles.length, sum = this.cycles.reduce((a, b) => a + b, 0);
    return {
      shots: this.shots, scored: this.scored, misses: this.misses,
      accuracy: this.shots ? this.scored / this.shots : 0,
      cycles: n, avgCycle: n ? sum / n : 0, bestCycle: n ? Math.min(...this.cycles) : 0,
      cyclesPerMin: sum > 0 ? n * 60 / sum : 0,
    };
  }
}
