// inputlog.js - input quantisation, change-only tick-tagged input log (replay format v1, spec section 6).
// Same-engine bit-exact replay needs the recorded input to equal the applied input, so live input is
// quantised with quantizeInput() before use. Fiedler: record inputs only, tagged by tick, delta-encoded
// (https://gafferongames.com/post/deterministic_lockstep/). Tags: [assumed] unless noted.
import { TS } from './params.js';
import { hashState } from './statehash.js';

const MAX_BUTTONS = 32; // [buttons] uint32 bitmask width

// Quantise an analog axis in [-1,1] to an integer count in [-q, q]. NaN -> 0. Units: counts.
export function quantAxis(v, q = TS.AXIS_QUANT) {
  if (!(v === v)) return 0;
  const c = v > 1 ? 1 : v < -1 ? -1 : v;
  const r = Math.round(c * q);
  return r === 0 ? 0 : r; // normalise -0
}

function checkSchema(schema) {
  if (!schema || !Array.isArray(schema.axes) || !Array.isArray(schema.buttons)) {
    throw new Error('schema must be { axes: string[], buttons: string[] }');
  }
  if (schema.buttons.length > MAX_BUTTONS) throw new Error(`at most ${MAX_BUTTONS} buttons`);
}

export function encodeInput(input, schema) {
  checkSchema(schema);
  const axes = (input && input.axes) || {};
  const btns = (input && input.buttons) || {};
  const a = schema.axes.map((n) => quantAxis(Number(axes[n] ?? 0)));
  let b = 0;
  for (let i = 0; i < schema.buttons.length; i++) if (btns[schema.buttons[i]]) b |= (1 << i);
  return { a, b: b >>> 0 };
}

export function decodeInput(enc, schema) {
  checkSchema(schema);
  const axes = {};
  const buttons = {};
  const a = (enc && enc.a) || [];
  const b = ((enc && enc.b) || 0) >>> 0;
  for (let i = 0; i < schema.axes.length; i++) axes[schema.axes[i]] = (a[i] || 0) / TS.AXIS_QUANT;
  for (let i = 0; i < schema.buttons.length; i++) buttons[schema.buttons[i]] = ((b >>> i) & 1) === 1;
  return { axes, buttons };
}

export function quantizeInput(input, schema) {
  return decodeInput(encodeInput(input, schema), schema);
}

export function inputsEqual(encA, encB) {
  if (!encA || !encB) return encA === encB;
  if ((encA.b >>> 0) !== (encB.b >>> 0)) return false;
  if (encA.a.length !== encB.a.length) return false;
  for (let i = 0; i < encA.a.length; i++) if (encA.a[i] !== encB.a[i]) return false;
  return true;
}

export class InputRecorder {
  constructor({ schema, seed, meta = {}, initialHash = null }) {
    checkSchema(schema);
    this.schema = { axes: [...schema.axes], buttons: [...schema.buttons] };
    this.seed = seed >>> 0;
    this.meta = meta;
    this.initialHash = initialHash;
    this.entries = [];  // [tick, int[], mask]
    this.hashes = [];   // [tick, exactHex, quantHex]
    this._last = null;
  }

  record(tick, input) {
    const enc = encodeInput(input, this.schema);
    if (this._last && inputsEqual(this._last, enc)) return;
    this.entries.push([tick, enc.a, enc.b]);
    this._last = enc;
  }

  recordHash(tick, h) {
    this.hashes.push([tick, h.exact, h.quant]);
  }

  finish(endTick) {
    return new InputLog({
      version: TS.REPLAY_VERSION,
      seed: this.seed,
      hz: TS.PHYSICS_HZ,
      controlDivider: TS.CONTROL_DIVIDER,
      paramsHash: hashState(TS),
      meta: this.meta,
      initialHash: this.initialHash,
      schema: this.schema,
      inputs: this.entries.map((e) => [e[0], e[1].slice(), e[2]]),
      hashes: this.hashes.map((h) => h.slice()),
      endTick,
    });
  }
}

export class InputLog {
  constructor(obj) {
    this.version = obj.version ?? TS.REPLAY_VERSION;
    this.seed = obj.seed >>> 0;
    this.hz = obj.hz ?? TS.PHYSICS_HZ;
    this.controlDivider = obj.controlDivider ?? TS.CONTROL_DIVIDER;
    this.paramsHash = obj.paramsHash ?? null;
    this.meta = obj.meta ?? {};
    this.initialHash = obj.initialHash ?? null;
    this.schema = obj.schema;
    this.endTick = obj.endTick ?? 0;
    this.entries = (obj.inputs || []).map((e) => [e[0], e[1].slice(), e[2] >>> 0]);
    this.hashes = (obj.hashes || []).map((h) => [h[0], h[1], h[2]]);
    this._hashMap = new Map(this.hashes.map((h) => [h[0], { exact: h[1], quant: h[2] }]));
  }

  // Latest entry with entry.tick <= tick (binary search); before the first entry -> zeros.
  inputAt(tick) {
    const e = this.entries;
    let lo = 0;
    let hi = e.length - 1;
    let idx = -1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (e[mid][0] <= tick) { idx = mid; lo = mid + 1; } else hi = mid - 1;
    }
    if (idx < 0) return decodeInput({ a: [], b: 0 }, this.schema);
    return decodeInput({ a: e[idx][1], b: e[idx][2] }, this.schema);
  }

  hashAt(tick) {
    return this._hashMap.get(tick) || null;
  }

  toJSON() {
    return {
      version: this.version,
      seed: this.seed,
      hz: this.hz,
      controlDivider: this.controlDivider,
      paramsHash: this.paramsHash,
      meta: this.meta,
      initialHash: this.initialHash,
      schema: this.schema,
      inputs: this.entries.map((e) => [e[0], e[1].slice(), e[2]]),
      hashes: this.hashes.map((h) => h.slice()),
      endTick: this.endTick,
    };
  }

  static fromJSON(objOrString, { allowMismatch = false } = {}) {
    const obj = typeof objOrString === 'string' ? JSON.parse(objOrString) : objOrString;
    if (!obj || obj.version !== TS.REPLAY_VERSION) {
      throw new Error(`replay version ${obj && obj.version} unsupported (expected ${TS.REPLAY_VERSION})`);
    }
    if (!allowMismatch) {
      if (obj.hz !== TS.PHYSICS_HZ) throw new Error(`replay hz ${obj.hz} != current ${TS.PHYSICS_HZ}`);
      if (obj.controlDivider !== TS.CONTROL_DIVIDER) {
        throw new Error(`replay controlDivider ${obj.controlDivider} != current ${TS.CONTROL_DIVIDER}`);
      }
    }
    checkSchema(obj.schema);
    let prev = -Infinity;
    for (const e of obj.inputs || []) {
      if (!(e[0] > prev)) throw new Error('input ticks must be strictly increasing');
      prev = e[0];
    }
    return new InputLog(obj);
  }
}
