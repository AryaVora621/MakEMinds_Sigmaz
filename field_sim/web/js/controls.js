// controls.js - remappable keyboard controls with presets. Pure ES module + one DOM helper (createControlsPanel).
// Key names are lowercase KeyboardEvent.key values ('w', 'arrowup', ' ', 'shift'); see normalizeKey().
// Sign convention (same as driver.js): +fwd forward, +strafe LEFT, +turn counter-clockwise (left).

export const STORAGE_KEY = 'biobuzz.controls.v1';

export const ACTIONS = [
  'forward', 'backward', 'strafeLeft', 'strafeRight', 'turnLeft', 'turnRight',
  'shoot', 'fast', 'slow', 'intakeToggle', 'fieldCentric', 'resetRobot',
  'aimTurretLeft', 'aimTurretRight', 'aimHoodUp', 'aimHoodDown', 'aimRpmDown', 'aimRpmUp',
  'aimStop', 'aimSnap', 'aimCycle', 'aimFine',
];

export const ACTION_LABELS = {
  forward: 'Drive forward', backward: 'Drive backward', strafeLeft: 'Strafe left', strafeRight: 'Strafe right',
  turnLeft: 'Turn left (CCW)', turnRight: 'Turn right (CW)', shoot: 'Shoot', fast: 'Fast / boost', slow: 'Slow / precision',
  intakeToggle: 'Intake on/off', fieldCentric: 'Field-centric toggle', resetRobot: 'Reset robot',
  aimTurretLeft: 'Aim: turret left', aimTurretRight: 'Aim: turret right', aimHoodUp: 'Aim: hood up',
  aimHoodDown: 'Aim: hood down', aimRpmDown: 'Aim: RPM down', aimRpmUp: 'Aim: RPM up',
  aimStop: 'Aim: RPM stop', aimSnap: 'Aim: snap to auto', aimCycle: 'Aim: cycle RPM preset', aimFine: 'Aim: fine adjust',
};

/** Keys that can never be bound (browser / OS / modifiers / cancel key of the rebind prompt). */
export const RESERVED_KEYS = ['escape', 'tab', 'control', 'alt', 'meta', 'capslock', 'f5', 'f11', 'f12', 'contextmenu'];

/** Keys the host (app.js) uses for non-remappable things: T/Y tip, M view, L labels, digits, backtick. */
export const HOST_KEYS = ['t', 'y', 'm', 'l', '`', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'];

// Pairs of actions that may legitimately share a key (Shift = fast drive and fine aim).
const ALLOWED_SHARED = [['fast', 'aimFine']];

const AIM_DEFAULTS = {
  aimTurretLeft: ['arrowleft'], aimTurretRight: ['arrowright'], aimHoodUp: ['arrowup'], aimHoodDown: ['arrowdown'],
  aimRpmDown: ['['], aimRpmUp: [']'], aimStop: ['z'], aimSnap: ['x'], aimCycle: ['c'], aimFine: ['shift'],
};
const COMMON = { shoot: [' '], fast: ['shift'], slow: [], intakeToggle: ['i'], fieldCentric: ['f'], resetRobot: ['r'] };

export const PRESETS = {
  classic: {
    label: 'Classic',
    description: 'W/S forward-back, A/D turn left/right, Q/E strafe left/right. Aim keys: arrows, [ ], Z X C.',
    bindings: { forward: ['w'], backward: ['s'], turnLeft: ['a'], turnRight: ['d'], strafeLeft: ['q'], strafeRight: ['e'], ...COMMON, ...AIM_DEFAULTS },
  },
  wasd_strafe: {
    label: 'WASD strafe (old)',
    description: 'W/S forward-back, A/D strafe left/right, Q/E turn left/right. Aim keys: arrows, [ ], Z X C.',
    bindings: { forward: ['w'], backward: ['s'], strafeLeft: ['a'], strafeRight: ['d'], turnLeft: ['q'], turnRight: ['e'], ...COMMON, ...AIM_DEFAULTS },
  },
  literal: {
    label: 'Literal (Q/E strafe, W/S turn)',
    description: 'Q/E strafe left/right, W/S turn left/right, Arrow Up/Down forward/back (arrow Left/Right unused). '
      + 'Because the arrows drive, the aim keys move to J/K = turret left/right and U/N = hood up/down ([ ] Z X C unchanged).',
    bindings: {
      forward: ['arrowup'], backward: ['arrowdown'], strafeLeft: ['q'], strafeRight: ['e'], turnLeft: ['w'], turnRight: ['s'],
      ...COMMON, ...AIM_DEFAULTS,
      aimTurretLeft: ['j'], aimTurretRight: ['k'], aimHoodUp: ['u'], aimHoodDown: ['n'],
    },
  },
};

const ALIASES = {
  up: 'arrowup', down: 'arrowdown', left: 'arrowleft', right: 'arrowright',
  space: ' ', spacebar: ' ', esc: 'escape', ctrl: 'control', command: 'meta', cmd: 'meta', win: 'meta', os: 'meta',
};

/** Normalise any key name to the canonical lowercase form ('Arrow Up' -> 'arrowup', 'Space' -> ' ', 'W' -> 'w'). */
export function normalizeKey(k) {
  if (k == null) return '';
  let s = String(k);
  if (s === ' ') return ' ';
  s = s.toLowerCase();
  if (s.trim() === '') return ' ';
  s = s.trim();
  if (s.length === 1) return s;
  s = s.replace(/[\s_-]+/g, '');
  return ALIASES[s] ?? s;
}

export function displayKey(k) {
  const m = { ' ': 'Space', arrowup: 'Up', arrowdown: 'Down', arrowleft: 'Left', arrowright: 'Right', shift: 'Shift', '`': '`' };
  if (m[k]) return m[k];
  return k.length === 1 ? k.toUpperCase() : k.charAt(0).toUpperCase() + k.slice(1);
}

const clonePreset = id => {
  const p = PRESETS[id];
  if (!p) throw new Error('unknown preset ' + id);
  const out = {};
  for (const a of ACTIONS) out[a] = [...(p.bindings[a] || [])];
  return out;
};

function validateOverrides(overrides) {
  if (overrides == null) return {};
  if (typeof overrides !== 'object' || Array.isArray(overrides)) throw new Error('overrides must be an object');
  const out = {};
  for (const [a, keys] of Object.entries(overrides)) {
    if (!ACTIONS.includes(a)) throw new Error('unknown action ' + a);
    if (!Array.isArray(keys) || keys.some(k => typeof k !== 'string')) throw new Error('bindings for ' + a + ' must be an array of strings');
    const norm = [...new Set(keys.map(normalizeKey).filter(Boolean))];
    if (norm.some(k => RESERVED_KEYS.includes(k))) throw new Error('reserved key bound to ' + a);
    out[a] = norm;
  }
  return out;
}

export class KeyBindings {
  constructor(presetId = 'classic', overrides = {}) {
    const ov = validateOverrides(overrides);
    this.presetId = presetId;
    this.map = clonePreset(presetId);
    for (const [a, keys] of Object.entries(ov)) this.map[a] = keys;
  }
  setPreset(id) { this.map = clonePreset(id); this.presetId = id; return this; }
  reset() { return this.setPreset(this.presetId); }
  bindingsFor(action) {
    if (!ACTIONS.includes(action)) throw new Error('unknown action ' + action);
    return [...this.map[action]];
  }
  actionForKey(key) {
    const k = normalizeKey(key);
    return ACTIONS.find(a => this.map[a].includes(k)) ?? null;
  }
  has(action, key) { return this.map[action]?.includes(normalizeKey(key)) ?? false; }
  /** Bind `key` as the only key of `action`, removing it from every other action. Returns {ok, reason?, stolenFrom}. */
  rebind(action, key) {
    if (!ACTIONS.includes(action)) return { ok: false, reason: 'unknown action', stolenFrom: [] };
    const k = normalizeKey(key);
    if (!k) return { ok: false, reason: 'empty key', stolenFrom: [] };
    if (RESERVED_KEYS.includes(k)) return { ok: false, reason: `'${k}' is reserved`, stolenFrom: [] };
    const stolenFrom = [];
    for (const a of ACTIONS) {
      if (a === action) continue;
      if (this.map[a].includes(k)) {
        if (ALLOWED_SHARED.some(([x, y]) => (x === a && y === action) || (y === a && x === action))) continue;
        this.map[a] = this.map[a].filter(x => x !== k); stolenFrom.push(a);
      }
    }
    this.map[action] = [k];
    return { ok: true, stolenFrom };
  }
  /** Unbind a single key (or every key when key omitted) from an action. */
  unbind(action, key) {
    if (!ACTIONS.includes(action)) return false;
    this.map[action] = key == null ? [] : this.map[action].filter(x => x !== normalizeKey(key));
    return true;
  }
  /** [{key, actions:[...], reason:'duplicate'|'host'}] for keys bound to two actions or to host-reserved keys. */
  conflicts() {
    const byKey = new Map();
    for (const a of ACTIONS) for (const k of this.map[a]) { if (!byKey.has(k)) byKey.set(k, []); byKey.get(k).push(a); }
    const out = [];
    for (const [key, actions] of byKey) {
      const dup = actions.length > 1 && !(actions.length === 2 && ALLOWED_SHARED.some(([x, y]) => actions.includes(x) && actions.includes(y)));
      if (dup) out.push({ key, actions, reason: 'duplicate' });
      if (HOST_KEYS.includes(key)) out.push({ key, actions, reason: 'host' });
    }
    return out;
  }
  toJSON() {
    const base = PRESETS[this.presetId].bindings;
    const overrides = {};
    for (const a of ACTIONS) {
      const cur = this.map[a], def = base[a] || [];
      if (cur.length !== def.length || cur.some((k, i) => k !== def[i])) overrides[a] = [...cur];
    }
    return { version: 1, preset: this.presetId, overrides };
  }
  static fromJSON(obj) {
    if (typeof obj === 'string') obj = JSON.parse(obj);
    if (!obj || typeof obj !== 'object' || Array.isArray(obj)) throw new Error('bad controls JSON');
    if (obj.version !== 1) throw new Error('unsupported controls version');
    if (typeof obj.preset !== 'string' || !PRESETS[obj.preset]) throw new Error('unknown preset ' + obj.preset);
    return new KeyBindings(obj.preset, obj.overrides ?? {});
  }
  /** Load from storage.getItem(STORAGE_KEY); on missing/invalid data leaves this unchanged and returns false. */
  load(storage) {
    try {
      const raw = storage && storage.getItem(STORAGE_KEY);
      if (!raw) return false;
      const b = KeyBindings.fromJSON(JSON.parse(raw));
      this.presetId = b.presetId; this.map = b.map;
      return true;
    } catch { return false; }
  }
  save(storage) {
    try { storage.setItem(STORAGE_KEY, JSON.stringify(this.toJSON())); return true; } catch { return false; }
  }
}

/** Same shape as driver.js mapKeyboard (+ intakeToggle, resetRobot, slow). keysSet: lowercase held key names. */
export function keyboardCommand(keysSet, bindings, { fastScale = 1, slowScale = 0.6 } = {}) {
  const held = new Set();
  if (keysSet) for (const k of keysSet) held.add(normalizeKey(k));
  const on = a => bindings.map[a].some(k => held.has(k));
  const aux = { a: false, b: false, x: false, y: false, lb: false, rb: false, lt: false, rt: false,
    back: false, start: false, ls: false, rs: false, dpadUp: false, dpadDown: false, dpadLeft: false, dpadRight: false };
  let f = (on('forward') ? 1 : 0) - (on('backward') ? 1 : 0);
  let l = (on('strafeLeft') ? 1 : 0) - (on('strafeRight') ? 1 : 0);
  const m = Math.hypot(f, l);
  if (m > 1) { f /= m; l /= m; }
  const fast = on('fast'), slow = on('slow');
  const scale = (fast ? fastScale : 1) * (slow ? slowScale : 1);
  aux.lb = fast;
  return {
    connected: true,
    fwd: f * scale || 0, strafe: l * scale || 0,
    turn: (on('turnLeft') ? 1 : 0) - (on('turnRight') ? 1 : 0),
    shoot: on('shoot'), fieldCentricToggle: on('fieldCentric'),
    intakeToggle: on('intakeToggle'), resetRobot: on('resetRobot'),
    aux,
  };
}

/** Same shape as aimcontrol.js aimInputFromKeys but through the bindings (so presets can move the aim keys). */
export function aimInputFromBindings(keysSet, bindings) {
  const held = new Set();
  if (keysSet) for (const k of keysSet) held.add(normalizeKey(k));
  const on = a => bindings.map[a].some(k => held.has(k));
  const axis = (p, n) => (on(p) ? 1 : 0) - (on(n) ? 1 : 0);
  return {
    turretAxis: axis('aimTurretLeft', 'aimTurretRight'),
    hoodAxis: axis('aimHoodUp', 'aimHoodDown'),
    rpmAxis: axis('aimRpmUp', 'aimRpmDown'),
    fine: on('aimFine'),
    rpmPreset: on('aimStop') ? 'stop' : on('aimCycle') ? 'cycle' : null,
    snapToAuto: on('aimSnap'),
  };
}

/** True when the key is bound to a drive/aim action (host should preventDefault for these). */
export function isBoundKey(bindings, key) { return bindings.actionForKey(key) !== null; }

// ---------------------------------------------------------------- DOM panel

const STYLE_ID = 'biobuzz-controls-style';
const CSS = `
.cp-root{background:#0f0f14;color:#e6e6ee;font:12px/1.4 system-ui,sans-serif;padding:10px;border:1px solid #2a2a35;border-radius:8px;max-width:420px}
.cp-root select,.cp-root button{background:#16161c;color:#e6e6ee;border:1px solid #33333f;border-radius:5px;padding:3px 8px;font:inherit;cursor:pointer}
.cp-root button:hover{border-color:#c2427c}
.cp-desc{color:#9a9aab;margin:6px 0 8px}
.cp-table{width:100%;border-collapse:collapse}
.cp-table td{padding:3px 4px;border-top:1px solid #1f1f28}
.cp-chip{display:inline-block;background:#16161c;border:1px solid #c2427c;color:#f2c6d9;border-radius:4px;padding:0 6px;margin-right:3px;font-family:monospace}
.cp-none{color:#6a6a78}
.cp-warn{color:#ffb86b;margin:8px 0 0;padding-left:16px}
.cp-listening{background:#c2427c !important;color:#fff !important}
`;

export function createControlsPanel({ bindings, doc, onChange } = {}) {
  if (!bindings || !doc) throw new Error('createControlsPanel needs bindings and doc');
  if (!doc.getElementById(STYLE_ID)) {
    const st = doc.createElement('style'); st.id = STYLE_ID; st.textContent = CSS;
    (doc.head || doc.body).appendChild(st);
  }
  const mk = (tag, cls, text) => { const e = doc.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; };
  const clear = e => { while (e.firstChild) e.removeChild(e.firstChild); };
  const root = mk('div', 'cp-root');
  const select = mk('select');
  for (const id of Object.keys(PRESETS)) { const o = mk('option', '', PRESETS[id].label); o.value = id; select.appendChild(o); }
  const resetBtn = mk('button', 'cp-reset', 'Reset');
  const desc = mk('div', 'cp-desc');
  const table = mk('table', 'cp-table');
  const warn = mk('ul', 'cp-warn');
  root.appendChild(select); root.appendChild(resetBtn); root.appendChild(desc); root.appendChild(table); root.appendChild(warn);

  let listening = null;   // { action, btn, handler }
  const changed = () => { refresh(); if (onChange) onChange(bindings); };
  const stopListening = () => {
    if (!listening) return;
    doc.removeEventListener('keydown', listening.handler, true);
    listening = null;
  };
  const startListening = (action) => {
    stopListening();
    const handler = ev => {
      if (ev.preventDefault) ev.preventDefault();
      if (ev.stopPropagation) ev.stopPropagation();
      const k = normalizeKey(ev.key);
      if (k === 'escape') { stopListening(); refresh(); return; }
      const r = bindings.rebind(action, k);
      if (r.ok) { stopListening(); changed(); }
      else { stopListening(); refresh(); warn.appendChild(mk('li', '', r.reason)); }
    };
    listening = { action, handler };
    doc.addEventListener('keydown', handler, true);
    refresh();
  };

  function refresh() {
    select.value = bindings.presetId;
    desc.textContent = PRESETS[bindings.presetId].description;
    clear(table);
    for (const a of ACTIONS) {
      const tr = mk('tr'), name = mk('td', '', ACTION_LABELS[a]), keysTd = mk('td'), btnTd = mk('td');
      const keys = bindings.bindingsFor(a);
      if (!keys.length) keysTd.appendChild(mk('span', 'cp-none', 'unbound'));
      for (const k of keys) keysTd.appendChild(mk('span', 'cp-chip', displayKey(k)));
      const isL = listening && listening.action === a;
      const btn = mk('button', isL ? 'cp-listening' : '', isL ? 'Press a key (Esc cancels)' : 'Rebind');
      btn.addEventListener('click', () => startListening(a));
      btnTd.appendChild(btn);
      tr.appendChild(name); tr.appendChild(keysTd); tr.appendChild(btnTd); table.appendChild(tr);
    }
    clear(warn);
    for (const c of bindings.conflicts()) {
      const names = c.actions.map(x => ACTION_LABELS[x]).join(', ');
      warn.appendChild(mk('li', '', c.reason === 'host'
        ? `${displayKey(c.key)} is also used by the simulator (${names})`
        : `${displayKey(c.key)} is bound to several actions: ${names}`));
    }
  }
  select.addEventListener('change', () => { stopListening(); bindings.setPreset(select.value); changed(); });
  resetBtn.addEventListener('click', () => { stopListening(); bindings.reset(); changed(); });
  refresh();
  return { el: root, refresh };
}
