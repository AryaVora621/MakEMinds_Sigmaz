// Auto data model (pure, no DOM): a PathPlanner-style autonomous = path library + command tree. Inches/radians, Pedro frame.
//
// Auto (JSON, schema_version 1):
//   { schema_version:1, name, alliance:'blue'|'red', start:{x,y,h(rad)},
//     paths:{ id:{ segments:[pathing.js segments], markers:[{t:0..1, command:<node>}] } },
//     root:<node>, settings:{ vmax, aMax, matchSeconds } }
// Node (every node has a stable `id`):
//   {type:'follow', path:id} | {type:'wait', seconds} | {type:'waitUntil', condition, timeout}
//   {type:'command', name, args:{}} | {type:'sequential'|'parallel'|'race'|'deadline', children:[...], deadline?:index}
//   waitUntil conditions: 'ballCount>=N' (also > <= < ==), 'flywheelReady', 'aimed', 'hopperFull', 'hopperEmpty',
//   'timeElapsed:S' (absolute auto time, adapter.time() >= S). timeout seconds (0/absent = none).
//
// ADAPTER (sim-side interface passed to buildCommandTree). All methods are optional except follow:
//   follow(pathId) -> handle { start(), update(dt), done(), progress() 0..1, stop?() }
//   intake(on) shoot(args) shootAll(args) aim(on) setRpm(rpm) flower(deploy) gate(open) park(args)
//   custom(name, args)                      -- for 'custom:<name>' commands
//   ballCount() flywheelReady() aimed() hopperFull() hopperEmpty() time()   -- condition sources
//   Action methods are fire-and-forget; if one returns an object with done(), the command waits for it,
//   otherwise it lasts COMMAND_DEFS[name].simDuration seconds (scheduler time, so it is deterministic).
// Markers: a path's markers become commands run in parallel with the follow; each starts once
// handle.progress() >= t (or the follow is done).

import { routeEvents, isEventPreset } from './pathing.js';
import { parallel, race, deadline, sequential, waitCmd, instantCmd, waitUntilCmd } from './scheduler.js';

export const SCHEMA_VERSION = 1;
export const FIELD = 144;
const GROUPS = ['sequential', 'parallel', 'race', 'deadline'];
const TAU = Math.PI * 2;
const wrapAngle = a => { a = ((a % TAU) + TAU) % TAU; return a > Math.PI ? a - TAU : a; };

// ---------- command registry ----------
const D = (label, category, args, simDuration, javaCall, icon, description) => ({ label, category, args, simDuration, javaCall, icon, description });
export const COMMAND_DEFS = {
  shoot: D('Shoot', 'shooter', { count: { type: 'number', default: 1, min: 1, max: 3 } }, a => 0.4 * (a?.count ?? 1), 'shooter.shoot({count})', 'S', 'Fire `count` balls.'),
  shootAll: D('Shoot all', 'shooter', {}, 1.2, 'shooter.shootAll()', 'S*', 'Fire everything in the hopper.'),
  intakeOn: D('Intake on', 'intake', {}, 0, 'intake.on()', 'I+', 'Start the intake.'),
  intakeOff: D('Intake off', 'intake', {}, 0, 'intake.off()', 'I-', 'Stop the intake.'),
  aimOn: D('Aim on', 'shooter', {}, 0, 'turret.aimOn()', 'A+', 'Enable auto-aim.'),
  aimOff: D('Aim off', 'shooter', {}, 0, 'turret.aimOff()', 'A-', 'Disable auto-aim.'),
  autoAim: D('Auto aim', 'shooter', { shootAfter: { type: 'boolean', default: false }, track: { type: 'boolean', default: false }, timeout: { type: 'number', default: 4, min: 0.5, max: 10 } }, 0, 'shooter.autoAim({shootAfter, track, timeout})', 'AA', 'Align flywheel RPM, turret and hood to the target and turn on the transfer. Optional: shoot once aligned.'),
  setRpm: D('Set RPM', 'shooter', { rpm: { type: 'number', default: 3000, min: 0, max: 6000 } }, 0, 'shooter.setRpm({rpm})', 'RPM', 'Set the flywheel target speed.'),
  flowerDeploy: D('Flower deploy', 'mechanism', {}, 0.5, 'flower.deploy()', 'F+', 'Deploy the flower.'),
  flowerStow: D('Flower stow', 'mechanism', {}, 0.5, 'flower.stow()', 'F-', 'Stow the flower.'),
  gateOpen: D('Gate open', 'mechanism', {}, 0.2, 'gate.open()', 'G+', 'Open the gate.'),
  gateClose: D('Gate close', 'mechanism', {}, 0.2, 'gate.close()', 'G-', 'Close the gate.'),
  park: D('Park', 'endgame', {}, 0, 'park.run()', 'P', 'Run the park routine.'),
};
const CUSTOM_DEF = name => D(name, 'custom', {}, 0, `custom.${name}()`, 'C', 'User-defined command.');
export function getCommandDef(name) {
  if (COMMAND_DEFS[name]) return COMMAND_DEFS[name];
  if (typeof name === 'string' && /^custom:.+/.test(name)) return CUSTOM_DEF(name.slice(7));
  return null;
}
export function commandDuration(name, args) {
  const d = getCommandDef(name); if (!d) return 0;
  return typeof d.simDuration === 'function' ? d.simDuration(args || {}) : d.simDuration;
}

// ---------- conditions ----------
export function parseCondition(c) {
  if (typeof c !== 'string') return null;
  let m = /^ballCount\s*(>=|<=|==|>|<)\s*(\d+)$/.exec(c.trim());
  if (m) return { kind: 'ballCount', op: m[1], n: +m[2] };
  m = /^timeElapsed:(\d+(?:\.\d+)?)$/.exec(c.trim());
  if (m) return { kind: 'timeElapsed', s: +m[1] };
  if (['flywheelReady', 'aimed', 'hopperFull', 'hopperEmpty'].includes(c.trim())) return { kind: c.trim() };
  return null;
}
const CMP = { '>=': (a, b) => a >= b, '<=': (a, b) => a <= b, '==': (a, b) => a === b, '>': (a, b) => a > b, '<': (a, b) => a < b };

// ---------- model ----------
export function newAuto(opts = {}) {
  const a = {
    schema_version: SCHEMA_VERSION, name: opts.name || 'MyAuto', alliance: opts.alliance === 'red' ? 'red' : 'blue',
    start: { x: opts.alliance === 'red' ? 24 : 120, y: 24, h: 0, ...(opts.start || {}) }, paths: {},
    root: { id: 'n1', type: 'sequential', children: [] },
    settings: { vmax: 60, aMax: 90, matchSeconds: 30, ...(opts.settings || {}) },
  };
  return a;
}
export function cloneAuto(a) { return JSON.parse(JSON.stringify(a)); }

function allNodes(auto) { const out = []; walkNodes(auto, n => { out.push(n); }, { markers: true }); return out; }
function nextIdNum(auto) {
  let mx = 0; for (const n of allNodes(auto)) { const m = /^n(\d+)$/.exec(n.id || ''); if (m) mx = Math.max(mx, +m[1]); }
  return mx + 1;
}
/** Gives every node (including marker commands) a unique id; keeps existing unique ids. */
export function ensureIds(auto) {
  const seen = new Set(); let k = nextIdNum(auto);
  for (const n of allNodes(auto)) { if (!n.id || seen.has(n.id)) n.id = 'n' + (k++); seen.add(n.id); }
  return auto;
}
function assignIds(auto, node) {
  let k = nextIdNum(auto); const used = new Set(allNodes(auto).map(n => n.id));
  const rec = n => { if (!n.id || used.has(n.id)) { while (used.has('n' + k)) k++; n.id = 'n' + k; } used.add(n.id); (n.children || []).forEach(rec); };
  rec(node);
}

/** Pre-order walk: fn(node, parent, index, depth). Returning false from fn stops the walk. Accepts an auto or a node. */
export function walkNodes(auto, fn, { markers = false } = {}) {
  let stop = false;
  const rec = (n, p, i, d) => {
    if (stop) return;
    if (fn(n, p, i, d) === false) { stop = true; return; }
    (n.children || []).forEach((c, j) => rec(c, n, j, d + 1));
  };
  if (auto && auto.root) {
    rec(auto.root, null, 0, 0);
    if (markers) for (const p of Object.values(auto.paths || {})) for (const m of p.markers || []) rec(m.command, null, 0, 0);
  } else rec(auto, null, 0, 0);
}
export function findNode(auto, id) {
  let r = null;
  walkNodes(auto, (n, parent, index) => { if (n.id === id) { r = { node: n, parent, index }; return false; } });
  return r;
}
function isDescendant(node, id) { let f = false; walkNodes(node, n => { if (n.id === id) { f = true; return false; } }); return f; }

function insertChild(parent, node, index) {
  const arr = parent.children; const i = index === undefined || index > arr.length ? arr.length : Math.max(0, index);
  arr.splice(i, 0, node);
  if (parent.type === 'deadline' && i <= (parent.deadline ?? 0) && arr.length > 1) parent.deadline = (parent.deadline ?? 0) + 1;
  return i;
}
function detach(parent, index) {
  const [n] = parent.children.splice(index, 1);
  if (parent.type === 'deadline') {
    let d = parent.deadline ?? 0;
    if (index < d) d--; else if (index === d) d = 0;
    parent.deadline = Math.max(0, Math.min(d, parent.children.length - 1));
  }
  return n;
}
/** Adds `node` under group `parentId` (default root) at `index` (default end). Returns the node (ids assigned). */
export function addNode(auto, parentId, node, index) {
  const p = parentId == null ? { node: auto.root } : findNode(auto, parentId);
  if (!p || !GROUPS.includes(p.node.type)) throw new Error('addNode: parent is not a group');
  assignIds(auto, node); insertChild(p.node, node, index); return node;
}
export function removeNode(auto, id) {
  const f = findNode(auto, id); if (!f || !f.parent) return null;
  return detach(f.parent, f.index);
}
export function moveNode(auto, id, newParentId, index) {
  const f = findNode(auto, id); if (!f || !f.parent) return false;
  const np = findNode(auto, newParentId ?? auto.root.id);
  if (!np || !GROUPS.includes(np.node.type) || isDescendant(f.node, np.node.id)) return false;
  let idx = index;
  if (np.node === f.parent && idx !== undefined && idx > f.index) idx--;
  detach(f.parent, f.index); insertChild(np.node, f.node, idx); return true;
}
export function wrapInGroup(auto, id, type = 'sequential') {
  if (!GROUPS.includes(type)) throw new Error('wrapInGroup: bad group type');
  const f = findNode(auto, id); if (!f) return null;
  const g = { type, children: [f.node] }; if (type === 'deadline') g.deadline = 0;
  g.id = 'n' + nextIdNum(auto);
  if (f.parent) f.parent.children[f.index] = g; else { // wrapping the root
    auto.root = g; }
  return g;
}
/** Replaces a group by its children in the parent. Returns false for the root or non-groups. */
export function unwrap(auto, id) {
  const f = findNode(auto, id); if (!f || !f.parent || !GROUPS.includes(f.node.type)) return false;
  const p = f.parent, dl = p.type === 'deadline' ? p.deadline ?? 0 : null;
  p.children.splice(f.index, 1, ...f.node.children);
  if (dl !== null) { if (f.index < dl) p.deadline = dl + f.node.children.length - 1; else if (f.index === dl) p.deadline = f.index; }
  return true;
}
export function addMarker(auto, pathId, t, command) {
  const p = auto.paths[pathId]; if (!p) throw new Error('addMarker: unknown path');
  p.markers ||= []; assignIds(auto, command); p.markers.push({ t, command }); return command;
}

// ---------- paths ----------
function pathStart(p) { return p?.segments?.[0]?.points?.[0] || null; }
function pathEnd(p) { return p?.segments?.length ? p.segments.at(-1).points.at(-1) : null; }

// ---------- validation ----------
export function validateAuto(auto, opts = {}) {
  const issues = [], add = (level, code, message, extra = {}) => issues.push({ level, code, message, ...extra });
  const fb = { x0: 0, y0: 0, x1: FIELD, y1: FIELD, ...(opts.fieldBounds || {}) };
  const known = opts.knownCommands ? new Set(opts.knownCommands) : null;
  if (!auto || typeof auto !== 'object' || !auto.root) { add('error', 'no-root', 'Auto has no root node'); return issues; }
  const paths = auto.paths || {}; const used = new Set(); const ids = new Set();

  for (const [pid, p] of Object.entries(paths)) {
    (p.segments || []).forEach((s, si) => (s.points || []).forEach(q => {
      if (q.x < fb.x0 || q.x > fb.x1 || q.y < fb.y0 || q.y > fb.y1) add('error', 'out-of-bounds', `Path "${pid}" segment ${si} leaves the field (${q.x.toFixed?.(1)}, ${q.y.toFixed?.(1)})`, { pathId: pid });
    }));
    for (const m of p.markers || []) {
      if (typeof m.t !== 'number' || !(m.t >= 0 && m.t <= 1)) add('error', 'marker-range', `Marker on "${pid}" has t=${m.t}, must be within 0..1`, { pathId: pid, nodeId: m.command?.id });
      if (m.command) checkNode(m.command);
    }
  }

  function checkNode(n) {
    walkNodes(n, nd => {
      if (nd.id) { if (ids.has(nd.id)) add('error', 'duplicate-id', `Duplicate node id ${nd.id}`, { nodeId: nd.id }); ids.add(nd.id); }
      const ref = { nodeId: nd.id };
      if (GROUPS.includes(nd.type)) {
        if (!Array.isArray(nd.children) || nd.children.length === 0) add('warning', 'empty-group', `Empty ${nd.type} group`, ref);
        if (nd.type === 'deadline' && Array.isArray(nd.children) && nd.children.length && !(Number.isInteger(nd.deadline ?? 0) && (nd.deadline ?? 0) >= 0 && (nd.deadline ?? 0) < nd.children.length))
          add('error', 'bad-deadline', 'Deadline index is out of range', ref);
      } else if (nd.type === 'follow') {
        used.add(nd.path);
        if (!paths[nd.path]) add('error', 'unknown-path', `Follow references unknown path "${nd.path}"`, ref);
        else if (!paths[nd.path].segments?.length) add('error', 'empty-path', `Path "${nd.path}" has no segments`, { ...ref, pathId: nd.path });
      } else if (nd.type === 'wait') {
        if (typeof nd.seconds !== 'number' || !(nd.seconds >= 0)) add('error', 'bad-wait', 'wait needs seconds >= 0', ref);
      } else if (nd.type === 'waitUntil') {
        const c = parseCondition(nd.condition);
        if (!c) add('error', 'bad-condition', `Unknown condition "${nd.condition}"`, ref);
        else if (c.kind !== 'timeElapsed' && !(nd.timeout > 0)) add('warning', 'no-timeout', 'waitUntil has no timeout and may block the auto forever', ref);
      } else if (nd.type === 'command') {
        if (!getCommandDef(nd.name) || (known && !known.has(nd.name) && !String(nd.name).startsWith('custom:'))) add('error', 'unknown-command', `Unknown command "${nd.name}"`, ref);
        else {
          const def = getCommandDef(nd.name);
          for (const [k, spec] of Object.entries(def.args || {})) {
            const v = nd.args?.[k];
            if (v !== undefined && spec.type === 'boolean' && typeof v !== 'boolean') add('error', 'bad-arg', `Command "${nd.name}" arg ${k}=${v} must be true or false`, ref);
            if (v !== undefined && (spec.type === 'number' && (typeof v !== 'number' || !Number.isFinite(v) || (spec.min !== undefined && v < spec.min) || (spec.max !== undefined && v > spec.max))))
              add('error', 'bad-arg', `Command "${nd.name}" arg ${k}=${v} is invalid`, ref);
          }
        }
      } else add('error', 'unknown-node', `Unknown node type "${nd.type}"`, ref);
    });
  }
  checkNode(auto.root);

  // discontinuity: follow start vs where the previous path ended
  const walk = (n, pos) => {
    if (n.type === 'follow') {
      const p = paths[n.path]; if (!p || !p.segments?.length) return pos;
      const s = pathStart(p), e = pathEnd(p);
      if (pos && Math.hypot(s.x - pos.x, s.y - pos.y) > 2) add('warning', 'discontinuity', `Path "${n.path}" starts ${Math.hypot(s.x - pos.x, s.y - pos.y).toFixed(1)} in from where the robot will be`, { nodeId: n.id, pathId: n.path });
      return { x: e.x, y: e.y };
    }
    if (!GROUPS.includes(n.type) || !Array.isArray(n.children)) return pos;
    if (n.type === 'sequential') return n.children.reduce((p, c) => walk(c, p), pos);
    const outs = n.children.map(c => walk(c, pos));
    if (n.type === 'deadline') return outs[n.deadline ?? 0] || pos;
    for (let i = outs.length - 1; i >= 0; i--) if (outs[i] !== pos) return outs[i];
    return pos;
  };
  walk(auto.root, auto.start ? { x: auto.start.x, y: auto.start.y } : null);

  for (const pid of Object.keys(paths)) if (!used.has(pid)) add('warning', 'unused-path', `Path "${pid}" is never followed`, { pathId: pid });
  return issues;
}

// ---------- mirror ----------
export function mirrorAuto(auto) {
  const a = cloneAuto(auto), mh = h => (h === undefined ? undefined : wrapAngle(Math.PI - h));
  a.alliance = auto.alliance === 'red' ? 'blue' : auto.alliance === 'blue' ? 'red' : auto.alliance;
  a.start.x = FIELD - a.start.x; a.start.h = mh(a.start.h);
  for (const p of Object.values(a.paths)) for (const s of p.segments) {
    s.points.forEach(q => { q.x = FIELD - q.x; });
    if (s.heading && s.heading.mode !== 'tangent') {
      if (s.heading.start !== undefined) s.heading.start = mh(s.heading.start);
      if (s.heading.end !== undefined) s.heading.end = mh(s.heading.end);
    }
  }
  return a;
}

// ---------- duration estimate ----------
/** ctx = { pathTime(pathId) -> seconds }. waitUntil counts as its timeout (0 if none). */
export function estimateDuration(auto, ctx = {}) {
  const pt = id => (ctx.pathTime ? ctx.pathTime(id) || 0 : 0);
  const est = n => {
    switch (n.type) {
      case 'follow': return pt(n.path);
      case 'wait': return n.seconds || 0;
      case 'waitUntil': return n.timeout > 0 ? n.timeout : 0;
      case 'command': return commandDuration(n.name, n.args);
      case 'sequential': return n.children.reduce((s, c) => s + est(c), 0);
      case 'parallel': return Math.max(0, ...n.children.map(est));
      case 'race': return n.children.length ? Math.min(...n.children.map(est)) : 0;
      case 'deadline': return n.children.length ? est(n.children[n.deadline ?? 0]) : 0;
      default: return 0;
    }
  };
  return est(auto.root);
}

// ---------- JSON ----------
export function toJSON(auto) { return JSON.stringify(auto, null, 2); }
const fin = (v, p) => { if (typeof v !== 'number' || !Number.isFinite(v)) throw new Error(`${p}: not a finite number`); return v; };
function parseNode(o, p) {
  if (!o || typeof o !== 'object') throw new Error(`${p}: must be an object`);
  const n = { ...o };
  if (n.id !== undefined && typeof n.id !== 'string') throw new Error(`${p}.id: must be a string`);
  if (GROUPS.includes(n.type)) {
    if (!Array.isArray(n.children)) throw new Error(`${p}.children: must be an array`);
    n.children = n.children.map((c, i) => parseNode(c, `${p}.children[${i}]`));
    if (n.type === 'deadline') n.deadline = n.deadline === undefined ? 0 : fin(n.deadline, `${p}.deadline`);
  } else if (n.type === 'follow') { if (typeof n.path !== 'string') throw new Error(`${p}.path: must be a string`); }
  else if (n.type === 'wait') fin(n.seconds, `${p}.seconds`);
  else if (n.type === 'waitUntil') {
    if (typeof n.condition !== 'string') throw new Error(`${p}.condition: must be a string`);
    if (n.timeout !== undefined) fin(n.timeout, `${p}.timeout`);
  } else if (n.type === 'command') {
    if (typeof n.name !== 'string') throw new Error(`${p}.name: must be a string`);
    if (n.args !== undefined && (typeof n.args !== 'object' || n.args === null || Array.isArray(n.args))) throw new Error(`${p}.args: must be an object`);
  } else throw new Error(`${p}.type: unknown node type "${n.type}"`);
  return n;
}
export function fromJSON(text) {
  let o; try { o = JSON.parse(text); } catch (e) { throw new Error('Invalid JSON: ' + e.message); }
  if (!o || typeof o !== 'object' || Array.isArray(o)) throw new Error('Auto must be a JSON object');
  if (o.schema_version !== SCHEMA_VERSION) throw new Error(`Unsupported schema_version ${o.schema_version} (expected ${SCHEMA_VERSION})`);
  if (!o.start || typeof o.start !== 'object') throw new Error('start: missing');
  const a = newAuto({ name: String(o.name || 'MyAuto'), alliance: o.alliance, start: { x: fin(o.start.x, 'start.x'), y: fin(o.start.y, 'start.y'), h: fin(o.start.h ?? 0, 'start.h') }, settings: o.settings && typeof o.settings === 'object' ? o.settings : {} });
  if (o.paths === null || typeof o.paths !== 'object' || Array.isArray(o.paths)) throw new Error('paths: must be an object');
  for (const [id, p] of Object.entries(o.paths)) {
    const pp = `paths.${id}`;
    if (!p || !Array.isArray(p.segments)) throw new Error(`${pp}.segments: must be an array`);
    p.segments.forEach((s, i) => {
      if (s.type !== 'line' && s.type !== 'curve') throw new Error(`${pp}.segments[${i}].type: must be 'line' or 'curve'`);
      if (!Array.isArray(s.points) || s.points.length < 2 || (s.type === 'line' && s.points.length !== 2) || (s.type === 'curve' && s.points.length < 3)) throw new Error(`${pp}.segments[${i}].points: wrong count`);
      s.points.forEach((q, j) => { fin(q?.x, `${pp}.segments[${i}].points[${j}].x`); fin(q?.y, `${pp}.segments[${i}].points[${j}].y`); });
    });
    const markers = (p.markers || []).map((m, i) => ({ t: fin(m.t, `${pp}.markers[${i}].t`), command: parseNode(m.command, `${pp}.markers[${i}].command`) }));
    a.paths[id] = { segments: JSON.parse(JSON.stringify(p.segments)), markers };
  }
  a.root = parseNode(o.root, 'root');
  return ensureIds(a);
}

/** Smallest free name: `base`, then `base 2`, `base 3`, ... `names` is an array or Set of taken names. */
export function uniqueName(names, base = 'Untitled') {
  const taken = names instanceof Set ? names : new Set(names || []);
  const b = String(base ?? '').trim() || 'Untitled';
  if (!taken.has(b)) return b;
  for (let i = 2; ; i++) if (!taken.has(b + ' ' + i)) return b + ' ' + i;
}
/** Name for a copy of `name`: `name copy`, then `name copy 2`, ... unique against `names`. */
export function duplicateName(names, name) { return uniqueName(names, String(name ?? '').trim() + ' copy'); }

// ---------- auto library (several autos, localStorage-backed; storage is injectable) ----------
export const LIBRARY_KEY = 'biobuzz.autos.v1';
const libWrite = (st, autos) => { try { st.setItem(LIBRARY_KEY, JSON.stringify({ version: 1, autos })); return true; } catch (_) { return false; } };
/** { name: auto } (invalid entries are dropped silently). Each auto keeps its savedAt (list order) while stored. */
export function libraryRead(st) {
  try {
    const o = JSON.parse(st?.getItem(LIBRARY_KEY) || '{}'), out = {};
    for (const [k, v] of Object.entries(o && o.autos && typeof o.autos === 'object' ? o.autos : {})) {
      try { const a = fromJSON(JSON.stringify(v)); if (typeof v.savedAt === 'number') a.savedAt = v.savedAt; out[k] = a; } catch (_) { /* skip corrupt entry */ }
    }
    return out;
  } catch (_) { return {}; }
}
export function libraryList(st) {
  let raw = {}; try { raw = JSON.parse(st?.getItem(LIBRARY_KEY) || '{}').autos || {}; } catch (_) { /* empty */ }
  return Object.entries(libraryRead(st)).map(([name, a]) => ({ name, alliance: a.alliance, savedAt: raw[name]?.savedAt || 0, nodes: allNodes(a).length })).sort((x, y) => y.savedAt - x.savedAt || x.name.localeCompare(y.name));
}
export function librarySave(st, auto, now = Date.now()) {
  const all = libraryRead(st), name = String(auto.name || 'MyAuto'), a = cloneAuto(auto); a.savedAt = now;
  all[name] = a; return libWrite(st, all) ? name : null;
}
const stripSaved = a => { const c = cloneAuto(a); delete c.savedAt; return c; };
export function libraryLoad(st, name) { const a = libraryRead(st)[name]; return a ? stripSaved(a) : null; }
export function libraryDelete(st, name) { const all = libraryRead(st); if (!(name in all)) return false; delete all[name]; return libWrite(st, all); }
/** Copies `name` as `duplicateName(...)` (e.g. "Left9 copy"). Returns the new name or null. */
export function libraryDuplicate(st, name) {
  const all = libraryRead(st); if (!all[name]) return null;
  const nn = duplicateName(Object.keys(all), name);
  const a = cloneAuto(all[name]); a.name = nn; a.savedAt = Date.now(); all[nn] = a; return libWrite(st, all) ? nn : null;
}
/** Renames a stored auto. Returns the new name, `from` when unchanged, or null when `from` is missing, the new name is empty, or it is taken. */
export function libraryRename(st, from, to) {
  const all = libraryRead(st), nn = String(to ?? '').trim();
  if (!all[from] || !nn) return null;
  if (nn === from) return from;
  if (all[nn]) return null;
  const out = {}; for (const [k, v] of Object.entries(all)) { if (k === from) { v.name = nn; out[nn] = v; } else out[k] = v; }
  return libWrite(st, out) ? nn : null;
}
/** Moves an auto one place up (dir -1) or down (dir +1) in the list. Returns false at the ends or when the name is unknown. */
export function libraryMove(st, name, dir) {
  const list = libraryList(st).map(x => x.name), i = list.indexOf(name), j = i + (dir < 0 ? -1 : 1);
  if (i < 0 || j < 0 || j >= list.length) return false;
  [list[i], list[j]] = [list[j], list[i]];
  const all = libraryRead(st), base = Date.now();
  list.forEach((n, k) => { if (all[n]) all[n].savedAt = base + (list.length - k); });   // first in the list = largest savedAt
  return libWrite(st, all);
}
export function libraryExport(st) { return JSON.stringify({ library: 1, autos: Object.values(libraryRead(st)).map(stripSaved) }, null, 2); }
/** Accepts a library export or a single auto JSON; returns the number of autos imported (name clashes are overwritten). */
export function libraryImport(st, text) {
  let o; try { o = JSON.parse(text); } catch (e) { throw new Error('Invalid JSON: ' + e.message); }
  const list = o && o.library === 1 && Array.isArray(o.autos) ? o.autos : [o];
  const all = libraryRead(st); let n = 0;
  for (const x of list) { const a = fromJSON(JSON.stringify(x)); all[a.name] = a; n++; }
  if (!libWrite(st, all)) throw new Error('Storage is full or unavailable'); return n;
}

// ---------- command tree ----------
/** Path-planner event markers (segment callbacks whose name is a known command) as auto markers: [{t (0..1 of path length), command}]. */
export function callbackMarkers(path) {
  if (!path || !Array.isArray(path.segments) || !path.segments.length) return [];
  try {
    return routeEvents({ segments: path.segments }).filter(e => isEventPreset(e.name) && getCommandDef(e.name))
      .map(e => ({ t: e.frac, command: { type: 'command', name: e.name, args: { ...(e.args || {}) } }, fromCallback: true }));
  } catch (_) { return []; }
}
export function buildCommandTree(auto, adapter) {
  const tag = (c, n) => { c.node = n.id ?? null; return c; };
  const act = (fn, name, ms, args) => { // fire-and-forget action, lasts simDuration unless it returns a done() handle
    let h = null, el = 0; const dur = commandDuration(name, args);
    return {
      name, initialize() { el = 0; h = fn(); },
      execute(_c, dt) { el += dt; },
      isFinished() { return h && typeof h.done === 'function' ? !!h.done() : el >= dur - 1e-9; },
    };
  };
  const cond = (str, timeout) => {
    const c = parseCondition(str); if (!c) throw new Error(`Unknown condition "${str}"`);
    const a = adapter;
    const pred = () => {
      switch (c.kind) {
        case 'ballCount': return CMP[c.op](a.ballCount(), c.n);
        case 'timeElapsed': return a.time() >= c.s;
        default: return !!a[c.kind]();
      }
    };
    return waitUntilCmd(pred, timeout, `waitUntil(${str})`);
  };
  const cmdNode = n => {
    const args = n.args || {}, a = adapter, nm = n.name;
    if (!getCommandDef(nm)) return { name: nm, initialize() { throw new Error(`Unknown command "${nm}"`); } };
    const call = (m, ...x) => () => a[m]?.(...x);
    switch (nm) {
      case 'shoot': return act(call('shoot', args), nm, 0, args);
      case 'shootAll': return act(call('shootAll', args), nm, 0, args);
      case 'intakeOn': return act(call('intake', true), nm, 0, args);
      case 'intakeOff': return act(call('intake', false), nm, 0, args);
      case 'aimOn': return act(call('aim', true), nm, 0, args);
      case 'aimOff': return act(call('aim', false), nm, 0, args);
      case 'autoAim': return act(call('autoAim', args), nm, 0, args);   // adapter.autoAim({shootAfter}): runtime lives in the host / runner
      case 'setRpm': return act(call('setRpm', args.rpm ?? COMMAND_DEFS.setRpm.args.rpm.default), nm, 0, args);
      case 'flowerDeploy': return act(call('flower', true), nm, 0, args);
      case 'flowerStow': return act(call('flower', false), nm, 0, args);
      case 'gateOpen': return act(call('gate', true), nm, 0, args);
      case 'gateClose': return act(call('gate', false), nm, 0, args);
      case 'park': return act(call('park', args), nm, 0, args);
      default: return act(call('custom', nm.slice(7), args), nm, 0, args);
    }
  };
  const followNode = n => {
    const path = auto.paths[n.path], box = { h: null };
    const f = {
      name: `follow(${n.path})`,
      initialize() {
        if (!path) throw new Error(`Unknown path "${n.path}"`);
        box.h = adapter.follow(n.path); box.h.start();
      },
      execute(_c, dt) { box.h.update(dt); },
      isFinished() { return !!box.h.done(); },
      end(_c, interrupted) { if (interrupted) box.h?.stop?.(); },
    };
    tag(f, n);
    const markers = [...(path?.markers || []), ...callbackMarkers(path)].sort((x, y) => x.t - y.t);
    if (!markers.length) return f;
    const ms = markers.map(m => sequential(
      waitUntilCmd(() => !!box.h && (box.h.progress() >= m.t - 1e-9 || !!box.h.done()), 0, `marker@${m.t}`),
      build(m.command)));
    return Object.assign(parallel(f, ...ms), { name: `follow+markers(${n.path})`, node: null });
  };
  function build(n) {
    switch (n.type) {
      case 'follow': return followNode(n);
      case 'wait': return tag(waitCmd(n.seconds || 0), n);
      case 'waitUntil': return tag(cond(n.condition, n.timeout), n);
      case 'command': return tag(cmdNode(n), n);
      case 'sequential': return tag(sequential(...n.children.map(build)), n);
      case 'parallel': return tag(parallel(...n.children.map(build)), n);
      case 'race': return tag(race(...n.children.map(build)), n);
      case 'deadline': {
        if (!n.children.length) return tag(sequential(), n);
        const di = n.deadline ?? 0, kids = n.children.map(build);
        return tag(deadline(kids[di], ...kids.filter((_, i) => i !== di)), n);
      }
      default: return tag({ name: n.type, initialize() { throw new Error(`Unknown node type "${n.type}"`); } }, n);
    }
  }
  return build(auto.root);
}
