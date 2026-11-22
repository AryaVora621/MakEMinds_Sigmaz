// Auto planner UI, modelled on FRC PathPlanner's auto editor (see docs/RESEARCH_PATHPLANNER.md).
// Browser module, DOM built with createElement/textContent; the only innerHTML use is for the static inline-SVG icon constants below.
//
// const planner = createAutoPlanner({ leftEl, rightEl, timelineEl, getPaths, getHost, toast, onChange, onPreviewPath,
//                                     exportFn, pathSampler })
//   leftEl      project sidebar: Autos list (+), Paths list (+), command palette
//   rightEl     editor: auto name + alliance, command sequence tree, inspector, start state, validation, export
//   timelineEl  transport (play/pause/step/speed, Run in sim, Preview) + time ruler + command lanes + playhead
//   getPaths() -> { id: route }          routes from the path planner ({name, start, segments}); the shared path library is merged in
//   getHost()  -> host (see autorunner.js header) or null while the sim is not ready
//   exportFn(auto, style) -> string      style: 'statemachine' | 'ivy'
//   pathSampler(route) -> samples        optional; defaults to pathing.sampleRoute(route, 1)
//   toast(msg, kind?) onChange(auto) onPreviewPath(id)   optional callbacks (onPreviewPath = "open this path in the path editor")
// planner: { auto (getter), load(auto), loadJSON(text), save() -> JSON string, render(), undo(), redo(), run(opts), pause(),
//            stop(), stepOnce(), seek(t), tick(dt) /*call every sim frame*/, runner (getter), selected (getter),
//            select(id), insertFromPalette(kind, parentId?, index?), dropAt(data, targetId, zone), exportJava(style),
//            setAlliance(a), mirror(), newAuto(), handleKey(e), focus(), destroy() }
// Edits use 'change' events (not 'input') so typing is never interrupted by a re-render.

import {
  COMMAND_DEFS, getCommandDef, newAuto, cloneAuto, findNode, walkNodes, addNode, removeNode, moveNode, wrapInGroup, unwrap,
  addMarker, validateAuto, mirrorAuto, toJSON, fromJSON, parseCondition, ensureIds,
  libraryList, librarySave, libraryLoad, libraryDelete, libraryDuplicate, libraryExport, libraryImport,
  libraryRename, libraryMove, libraryRead, uniqueName, duplicateName,
} from './autos.js';
import { AutoRunner, buildTimeline } from './autorunner.js';
import { sampleRoute, estimateTime, routeLength, planTrajectory, sharedPathLibrary, PLAN_VIEW, retargetStart } from './pathing.js';

const GROUPS = ['sequential', 'parallel', 'race', 'deadline'];
const KIND_COLOR = { follow: '#4c8dff', intake: '#2fb37a', shooter: '#e0568f', mechanism: '#9b86f3', endgame: '#35b6e0', wait: '#7b8394', waitUntil: '#8f98ab', group: '#c4cad6', custom: '#d9893f' };
const KIND_ICON = { follow: 'route', wait: 'clock', waitUntil: 'clock', sequential: 'seq', parallel: 'par', race: 'race', deadline: 'dead', intake: 'intake', shooter: 'target', mechanism: 'cog', endgame: 'flag', custom: 'bolt' };
const PALETTE = [
  ['Drive', [['follow', 'Follow path']]],
  ['Intake', [['intakeOn', 'Intake on'], ['intakeOff', 'Intake off']]],
  ['Shoot', [['shoot', 'Shoot'], ['shootAll', 'Shoot all'], ['autoAim', 'Auto aim'], ['setRpm', 'Set RPM'], ['aimOn', 'Aim on'], ['aimOff', 'Aim off']]],
  ['Mechanisms', [['flowerDeploy', 'Flower deploy'], ['flowerStow', 'Flower stow'], ['gateOpen', 'Gate open'], ['gateClose', 'Gate close'], ['park', 'Park']]],
  ['Wait', [['wait', 'Wait'], ['waitUntil', 'Wait until']]],
  ['Groups', [['sequential', 'Sequential'], ['parallel', 'Parallel'], ['race', 'Race'], ['deadline', 'Deadline']]],
];
const paletteKind = k => (k === 'follow' ? 'follow' : GROUPS.includes(k) ? k : k === 'wait' || k === 'waitUntil' ? k : (COMMAND_DEFS[k]?.category || 'custom'));
const COND_KINDS = [['ballCount', 'Ball count'], ['flywheelReady', 'Flywheel ready'], ['aimed', 'Aimed'], ['hopperFull', 'Hopper full'], ['hopperEmpty', 'Hopper empty'], ['timeElapsed', 'Time elapsed (s)']];
const LANE_H = 26, RULER_H = 26;
const TIP_FOR = {
  run: 'Run this auto in the live 3D simulator from the start', preview: 'Start paused so you can scrub the timeline and watch the robot', pause: 'Play / pause (runs the auto if it is not running)',
  stop: 'Stop and put the robot back at the start', step: 'Advance one frame (1/30 s)', 'tl-back': 'Scrub back 0.1 s', 'tl-fwd': 'Scrub forward 0.1 s', 'tl-first': 'Jump to the start', 'tl-last': 'Jump to the end',
  undo: 'Undo (Ctrl+Z)', redo: 'Redo (Ctrl+Y)', new: 'New auto', 'save-json': 'Download this auto as JSON', 'load-json': 'Load an auto from a JSON file',
  'export-statemachine': 'Export Pedro 2.1.0 Java (switch state machine, runs on any OpMode)', 'export-ivy': 'Export Pedro Ivy Java (command based)',
  mirror: 'Mirror the whole auto to the other alliance', 'full-field': 'Show the whole field instead of only the chosen alliance half', 'lib-save': 'Save this auto to the project', 'lib-export': 'Download all autos as JSON', 'lib-import': 'Import autos from a JSON file',
  'lib-load': 'Open this auto', 'lib-dup': 'Duplicate', 'lib-del': 'Delete', 'lib-rename': 'Rename (Enter saves, Esc cancels)', 'lib-up': 'Move up in the list', 'lib-down': 'Move down in the list',
  'path-new': 'New path', preview2: 'Open in the path editor', 'add-follow': 'Add a Follow path command for this path',
  'path-dup': 'Duplicate path', 'path-rename': 'Rename path (Enter saves, Esc cancels)', 'path-del': 'Delete path', 'path-up': 'Move path up', 'path-down': 'Move path down', 'undo-list': 'Undo the last delete',
  preview3: '', up: 'Move up', down: 'Move down', delete: 'Delete command', toggle: 'Collapse / expand', indent: 'Indent into the group above', outdent: 'Move out of the parent group', unwrap: 'Ungroup (keep the children)',
};
const ICONS = {
  play: '<path d="M7 4.5v15l12-7.5z" fill="currentColor" stroke="none"/>', pause: '<path d="M7 4h3.5v16H7zM13.5 4H17v16h-3.5z" fill="currentColor" stroke="none"/>', stop: '<rect x="6" y="6" width="12" height="12" rx="1.5" fill="currentColor" stroke="none"/>',
  first: '<path d="M6 5v14"/><path d="M19 5L9 12l10 7z" fill="currentColor"/>', last: '<path d="M18 5v14"/><path d="M5 5l10 7-10 7z" fill="currentColor"/>',
  back: '<path d="M18 5l-9 7 9 7z" fill="currentColor"/><path d="M6 5v14"/>', fwd: '<path d="M6 5l9 7-9 7z" fill="currentColor"/><path d="M18 5v14"/>',
  eye: '<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>', undo: '<path d="M9 7L4 12l5 5"/><path d="M4 12h10a6 6 0 010 8"/>', redo: '<path d="M15 7l5 5-5 5"/><path d="M20 12H10a6 6 0 000 8"/>',
  plus: '<path d="M12 5v14M5 12h14"/>', save: '<path d="M5 4h11l3 3v13H5z"/><path d="M8 4v5h7V4M8 20v-6h8v6"/>', load: '<path d="M4 7h6l2 2h8v10H4z"/>', download: '<path d="M12 4v12M7 11l5 5 5-5M5 20h14"/>',
  mirror: '<path d="M12 3v18"/><path d="M8 7L3 12l5 5zM16 7l5 5-5 5z"/>', copy: '<rect x="8" y="8" width="12" height="12" rx="2"/><path d="M16 8V5a1 1 0 00-1-1H5a1 1 0 00-1 1v10a1 1 0 001 1h3"/>',
  trash: '<path d="M5 7h14M10 7V4h4v3M7 7l1 13h8l1-13"/>', pencil: '<path d="M4 20l1-4L16 5l3 3L8 19z"/><path d="M14 7l3 3"/>', chev: '<path d="M9 6l6 6-6 6"/>',
  grip: '<circle cx="9" cy="6" r="1.4" fill="currentColor" stroke="none"/><circle cx="15" cy="6" r="1.4" fill="currentColor" stroke="none"/><circle cx="9" cy="12" r="1.4" fill="currentColor" stroke="none"/><circle cx="15" cy="12" r="1.4" fill="currentColor" stroke="none"/><circle cx="9" cy="18" r="1.4" fill="currentColor" stroke="none"/><circle cx="15" cy="18" r="1.4" fill="currentColor" stroke="none"/>',
  up: '<path d="M6 15l6-6 6 6"/>', down: '<path d="M6 9l6 6 6-6"/>', indent: '<path d="M4 6h16M12 12h8M12 18h8"/><path d="M4 10l4 3-4 3z" fill="currentColor"/>', outdent: '<path d="M4 6h16M12 12h8M12 18h8"/><path d="M8 10l-4 3 4 3z" fill="currentColor"/>',
  unwrap: '<path d="M4 8h7M13 16h7M4 16h3M17 8h3"/>', clear: '<path d="M6 6l12 12M18 6L6 18"/>',
  route: '<circle cx="5.5" cy="18" r="2"/><circle cx="18.5" cy="6" r="2"/><path d="M7.5 18H13a4 4 0 000-8h-2a4 4 0 010-8h5.5"/>', clock: '<circle cx="12" cy="12" r="8"/><path d="M12 7v5l3 2"/>',
  seq: '<path d="M5 6h10M5 12h14M5 18h10"/>', par: '<path d="M8 5v14M16 5v14"/>', race: '<path d="M6 21V4M6 5h12l-3 4 3 4H6"/>', dead: '<circle cx="12" cy="13" r="7"/><path d="M12 9v4M9 3h6"/>',
  intake: '<path d="M4 12h12M11 7l5 5-5 5M20 5v14"/>', target: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3.5"/>', cog: '<circle cx="12" cy="12" r="3"/><path d="M12 3v3M12 18v3M3 12h3M18 12h3M5.6 5.6l2.1 2.1M16.3 16.3l2.1 2.1M18.4 5.6l-2.1 2.1M7.7 16.3l-2.1 2.1"/>',
  flag: '<path d="M6 21V4M6 5h11l-2.5 4L17 13H6"/>', bolt: '<path d="M13 3L5 14h6l-1 7 8-11h-6z"/>', folder: '<path d="M3 7h7l2 2h9v10H3z"/>', left: '<path d="M15 6l-6 6 6 6"/>', doc: '<path d="M6 3h8l4 4v14H6z"/><path d="M14 3v4h4"/>',
  warn: '<path d="M12 4l9 16H3z"/><path d="M12 10v4M12 17v.5"/>', check: '<path d="M5 12l5 5 9-10"/>',
  edit: '<path d="M4 20h4L19 9l-4-4L4 16z"/><path d="M13.5 6.5l4 4"/>',
};
const ico = (doc, name, size = 16) => {
  const s = doc.createElement('span'); s.className = 'ap-ico'; s.setAttribute('aria-hidden', 'true');
  s.innerHTML = `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">${ICONS[name] || ''}</svg>`;
  return s;
};
// Fallback stylesheet (used only when planner.css is not linked); the real styles live in css/planner.css.
const CSS = `.ap-root{color:#e8ecf4;font:12px/1.4 system-ui,sans-serif}.ap-btn{background:#1d2330;color:#e8ecf4;border:1px solid #2c3446;border-radius:6px;padding:3px 8px;cursor:pointer}.ap-in{background:#0d1017;color:#e8ecf4;border:1px solid #2c3446;border-radius:5px}`;

function el(doc, tag, props = {}, ...kids) {
  const e = doc.createElement(tag);
  if (props.cls) e.className = props.cls;
  if (props.text !== undefined) e.textContent = String(props.text);
  if (props.attrs) for (const [k, v] of Object.entries(props.attrs)) if (v !== undefined && v !== null) e.setAttribute(k, String(v));
  if (props.style) Object.assign(e.style, props.style);
  if (props.on) for (const [k, f] of Object.entries(props.on)) e.addEventListener(k, f);
  for (const k of kids.flat()) if (k) e.appendChild(k);
  if (props.value !== undefined) e.value = props.value;
  if (props.checked !== undefined) e.checked = props.checked;
  if (props.disabled !== undefined) e.disabled = props.disabled;
  return e;
}
const keepId = c => { try { return c.getAttribute && c.getAttribute('id') === 'timeline-resize'; } catch (_) { return false; } };
function clear(e) { for (const c of [...(e.children || [])]) if (!keepId(c)) e.removeChild(c); if (!e.children) while (e.firstChild) e.removeChild(e.firstChild); }
const fmt = v => (Number.isInteger(v) ? String(v) : (+v).toFixed(2).replace(/0+$/, '').replace(/\.$/, ''));
const deg = r => (r * 180) / Math.PI, rad = d => (d * Math.PI) / 180;

export function createAutoPlanner(opts = {}) {
  const { leftEl, rightEl, timelineEl } = opts;
  const doc = (leftEl || rightEl || timelineEl)?.ownerDocument || (typeof document !== 'undefined' ? document : null);
  if (!doc) throw new Error('createAutoPlanner: no document');
  const toast = (m, k) => { try { opts.toast?.(m, k); } catch (_) { /* ignore */ } };
  const sampler = opts.pathSampler || (route => sampleRoute(route, 1));
  const pathLib = opts.pathLibrary !== undefined ? opts.pathLibrary : sharedPathLibrary;

  const hasSheet = !!(doc.querySelector && doc.querySelector('link[href*="planner.css"]'));
  if (doc.head && !hasSheet && !(doc.getElementById && doc.getElementById('autoplanner-style'))) {
    const st = el(doc, 'style', { attrs: { id: 'autoplanner-style' }, text: CSS }); doc.head.appendChild(st);
  }
  for (const r of [leftEl, rightEl, timelineEl]) if (r) r.className = ((r.className || '') + ' ap-root').trim();
  if (leftEl) leftEl.className += ' ap-side'; if (rightEl) rightEl.className += ' ap-edit';

  // ---------- state ----------
  let auto = newAuto(); ensureIds(auto);
  let selected = null, runner = null, runnerJson = '', bars = [], playT = 0, speed = 1, active = false, lastState = 'idle', libName = null;
  const collapsed = new Set(), closedSecs = new Set(['start', 'exp', 'val']), srcSig = {}, infoCache = new Map(), rowEls = new Map(), barEls = new Map();
  let hist = [JSON.stringify(auto)], pos = 0, timeLabel = null, playhead = null, runBtns = {}, planner_importLibrary = null, tlPps = 24;
  let renameKey = null, undoBar = null, undoTimer = null;   // list state: 'lib:<auto>' / 'path:<id>' while renaming; last delete (undoable for UNDO_MS)
  const UNDO_MS = 8000;
  // collapsed sections are remembered (localStorage); first run starts with the advanced cards closed
  const CLOSED_KEY = 'biobuzz.autoui.closed.v1';
  try { const raw = typeof localStorage !== 'undefined' ? localStorage.getItem(CLOSED_KEY) : null; if (raw) { closedSecs.clear(); for (const k of JSON.parse(raw)) closedSecs.add(k); } } catch (_) { /* default set */ }
  const saveClosed = () => { try { if (typeof localStorage !== 'undefined') localStorage.setItem(CLOSED_KEY, JSON.stringify([...closedSecs])); } catch (_) { /* ignore */ } };
  let tlMin = true;   // timeline starts as one compact transport row; the chevron expands the ruler and lanes
  try { const v = typeof localStorage !== 'undefined' ? localStorage.getItem('biobuzz.autoui.tlmin.v1') : null; if (v !== null) tlMin = v === '1'; } catch (_) { /* default */ }

  // ---------- paths ----------
  const hostPaths = () => {
    let p = {}; try { p = opts.getPaths?.() || {}; } catch (_) { p = {}; }
    if (p instanceof Map) p = Object.fromEntries(p);
    if (Array.isArray(p)) p = Object.fromEntries(p.map((r, i) => [r.id ?? r.name ?? 'path' + (i + 1), r]));
    let items = {}; try { items = pathLib ? pathLib.all() : {}; } catch (_) { items = {}; }
    if (pathLib || Object.keys(items).length) { p = { ...p }; delete p.route; }   // the live editor route is only a library item; with an emptied library it must not show up as a ghost path      // the live editor route is one of the library items
    return { ...p, ...items };
  };
  const pathIds = () => [...new Set([...Object.keys(hostPaths()), ...Object.keys(auto.paths || {})])];
  const sigOf = r => JSON.stringify(r?.segments || []);
  function routeOf(id) {
    const r = hostPaths()[id] || auto.paths[id]; if (!r) return null;
    return { name: r.name || id, alliance: auto.alliance, start: r.start || (r.segments?.[0]?.points?.[0] ? { ...r.segments[0].points[0], h: auto.start.h } : auto.start), segments: r.segments || [] };
  }
  function pathInfo(id) {
    const r = routeOf(id); if (!r) return { len: 0, time: 0 };
    const key = id + sigOf(r) + auto.settings.vmax + auto.settings.aMax, c = infoCache.get(id);
    if (c && c.key === key) return c.v;
    let v = { len: 0, time: 0 };
    try { v = { len: routeLength(r), time: estimateTime(r, auto.settings.vmax, auto.settings.aMax) }; } catch (_) { /* keep zeros */ }
    const ft = followerTimes.get(id); if (ft && ft.time != null && ft.sig === sigOf(r) + auto.settings.vmax + auto.settings.aMax) v = { ...v, time: ft.time };
    infoCache.set(id, { key, v }); return v;
  }
  // Time-parameterised motion from the real Pedro follower (pedrofollow.js). Only used in a served page and only when that module
  // produced a trajectory; otherwise the built-in trapezoid model keeps driving (and node tests stay offline).
  const wantFollower = opts.trajectorySampler !== undefined ? !!opts.trajectorySampler : (typeof location !== 'undefined' && /^https?:/.test(String(location.protocol)));
  const trajSampler = !wantFollower ? null : typeof opts.trajectorySampler === 'function' ? opts.trajectorySampler
    : route => planTrajectory(route, { vmax: auto.settings.vmax, aMax: auto.settings.aMax }).then(r => (r.source === 'pedro-follower' ? r.traj : null));
  const followerTimes = new Map();
  function refineTimes() {
    if (!trajSampler) return;
    for (const id of usedPaths()) {
      const r = routeOf(id); if (!r || !r.segments.length) continue;
      const sig = sigOf(r) + auto.settings.vmax + auto.settings.aMax, c = followerTimes.get(id); if (c && c.sig === sig) continue;
      followerTimes.set(id, { sig, time: null });
      Promise.resolve().then(() => trajSampler(r)).then(tr => {
        const a = Array.isArray(tr) ? tr : tr && tr.traj; if (!a || a.length < 2) return;
        followerTimes.set(id, { sig, time: a.at(-1).t }); infoCache.delete(id); renderTimeline();
      }).catch(() => {});
    }
  }
  const pathTime = id => pathInfo(id).time;
  function usedPaths() { const s = new Set(); walkNodes(auto, n => { if (n.type === 'follow' && n.path) s.add(n.path); }); return s; }
  /** Copies host routes into auto.paths for every followed path (keeps markers); refreshes when the host route changed. */
  function syncPaths() {
    const hp = hostPaths();
    for (const id of usedPaths()) {
      const r = hp[id]; if (!r || !r.segments) continue;
      const sig = sigOf(r);
      if (!auto.paths[id]) auto.paths[id] = { segments: JSON.parse(sig), markers: [] };
      else if (srcSig[id] !== sig) auto.paths[id].segments = JSON.parse(sig);
      srcSig[id] = sig;
    }
  }
  function markSynced() { const hp = hostPaths(); for (const id of Object.keys(auto.paths)) if (hp[id]) srcSig[id] = sigOf(hp[id]); }
  /** Publishes what the 2D map should draw in the Auto tab (followed paths in order, markers, start pose). */
  function publishView() {
    try {
      const items = [], seen = new Set(); let sel = selected ? findNode(auto, selected)?.node : null;
      walkNodes(auto, n => {
        if (n.type !== 'follow' || !n.path || seen.has(n.path)) return; const r = routeOf(n.path); if (!r || !r.segments.length) return; seen.add(n.path);
        items.push({ id: n.path, route: r, selected: !!sel && sel.type === 'follow' && sel.path === n.path, markers: (auto.paths[n.path]?.markers || []).map(m => ({ t: m.t, label: getCommandDef(m.command?.name)?.label || m.command?.name || 'marker' })) });
      });
      PLAN_VIEW.items = items; PLAN_VIEW.start = auto.start; PLAN_VIEW.alliance = auto.alliance; PLAN_VIEW.openPath = openPath;
    } catch (_) { /* drawing hints are optional */ }
  }
  const unsubLib = pathLib && pathLib.subscribe ? pathLib.subscribe(() => { try { syncPaths(); infoCache.clear(); renderLeft(); renderTimeline(); renderValidation(); publishView(); } catch (_) { /* ignore */ } }) : null;

  // ---------- history / mutation ----------
  function record() {
    const j = JSON.stringify(auto);
    if (j === hist[pos]) return;
    hist.splice(pos + 1); hist.push(j); if (hist.length > 200) hist.shift(); pos = hist.length - 1;
  }
  function autosave() {
    const st = storage(); if (!st) return;
    try {
      if (libName && libName !== auto.name && libraryList(st).some(x => x.name === libName)) libraryDelete(st, libName);
      if (!libName && libraryList(st).some(x => x.name === auto.name)) auto.name = uniqueAutoName(auto.name);
      if (librarySave(st, auto)) libName = auto.name;
    } catch (_) { /* storage problems are not fatal */ }
  }
  function uniqueAutoName(base) { const st = storage(); return uniqueName(st ? libraryList(st).map(x => x.name) : [], base); }
  function commit(fn, { select } = {}) {
    try { fn(); } catch (e) { toast('Edit failed: ' + e.message, 'error'); return false; }
    if (select !== undefined) selected = select;
    if (selected && !findNode(auto, selected)) selected = null;
    syncPaths(); record(); autosave(); refresh(); try { opts.onChange?.(auto); } catch (_) { /* ignore */ }
    return true;
  }
  function restore(json) {
    stopRunner(); auto = JSON.parse(json);
    if (selected && !findNode(auto, selected)) selected = null;
    markSynced(); autosave(); refresh(); try { opts.onChange?.(auto); } catch (_) { /* ignore */ }
  }
  const undo = () => { if (pos > 0) { pos--; restore(hist[pos]); } };
  const redo = () => { if (pos < hist.length - 1) { pos++; restore(hist[pos]); } };

  // ---------- node helpers ----------
  function makeNode(kind) {
    if (kind === 'follow') return { type: 'follow', path: pathIds()[0] || '' };
    if (kind === 'wait') return { type: 'wait', seconds: 1 };
    if (kind === 'waitUntil') return { type: 'waitUntil', condition: 'ballCount<=0', timeout: 3 };
    if (GROUPS.includes(kind)) return kind === 'deadline' ? { type: kind, children: [], deadline: 0 } : { type: kind, children: [] };
    const d = COMMAND_DEFS[kind]; if (!d) throw new Error('unknown palette item ' + kind);
    const args = {}; for (const [k, s] of Object.entries(d.args || {})) if (s.default !== undefined) args[k] = s.default;
    return { type: 'command', name: kind, args };
  }
  function insertFromPalette(kind, parentId, index) {
    let pid = parentId, idx = index;
    if (pid === undefined) {
      const f = selected ? findNode(auto, selected) : null;
      if (f && GROUPS.includes(f.node.type)) pid = f.node.id;
      else if (f && f.parent) { pid = f.parent.id; idx = f.index + 1; }
      else pid = auto.root.id;
    }
    let node = null;
    commit(() => { node = makeNode(kind); addNode(auto, pid, node, idx); }, { select: undefined });
    if (node) { selected = node.id; refresh(); }
    return node;
  }
  function dropAt(data, targetId, zone) {
    const t = findNode(auto, targetId); if (!t) return false;
    let pid, idx;
    if (zone === 'into' && GROUPS.includes(t.node.type)) { pid = t.node.id; idx = undefined; }
    else if (!t.parent) { pid = t.node.id; idx = undefined; }
    else { pid = t.parent.id; idx = zone === 'before' ? t.index : t.index + 1; }
    if (typeof data !== 'string') return false;
    if (data.startsWith('palette:')) { insertFromPalette(data.slice(8), pid, idx); return true; }
    if (data.startsWith('node:')) { return commit(() => { if (!moveNode(auto, data.slice(5), pid, idx)) throw new Error('cannot move there'); }); }
    return false;
  }
  function moveBy(id, dir) {
    const f = findNode(auto, id); if (!f || !f.parent) return;
    const j = f.index + dir; if (j < 0 || j >= f.parent.children.length) return;
    commit(() => moveNode(auto, id, f.parent.id, dir < 0 ? j : j + 1));
  }
  function indent(id) {
    const f = findNode(auto, id); if (!f || !f.parent) return;
    const prev = f.parent.children[f.index - 1];
    if (!prev || !GROUPS.includes(prev.type)) { toast('Indent needs a group directly above'); return; }
    commit(() => moveNode(auto, id, prev.id));
  }
  function outdent(id) {
    const f = findNode(auto, id); if (!f || !f.parent || f.parent === auto.root) return;
    const g = findNode(auto, f.parent.id); if (!g || !g.parent) return;
    commit(() => moveNode(auto, id, g.parent.id, g.index + 1));
  }
  function kindOf(n) { return n.type === 'command' ? (getCommandDef(n.name)?.category || 'custom') : n.type === 'follow' ? 'follow' : GROUPS.includes(n.type) ? 'group' : n.type; }
  function summary(n) {
    switch (n.type) {
      case 'follow': return { chip: 'follow', title: `Follow ${routeOf(n.path)?.name || n.path || '(no path)'}`, sub: n.path && (auto.paths[n.path] || routeOf(n.path)) ? `${pathTime(n.path).toFixed(1)} s, ${pathInfo(n.path).len.toFixed(0)} in` : 'path missing' };
      case 'wait': return { chip: 'wait', title: 'Wait', sub: `${fmt(n.seconds ?? 0)} s` };
      case 'waitUntil': return { chip: 'until', title: 'Wait until', sub: `${n.condition}${n.timeout > 0 ? ` (timeout ${fmt(n.timeout)} s)` : ''}` };
      case 'command': { const d = getCommandDef(n.name); return { chip: d ? d.category : '?', title: d ? d.label : String(n.name), sub: Object.entries(n.args || {}).map(([k, v]) => `${k} ${fmt(v)}`).join(', ') }; }
      default: return { chip: n.type, title: n.type[0].toUpperCase() + n.type.slice(1), sub: `${(n.children || []).length} command(s)${n.type === 'deadline' ? `, deadline #${(n.deadline ?? 0) + 1}` : ''}` };
    }
  }

  // ---------- small widgets ----------
  const btn = (text, act, onClick, extra = {}) => {
    const b = el(doc, 'button', { cls: 'ap-btn' + (extra.cls ? ' ' + extra.cls : ''), attrs: { 'data-act': act, type: 'button', title: extra.title || TIP_FOR[act] || text, 'aria-label': extra.title || TIP_FOR[act] || text, 'data-ico': extra.icon || undefined }, on: { click: e => { e?.stopPropagation?.(); onClick(e); } }, disabled: extra.disabled });
    if (extra.icon) b.appendChild(ico(doc, extra.icon, extra.size || 16));
    if (text && !extra.iconOnly) b.appendChild(el(doc, 'span', { cls: 'ap-btn-t', text }));
    return b;
  };
  const ibtn = (icon, act, onClick, extra = {}) => btn('', act, onClick, { ...extra, icon, iconOnly: true, cls: 'ap-ib' + (extra.cls ? ' ' + extra.cls : '') });
  const rowOf = (label, ...ctl) => el(doc, 'div', { cls: 'ap-row' }, label ? el(doc, 'span', { cls: 'ap-lbl', text: label }) : null, ...ctl);
  function numField(label, value, onSet, o = {}) {
    const inp = el(doc, 'input', { cls: 'ap-in', attrs: { type: 'number', 'data-field': o.field || label, min: o.min, max: o.max, step: o.step ?? 'any' }, value: value === undefined ? '' : String(value) });
    inp.addEventListener('change', () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) onSet(v); });
    return rowOf(label, inp);
  }
  function selField(label, options, current, onSet, field) {
    const s = el(doc, 'select', { cls: 'ap-in', attrs: { 'data-field': field || label } }, options.map(([v, t]) => el(doc, 'option', { text: t, attrs: { value: v }, value: v })));
    s.value = current; s.addEventListener('change', () => onSet(s.value));
    return rowOf(label, s);
  }
  function textField(label, value, onSet, field) {
    const inp = el(doc, 'input', { cls: 'ap-in', attrs: { type: 'text', 'data-field': field || label, spellcheck: 'false' }, value: value ?? '' });
    inp.addEventListener('change', () => onSet(inp.value)); return rowOf(label, inp);
  }
  function section(title, id, body, { right, count } = {}) {
    const open = !closedSecs.has(id);
    const head = el(doc, 'div', { cls: 'ap-sec-h' },
      el(doc, 'button', { cls: 'ap-sec-t', attrs: { type: 'button', 'aria-expanded': open ? 'true' : 'false', 'data-sec': id }, on: { click: () => { if (closedSecs.has(id)) closedSecs.delete(id); else closedSecs.add(id); saveClosed(); refreshAll(); } } }, ico(doc, 'chev', 12), el(doc, 'span', { text: title }), count !== undefined ? el(doc, 'span', { cls: 'ap-count', text: count }) : null),
      right ? el(doc, 'span', { cls: 'ap-sec-r' }, right) : null);
    const s = el(doc, 'section', { cls: 'ap-sec' + (open ? ' open' : '') }, head);
    s.appendChild(body); if (!open) body.style.display = 'none';   // closed: still in the DOM (tests, find-in-page) but not shown
    return s;
  }

  // ---------- LEFT: project sidebar (Autos, Paths, Commands) ----------
  const storage = () => { try { return opts.storage !== undefined ? opts.storage : (typeof localStorage !== 'undefined' ? localStorage : null); } catch (_) { return null; } };
  let leftBox = null;
  function renderLeft() {
    if (!leftEl) return; clear(leftEl);
    leftBox = el(doc, 'div', { cls: 'ap-pane' }); leftEl.appendChild(leftBox);
    const u = undoStrip(); if (u) leftBox.appendChild(u);
    leftBox.appendChild(autosSection()); leftBox.appendChild(pathsSection()); leftBox.appendChild(paletteSection());
  }
  /** Inline rename field for a list row. Enter or blur commits, Esc cancels; keys and clicks stay inside the field (the row itself opens the item). */
  function renameField(value, onCommit, onCancel) {
    const inp = el(doc, 'input', { cls: 'ap-in ap-rename', attrs: { type: 'text', 'data-role': 'rename', 'aria-label': 'New name', spellcheck: 'false' }, value });
    let done = false;
    const stop = e => { try { e?.stopPropagation?.(); } catch (_) { /* ignore */ } };
    inp.addEventListener('mousedown', stop); inp.addEventListener('click', stop);
    inp.addEventListener('keydown', e => { stop(e); if (e.key === 'Enter') { e.preventDefault?.(); if (!done) { done = true; onCommit(inp.value); } } else if (e.key === 'Escape') { done = true; onCancel(); } });
    inp.addEventListener('blur', () => { if (!done) { done = true; onCommit(inp.value); } });
    return inp;
  }
  function itemRow(iconName, name, meta, o = {}) {
    const editing = !!o.editKey && renameKey === o.editKey;
    const nameEl = editing ? renameField(name, v => finishRename(o.editKey, v), () => { renameKey = null; renderLeft(); }) : el(doc, 'span', { cls: 'ap-item-n', text: name });
    const row = el(doc, 'div', { cls: 'ap-item', attrs: { ...(o.attrs || {}), 'data-on': o.on ? '1' : '0', tabindex: '0', title: o.title || name }, on: o.onClick ? { click: o.onClick, keydown: e => { if (e.key === 'Enter') o.onClick(e); else if ((e.key === 'Delete' || e.key === 'Backspace') && o.onDelete && !editable(e.target)) { e.preventDefault?.(); e.stopPropagation?.(); o.onDelete(); } } } : {} },
      ico(doc, iconName, 15), nameEl, meta && !editing ? el(doc, 'span', { cls: 'ap-item-m', text: meta }) : null, el(doc, 'span', { cls: 'ap-item-a' }, ...(editing ? [] : (o.actions || []))));
    if (editing) setTimeout(() => { try { nameEl.focus?.(); nameEl.select?.(); } catch (_) { /* ignore */ } }, 0);
    return row;
  }
  // ---------- list operations (Autos and Paths). Each one edits storage or the path library, then re-renders the sidebar ----------
  const libNames = st => { try { return st ? libraryList(st).map(x => x.name) : []; } catch (_) { return []; } };
  const pathNames = () => { try { return pathLib ? pathLib.names() : []; } catch (_) { return []; } };
  const inPathLib = id => pathNames().includes(id);
  function showUndo(msg, undo) {
    undoBar = { msg, undo }; clearTimeout(undoTimer);
    undoTimer = setTimeout(() => { undoBar = null; undoTimer = null; renderLeft(); }, UNDO_MS);
    try { undoTimer?.unref?.(); } catch (_) { /* browsers return a number */ }
  }
  function runUndo() {
    const u = undoBar; if (!u) return; undoBar = null; clearTimeout(undoTimer); undoTimer = null;
    try { u.undo(); } catch (e) { toast('Undo failed: ' + e.message, 'error'); }
    refresh();
  }
  function undoStrip() {
    if (!undoBar) return null;
    return el(doc, 'div', { cls: 'ap-undo', attrs: { 'data-role': 'undo-bar', role: 'status' } }, el(doc, 'span', { cls: 'ap-undo-t', text: undoBar.msg }), btn('Undo', 'undo-list', runUndo, { cls: 'ap-undo-b' }));
  }
  function finishRename(key, value) {
    renameKey = null; const i = key.indexOf(':'), kind = key.slice(0, i), from = key.slice(i + 1);
    if (kind === 'lib') renameAuto(from, value); else renamePath(from, value);
    renderLeft();
  }
  function renameAuto(from, to) {
    const st = storage(); if (!st) return false; const nn = String(to ?? '').trim();
    if (!nn) { toast('Name cannot be empty', 'error'); return false; }
    if (nn === from) return true;
    if (libNames(st).includes(nn)) { toast(`An auto named "${nn}" already exists`, 'error'); return false; }
    const r = libraryRename(st, from, nn); if (!r) { toast('Could not rename "' + from + '"', 'error'); return false; }
    if (libName === from) { libName = r; commit(() => { auto.name = r; }); }
    return true;
  }
  function duplicateAuto(name) {
    const st = storage(); if (!st) return;
    const nn = libraryDuplicate(st, name); if (nn) toast(`Copied as "${nn}"`); else toast('Could not duplicate "' + name + '"', 'error');
    renderLeft();
  }
  function deleteAuto(name) {
    const st = storage(); if (!st) return;
    const rec = libraryRead(st)[name]; if (!rec || !libraryDelete(st, name)) return;
    if (libName === name) libName = null;
    const savedAt = rec.savedAt || Date.now();
    showUndo(`Deleted "${name}"`, () => {
      const taken = libNames(st), n = taken.includes(name) ? uniqueName(taken, name) : name;
      const back = cloneAuto(rec); back.name = n; librarySave(st, back, savedAt);
    });
    renderLeft();
  }
  function moveAuto(name, dir) { const st = storage(); if (st && libraryMove(st, name, dir)) renderLeft(); }
  function renamePath(from, to) {
    const nn = String(to ?? '').trim();
    if (!inPathLib(from)) { toast('Only paths in the project library can be renamed', 'error'); return false; }
    if (!nn) { toast('Name cannot be empty', 'error'); return false; }
    if (nn === from) return true;
    if (pathNames().includes(nn)) { toast(`A path named "${nn}" already exists`, 'error'); return false; }
    let ok = false; try { ok = pathLib.rename(from, nn); } catch (_) { ok = false; }
    if (!ok) { toast('Could not rename "' + from + '"', 'error'); return false; }
    // follows in the current auto keep pointing at the path: move their node references and the copied segments/markers
    commit(() => {
      walkNodes(auto, n => { if (n.type === 'follow' && n.path === from) n.path = nn; });
      if (auto.paths[from]) { auto.paths[nn] = auto.paths[from]; delete auto.paths[from]; }
      if (srcSig[from] !== undefined) { srcSig[nn] = srcSig[from]; delete srcSig[from]; }
    });
    return true;
  }
  function duplicatePath(id) {
    if (!inPathLib(id)) { toast('Only paths in the project library can be duplicated', 'error'); return; }
    const route = pathLib.get(id); if (!route) return;
    const nn = duplicateName(pathNames(), id);
    try { pathLib.save(nn, route); pathLib.setActive(nn); } catch (_) { toast('Could not duplicate "' + id + '"', 'error'); return; }
    toast(`Copied as "${nn}"`);
  }
  function deletePath(id) {
    if (!inPathLib(id)) { toast('Only paths in the project library can be deleted', 'error'); return; }
    const route = pathLib.get(id), wasOpen = editing && pathLib.active?.() === id;
    if (!pathLib.remove(id)) return;
    if (wasOpen) closePath();
    if (auto.paths[id]) commit(() => { delete auto.paths[id]; delete srcSig[id]; });   // the copy kept for followed paths goes too, so the row really disappears (follows now report a missing path)
    showUndo(`Deleted path "${id}"`, () => { const n = pathNames().includes(id) ? uniqueName(pathNames(), id) : id; pathLib.save(n, { ...route, name: n }); pathLib.setActive(n); });
    refresh();
  }
  function movePath(id, dir) { if (inPathLib(id) && pathLib.move?.(id, dir)) renderLeft(); }
  function autosSection() {
    const st = storage(), list = st ? libraryList(st) : [];
    const body = el(doc, 'div', { cls: 'ap-list', attrs: { 'data-role': 'library' } });
    const right = [ibtn('plus', 'new', newAutoCmd, { title: 'New auto' }), ibtn('save', 'lib-save', () => {
      if (!st) { saveFile(); return; } syncPaths(); const n = librarySave(st, auto); if (n) { libName = n; toast('Saved "' + n + '" to the project'); renderLeft(); } else toast('Could not save (storage full?)', 'error');
    }, { cls: 'ap-adv' }), ibtn('download', 'lib-export', () => { if (st) download('autos_library.json', libraryExport(st)); }), ibtn('load', 'lib-import', importLibFile)];
    if (!st) body.appendChild(el(doc, 'div', { cls: 'ap-empty', text: 'Browser storage is unavailable; use Save JSON / Load JSON.' }));
    const inLib = list.some(x => x.name === auto.name && libName === x.name);
    if (!inLib) body.appendChild(itemRow('doc', auto.name || 'MyAuto', 'unsaved', { on: true, attrs: { 'data-role': 'current' }, title: 'The auto you are editing (saved to the project on its first edit or with Save)' }));
    list.forEach((it, i) => {
      const cur = libName === it.name && it.name === auto.name, key = 'lib:' + it.name;
      body.appendChild(itemRow('doc', it.name, it.alliance === 'red' ? 'Red' : 'Blue', {
        on: cur, attrs: { 'data-lib': it.name }, title: `${it.name}: ${it.alliance} alliance, ${it.nodes} nodes`, editKey: key,
        onClick: () => { const a = libraryLoad(st, it.name); if (a) { load(a); libName = it.name; renderLeft(); } }, onDelete: () => deleteAuto(it.name),
        actions: [ibtn('folder', 'lib-load', () => { const a = libraryLoad(st, it.name); if (a) { load(a); libName = it.name; renderLeft(); toast('Opened "' + it.name + '"'); } }, { cls: 'ap-adv' }),
          ibtn('edit', 'lib-rename', () => { renameKey = key; renderLeft(); }),
          ibtn('copy', 'lib-dup', () => duplicateAuto(it.name)),
          ibtn('up', 'lib-up', () => moveAuto(it.name, -1), { disabled: i === 0, size: 14, cls: 'ap-adv' }), ibtn('down', 'lib-down', () => moveAuto(it.name, 1), { disabled: i === list.length - 1, size: 14, cls: 'ap-adv' }),
          ibtn('trash', 'lib-del', () => deleteAuto(it.name))],
      }));
    });
    return section('Autos', 'autos', body, { right, count: String(list.length + (inLib ? 0 : 1)) });
  }
  function pathsSection() {
    const ids = pathIds(), body = el(doc, 'div', { cls: 'ap-list' });
    if (!ids.length) body.appendChild(el(doc, 'div', { cls: 'ap-empty', attrs: { 'data-role': 'no-paths' }, text: 'No paths yet. Press + to create one, then draw it on the field.' }));
    const lib = pathNames();
    ids.forEach(id => {
      const r = routeOf(id), inf = pathInfo(id), used = usedPaths().has(id), key = 'path:' + id, inLib = lib.includes(id), libIdx = lib.indexOf(id);
      const actions = [ibtn('pencil', 'preview', () => openPath(id), { title: 'Open in the path editor', cls: 'ap-adv' }), ibtn('plus', 'add-follow', () => { const n = insertFromPalette('follow'); if (n) commit(() => { n.path = id; }); })];
      if (inLib) actions.push(ibtn('edit', 'path-rename', () => { renameKey = key; renderLeft(); }), ibtn('copy', 'path-dup', () => duplicatePath(id)),
        ibtn('up', 'path-up', () => movePath(id, -1), { disabled: libIdx <= 0, size: 14, cls: 'ap-adv' }), ibtn('down', 'path-down', () => movePath(id, 1), { disabled: libIdx < 0 || libIdx >= lib.length - 1, size: 14, cls: 'ap-adv' }),
        ibtn('trash', 'path-del', () => deletePath(id)));
      body.appendChild(itemRow('route', r?.name || id, `${inf.len.toFixed(0)} in, ${inf.time.toFixed(1)} s`, {
        attrs: { 'data-path': id, 'data-used': used ? '1' : '0', 'data-inlib': inLib ? '1' : '0' }, on: editing && pathLib?.active?.() === id, title: `${r?.name || id}: click to edit this path`, onClick: () => openPath(id), onDelete: inLib ? () => deletePath(id) : undefined,
        editKey: inLib ? key : undefined, actions,
      }));
    });
    return section('Paths', 'paths', body, { right: [ibtn('plus', 'path-new', () => { let n = null; try { n = pathLib && pathLib.create(); } catch (_) { n = null; } if (n) openPath(n); else toast('Open the path editor to create a path'); })], count: String(ids.length) });
  }
  let editing = false, prevRoot = null;
  /** Opens a path for editing in the right panel (the field accepts waypoint edits while open). Falls back to onPreviewPath when no path editor exists. */
  function openPath(id) {
    try { pathLib?.setActive?.(id); } catch (_) { /* ignore */ }
    const ed = PLAN_VIEW.editor;
    if (ed && rightEl) { editing = true; ed.editing = true; renderRight(); renderLeft(); publishView(); }
    else { try { opts.onPreviewPath?.(id); } catch (_) { /* ignore */ } }
  }
  function closePath() {
    editing = false; const ed = PLAN_VIEW.editor;
    hideEditorChrome();
    if (ed) { ed.editing = false; ed.playing = false; try { ed.bindPlayback(null); } catch (_) { /* ignore */ } if (prevRoot) { try { ed.bindPanel(prevRoot); } catch (_) { /* ignore */ } } }
    syncPaths(); renderRight(); renderLeft(); renderTimeline(); publishView();
  }
  // Full-screen path editor like PathPlanner: a top bar (back arrow, path name) over the whole width, the field on the left, the editor panel on the right,
  // and a small floating playback bar at the bottom-left of the field. #autopanel gets the class "editing" (css hides the project sidebar and the timeline).
  let chrome = null;
  function showEditorChrome(ed) {
    hideEditorChrome();
    const host = rightEl && rightEl.parentNode; if (!host || !host.appendChild) return;
    const nameIn = el(doc, 'input', { cls: 'pp-title-in', attrs: { type: 'text', 'data-field': 'path-name', 'aria-label': 'Path name', title: 'Rename this path', spellcheck: 'false' }, value: ed.route.name });
    nameIn.addEventListener('change', () => { ed.rename(nameIn.value); nameIn.value = ed.route.name; });
    const bar = el(doc, 'div', { cls: 'ap-root pp-topbar', attrs: { id: 'pp-topbar' } }, ibtn('left', 'path-done', closePath, { title: 'Back to the auto editor', size: 20 }), nameIn, el(doc, 'span', { cls: 'ap-grow' }),
      ibtn('trash', 'path-delete-open', () => { const id = ed.route.name; if (inPathLib(id)) deletePath(id); else toast('This path is not in the project library', 'error'); }, { title: 'Delete this path (Undo is offered on the main screen)' }),
      ibtn('eye', 'full-field', () => { toggleFullField(); bar.querySelector?.('[data-act="full-field"]')?.classList?.toggle('on', !!PLAN_VIEW.fullField); }, { cls: PLAN_VIEW.fullField ? 'on' : '', title: PLAN_VIEW.fullField ? 'Full field is on: click to show only the ' + auto.alliance + ' half' : 'Show the whole field instead of only the ' + auto.alliance + ' half' }));
    const fl = el(doc, 'div', { cls: 'ap-root pp-root', attrs: { id: 'pp-float' } });
    host.appendChild(bar); host.appendChild(fl); chrome = { bar, fl, host };
    host.classList?.add('editing');
    ed.bindPlayback(fl);
  }
  function hideEditorChrome() {
    if (!chrome) return;
    try { chrome.host.removeChild(chrome.bar); chrome.host.removeChild(chrome.fl); chrome.host.classList?.remove('editing'); } catch (_) { /* ignore */ }
    chrome = null;
  }
  function renderPathMode() {
    const ed = PLAN_VIEW.editor, pane = el(doc, 'div', { cls: 'ap-pane ap-pathmode' }); rightEl.appendChild(pane);
    const host = el(doc, 'div', { cls: 'ap-pathhost', attrs: { 'data-role': 'path-editor' } }); pane.appendChild(host);
    if (!prevRoot) prevRoot = ed.panelRoot();
    showEditorChrome(ed);
    ed.bindPanel(host);
  }
  function paletteSection() {
    const body = el(doc, 'div', { cls: 'ap-palette' });
    for (const [group, items] of PALETTE) {
      body.appendChild(el(doc, 'div', { cls: 'ap-pal-h', text: group }));
      const grid = el(doc, 'div', { cls: 'ap-pal-g' });
      for (const [kind, label] of items) {
        const k = paletteKind(kind);
        const chip = el(doc, 'button', { cls: 'ap-chip', attrs: { type: 'button', draggable: 'true', 'data-palette': kind, title: `Add ${label} (click, or drag into the sequence)` } },
          el(doc, 'span', { cls: 'ap-dot', style: { background: KIND_COLOR[k] || KIND_COLOR.custom } }), el(doc, 'span', { cls: 'ap-chip-t', text: label }));
        chip.addEventListener('dragstart', e => { try { e.dataTransfer.setData('text/plain', 'palette:' + kind); e.dataTransfer.effectAllowed = 'copy'; } catch (_) { /* ignore */ } });
        chip.addEventListener('click', () => insertFromPalette(kind));
        grid.appendChild(chip);
      }
      body.appendChild(grid);
    }
    return section('Commands', 'cmds', body);
  }
  function importLibFile() {
    try {
      const inp = el(doc, 'input', { attrs: { type: 'file', accept: '.json,application/json' } });
      inp.addEventListener('change', async () => { try { const f = inp.files?.[0]; if (f) { const n = libraryImport(storage(), await f.text()); toast('Imported ' + n + ' auto(s)'); renderLeft(); } } catch (e) { toast('Import failed: ' + e.message, 'error'); } });
      inp.click?.();
    } catch (e) { toast('Import failed: ' + e.message, 'error'); }
  }
  planner_importLibrary = text => { const n = libraryImport(storage(), text); renderLeft(); return n; };
  function setAlliance(a) { if (a !== auto.alliance) commit(() => { retargetStart(auto, a); }); }   // retargetStart sets auto.alliance and moves auto.start into that half if needed
  function toggleFullField() { PLAN_VIEW.fullField = !PLAN_VIEW.fullField; publishView(); renderHead(); }
  function mirror() { commit(() => { const m = mirrorAuto(auto); auto = m; markSynced(); }); }

  // ---------- RIGHT: auto header, sequence tree, inspector, start state, validation, export ----------
  let treeBox = null, inspBox = null, valBox = null, headBox = null, setBox = null, expBox = null;
  function renderRight() {
    if (!rightEl) return; clear(rightEl); rowEls.clear();
    if (editing && PLAN_VIEW.editor) { renderPathMode(); return; }
    const pane = el(doc, 'div', { cls: 'ap-pane' }); rightEl.appendChild(pane);
    headBox = el(doc, 'div', { cls: 'ap-head' }); pane.appendChild(headBox);
    treeBox = el(doc, 'div', { attrs: { 'data-role': 'tree' } }); pane.appendChild(treeBox);
    inspBox = el(doc, 'div', { attrs: { 'data-role': 'inspector' } }); pane.appendChild(inspBox);
    setBox = el(doc, 'div'); pane.appendChild(setBox);
    valBox = el(doc, 'div', { attrs: { 'data-role': 'validation' } }); pane.appendChild(valBox);
    expBox = el(doc, 'div'); pane.appendChild(expBox);
    renderHead(); renderTree(); renderInspector(); renderSettings(); renderValidation(); renderExport();
  }
  function renderHead() {
    if (!headBox) return; clear(headBox);
    const name = el(doc, 'input', { cls: 'ap-name', attrs: { type: 'text', 'data-field': 'name', 'aria-label': 'Auto name', title: 'Rename this auto', spellcheck: 'false' }, value: auto.name });
    name.addEventListener('change', () => commit(() => { auto.name = name.value.trim() || 'MyAuto'; }));
    headBox.appendChild(el(doc, 'div', { cls: 'ap-name-row' }, ico(doc, 'doc', 16), name));
    headBox.appendChild(btn('Run in sim', 'run-top', () => run(), { icon: 'play', cls: 'primary ap-runbig', title: 'Run this auto in the 3D simulator' }));
    const seg = el(doc, 'div', { cls: 'ap-seg', attrs: { role: 'group', 'aria-label': 'Alliance' } },
      btn('Blue', 'alliance-blue', () => setAlliance('blue'), { cls: auto.alliance === 'blue' ? 'on blue' : 'blue', title: 'Blue alliance' }),
      btn('Red', 'alliance-red', () => setAlliance('red'), { cls: auto.alliance === 'red' ? 'on red' : 'red', title: 'Red alliance' }));
    headBox.appendChild(el(doc, 'div', { cls: 'ap-bar' }, seg, el(doc, 'span', { cls: 'ap-grow' }), ibtn('eye', 'full-field', toggleFullField, { cls: PLAN_VIEW.fullField ? 'on' : '', title: PLAN_VIEW.fullField ? 'Full field is on: click to show only the ' + auto.alliance + ' half' : 'Show the whole field instead of only the ' + auto.alliance + ' half' }), ibtn('mirror', 'mirror', mirror),
      ibtn('undo', 'undo', undo, { disabled: pos <= 0 }), ibtn('redo', 'redo', redo, { disabled: pos >= hist.length - 1 })));
  }
  function renderTree() {
    if (editing || !treeBox) return; clear(treeBox); rowEls.clear();
    const wrap = el(doc, 'div', { cls: 'ap-tree', attrs: { 'data-role': 'tree-body' } });
    wrap.addEventListener('dragover', e => { e.preventDefault?.(); });
    wrap.addEventListener('drop', e => { e.preventDefault?.(); const d = e.dataTransfer?.getData?.('text/plain'); if (d) dropAt(d, auto.root.id, 'into'); });
    const rec = (n, parent, index, depth) => {
      wrap.appendChild(nodeRow(n, parent, index, depth));
      if (GROUPS.includes(n.type) && !collapsed.has(n.id)) (n.children || []).forEach((c, i) => rec(c, n, i, depth + 1));
    };
    rec(auto.root, null, 0, 0);
    if (!(auto.root.children || []).length) wrap.appendChild(el(doc, 'div', { cls: 'ap-empty', attrs: { 'data-role': 'tree-empty' }, text: 'Empty auto. Click or drag a command from the Commands list, or press + on a path.' }));
    treeBox.appendChild(section('Auto sequence', 'tree', wrap, { count: String(countNodes()) }));
    updateLive();
  }
  function countNodes() { let n = 0; walkNodes(auto, x => { if (x !== auto.root) n++; }); return n; }
  function nodeRow(n, parent, index, depth) {
    const s = summary(n), isGroup = GROUPS.includes(n.type), last = parent ? parent.children.length - 1 : 0, kind = kindOf(n);
    const row = el(doc, 'div', { cls: 'ap-node', attrs: { 'data-node': n.id, draggable: parent ? 'true' : 'false', 'data-sel': selected === n.id ? '1' : '0', title: `${s.title}${s.sub ? ': ' + s.sub : ''}` }, style: { marginLeft: depth * 12 + 'px' } });
    row.addEventListener('click', () => select(n.id));
    row.addEventListener('dragstart', e => { try { e.dataTransfer.setData('text/plain', 'node:' + n.id); e.dataTransfer.effectAllowed = 'move'; } catch (_) { /* ignore */ } });
    row.addEventListener('dragover', e => { e.preventDefault?.(); row.setAttribute('data-drop', '1'); });
    row.addEventListener('dragleave', () => row.removeAttribute('data-drop'));
    row.addEventListener('drop', e => {
      e.preventDefault?.(); e.stopPropagation?.(); row.removeAttribute('data-drop');
      const d = e.dataTransfer?.getData?.('text/plain'); if (!d) return;
      let frac = 0.5; try { const r = row.getBoundingClientRect(); if (r && r.height > 0) frac = (e.clientY - r.top) / r.height; } catch (_) { /* ignore */ }
      const zone = isGroup ? (frac < 0.25 ? 'before' : frac > 0.75 ? 'after' : 'into') : frac < 0.5 ? 'before' : 'after';
      dropAt(d, n.id, parent ? zone : 'into');
    });
    row.appendChild(el(doc, 'span', { cls: 'ap-grip' }, parent ? ico(doc, 'grip', 12) : null));
    if (isGroup) row.appendChild(btn('', 'toggle', () => { if (collapsed.has(n.id)) collapsed.delete(n.id); else collapsed.add(n.id); renderTree(); }, { cls: 'ap-ib ap-tog' + (collapsed.has(n.id) ? '' : ' open'), icon: 'chev', size: 12, iconOnly: true }));
    else row.appendChild(el(doc, 'span', { cls: 'ap-tog-sp' }));
    const badge = el(doc, 'span', { cls: 'ap-badge', attrs: { 'data-kind': kind, title: s.chip }, style: { background: KIND_COLOR[kind] || KIND_COLOR.custom } }, ico(doc, KIND_ICON[GROUPS.includes(n.type) ? n.type : kind] || 'bolt', 13));
    row.appendChild(badge);
    row.appendChild(el(doc, 'span', { cls: 'ap-txt' }, el(doc, 'span', { cls: 'ap-t1', attrs: { 'data-role': 'label' }, text: s.title }), el(doc, 'span', { cls: 'ap-t2', attrs: { 'data-role': 'summary' }, text: s.sub })));
    if (parent) {
      row.appendChild(el(doc, 'span', { cls: 'ap-acts' },
        ibtn('up', 'up', () => moveBy(n.id, -1), { disabled: index === 0, size: 14 }), ibtn('down', 'down', () => moveBy(n.id, 1), { disabled: index === last, size: 14 }), ibtn('trash', 'delete', () => deleteNode(n.id), { size: 14 })));
    }
    rowEls.set(n.id, row);
    return row;
  }
  function deleteNode(id) { const f = findNode(auto, id); if (f && f.parent) commit(() => { removeNode(auto, id); }); }
  function select(id) { if (id && !findNode(auto, id)) return; selected = id; renderTree(); renderInspector(); publishView(); }

  function renderInspector() {
    if (editing || !inspBox) return; clear(inspBox);
    const body = el(doc, 'div', { cls: 'ap-body' });
    const f = selected ? findNode(auto, selected) : null;
    if (!f) return;   // nothing selected: no inspector card (less clutter)
    const n = f.node, saved = inspBox; inspBox = body;                       // helpers below append to inspBox
    try {
      if (n.type === 'follow') {
        const ids = pathIds(); if (n.path && !ids.includes(n.path)) ids.push(n.path);
        inspBox.appendChild(selField('Path', [['', '(none)'], ...ids.map(i => [i, routeOf(i)?.name || i])], n.path || '', v => commit(() => { n.path = v; }), 'path'));
        if (n.path) inspBox.appendChild(rowOf('', btn('Edit path', 'preview', () => openPath(n.path), { icon: 'pencil', title: 'Open this path in the path editor' })));
        renderMarkers(n);
      } else if (n.type === 'wait') {
        inspBox.appendChild(numField('Seconds', n.seconds ?? 0, v => commit(() => { n.seconds = Math.max(0, v); }), { min: 0, field: 'seconds' }));
      } else if (n.type === 'waitUntil') {
        renderCondition(n);
        inspBox.appendChild(numField('Timeout (s)', n.timeout ?? 0, v => commit(() => { n.timeout = Math.max(0, v); }), { min: 0, field: 'timeout' }));
      } else if (n.type === 'command') {
        const def = getCommandDef(n.name);
        if (/^custom:/.test(n.name || '') || !def) inspBox.appendChild(textField('Command', n.name, v => commit(() => { n.name = v; }), 'name'));
        else inspBox.appendChild(selField('Command', Object.entries(COMMAND_DEFS).map(([k, d]) => [k, d.label]), n.name, v => commit(() => { n.name = v; n.args = makeNode(v).args; }), 'name'));
        if (def) {
          for (const [k, spec] of Object.entries(def.args || {})) {
            const cur = n.args?.[k] ?? spec.default;
            if (spec.type === 'boolean') { const cb = el(doc, 'input', { attrs: { type: 'checkbox', 'data-field': 'arg.' + k } }); cb.checked = !!cur; cb.addEventListener('change', () => commit(() => { n.args = { ...(n.args || {}), [k]: cb.checked }; })); inspBox.appendChild(rowOf(k === 'shootAfter' ? 'Shoot after' : k, cb)); }
            else if (spec.type === 'number') inspBox.appendChild(numField(k, cur, v => commit(() => { n.args = { ...(n.args || {}), [k]: v }; }), { min: spec.min, max: spec.max, field: 'arg.' + k }));
            else inspBox.appendChild(textField(k, cur, v => commit(() => { n.args = { ...(n.args || {}), [k]: v }; }), 'arg.' + k));
          }
          inspBox.appendChild(el(doc, 'div', { cls: 'ap-note', text: def.description }));
        }
      } else if (GROUPS.includes(n.type)) {
        inspBox.appendChild(selField('Type', GROUPS.map(g => [g, g[0].toUpperCase() + g.slice(1)]), n.type, v => commit(() => { n.type = v; if (v === 'deadline' && !(n.deadline >= 0)) n.deadline = 0; }), 'group-type'));
        if (n.type === 'deadline' && n.children.length) {
          inspBox.appendChild(selField('Deadline', n.children.map((c, i) => [String(i), `#${i + 1} ${summary(c).title}`]), String(n.deadline ?? 0), v => commit(() => { n.deadline = parseInt(v, 10) || 0; }), 'deadline'));
        }
      }
      if (f.parent) {
        const w = el(doc, 'select', { cls: 'ap-in', attrs: { 'data-act': 'wrap', title: 'Wrap this command in a group', 'aria-label': 'Wrap in group' } }, ['Wrap in...', ...GROUPS].map((g, i) => el(doc, 'option', { text: i ? g[0].toUpperCase() + g.slice(1) : g, attrs: { value: i ? g : '' }, value: i ? g : '' })));
        w.value = ''; w.addEventListener('change', () => { if (w.value) { const t = w.value; commit(() => { wrapInGroup(auto, n.id, t); }); } });
        inspBox.appendChild(el(doc, 'details', { cls: 'pp-adv' }, el(doc, 'summary', { text: 'Move and group' }), el(doc, 'div', { cls: 'ap-bar ap-struct' }, btn('Indent', 'indent', () => indent(n.id), { icon: 'indent' }), btn('Outdent', 'outdent', () => outdent(n.id), { icon: 'outdent' }), w,
          GROUPS.includes(n.type) ? btn('Ungroup', 'unwrap', () => commit(() => { unwrap(auto, n.id); }), { icon: 'unwrap' }) : null)));
        inspBox.appendChild(rowOf('', btn('Delete command', 'delete-selected', () => deleteNode(n.id), { icon: 'trash', cls: 'danger' })));
      }
    } finally { inspBox = saved; }
    inspBox.appendChild(section('Inspector', 'insp', body, { count: summary(n).chip }));
  }
  function renderCondition(n) {
    const c = parseCondition(n.condition) || { kind: 'ballCount', op: '>=', n: 1 };
    const kind = el(doc, 'select', { cls: 'ap-in', attrs: { 'data-field': 'cond-kind' } }, COND_KINDS.map(([v, t]) => el(doc, 'option', { text: t, attrs: { value: v }, value: v })));
    const op = el(doc, 'select', { cls: 'ap-in ap-op', attrs: { 'data-field': 'cond-op' } }, ['>=', '>', '<=', '<', '=='].map(v => el(doc, 'option', { text: v, attrs: { value: v }, value: v })));
    const num = el(doc, 'input', { cls: 'ap-in ap-num', attrs: { type: 'number', 'data-field': 'cond-n', step: 'any' }, value: String(c.kind === 'timeElapsed' ? c.s : c.n ?? 1) });
    kind.value = c.kind; op.value = c.op || '>=';
    const apply = () => {
      const k = kind.value, v = parseFloat(num.value), nv = Number.isFinite(v) ? v : 0;
      const str = k === 'ballCount' ? `ballCount${op.value}${Math.max(0, Math.round(nv))}` : k === 'timeElapsed' ? `timeElapsed:${Math.max(0, nv)}` : k;
      commit(() => { n.condition = str; });
    };
    for (const e of [kind, op, num]) e.addEventListener('change', apply);
    inspBox.appendChild(rowOf('Condition', kind));
    inspBox.appendChild(rowOf('', op, num));
  }
  function renderMarkers(n) {
    const p = auto.paths[n.path];
    inspBox.appendChild(el(doc, 'div', { cls: 'ap-sub-h', text: 'Event markers' }));
    if (!p) { inspBox.appendChild(el(doc, 'div', { cls: 'ap-empty', text: 'Path is not available yet.' })); return; }
    (p.markers || []).forEach((m, i) => {
      const cmdSel = el(doc, 'select', { cls: 'ap-in', attrs: { 'data-field': 'marker-cmd' } }, Object.entries(COMMAND_DEFS).map(([k, d]) => el(doc, 'option', { text: d.label, attrs: { value: k }, value: k })));
      cmdSel.value = m.command?.name; cmdSel.addEventListener('change', () => commit(() => { m.command = { ...makeNode(cmdSel.value), id: m.command.id }; }));
      const t = el(doc, 'input', { cls: 'ap-in ap-num', attrs: { type: 'number', 'data-field': 'marker-t', min: 0, max: 1, step: 0.05, title: 'Position along the path (0 to 1)', 'aria-label': 'Marker position' }, value: String(m.t) });
      t.addEventListener('change', () => { const v = parseFloat(t.value); if (Number.isFinite(v)) commit(() => { m.t = Math.min(1, Math.max(0, v)); }); });
      inspBox.appendChild(el(doc, 'div', { cls: 'ap-row ap-marker' }, t, cmdSel, ibtn('clear', 'marker-delete', () => commit(() => { p.markers.splice(i, 1); }), { title: 'Delete marker', size: 14 })));
    });
    inspBox.appendChild(rowOf('', btn('Add marker', 'marker-add', () => commit(() => { addMarker(auto, n.path, 0.5, makeNode('intakeOn')); }), { icon: 'plus' })));
  }
  function renderSettings() {
    if (editing || !setBox) return; clear(setBox);
    const body = el(doc, 'div', { cls: 'ap-body' }); const saved = inspBox; inspBox = body;
    try {
      body.appendChild(el(doc, 'div', { cls: 'ap-sub-h', text: 'Ideal starting state' }));
      body.appendChild(el(doc, 'div', { cls: 'ap-grid3' }, ...[['X (in)', 'start.x', fmt(auto.start.x), v => { auto.start.x = v; }], ['Y (in)', 'start.y', fmt(auto.start.y), v => { auto.start.y = v; }], ['Rot (deg)', 'start.h', fmt(deg(auto.start.h || 0)), v => { auto.start.h = rad(v); }]].map(([l, f, val, set]) => {
        const inp = el(doc, 'input', { cls: 'ap-in', attrs: { type: 'number', step: 'any', 'data-field': f, 'aria-label': l }, value: String(val) });
        inp.addEventListener('change', () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) commit(() => set(v)); });
        return el(doc, 'label', { cls: 'ap-f' }, el(doc, 'span', { text: l }), inp);
      })));
      body.appendChild(el(doc, 'div', { cls: 'ap-sub-h', text: 'Global constraints' }));
      body.appendChild(numField('Max speed', fmt(auto.settings.vmax), v => commit(() => { auto.settings.vmax = Math.max(1, v); }), { field: 'vmax' }));
      body.appendChild(numField('Max accel', fmt(auto.settings.aMax), v => commit(() => { auto.settings.aMax = Math.max(1, v); }), { field: 'aMax' }));
    } finally { inspBox = saved; }
    setBox.appendChild(section('Start state and limits', 'start', body));
  }
  function issues() {
    let list = []; try { list = validateAuto(auto); } catch (e) { list = [{ level: 'error', code: 'validate', message: e.message }]; }
    if (runner && runner.state === 'error') for (const e of runner.errors) list.push({ level: 'error', code: 'runtime', message: 'Run: ' + e });
    return list;
  }
  function renderValidation() {
    if (editing || !valBox) return; clear(valBox);
    const list = issues(), body = el(doc, 'div', { cls: 'ap-body' });
    if (!list.length) body.appendChild(el(doc, 'div', { cls: 'ap-issue ok' }, ico(doc, 'check', 14), el(doc, 'span', { text: 'No problems found.' })));
    for (const i of list) {
      const r = el(doc, 'div', { cls: 'ap-issue', attrs: { 'data-act': 'issue', 'data-level': i.level, 'data-node': i.nodeId, tabindex: '0' } }, ico(doc, 'warn', 14), el(doc, 'span', { text: `${i.level}: ${i.message}` }));
      r.addEventListener('click', () => { if (i.nodeId) select(i.nodeId); });
      body.appendChild(r);
    }
    if (list.length) valBox.appendChild(section('Validation', 'val', body, { count: String(list.length) }));
  }
  function renderExport() {
    if (editing || !expBox) return; clear(expBox);
    const body = el(doc, 'div', { cls: 'ap-body' });
    body.appendChild(el(doc, 'div', { cls: 'ap-bar' }, btn('State machine', 'export-statemachine', () => exportJava('statemachine'), { icon: 'download' }), btn('Ivy', 'export-ivy', () => exportJava('ivy'), { icon: 'download' })));
    body.appendChild(el(doc, 'div', { cls: 'ap-bar' }, btn('Save JSON', 'save-json', saveFile, { icon: 'save' }), btn('Load JSON', 'load-json', loadFile, { icon: 'load' })));
    body.appendChild(el(doc, 'div', { cls: 'ap-note', text: 'Pedro Pathing 2.1.0 Java. The file downloads and is copied to the clipboard.' }));
    expBox.appendChild(section('Export', 'exp', body));
  }

  // ---------- timeline: transport + ruler + lanes ----------
  function computeBars() {
    try { bars = buildTimeline(auto, { pathTime }); } catch (e) { bars = []; bars.total = 0; bars.errors = [e.message]; bars.lanes = 0; }
  }
  const fmtT = v => (+v).toFixed(2);
  /** Compact timeline = just the transport row (56 px); expanded = 180 px unless the user dragged the handle. #autopanel's --tl-h keeps the side panels above it. */
  function applyTlHeight() {
    try {
      const h = tlMin ? 56 : Math.max(120, parseInt(timelineEl.style.height, 10) || 180), host = timelineEl.parentNode;
      if (tlMin || !timelineEl.style.height || parseInt(timelineEl.style.height, 10) < 120) { timelineEl.style.height = h + 'px'; host?.style?.setProperty?.('--tl-h', h + 'px'); }
      else host?.style?.setProperty?.('--tl-h', h + 'px');
    } catch (_) { /* fake DOMs */ }
  }
  function renderTimeline() {
    if (!timelineEl) return; clear(timelineEl); barEls.clear();
    computeBars();
    const limit = auto.settings.matchSeconds || 30, total = bars.total || 0, over = total > limit;
    timelineEl.className = (timelineEl.className || '').replace(/\bap-tl\b/g, '').replace(/\btl-min\b/g, '').replace(/\s+/g, ' ').trim() + ' ap-tl' + (tlMin ? ' tl-min' : '');
    applyTlHeight();
    if (!/\bap-root\b/.test(timelineEl.className)) timelineEl.className += ' ap-root';
    // header: transport
    runBtns = {};
    const add = (k, b) => { runBtns[k] = b; return b; };
    const head = el(doc, 'div', { cls: 'ap-tl-head' },
      el(doc, 'div', { cls: 'ap-grp' }, add('tl-first', ibtn('first', 'tl-first', () => seek(0), { cls: 'ap-adv' })), add('tl-back', ibtn('back', 'tl-back', () => seek(playT - 0.1), { cls: 'ap-adv' })),
        add('pause', ibtn('play', 'pause', pause, { cls: 'ap-play' })), add('tl-fwd', ibtn('fwd', 'tl-fwd', () => seek(playT + 0.1), { cls: 'ap-adv' })), add('stop', ibtn('stop', 'stop', stop)),
        add('step', btn('Step', 'step', stepOnce, { cls: 'ap-step ap-adv' }))));
    timeLabel = el(doc, 'span', { cls: 'ap-time', attrs: { 'data-role': 'time' }, text: `${fmtT(playT)} s` });
    head.appendChild(timeLabel);
    const sp = el(doc, 'select', { cls: 'ap-in ap-speed ap-adv', attrs: { 'data-field': 'speed', title: 'Playback speed', 'aria-label': 'Playback speed' } }, [0.1, 0.25, 0.5, 1, 2, 4].map(v => el(doc, 'option', { text: v + 'x', attrs: { value: v }, value: String(v) })));
    sp.value = String(speed); sp.addEventListener('change', () => { speed = parseFloat(sp.value) || 1; if (runner) runner.speed = speed; });
    head.appendChild(sp);
    head.appendChild(el(doc, 'span', { cls: 'ap-grow' }));
    if (bars.errors?.length) head.appendChild(el(doc, 'span', { cls: 'ap-tl-msg warn', attrs: { 'data-role': 'dry-error', title: 'Dry run: ' + bars.errors[0] }, text: 'Dry run: ' + bars.errors[0] }));
    if (over) head.appendChild(el(doc, 'span', { cls: 'ap-tl-msg bad', attrs: { 'data-role': 'warn', title: `Exceeds the ${limit} s autonomous period by ${(total - limit).toFixed(1)} s` }, text: `Over by ${(total - limit).toFixed(1)} s` }));
    head.appendChild(el(doc, 'span', { cls: 'ap-total' + (over ? ' bad' : ''), attrs: { 'data-role': 'total', 'data-over': over ? '1' : '0' }, text: `Estimated ${total.toFixed(1)} s / ${limit} s` }));
    head.appendChild(add('preview', btn('Preview', 'preview', () => run({ paused: true }), { icon: 'eye' })));
    head.appendChild(add('run', btn('Run in sim', 'run', () => run(), { icon: 'play', cls: 'primary' })));
    head.appendChild(ibtn(tlMin ? 'up' : 'down', 'tl-toggle', () => { tlMin = !tlMin; try { localStorage.setItem('biobuzz.autoui.tlmin.v1', tlMin ? '1' : '0'); } catch (_) { /* ignore */ } renderTimeline(); }, { title: tlMin ? 'Show the timeline lanes' : 'Hide the timeline lanes' }));
    timelineEl.appendChild(head);

    // body: ruler + lanes (pixels per second adapts to the panel width so the whole auto fits without horizontal scrolling)
    const lanes = Math.max(1, bars.lanes || 1), span = Math.max(total, limit) + 1;
    let avail = 0; try { avail = (timelineEl.clientWidth || 0) - 24; } catch (_) { avail = 0; }
    const pps = tlPps = Math.max(10, Math.min(48, avail > 100 ? avail / span : 24)), W = Math.ceil(span * pps), H = RULER_H + lanes * LANE_H + 8;
    const scroll = el(doc, 'div', { cls: 'ap-tl-scroll' });
    const track = el(doc, 'div', { cls: 'ap-track', attrs: { 'data-role': 'track' }, style: { width: W + 'px', height: H + 'px' } });
    const step = [1, 2, 5, 10, 15, 30].find(s => s * pps >= 44) || 30;
    for (let s = 0; s * pps < W; s += step) {
      track.appendChild(el(doc, 'div', { cls: 'ap-tick', style: { left: s * pps + 'px', height: H + 'px' } }));
      track.appendChild(el(doc, 'span', { cls: 'ap-tick-l', text: s + ' s', style: { left: s * pps + 4 + 'px' } }));
    }
    track.appendChild(el(doc, 'div', { cls: 'ap-limit', attrs: { 'data-role': 'limit' }, style: { left: limit * pps + 'px', height: H + 'px' } }));
    track.appendChild(el(doc, 'span', { cls: 'ap-limit-l', text: `${limit} s`, style: { left: limit * pps + 4 + 'px' } }));
    for (const b of bars) {
      const isG = b.kind === 'group', color = KIND_COLOR[b.kind] || KIND_COLOR.custom;
      const style = { left: b.start * pps + 'px', width: Math.max(isG ? 2 : 6, (b.end - b.start) * pps - 1) + 'px', top: RULER_H + b.lane * LANE_H + (isG ? 0 : 2) + 'px', background: color };
      if (isG) Object.assign(style, { height: '3px', opacity: '0.5', padding: '0' });
      const bar = el(doc, 'div', { cls: 'ap-cbar' + (isG ? ' grp' : ''), text: isG ? '' : b.label, attrs: { 'data-bar': b.id, 'data-kind': b.kind, title: `${b.label}  ${b.start.toFixed(2)} to ${b.end.toFixed(2)} s` }, style });
      if (!isG) bar.addEventListener('mousedown', e => { e?.stopPropagation?.(); select(b.id); });
      if (!barEls.has(b.id)) barEls.set(b.id, []); barEls.get(b.id).push(bar); track.appendChild(bar);
    }
    playhead = el(doc, 'div', { cls: 'ap-playhead', attrs: { 'data-role': 'playhead' }, style: { left: playT * pps + 'px', height: H + 'px' } });
    track.appendChild(playhead);
    track.addEventListener('mousedown', e => {
      const at = ev => { const r = track.getBoundingClientRect?.() || { left: 0 }; return Math.max(0, (ev.clientX - r.left) / tlPps); };
      seek(at(e));
      const mv = ev => seek(at(ev)), up = () => { doc.removeEventListener('mousemove', mv); doc.removeEventListener('mouseup', up); };
      doc.addEventListener('mousemove', mv); doc.addEventListener('mouseup', up);
    });
    scroll.appendChild(track); timelineEl.appendChild(scroll);
    updateButtons(); updateLive();
  }
  function updateButtons() {
    const st = runner ? runner.state : 'idle';
    const pb = runBtns.pause;
    if (pb) {
      const playing = st === 'running'; pb.setAttribute('data-state', st);
      const t = playing ? 'Pause' : st === 'paused' ? 'Resume' : 'Play (run the auto in the simulator)'; pb.setAttribute('title', t); pb.setAttribute('aria-label', t);
      const ic = pb.children?.[0]; if (ic) ic.innerHTML = `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">${playing ? ICONS.pause : ICONS.play}</svg>`;
    }
    if (runBtns.stop) runBtns.stop.disabled = st === 'idle';
    if (runBtns.run) runBtns.run.setAttribute('data-state', st);
  }
  let liveSet = new Set();
  function updateLive() {
    const t = runner && runner.state !== 'idle' ? runner.time : playT;
    if (runner && runner.state !== 'idle') playT = runner.time;
    if (timeLabel) timeLabel.textContent = `${fmtT(t)} s`;
    if (playhead) playhead.style.left = playT * tlPps + 'px';
    const now = runner && runner.state !== 'idle' ? runner.activeNodes : new Set();
    for (const id of liveSet) if (!now.has(id)) { rowEls.get(id)?.removeAttribute('data-live'); barEls.get(id)?.forEach(b => b.removeAttribute('data-live')); }
    for (const id of now) { rowEls.get(id)?.setAttribute('data-live', '1'); barEls.get(id)?.forEach(b => b.setAttribute('data-live', '1')); }
    liveSet = new Set(now);
  }

  // ---------- runner control ----------
  function stopRunner() { try { runner?.stop(); } catch (_) { /* ignore */ } runner = null; updateButtons(); }
  async function ensureRunner() {
    const host = opts.getHost?.(); if (!host) { toast('Simulator is not ready yet', 'error'); return null; }
    syncPaths();
    const json = JSON.stringify(auto);
    if (!runner || runnerJson !== json || runner.host !== host) {
      try { runner?.stop(); } catch (_) { /* ignore */ }
      runner = new AutoRunner({ host, pathSampler: sampler, trajectorySampler: trajSampler }); runner.speed = speed; runner.load(cloneAuto(auto)); runnerJson = json;
    }
    await runner.prepare(); return runner;
  }
  async function run({ paused = false } = {}) {
    syncPaths();
    const errs = issues().filter(i => i.level === 'error');
    if (errs.length) { toast('Fix errors first: ' + errs[0].message, 'error'); if (errs[0].nodeId) select(errs[0].nodeId); return false; }
    try { runner?.stop(); } catch (_) { /* ignore */ } runner = null;
    const r = await ensureRunner(); if (!r) return false;
    r.start(); if (paused && r.state === 'running') r.pause();
    updateButtons(); updateLive();
    if (r.state === 'error') { toast('Run failed: ' + r.errors.join('; '), 'error'); renderValidation(); }
    return true;
  }
  function pause() {
    if (!runner || runner.state === 'idle' || runner.state === 'done' || runner.state === 'error') { run(); return; }
    if (runner.state === 'running') runner.pause(); else if (runner.state === 'paused') runner.resume(); updateButtons();
  }
  function stop() { if (runner) { runner.reset(); } playT = 0; updateButtons(); updateLive(); }
  async function stepOnce() {
    if (!runner || runner.state === 'idle' || runner.state === 'done' || runner.state === 'error') { if (!(await run({ paused: true }))) return; }
    if (!runner) return;
    if (runner.state === 'paused' || runner.state === 'running') { runner.state = 'running'; runner.step(1 / 30); if (runner.state === 'running') runner.pause(); }
    afterStep();
  }
  async function seek(t) {
    playT = Math.max(0, t);
    const r = await ensureRunner().catch(() => null);
    if (r) { r.seek(playT); playT = r.time; }
    updateButtons(); updateLive();
  }
  function afterStep() {
    updateLive(); updateButtons();
    const st = runner ? runner.state : 'idle';
    if (st !== lastState) {
      lastState = st;
      if (st === 'error') { toast('Auto aborted: ' + runner.errors.join('; '), 'error'); renderValidation(); }
      else if (st === 'done') toast(`Auto finished in ${runner.time.toFixed(1)} s`);
    }
  }
  function tick(dt) {
    try { PLAN_VIEW.autoVisible = !!((rightEl || leftEl) && (rightEl || leftEl).offsetParent); } catch (_) { PLAN_VIEW.autoVisible = false; }
    if (runner && runner.state === 'running') { runner.step(dt); afterStep(); }
  }

  // ---------- files / export ----------
  function download(name, text) {
    try {
      if (typeof Blob === 'undefined' || typeof URL === 'undefined' || !URL.createObjectURL || !doc.body) return;
      const a = el(doc, 'a', { attrs: { download: name, href: URL.createObjectURL(new Blob([text], { type: 'text/plain' })) } });
      doc.body.appendChild(a); a.click?.(); doc.body.removeChild(a);
    } catch (_) { /* ignore */ }
  }
  function exportJava(style) {
    if (!opts.exportFn) { toast('Java export is not available', 'error'); return null; }
    const errs = issues().filter(i => i.level === 'error');
    if (errs.length) { toast('Export blocked: ' + errs[0].message, 'error'); return null; }
    let text; try { text = opts.exportFn(cloneAuto(auto), style); } catch (e) { toast('Export failed: ' + e.message, 'error'); return null; }
    try { const p = (typeof navigator !== 'undefined') && navigator.clipboard?.writeText?.(text); p?.catch?.(() => {}); } catch (_) { /* ignore */ }
    const cls = (auto.name || 'MyAuto').replace(/[^A-Za-z0-9_]/g, '') || 'MyAuto';
    download(cls + '.java', text); toast('Java copied to clipboard and downloaded');
    return text;
  }
  function save() { return toJSON(auto); }
  function saveFile() { download((auto.name || 'auto').replace(/[^A-Za-z0-9_-]/g, '_') + '.json', save()); toast('Auto saved'); }
  function loadFile() {
    try {
      const inp = el(doc, 'input', { attrs: { type: 'file', accept: '.json,application/json' } });
      inp.addEventListener('change', async () => { try { const f = inp.files?.[0]; if (f) loadJSON(await f.text()); } catch (e) { toast('Load failed: ' + e.message, 'error'); } });
      inp.click?.();
    } catch (e) { toast('Load failed: ' + e.message, 'error'); }
  }
  function load(a) {
    stopRunner(); auto = cloneAuto(a); ensureIds(auto); selected = null; collapsed.clear(); playT = 0; libName = null;
    for (const k of Object.keys(srcSig)) delete srcSig[k];
    markSynced(); hist = [JSON.stringify(auto)]; pos = 0; refresh(); try { opts.onChange?.(auto); } catch (_) { /* ignore */ }
  }
  function loadJSON(text) { try { load(fromJSON(text)); toast('Auto loaded'); return true; } catch (e) { toast('Load failed: ' + e.message, 'error'); return false; } }
  function newAutoCmd() { const st = storage(); load(newAuto({ alliance: auto.alliance, name: st ? uniqueAutoName('New Auto') : 'MyAuto' })); }

  // ---------- keyboard ----------
  const editable = t => { const tag = String(t?.tagName || '').toUpperCase(); return tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA' || !!t?.isContentEditable; };
  function handleKey(e) {
    if (!active || editable(e.target)) return;
    const k = String(e.key || '').toLowerCase(), ctrl = e.ctrlKey || e.metaKey;
    if (k === 'delete' && selected) { deleteNode(selected); e.preventDefault?.(); }
    else if (ctrl && k === 'z' && !e.shiftKey) { undo(); e.preventDefault?.(); }
    else if (ctrl && (k === 'y' || (k === 'z' && e.shiftKey))) { redo(); e.preventDefault?.(); }
  }
  const inside = t => { for (let n = t; n; n = n.parentNode) if (n === leftEl || n === rightEl || n === timelineEl) return true; return false; };
  const onDown = e => { active = inside(e.target); };
  doc.addEventListener('keydown', handleKey); doc.addEventListener('mousedown', onDown);
  for (const r of [leftEl, rightEl, timelineEl]) r?.addEventListener('mousedown', () => { active = true; });

  function refreshAll() { renderLeft(); renderRight(); }
  function refresh() { renderLeft(); renderRight(); renderTimeline(); publishView(); try { refineTimes(); } catch (_) { /* optional */ } }
  let ro = null, roTimer = null, roW = 0;
  try {
    if (timelineEl && typeof ResizeObserver !== 'undefined') {
      ro = new ResizeObserver(() => { const w = timelineEl.clientWidth; if (Math.abs(w - roW) < 2) return; roW = w; clearTimeout(roTimer); roTimer = setTimeout(renderTimeline, 120); });
      ro.observe(timelineEl);
    }
  } catch (_) { ro = null; }

  // open the most recently saved auto (autosave keeps it current), like reopening a PathPlanner project
  try {
    const st = storage(); const first = st ? libraryList(st)[0] : null;
    if (first) { const a = libraryLoad(st, first.name); if (a) { auto = cloneAuto(a); ensureIds(auto); libName = first.name; hist = [JSON.stringify(auto)]; pos = 0; markSynced(); } }
  } catch (_) { /* start empty */ }

  const planner = {
    get auto() { return auto; }, get runner() { return runner; }, get selected() { return selected; }, get bars() { return bars; },
    load, loadJSON, save, render: refresh, undo, redo, run, pause, stop, stepOnce, seek, tick, select, insertFromPalette, dropAt,
    exportJava, setAlliance, mirror, importLibrary: text => planner_importLibrary(text), get usesFollower() { return !!trajSampler; }, newAuto: newAutoCmd, handleKey, focus() { active = true; },
    destroy() { clearTimeout(undoTimer); hideEditorChrome(); if (editing) { try { closePath(); } catch (_) { /* ignore */ } } doc.removeEventListener('keydown', handleKey); doc.removeEventListener('mousedown', onDown); try { unsubLib?.(); ro?.disconnect(); } catch (_) { /* ignore */ } clearTimeout(roTimer); stopRunner(); PLAN_VIEW.items = []; PLAN_VIEW.autoVisible = false; },
  };
  refresh();
  return planner;
}
