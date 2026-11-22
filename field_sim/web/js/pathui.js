// Canvas path editor for the 2D map, styled after the official Pedro Pathing Visualizer. DOM-free except bindPanel/export buttons.
// Host wiring: const ed = createPathEditor({ mx, my, getK, toast, getFootprint, getObstacles });
//   pointer events -> if (ed.active && ed.pointer('down'|'move'|'up', fieldX, fieldY, e)) return;
//   keydown -> if (ed.active && ed.key(e)) e.preventDefault();  drawMap -> ed.draw(mctx);
//   each frame while ed.playing -> ed.step(dt); ed.playPose holds the ghost robot pose {x,y,h(rad)}.
// The curve drawn is the REAL Pedro 2.1.0 sampling (PedroSampler.latest, via /api/pedro/sample); pathing.sampleRoute is only the fallback.
import * as P from './pathing.js';
import { PedroSampler } from './pedrosample.js';
import { exportPP, ppToRoute } from './ppformat.js';
import { drawPlannerField, drawFieldOverlay } from './plannerfield.js';

const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const deg = r => (r * 180 / Math.PI);
const rad = d => d * Math.PI / 180;
const clamp = v => Math.max(0, Math.min(P.FIELD, v));
/** Grid line coordinates inside bounds {x0,x1,y0,y1}: { vertical: [x...], horizontal: [y...] }, both ends included when on the step.
 *  Pure (no DOM). step <= 0 or no bounds gives empty lists. gridLines({x0:0,x1:144,y0:0,y1:144}, 24) gives 7 lines per axis. */
export function gridLines(b, step) {
  const out = { vertical: [], horizontal: [] };
  if (!b || !(step > 0)) return out;
  const axis = (lo, hi) => {
    const arr = [], eps = 1e-9;
    for (let n = Math.ceil(lo / step - eps); n * step <= hi + eps * step; n++) arr.push((Math.round(n * step * 1e6) + 0) / 1e6);   // +0 turns -0 into 0
    return arr;
  };
  out.vertical = axis(b.x0, b.x1); out.horizontal = axis(b.y0, b.y1);
  return out;
}
// PathPlanner palette. Source: mjansen4857/pathplanner (Flutter), https://raw.githubusercontent.com/mjansen4857/pathplanner/main/lib/widgets/editor/path_painter.dart
//  Line refs come from the R1 research digest (path_painter.dart; not re-fetched in this edit task):
//  rotation target default grey[700] (l.591), selected orange (l.594), hovered deepPurpleAccent (l.597); event marker grey[700] (l.520), selected orange (l.523), hovered deepPurple (l.525), marker stroke surfaceContainer (l.521);
//  constraint zone orange when selected (l.358) and deep purple when hovered (l.383); default zone colour not in the digest (see THEME.zone); start/end rotation outlines green/red alpha 125 (l.620, l.632).
//  Hex values are Flutter Colors.* constants (orange #ff9800, deepPurpleAccent #7c4dff, grey[700] #616161, green #4caf50, red #f44336).
//  lib/util/prefs.dart Defaults.teamColor = 0xFF3F51B5 -> Material 3 dark scheme (tonal spot), computed with @material/material-color-utilities.
export const THEME = {
  base: '#1b1b1f', surface: '#23232a', border: '#34343a', body: '#c7c5d0', heading: '#e4e1e9',   // M3 dark: surface, surfaceContainer, surfaceContainerHighest, onSurfaceVariant, onSurface
  accent: '#bac3ff', secondary: '#c3c5dd', danger: '#f44336', warn: '#ff9800', ok: '#4caf50',   // primary, secondary, Colors.red, orange, green
  path: '#c3c5dd', line: '#c3c5dd', lineHi: '#ff9800', ghost: '#ff9800', arrow: '#ffffff',         // path = colorScheme.secondary, hovered/selected path = Colors.orange
  select: '#ff9800', hover: '#7c4dff', handleLine: '#616161', startFill: 'rgba(76,175,80,0.78)', endFill: 'rgba(244,67,54,0.78)', // orange, deepPurpleAccent, grey[700], green/red alpha 125
  grid: 'rgba(195,197,221,0.196)', warnBreak: '#f9a825',                                          // secondary alpha 50, yellow[800]
  rot: '#616161', rotSel: '#ff9800',                    // rotation target: grey[700] default, orange when its path is selected (path_painter.dart l.591, l.594)
  zone: '#616161', zoneSel: '#ff9800', zoneHot: '#7c4dff' // constraint zone: selected orange (l.358), hovered deepPurple (l.383). Default colour is UNVERIFIED (not in the digest): grey[700] assumed
};
const T = THEME, FONT = "'Instrument Sans', 'Segoe UI', sans-serif";
const segColor = s => s?.color || T.line;
const ICONS = {
  select: '<path d="M5 3l13 8-6 1.5L9 19z"/>', line: '<path d="M5 19L19 5"/><circle cx="5" cy="19" r="2"/><circle cx="19" cy="5" r="2"/>',
  curve: '<path d="M4 19C4 7 20 17 20 5"/><circle cx="4" cy="19" r="1.8"/><circle cx="20" cy="5" r="1.8"/>', pan: '<path d="M12 3v18M3 12h18M12 3l-3 3M12 3l3 3M12 21l-3-3M12 21l3-3M3 12l3-3M3 12l3 3M21 12l-3-3M21 12l-3 3"/>',
  delete: '<path d="M5 7h14M10 7V4h4v3M7 7l1 13h8l1-13"/>', undo: '<path d="M9 7L4 12l5 5M4 12h10a6 6 0 010 8"/>', redo: '<path d="M15 7l5 5-5 5M20 12H10a6 6 0 000 8"/>',
  mirror: '<path d="M12 3v18M8 7L3 12l5 5zM16 7l5 5-5 5z"/>', clear: '<path d="M4 4l16 16M20 4L4 20"/>',
  toggle: '<path d="M7 4l13 8-13 8z"/>', pause: '<path d="M7 4v16M17 4v16"/>', stop: '<rect x="6" y="6" width="12" height="12"/>',
  first: '<path d="M6 4v16M20 4L9 12l11 8z"/>', last: '<path d="M18 4v16M4 4l11 8L4 20z"/>', back: '<path d="M19 4L8 12l11 8zM5 4v16"/>', fwd: '<path d="M5 4l11 8L5 20zM19 4v16"/>',
  eye: '<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>', lock: '<rect x="5" y="11" width="14" height="9" rx="2"/><path d="M8 11V8a4 4 0 018 0v3"/>',
  up: '<path d="M6 15l6-6 6 6"/>', down: '<path d="M6 9l6 6 6-6"/>', del: '<path d="M5 7h14M10 7V4h4v3M7 7l1 13h8l1-13"/>',
  copy: '<rect x="8" y="8" width="12" height="12" rx="2"/><path d="M16 8V5a1 1 0 00-1-1H5a1 1 0 00-1 1v10a1 1 0 001 1h3"/>', download: '<path d="M12 4v12M7 11l5 5 5-5M5 20h14"/>',
  save: '<path d="M5 4h11l3 3v13H5zM8 4v5h7V4M8 20v-6h8v6"/>', load: '<path d="M4 7h6l2 2h8v10H4z"/>', exportpp: '<path d="M12 16V4M7 9l5-5 5 5M5 20h14"/>', importpp: '<path d="M12 4v12M7 11l5 5 5-5M5 20h14"/>',
  split: '<path d="M12 3v18M5 8l-2 4 2 4M19 8l2 4-2 4"/>', addctl: '<circle cx="12" cy="12" r="3"/><path d="M12 3v4M12 17v4M3 12h4M17 12h4"/>', delseg: '<path d="M5 7h14M10 7V4h4v3M7 7l1 13h8l1-13"/>',
  cbadd: '<path d="M12 5v14M5 12h14"/>', pnew: '<path d="M12 5v14M5 12h14"/>', pdup: '<rect x="8" y="8" width="12" height="12" rx="2"/><path d="M16 8V5a1 1 0 00-1-1H5a1 1 0 00-1 1v10a1 1 0 001 1h3"/>', pdel: '<path d="M5 7h14M10 7V4h4v3M7 7l1 13h8l1-13"/>',
  plus: '<path d="M12 5v14M5 12h14"/>', reset: '<path d="M4 12a8 8 0 108-8H8"/><path d="M11 1L8 4l3 3"/>',
  waypoints: '<polyline points="4 18 12 6 20 18"/><circle cx="4" cy="18" r="2"/><circle cx="12" cy="6" r="2"/><circle cx="20" cy="18" r="2"/>',
  pin: '<path d="M12 22s7-7.5 7-13a7 7 0 10-14 0c0 5.5 7 13 7 13z" fill="currentColor" stroke="none"/><circle cx="12" cy="9" r="2.6" fill="#23232a" stroke="none"/>', marker: '<path d="M12 22s7-7.5 7-13a7 7 0 10-14 0c0 5.5 7 13 7 13z" fill="currentColor" stroke="none"/><path d="M9 9l2.2 2.2L15 7.4" stroke="#23232a"/>',
  rot: '<path d="M20 12a8 8 0 11-3-6.2"/><path d="M20 3v5h-5"/><circle cx="12" cy="12" r="2"/>', toward: '<circle cx="12" cy="12" r="8" stroke-dasharray="3 3"/><path d="M12 12l5-5"/><circle cx="12" cy="12" r="1.6"/>',
  start: '<path d="M4 4v16"/><path d="M8 12h12M15 7l5 5-5 5"/>', final: '<circle cx="12" cy="12" r="9"/><path d="M9 8v8M9 8h6l-1.5 2.5L15 13H9"/>',
  speedo: '<rect x="4" y="4" width="16" height="16" rx="3"/><path d="M8 15l4-6 4 3"/>', zonei: '<path d="M4 16a8 8 0 1116 0"/><path d="M12 16l4-5"/><path d="M4 20h16"/>',
  optim: '<path d="M3 17l5-6 4 3 6-8"/><path d="M15 6h4v4"/>', cog: '<circle cx="12" cy="12" r="3"/><path d="M12 3v3M12 18v3M3 12h3M18 12h3M5.6 5.6l2.1 2.1M16.3 16.3l2.1 2.1M18.4 5.6l-2.1 2.1M7.7 16.3l-2.1 2.1"/>',
  chev: '<path d="M9 6l6 6-6 6"/>', route: '<circle cx="5" cy="18" r="2"/><circle cx="19" cy="6" r="2"/><path d="M7 18h6a4 4 0 000-8h-2a4 4 0 010-8h6"/>',
};
const TIPS = {
  select: 'Select (V): click a path or point; clicking the field adds nothing', line: 'Straight waypoint (L): click the field to append a straight path', curve: 'Waypoint with Bezier handles (C): click the field to append, click a path to insert', delete: 'Delete tool: click a point',
  undo: 'Undo (Ctrl+Z)', redo: 'Redo (Ctrl+Y / Ctrl+Shift+Z)', mirror: 'Mirror to the other alliance (flip x)', addev: 'Add a custom event marker halfway along the selected path', noop: 'Not supported', clear: 'Clear all paths',
  toggle: 'Play / pause (Space)', stop: 'Reset the preview to the start', first: 'Jump to start (Home)', last: 'Jump to end (End)', back: 'Step back 0.05 s (,)', fwd: 'Step forward 0.05 s (.)',
  eye: 'Show / hide path', lock: 'Lock / unlock path', up: 'Move path earlier', down: 'Move path later', del: 'Delete path', copy: 'Copy Pedro 2.1.0 Java to the clipboard', pnew: 'New path', pdup: 'Duplicate path', pdel: 'Delete path', download: 'Download the .java file',
  save: 'Save route JSON', load: 'Load route JSON', exportpp: 'Export Pedro Visualizer .pp', importpp: 'Import Pedro Visualizer .pp', split: 'Split the path in two at 50%', addctl: 'Add a control point (makes a curve)', delseg: 'Delete this path', cbadd: 'Add a custom named event',
  addrot: 'Add a rotation target at t 0.5 (heading = the current heading there)', addzone: 'Add a constraint zone over t 0.25 to 0.75 at half the global speed and accel (UI default)',
};
export const ico = (n, sz = 15) => `<svg class="pp-ico" width="${sz}" height="${sz}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${ICONS[n] || ''}</svg>`;
const SNAPS = [[0, 'Off'], [0.5, '0.5 in'], [1, '1 in'], [2, '2 in'], [3, '3 in'], [6, '6 in'], [12, '12 in'], [24, '24 in (tile)']];
const TOOLS = [['select', 'Select'], ['curve', 'Curve'], ['line', 'Line'], ['delete', 'Delete']];

// ---------- waypoint table: pure helpers (no DOM; used by the panel and by tests) ----------
/** One row per anchor of the route: the start point first ({seg:0, idx:0, start:true}), then the end anchor of every segment
 *  ({seg:i, idx:last, start:false}). x,y in inches; deg = waypoint heading (route.start.h, or the resolved end heading of the segment). */
export function waypointRows(route) {
  const rows = [{ seg: 0, idx: 0, start: true, x: route.start.x, y: route.start.y, deg: deg(route.start.h || 0), tangent: false, locked: false }];
  if (!route.segments.length) return rows;
  const res = P.resolveHeadings(route);
  route.segments.forEach((s, i) => {
    const e = s.points.at(-1);
    rows.push({ seg: i, idx: s.points.length - 1, start: false, x: e.x, y: e.y, deg: deg(res[i].h1), tangent: s.heading?.mode === 'tangent', locked: !!s.locked });
  });
  return rows;
}
/** Applies one edit from the waypoint table and mutates route. field: 'x' | 'y' (inches, clamped to 0..FIELD) or 'deg' (waypoint heading, wrapped to +-180).
 *  Moving an anchor carries its Bezier handles (P.moveAnchor). Returns {ok, value?, reason?}; ok:false leaves route untouched. */
export function applyWaypointEdit(route, row, field, value) {
  const n = typeof value === 'number' ? value : parseFloat(value);
  if (!Number.isFinite(n)) return { ok: false, reason: 'Enter a number' };
  if (!row.start && route.segments[row.seg]?.locked) return { ok: false, reason: 'Path is locked' };
  if (field === 'x' || field === 'y') {
    const v = Math.max(0, Math.min(P.FIELD, n)), cur = row.start ? route.start : route.segments[row.seg]?.points[row.idx];
    if (!cur) return { ok: false, reason: 'No such waypoint' };
    const x = field === 'x' ? v : cur.x, y = field === 'y' ? v : cur.y;
    if (row.start && !route.segments.length) { route.start.x = x; route.start.y = y; }
    else if (!P.moveAnchor(route, row.seg, row.idx, x, y)) return { ok: false, reason: 'No such waypoint' };
    return { ok: true, value: v };
  }
  if (field === 'deg') {
    const h = P.wrapAngle(rad(n));
    if (row.start) { route.start.h = h; return { ok: true, value: deg(h) }; }
    const s = route.segments[row.seg];
    if (!s) return { ok: false, reason: 'No such waypoint' };
    if (s.heading?.mode === 'tangent') return { ok: false, reason: 'Tangent paths follow the curve; change the heading mode first' };
    P.setEndHeading(route, row.seg, h);
    return { ok: true, value: deg(h) };
  }
  return { ok: false, reason: 'Unknown field ' + field };
}
/** Inserts an anchor after the row: the path that follows the row is split at its midpoint (the new anchor is its end);
 *  after the last anchor a short straight-ish path (12 in, toward the field centre) is appended. Returns {seg, idx} of the new anchor, or null when refused (locked path). */
export function insertWaypointAfter(route, row) {
  const target = row.start ? 0 : row.seg + 1;
  if (route.segments[target]) {
    if (route.segments[target].locked) return null;
    if (!P.splitSegment(route, target, 0.5)) return null;
    return { seg: target, idx: route.segments[target].points.length - 1 };
  }
  const e = row.start ? route.start : route.segments[row.seg]?.points.at(-1);
  if (!e) return null;
  const x = e.x + 12 <= P.FIELD ? e.x + 12 : e.x - 12;
  const at = P.addWaypoint(route, x, e.y);
  return { seg: at, idx: route.segments[at].points.length - 1 };
}
/** Deletes the anchor of a row. The start point and anchors of locked paths cannot be deleted. Returns true when the route changed. */
export function deleteWaypoint(route, row) {
  if (row.start || route.segments[row.seg]?.locked) return false;
  return P.deletePoint(route, row.seg, row.idx);
}
/** Keyboard handling of the path editor, DOM-free so it can be tested. ed must expose isLive, undo, redo, toggle, stepBy, seek,
 *  timeEst, setTool, deleteSelection and refresh (createPathEditor does). e = {key, ctrlKey, metaKey, shiftKey, altKey, targetTag}.
 *  Returns true when the key was handled. Keys typed into INPUT, SELECT or TEXTAREA are never handled.
 *  Ctrl+Z undo; Ctrl+Y or Ctrl+Shift+Z redo; Escape clears the selection; Delete / Backspace removes the selection (see deleteSelection). */
export function handleKey(ed, e = {}) {
  if (!ed.isLive()) return false;
  const tag = String(e.targetTag || '').toUpperCase();
  if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return false;
  const k = String(e.key || '').toLowerCase(), mod = !!(e.ctrlKey || e.metaKey);
  if (mod && k === 'z' && !e.shiftKey) { ed.undo(); return true; }
  if (mod && (k === 'y' || (k === 'z' && e.shiftKey))) { ed.redo(); return true; }
  if (k === 'escape') { ed.sel = -1; ed.selPt = null; ed.evSel = null; ed.rotSel = null; ed.refresh(); return true; }
  if (k === ' ' || k === 'spacebar') { ed.toggle(); return true; }
  if (k === ',') { ed.stepBy(-0.05); return true; }
  if (k === '.') { ed.stepBy(0.05); return true; }
  if (k === 'home') { ed.seek(0); return true; }
  if (k === 'end') { ed.seek(ed.timeEst()); return true; }
  if (!mod && !e.altKey) { const tk = { v: 'select', l: 'line', c: 'curve' }[k]; if (tk) { ed.setTool(tk); return true; } }
  if (k === 'delete' || k === 'backspace') return ed.deleteSelection();
  return false;
}

export function createPathEditor({ mx, my, getK, toast = () => {}, getFootprint, getObstacles, fetchImpl, debounceMs = 70, library }) {
  let hasLS = false; try { hasLS = typeof localStorage !== 'undefined' && !!localStorage; } catch { hasLS = false; }
  const lib = library || (hasLS ? P.sharedPathLibrary : P.createPathLibrary(null));
  let selfWrite = false;
  const route = (lib.active() && lib.get(lib.active())) || P.newRoute({ name: 'Path 1', alliance: P.PLAN_VIEW.alliance, start: P.defaultStart(P.PLAN_VIEW.alliance) });
  function libSave() { const was = selfWrite; selfWrite = true; try { lib.save(route.name, route); lib.setActive(route.name); } finally { selfWrite = was; } }
  let hist = P.createHistory(route);
  const sampler = new PedroSampler(fetchImpl ?? (globalThis.fetch ? globalThis.fetch.bind(globalThis) : undefined), { debounceMs });
  const ed = {
    route, active: false, playing: false, playPose: null, sel: 0, selPt: null, newType: 'line', tool: 'curve', tab: 'paths', className: 'MyAuto', hover: null, drag: null, obstacles: [],
    vmax: 60, aMax: 90, speed: 1, loop: false, progress: 0, ppWarnings: [], sampler,
    grid: 1, showGrid: false, magnet: true, hdgSnap: 15, traj: null, trajSource: null, time: 0, evSel: null, rotSel: null, snapHit: null,
  };
  let editFlag = false;
  /** true while a path is opened for waypoint editing (set by autoui). Reads true on the whole Auto tab too, so a host that routes events when 'editing' also swallows clicks there (the robot is never placed by clicking the plan map). */
  Object.defineProperty(ed, 'editing', { get: () => editFlag || !!P.PLAN_VIEW.autoVisible, set: v => { editFlag = !!v; }, enumerable: true });
  let root = null, pbRoot = null, simState = {}, liveDrag = null, rev = 0, jsCache = { rev: -1, s: null }, timeCache = { key: '', t: 0 };

  /** true while the host routes input to the editor: the Path tab (active) or the Auto tab with a path opened for editing (editing). */
  const live = () => ed.active || (editFlag && P.PLAN_VIEW.autoVisible);
  /** Alliance-limited view of the Auto tab: the half of the field of the alliance being planned (PLAN_VIEW.fullField lifts it). */
  const bounds = () => { const V = P.PLAN_VIEW; return (V.bounds = P.viewBounds(V.alliance, V.autoVisible ? V.fullField : true)); };   // the standalone Path tab always shows the full field
  ed.bounds = bounds;
  const vclamp = (x, y) => P.clampToView(bounds(), clamp(x), clamp(y));
  const hitR = () => 9 / Math.max(0.1, getK());
  const obstacles = () => { try { const o = getObstacles && getObstacles(); if (Array.isArray(o)) return o; } catch { /* host error: ignore */ } return ed.obstacles; };
  const isLocked = i => !!route.segments[i]?.locked;
  /** All draggable points as {seg, idx, x, y}; shared joins are listed once (as the previous segment's end). */
  function handles() {
    const out = [{ seg: 0, idx: 0, x: route.start.x, y: route.start.y, start: true }];
    route.segments.forEach((s, i) => { if (s.hidden) return; s.points.forEach((p, j) => { if (j > 0) out.push({ seg: i, idx: j, x: p.x, y: p.y, end: j === s.points.length - 1 }); }); });
    return out;
  }
  function pick(x, y) {
    let best = null, bd = hitR();
    for (const h of handles()) { const d = Math.hypot(h.x - x, h.y - y); if (d <= bd) { bd = d; best = h; } }
    return best;
  }
  const snap = (v, e) => (e && e.altKey ? Math.round(v * 100) / 100 : ed.grid > 0 ? Math.round(v / ed.grid) * ed.grid : v);
  /** Snapped field point (grid + magnet to other points); Alt disables snapping. exclude = Set of 'seg:idx' keys not to snap to. */
  function snapXY(x, y, e, exclude) {
    ed.snapHit = null;
    if (e && e.altKey) return vclamp(Math.round(x * 100) / 100, Math.round(y * 100) / 100);
    const r = P.snapPoint(route, x, y, { grid: ed.grid, magnet: ed.magnet, radius: hitR() * 0.8, exclude });
    if (r.snapped === 'point') ed.snapHit = { x: r.x, y: r.y };
    return { ...r, ...vclamp(r.x, r.y) };
  }
  const HDG_PX = 24;
  /** Heading handles: the tip of the heading arrow at the start and at every path end (not for tangent paths). */
  function hdgHandles() {
    const k = Math.max(0.1, getK()), L = HDG_PX / k, res = P.resolveHeadings(route), out = [{ kind: 'hdg', seg: -1, x: route.start.x, y: route.start.y, h: route.start.h || 0 }];
    route.segments.forEach((sg, i) => { if (sg.hidden || sg.heading.mode === 'tangent') return; const e = sg.points.at(-1); out.push({ kind: 'hdg', seg: i, x: e.x, y: e.y, h: res[i].h1 }); });
    return out.map(h => ({ ...h, hx: h.x + Math.cos(h.h) * L, hy: h.y + Math.sin(h.h) * L }));
  }
  function pickHdg(x, y) { let best = null, bd = hitR(); for (const h of hdgHandles()) { const d = Math.hypot(h.hx - x, h.hy - y); if (d <= bd) { bd = d; best = h; } } return best; }
  /** Rotation target dot under (x, y): {seg, k} (k = index in segment.rotations) or null. Dots sit on the path at c.t. */
  function pickRot(x, y) {
    let best = null, bd = hitR();
    route.segments.forEach((s, seg) => { if (s.hidden) return; (s.rotations || []).forEach((c, k) => { const q = P.bezierPoint(s.points, c.t), d = Math.hypot(q.x - x, q.y - y); if (d <= bd) { bd = d; best = { seg, k }; } }); });
    return best;
  }
  function pickEvent(x, y) { let best = null, bd = hitR(); for (const ev of P.routeEvents(route)) { if (route.segments[ev.seg]?.hidden) continue; const d = Math.hypot(ev.x - x, ev.y - y); if (d <= bd) { bd = d; best = ev; } } return best; }
  const setPt = (seg, idx, x, y, withHandles = false) => { if (!route.segments.length) { route.start.x = x; route.start.y = y; return true; } return withHandles ? P.moveAnchor(route, seg, idx, x, y) : P.movePoint(route, seg, idx, x, y); };
  const bump = () => { rev++; simState = {}; };
  const clampSel = () => { if (ed.sel < 0) return; ed.sel = Math.max(0, Math.min(route.segments.length - 1, ed.sel)); if (!route.segments.length) ed.sel = 0; if (ed.selPt && !route.segments[ed.selPt.seg] && route.segments.length) ed.selPt = null; };

  // ---------- real sampling ----------
  function resample() { sampler.request(route).then(r => { if (r && !r.aborted) { if (!ed.drag) refreshPanel(); } }, () => {}); }
  /** Sample list drawn / used for playback: real Pedro samples when they match the route, else the JS fallback. */
  function samples() {
    const l = sampler.latest;
    if (l && l.samples.length && l.segCount === route.segments.length && l.source !== 'empty') return l.samples;
    if (jsCache.rev !== rev || !jsCache.s) jsCache = { rev, s: P.sampleRoute(route, 1) };
    return jsCache.s;
  }
  ed.samples = samples;
  ed.sourceInfo = () => {
    const src = sampler.lastSource, busy = sampler.busy;
    if (src === 'pedro-core') return { label: 'Pedro 2.1.0 core', color: T.ok, source: src, busy };
    if (src === 'js-fallback') return { label: 'JS fallback', color: T.warn, source: src, busy, error: sampler.lastError };
    return { label: route.segments.length ? 'Pedro: sampling...' : 'Pedro: idle', color: T.body, source: null, busy };
  };
  const totalLen = () => { const l = sampler.latest; return l && l.segCount === route.segments.length && l.source !== 'empty' && Number.isFinite(l.total) ? l.total : P.routeLength(route); };
  function timeEst() {
    if (ed.traj && ed.traj.length > 1) return ed.traj.at(-1).t;
    const key = rev + ':' + ed.vmax + ':' + ed.aMax;
    if (timeCache.key !== key) timeCache = { key, t: P.estimateTime(route, ed.vmax, ed.aMax) };
    return timeCache.t;
  }
  function poseFromProgress(p) {
    const pts = samples(); if (pts.length < 2) return { x: route.start.x, y: route.start.y, h: route.start.h || 0 };
    const target = Math.max(0, Math.min(1, p)) * pts.at(-1).s;
    let lo = 0, hi = pts.length - 1; while (lo < hi - 1) { const m = (lo + hi) >> 1; if (pts[m].s <= target) lo = m; else hi = m; }
    const a = pts[lo], b = pts[hi], f = b.s > a.s ? Math.min(1, Math.max(0, (target - a.s) / (b.s - a.s))) : 0;
    return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, h: P.wrapAngle(a.h + P.angDiff(a.h, b.h) * f) };
  }

  let trajTimer = null, cum = null;
  /** Re-plans the time-parameterised trajectory (real Pedro follower when pedrofollow.js exists, else the local model). */
  function scheduleTraj() {
    clearTimeout(trajTimer); ed.traj = null; cum = null; if (!route.segments.length) return;
    const my = rev;
    trajTimer = setTimeout(() => { P.planTrajectory(route, { vmax: ed.vmax, aMax: ed.aMax }).then(r => { if (my !== rev) return; ed.traj = r.traj; ed.trajSource = r.source; if (!ed.drag) refreshPanel(); }).catch(() => {}); }, 120);
  }
  /** Time (s) at which the trajectory has covered fraction f (0..1) of its distance; used to place event ticks on the scrubber. */
  function timeAtFrac(f) {
    const tr = ed.traj; if (!tr || tr.length < 2) return f * timeEst();
    if (!cum) { cum = [0]; for (let i = 1; i < tr.length; i++) cum.push(cum[i - 1] + Math.hypot(tr[i].x - tr[i - 1].x, tr[i].y - tr[i - 1].y)); }
    const tot = cum.at(-1) || 1, d = f * tot; let lo = 0, hi = tr.length - 1; while (lo < hi - 1) { const m = (lo + hi) >> 1; if (cum[m] <= d) lo = m; else hi = m; }
    const fr = cum[hi] > cum[lo] ? (d - cum[lo]) / (cum[hi] - cum[lo]) : 0; return tr[lo].t + (tr[hi].t - tr[lo].t) * fr;
  }
  ed.timeAtFrac = timeAtFrac;
  const commit = () => { hist.commit(); libSave(); bump(); ed.playPose = null; ed.progress = 0; ed.time = 0; resample(); scheduleTraj(); refreshPanel(); };
  const fresh = () => { bump(); resample(); ed.traj = null; cum = null; };
  function replaceRoute(src) { for (const k of Object.keys(route)) delete route[k]; Object.assign(route, src); ed.selPt = null; clearPicks(); clampSel(); commit(); }
  /** Swaps in another path from the library. Not an edit: no undo entry; the undo history restarts at the new path. */
  function switchRoute(src) {
    for (const k of Object.keys(route)) delete route[k];
    Object.assign(route, src); ed.selPt = null; clearPicks(); clampSel(); hist = P.createHistory(route);
    bump(); ed.playPose = null; ed.progress = 0; ed.time = 0; resample(); scheduleTraj(); refreshPanel();
  }
  /** Index-based selections (rotation target, event marker) are dropped whenever the route is replaced or undone/redone. */
  const clearPicks = () => { ed.rotSel = null; ed.evSel = null; };

  /** Keeps the alliance view in step with the route's alliance (the view bounds and the start both follow it). */
  const syncAlliance = () => { if (P.PLAN_VIEW.alliance !== route.alliance) P.PLAN_VIEW.alliance = route.alliance; };
  ed.undo = () => { if (hist.undo()) { clearPicks(); clampSel(); syncAlliance(); libSave(); fresh(); refreshPanel(); return true; } return false; };
  ed.redo = () => { if (hist.redo()) { clearPicks(); clampSel(); syncAlliance(); libSave(); fresh(); refreshPanel(); return true; } return false; };
  ed.commit = () => commit();      // one undo entry per call (no entry when nothing changed); used by the panel and tests
  ed.isLive = () => live();
  ed.timeEst = () => timeEst();
  /** Switches the route to alliance a: moves the start into that half (mirrors it only when it is outside), one undo entry, saved to the library. */
  ed.setAlliance = a => {
    if (a !== 'red' && a !== 'blue') return false;
    if (a === route.alliance) return false;
    P.retargetStart(route, a); syncAlliance(); commit(); return true;
  };
  ed.mirror = () => {
    const m = P.mirrorRoute(route); P.retargetStart(m, m.alliance);   // mirrorRoute already flips the start; retarget keeps it inside the half
    replaceRoute(m); syncAlliance(); toast('Mirrored to ' + route.alliance);
  };
  ed.clear = () => { const r = P.newRoute({ name: route.name, alliance: route.alliance, start: route.start }); P.retargetStart(r, r.alliance); replaceRoute(r); };
  ed.javaText = () => P.toJava(route, { className: ed.className, opModeName: route.name });
  ed.setTool = t => { if (!TOOLS.some(x => x[0] === t)) return false; ed.tool = t; if (t === 'line' || t === 'curve') ed.newType = t; refreshPanel(); return true; };

  // ---------- playback ----------
  function updatePlaybackUi() {
    const host = pbRoot || root; if (!host) return;
    const sl = host.querySelector('[data-f="scrub"]'), tm = host.querySelector('[data-time]');
    if (sl) { sl.value = Math.round(ed.progress * 1000); sl.style.setProperty('--fill', (ed.progress * 100).toFixed(1) + '%'); }
    if (tm) tm.textContent = `${(ed.progress * timeEst()).toFixed(2)} / ${timeEst().toFixed(2)} s`;
    const chip = root && root.querySelector('[data-timechip]'); if (chip) chip.textContent = `~${timeEst().toFixed(2)}s`;
  }
  function setTime(t) {
    const T = ed.traj.at(-1).t; ed.time = Math.max(0, Math.min(T, t)); ed.progress = T > 0 ? ed.time / T : 1;
    ed.playPose = P.trajectoryPoseAt(ed.traj, ed.time); updatePlaybackUi(); return ed.playPose;
  }
  ed.step = dt => {
    if (!ed.playing || !route.segments.length) return null;
    if (ed.traj) {
      const T = ed.traj.at(-1).t; ed.time += dt * ed.speed;
      if (ed.time >= T) { if (ed.loop) ed.time = 0; else { ed.time = T; ed.playing = false; setTime(T); refreshPanel(); return ed.playPose; } }
      return setTime(ed.time);
    }
    const r = P.followSim(route, dt * ed.speed, simState, { vmax: ed.vmax, aMax: ed.aMax });
    ed.progress = r.progress; ed.playPose = poseFromProgress(r.progress);
    if (r.done) { if (ed.loop) simState = {}; else { ed.playing = false; refreshPanel(); } }
    updatePlaybackUi(); return ed.playPose;
  };
  ed.play = () => { if (!route.segments.length) return; if (ed.traj) { if (ed.time >= ed.traj.at(-1).t - 1e-9) ed.time = 0; } else if (!simState._pts || simState.done) { simState = {}; ed.progress = 0; } ed.playing = true; refreshPanel(); };
  ed.pause = () => { ed.playing = false; refreshPanel(); };
  ed.toggle = () => (ed.playing ? ed.pause() : ed.play());
  ed.stop = () => { ed.playing = false; simState = {}; ed.progress = 0; ed.time = 0; ed.playPose = null; refreshPanel(); };
  /** Seek to absolute time t (s). Step helpers pause playback. */
  ed.seek = t => { if (!route.segments.length) return null; if (ed.traj) return setTime(t); return ed.scrub(t / Math.max(1e-6, timeEst())); };
  ed.stepBy = dt => { ed.playing = false; return ed.seek((ed.progress * timeEst()) + dt); };
  /** Jump the ghost to progress p (0..1) along the route. */
  ed.scrub = p => {
    if (!route.segments.length) return null;
    if (ed.traj) return setTime(Math.max(0, Math.min(1, p)) * ed.traj.at(-1).t);
    p = Math.max(0, Math.min(1, p)); P.followSim(route, 0, simState, { vmax: ed.vmax, aMax: ed.aMax });
    simState.s = p * simState._L; simState.v = 0; simState.done = p >= 1; ed.progress = p; ed.playPose = poseFromProgress(p); updatePlaybackUi(); return ed.playPose;
  };

  // ---------- input ----------
  function hitSeg(x, y) {
    let best = -1, bd = hitR() * 1.2;
    for (const p of samples()) if (p.seg >= 0 && !route.segments[p.seg]?.hidden) { const d = Math.hypot(p.x - x, p.y - y); if (d <= bd) { bd = d; best = p.seg; } }
    return best;
  }
  /** Auto tab with no path open: the map is display-only. Clicking a drawn path opens it for waypoint editing; nothing else reacts (the robot is never placed or dragged here). */
  function overviewPointer(type, x, y, e) {
    if (type === 'down') {
      if (e.ctrlKey) return false;                                  // host copies a Pose
      if (e.button !== undefined && e.button !== 0) return true;
      let best = null, bd = hitR() * 1.4;
      for (const it of P.PLAN_VIEW.items) for (const p of autoSamples(it.route)) { const d = Math.hypot(p.x - x, p.y - y); if (d <= bd) { bd = d; best = it; } }
      if (best && P.PLAN_VIEW.openPath) { try { P.PLAN_VIEW.openPath(best.id); } catch { /* host error: ignore */ } }
      return true;
    }
    return false;
  }
  ed.pointer = (type, x, y, e = {}) => {
    if (!live()) return !ed.active && P.PLAN_VIEW.autoVisible && !editFlag ? overviewPointer(type, x, y, e) : false;
    if (type === 'down') {
      if (e.ctrlKey) return false;                                  // host copies a Pose
      const hd = e.button === 2 ? null : pickHdg(x, y);
      if (hd) {
        if (hd.seg >= 0 && isLocked(hd.seg)) { toast('Path is locked'); return true; }
        ed.drag = { kind: 'hdg', seg: hd.seg, x: hd.x, y: hd.y }; liveDrag = { x: hd.x, y: hd.y, moved: false, h: hd.h }; return true;
      }
      const evh = pickEvent(x, y);
      if (evh && ed.tool !== 'line' && ed.tool !== 'curve' || (evh && !pick(x, y))) {
        if (e.button === 2 || ed.tool === 'delete') { if (isLocked(evh.seg)) { toast('Path is locked'); return true; } if (P.removeEvent(route, evh.seg, evh.i)) { ed.evSel = null; commit(); } return true; }
        ed.sel = evh.seg; ed.selPt = null; ed.rotSel = null; ed.evSel = { seg: evh.seg, i: evh.i }; ed.drag = { kind: 'evt', seg: evh.seg, i: evh.i, x: evh.x, y: evh.y }; liveDrag = { x: evh.x, y: evh.y, moved: false }; refreshPanel(); return true;
      }
      const h = pick(x, y);
      if (e.button === 2 || (h && ed.tool === 'delete')) {          // right-click (or Delete tool) removes the waypoint / handle under the cursor
        if (!h) return true;
        if (isLocked(h.seg)) { toast('Path is locked'); return true; }
        if (P.deletePoint(route, h.seg, h.idx)) { ed.selPt = null; clampSel(); commit(); } else toast('The start point cannot be deleted');
        return true;
      }
      if (e.button !== undefined && e.button !== 0) return true;
      if (h) {
        ed.sel = h.seg; ed.selPt = { seg: h.seg, idx: h.idx }; ed.rotSel = null; ed.evSel = null;
        if (isLocked(h.seg)) { toast('Path is locked'); refreshPanel(); return true; }
        ed.drag = h; liveDrag = { x: h.x, y: h.y, moved: false }; refreshPanel(); return true;
      }
      const rt = pickRot(x, y);                                     // rotation target dot: select it (Delete removes it)
      if (rt) { ed.sel = rt.seg; ed.selPt = null; ed.evSel = null; ed.rotSel = rt; refreshPanel(); return true; }
      if (x < 0 || x > P.FIELD || y < 0 || y > P.FIELD) return true;
      const vb = bounds();
      if (!P.inView(vb, x, y)) { toast('Outside the ' + vb.alliance + ' half. Turn on Full field to edit there.'); return true; }
      if (ed.tool === 'select' || ed.tool === 'delete') { ed.sel = hitSeg(x, y); ed.selPt = null; ed.rotSel = null; ed.evSel = null; refreshPanel(); return true; }
      if (route.segments.length) {                                  // click on a drawn path: insert a waypoint there
        const n = P.nearestOnRoute(route, x, y);
        if (n && n.d <= hitR() * 1.2) {
          if (isLocked(n.seg)) { toast('Path is locked'); return true; }
          const at = P.insertWaypointAt(route, n.x, n.y, hitR() * 1.3);
          if (at >= 0) { ed.sel = at; ed.selPt = { seg: at, idx: route.segments[at].points.length - 1 }; commit(); return true; }
        }
      }
      const sp0 = snapXY(x, y, e), px = sp0.x, py = sp0.y;           // empty field: append a connected waypoint (Shift flips straight / curved)
      const curved = (ed.tool === 'curve') !== !!e.shiftKey;
      ed.sel = curved ? P.addWaypoint(route, px, py) : P.addLine(route, px, py);
      ed.selPt = { seg: ed.sel, idx: route.segments[ed.sel].points.length - 1 };
      commit(); return true;
    }
    if (type === 'move') {
      if (ed.drag && ed.drag.kind === 'hdg') {
        const base = ed.drag.seg < 0 ? route.start : route.segments[ed.drag.seg].points.at(-1);
        let a = Math.atan2(y - base.y, x - base.x);
        // heading snap: the hdgSnap setting (0 = free) or Shift (forces 15 deg steps); Alt = free
        if (!e.altKey && (ed.hdgSnap > 0 || e.shiftKey)) a = P.snapAngle(a, ed.hdgSnap > 0 ? ed.hdgSnap : 15);
        if (ed.drag.seg < 0) route.start.h = a; else P.setEndHeading(route, ed.drag.seg, a);
        liveDrag = { x: base.x, y: base.y, moved: true, h: a }; fresh(); return true;
      }
      if (ed.drag && ed.drag.kind === 'evt') {
        const t = P.nearestT(route, ed.drag.seg, x, y); P.moveEvent(route, ed.drag.seg, ed.drag.i, t);
        const q = P.bezierPoint(route.segments[ed.drag.seg].points, t); liveDrag = { x: q.x, y: q.y, moved: true, t }; fresh(); return true;
      }
      if (ed.drag) {
        const sp1 = snapXY(x, y, e, new Set([ed.drag.seg + ':' + ed.drag.idx])), px = sp1.x, py = sp1.y;
        setPt(ed.drag.seg, ed.drag.idx, px, py, true); ed.drag.x = px; ed.drag.y = py; liveDrag = { x: px, y: py, moved: true }; fresh(); return true;
      }
      ed.hover = pick(x, y); ed.hoverHdg = !!pickHdg(x, y); return false;
    }
    if (type === 'up') {
      if (!ed.drag) return false;
      const moved = liveDrag?.moved; ed.drag = null; liveDrag = null; ed.snapHit = null;
      if (moved) commit(); return true;
    }
    return false;
  };
  ed.key = e => handleKey(ed, { key: e.key, ctrlKey: e.ctrlKey, metaKey: e.metaKey, shiftKey: e.shiftKey, altKey: e.altKey, targetTag: e.target && e.target.tagName });
  /** Delete / Backspace: removes the selected rotation target, else the selected event marker, else the selected waypoint
   *  (the hovered point when none is selected), else the selected path. The start point cannot be deleted. Always returns true (key consumed). */
  ed.deleteSelection = () => {
    const rs = ed.rotSel;
    if (rs && route.segments[rs.seg]?.rotations?.[rs.k]) {
      if (isLocked(rs.seg)) { toast('Path is locked'); return true; }
      if (P.removeRotation(route, rs.seg, rs.k)) { ed.rotSel = null; commit(); }
      return true;
    }
    ed.rotSel = null;
    const es = ed.evSel;
    if (es && route.segments[es.seg]?.callbacks?.[es.i]) {
      if (isLocked(es.seg)) { toast('Path is locked'); return true; }
      if (P.removeEvent(route, es.seg, es.i)) { ed.evSel = null; commit(); }
      return true;
    }
    ed.evSel = null;
    const pt = ed.selPt || ed.hover;
    if (pt) {
      if (pt.idx === 0) { toast('The start point cannot be deleted'); return true; }
      if (isLocked(pt.seg)) { toast('Path is locked'); return true; }
      if (P.deletePoint(route, pt.seg, pt.idx)) { ed.hover = null; ed.selPt = null; clampSel(); commit(); }
      return true;
    }
    const n = route.segments.length; if (!n) return true;
    const i = ed.sel >= 0 ? Math.min(ed.sel, n - 1) : n - 1;
    if (isLocked(i)) { toast('Path is locked'); return true; }
    P.deleteSegment(route, i); ed.selPt = null; clampSel(); commit();
    return true;
  };
  ed.act = a => act(a, root, null);   // panel actions without a DOM event (tests and hosts); cbadd needs the panel and is not covered

  // ---------- drawing ----------
  const TAU = Math.PI * 2;
  function footprint() {
    const f = getFootprint && getFootprint(); let w = 18, l = 18;
    if (typeof f === 'number') w = l = f; else if (f) { w = f.w ?? f.width ?? f.x ?? w; l = f.l ?? f.length ?? f.h ?? f.height ?? f.y ?? w; }
    return { w, l };
  }
  /** Canvas rotation (clockwise radians) of a field heading h at (x, y). Measured through mx/my, so it is right in every view rotation. */
  const screenAng = (x, y, h) => Math.atan2(my(y + Math.sin(h)) - my(y), mx(x + Math.cos(h)) - mx(x));
  function dot(ctx, x, y, r, fill, ring, lw = 2) { ctx.beginPath(); ctx.arc(x, y, r, 0, TAU); ctx.fillStyle = fill; ctx.fill(); if (ring) { ctx.strokeStyle = ring; ctx.lineWidth = lw; ctx.stroke(); } }
  /** Plain label with a dark halo (no box behind it). */
  function tag(ctx, x, y, txt, color) {
    const q = P.PLAN_VIEW.viewQuarter || 0;   // the view is drawn through a canvas quarter turn: counter-rotate so text stays upright
    ctx.save(); if (q) { ctx.translate(x, y); ctx.rotate(-q * Math.PI / 2); ctx.translate(-x, -y); }
    ctx.font = `600 11px ${FONT}`; ctx.textAlign = 'left'; ctx.lineJoin = 'round'; ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(27,27,31,0.95)'; ctx.strokeText(txt, x, y); ctx.fillStyle = color; ctx.fillText(txt, x, y); ctx.restore();
  }
  /** Dotted trajectory like PathPlanner's preview: a dot every `gap` px along the polyline (samples, field space), yellow when slow to green when fast (speed(f), f = distance fraction). */
  function dotted(ctx, pts, color, r = 2.4, gap = 7, speed = null) {
    if (pts.length < 2) return;
    const total = pts.at(-1).s || 1; let carry = gap / 2;
    for (let i = 1; i < pts.length; i++) {
      const ax = mx(pts[i - 1].x), ay = my(pts[i - 1].y), bx = mx(pts[i].x), by = my(pts[i].y), d = Math.hypot(bx - ax, by - ay);
      if (d < 1e-6) continue;
      for (; carry <= d; carry += gap) {
        const f = carry / d, x = ax + (bx - ax) * f, y = ay + (by - ay) * f;
        const sf = speed ? speed(Math.min(1, ((pts[i - 1].s || 0) + ((pts[i].s || 0) - (pts[i - 1].s || 0)) * f) / total)) : 1;
        ctx.beginPath(); ctx.arc(x, y, r, 0, TAU); ctx.fillStyle = speed ? speedColor(sf) : color; ctx.fill();
      }
      carry -= d;
    }
  }
  const speedColor = f => { const t = Math.max(0, Math.min(1, f)), a = [255, 214, 0], b = [76, 175, 80]; return `rgb(${a.map((v, i) => Math.round(v + (b[i] - v) * t)).join(',')})`; };
  /** Per-position speed fraction (0..1) from the planned trajectory, or null when none is planned yet (then every dot is plain green). */
  function speedFn(pts) {
    if (!ed.traj || ed.traj.length < 2 || pts.length < 2) return null;
    const L = pts.at(-1).s || 1, e = 0.01, v = f => { const a = Math.max(0, f - e), b = Math.min(1, f + e), dt = timeAtFrac(b) - timeAtFrac(a); return dt > 1e-6 ? (b - a) * L / dt : 0; };
    let top = 1e-6; for (let i = 0; i <= 40; i++) top = Math.max(top, v(i / 40));
    return f => v(f) / top;
  }
  const autoCache = new WeakMap();
  const autoSamples = r => { let s = autoCache.get(r); if (!s) { try { s = P.sampleRoute(r, 1); } catch { s = []; } autoCache.set(r, s); } return s; };
  const pointAtFrac = (smp, f) => { if (smp.length < 2) return smp[0] || null; const tot = smp.at(-1).s || 1, d = f * tot; let i = 1; while (i < smp.length - 1 && smp[i].s < d) i++; return smp[i]; };
  /** PathPlanner point sizes: anchor 25 and control 20 "ui points" on a 3240 px wide field image, i.e. 25/3240 and 20/3240 of the field width (min 5 / 4 px so they stay grabbable). */
  const anchorR = () => Math.max(5, 25 / 3240 * P.FIELD * getK()), ctrlR = () => Math.max(4, 20 / 3240 * P.FIELD * getK());
  /** Robot outline (PathPlanner paintRobotOutline): stroke only, no fill, plus a short tick on the front edge. Heading h in radians (Pedro frame, y up). */
  function robotOutline(ctx, x, y, h, stroke, lw = 2) {
    const k = getK(), fp = footprint();
    ctx.save(); ctx.translate(mx(x), my(y)); ctx.rotate(screenAng(x, y, h));
    ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.strokeRect(-fp.l / 2 * k, -fp.w / 2 * k, fp.l * k, fp.w * k);
    ctx.beginPath(); ctx.moveTo(fp.l / 2 * k - 7, 0); ctx.lineTo(fp.l / 2 * k, 0); ctx.stroke(); ctx.restore();
  }
  /** Preview robot (the white box with corner dashes in PathPlanner). */
  function previewRobot(ctx, x, y, h) {
    const k = getK(), fp = footprint(), hl = fp.l / 2 * k, hw = fp.w / 2 * k, d = Math.min(8, hl * 0.4);
    ctx.save(); ctx.translate(mx(x), my(y)); ctx.rotate(screenAng(x, y, h)); ctx.strokeStyle = '#dee0ff'; ctx.lineWidth = 2; ctx.strokeRect(-hl, -hw, 2 * hl, 2 * hw);
    ctx.beginPath(); for (const [sx, sy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) { ctx.moveTo(sx * (hl - d - 3), sy * (hw - 4)); ctx.lineTo(sx * (hl - 3), sy * (hw - 4)); }
    ctx.stroke(); ctx.restore();
  }
  /** Auto planner overview: every followed path (in order), anchors, markers, start and end outlines. Display only; click a path to open it for editing. */
  function drawAuto(ctx) {
    const V = P.PLAN_VIEW, anySel = V.items.some(it => it.selected), ar = anchorR();
    V.items.forEach((it, n) => {
      const smp = autoSamples(it.route); if (smp.length < 2) return;
      ctx.globalAlpha = anySel && !it.selected ? 0.55 : 1;
      dotted(ctx, smp, it.selected ? T.select : T.ok, it.selected ? 3 : 2.4);
      for (const sg of it.route.segments) { const e = sg.points.at(-1); dot(ctx, mx(e.x), my(e.y), ar * 0.8, '#ffffff', T.surface, 2); }
      const last = smp.at(-1), res = (() => { try { return P.resolveHeadings(it.route); } catch { return []; } })();
      robotOutline(ctx, last.x, last.y, res.length ? res.at(-1).h1 : last.h || 0, T.endFill, 2);
      const a = smp[Math.min(smp.length - 1, 2)]; tag(ctx, mx(a.x) + 8, my(a.y) - 8, `${n + 1}  ${it.route.name || it.id}`, it.selected ? T.select : '#c3c5dd');
      ctx.globalAlpha = 1;
      for (const m of it.markers || []) {
        const q = pointAtFrac(smp, m.t); if (!q) continue; ctx.save(); ctx.translate(mx(q.x), my(q.y)); ctx.rotate(Math.PI / 4);
        ctx.fillStyle = T.warn; ctx.strokeStyle = T.surface; ctx.lineWidth = 1.5; ctx.fillRect(-4.5, -4.5, 9, 9); ctx.strokeRect(-4.5, -4.5, 9, 9); ctx.restore();
        tag(ctx, mx(q.x) + 9, my(q.y) + 16, m.label, T.warn);
      }
    });
    const st = V.start; if (st) {                                    // the robot is only an outline at the start pose, never draggable
      robotOutline(ctx, st.x, st.y, st.h || 0, T.startFill);
      dot(ctx, mx(st.x), my(st.y), ar, T.ok, T.surface, 2);
    }
    if (!V.items.length && !V.start) { ctx.textAlign = 'center'; tag(ctx, mx(72) - 120, my(72), 'Add a Follow path command to see paths here', T.body); }
  }
  /** Faint grid lines every ed.grid inches over the visible bounds. Skipped when denser than 300 lines in one frame. */
  function drawGrid(ctx) {
    const b = bounds(), g = gridLines(b, ed.grid);
    if (g.vertical.length + g.horizontal.length > 300) return;
    ctx.save(); ctx.strokeStyle = T.grid; ctx.lineWidth = 1; ctx.beginPath();
    for (const x of g.vertical) { ctx.moveTo(mx(x), my(b.y0)); ctx.lineTo(mx(x), my(b.y1)); }
    for (const y of g.horizontal) { ctx.moveTo(mx(b.x0), my(y)); ctx.lineTo(mx(b.x1), my(y)); }
    ctx.stroke(); ctx.restore();
  }
  /** Field background of the planner (host passes scene data), see plannerfield.js. */
  ed.drawField = (ctx, env) => drawPlannerField(ctx, env);
  ed.drawOverlay = (ctx, env) => drawFieldOverlay(ctx, env);
  ed.draw = ctx => {
    if (!ed.active && !live()) return;
    ctx.save();
    if (P.PLAN_VIEW.autoVisible && !editFlag) { try { drawAuto(ctx); } finally { ctx.restore(); } return; }
    if (ed.showGrid && ed.grid > 0) drawGrid(ctx);
    const pts = samples(), bySeg = route.segments.map(() => []), ar = anchorR(), cr = ctrlR();
    for (const p of pts) if (bySeg[p.seg]) bySeg[p.seg].push(p);
    for (const o of obstacles()) {
      ctx.setLineDash([4, 3]); ctx.strokeStyle = 'rgba(190,90,90,0.55)'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.rect(mx(o.minX), my(o.maxY), mx(o.maxX) - mx(o.minX), my(o.minY) - my(o.maxY)); ctx.stroke(); ctx.setLineDash([]);
    }
    const isHot = (seg, idx) => (ed.hover && ed.hover.seg === seg && ed.hover.idx === idx) || (ed.drag && ed.drag.seg === seg && ed.drag.idx === idx);
    const isSelPt = (seg, idx) => ed.selPt && ed.selPt.seg === seg && ed.selPt.idx === idx;
    // paths: a dotted trajectory (yellow slow, green fast); the selected path is slightly bolder
    const spd = speedFn(pts);
    route.segments.forEach((s, i) => {
      if (s.hidden) return;
      const line = [{ x: s.points[0].x, y: s.points[0].y, s: bySeg[i][0] ? bySeg[i][0].s : 0 }, ...bySeg[i]];
      dotted(ctx, line, segColor(s) === T.line ? T.ok : segColor(s), i === ed.sel ? 3.2 : 2.8, 8, spd);
    });
    // constraint zones: a thicker stretch of the path between t0 and t1, labelled with its vmax
    route.segments.forEach((s, i) => {
      if (s.hidden || !s.zones?.length) return;
      const col = i === ed.sel ? T.zoneSel : T.zone;
      for (const z of s.zones) {
        const stretch = P.sampleSegment(s, 80).filter(q => q.t >= z.t0 - 1e-9 && q.t <= z.t1 + 1e-9);
        if (stretch.length < 2) continue;
        ctx.beginPath(); stretch.forEach((p, j) => (j ? ctx.lineTo(mx(p.x), my(p.y)) : ctx.moveTo(mx(p.x), my(p.y)))); ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.strokeStyle = col; ctx.lineWidth = 6; ctx.globalAlpha = 0.85; ctx.stroke(); ctx.globalAlpha = 1;
        const m = stretch[stretch.length >> 1];
        tag(ctx, mx(m.x) + 8, my(m.y) + 14, `${z.vmax} in/s`, col);
      }
    });
    // handle lines (grey[700], orange for the selected path), anchor to its adjacent control point
    ctx.lineWidth = 2; ctx.lineCap = 'round';
    route.segments.forEach((s, i) => {
      if (s.hidden || s.points.length < 3) return;
      ctx.strokeStyle = i === ed.sel ? T.select : T.handleLine;
      const n = s.points.length, a = s.points[0], c1 = s.points[1], b = s.points[n - 1], c2 = s.points[n - 2];
      ctx.beginPath(); ctx.moveTo(mx(a.x), my(a.y)); ctx.lineTo(mx(c1.x), my(c1.y)); ctx.moveTo(mx(b.x), my(b.y)); ctx.lineTo(mx(c2.x), my(c2.y)); ctx.stroke();
    });
    // robot outlines: start (green), goal (red) and the selected waypoint, nothing else
    const res = P.resolveHeadings(route), last = route.segments.length - 1;
    robotOutline(ctx, route.start.x, route.start.y, route.start.h || 0, T.startFill);
    if (last >= 0) { const e = route.segments[last].points.at(-1); robotOutline(ctx, e.x, e.y, res[last].h1, T.endFill); }
    if (ed.selPt && ed.selPt.idx > 0) { const sg = route.segments[ed.selPt.seg]; if (sg && ed.selPt.idx === sg.points.length - 1) { const e = sg.points.at(-1); robotOutline(ctx, e.x, e.y, res[ed.selPt.seg].h1, T.select, 2); } }
    const obs = obstacles();
    if (obs.length) for (const p of pts) if (obs.some(o => p.x >= o.minX && p.x <= o.maxX && p.y >= o.minY && p.y <= o.maxY)) dot(ctx, mx(p.x), my(p.y), 2.5, T.danger);
    // control points (every segment) then anchors on top: start green, last red, others white; selected orange, hovered deepPurpleAccent
    const hs = handles();
    for (const h of hs) {
      if (h.start || h.end) continue;
      const sg = route.segments[h.seg]; if (sg.hidden) continue;
      const fill = isSelPt(h.seg, h.idx) ? T.select : isHot(h.seg, h.idx) ? T.hover : '#8f9bff';
      dot(ctx, mx(h.x), my(h.y), cr, fill, T.surface, 2);
    }
    for (const h of hs) {
      if (!h.start && !h.end) continue;
      const fill = isSelPt(h.seg, h.idx) || (h.start && ed.selPt && ed.selPt.idx === 0 && ed.selPt.seg === 0) ? T.select : isHot(h.seg, h.idx) ? T.hover : h.start ? T.ok : h.seg === last ? T.danger : '#ffffff';
      dot(ctx, mx(h.x), my(h.y), ar, fill, T.surface, 2);
    }
    // rotation targets: an orange ring on the path at t with a short heading line
    route.segments.forEach((s, i) => {
      if (s.hidden || !s.rotations?.length) return;
      for (const [k, c] of s.rotations.entries()) {
        const col = ed.rotSel && ed.rotSel.seg === i && ed.rotSel.k === k ? '#ffd180' : T.select;
        const q = P.bezierPoint(s.points, c.t), h = P.rotationAt(route, i, c.t) ?? c.h, cx = mx(q.x), cy = my(q.y), a = screenAng(q.x, q.y, h), L = 20;
        ctx.lineCap = 'round'; ctx.strokeStyle = col; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(cx, cy, Math.max(8, ar + 2), 0, TAU); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(a) * L, cy + Math.sin(a) * L); ctx.stroke();
        tag(ctx, cx + 12, cy - 12, `${deg(h).toFixed(0)} deg`, col);
      }
    });
    // rotation handles: drag the dot at the tip of the heading arrow
    for (const hh of hdgHandles()) {
      const hot = ed.drag && ed.drag.kind === 'hdg' && ed.drag.seg === hh.seg;
      ctx.strokeStyle = hot ? T.select : 'rgba(228,225,233,0.85)'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(mx(hh.x), my(hh.y)); ctx.lineTo(mx(hh.hx), my(hh.hy)); ctx.stroke();
      dot(ctx, mx(hh.hx), my(hh.hy), hot || ed.hoverHdg ? cr + 1.5 : cr, hot ? T.select : T.heading, T.surface, 2);
    }
    for (const ev of P.routeEvents(route)) {
      if (route.segments[ev.seg]?.hidden) continue;
      const pr = P.EVENT_PRESETS[ev.name], col = pr ? pr.color : T.warn, sel = ed.evSel && ed.evSel.seg === ev.seg && ed.evSel.i === ev.i, px = mx(ev.x), py = my(ev.y);
      ctx.save(); ctx.translate(px, py); ctx.rotate(Math.PI / 4); ctx.fillStyle = col; ctx.strokeStyle = sel ? T.select : T.surface; ctx.lineWidth = sel ? 2.5 : 1.5; ctx.fillRect(-5, -5, 10, 10); ctx.strokeRect(-5, -5, 10, 10); ctx.restore();
      tag(ctx, px + 9, py - 6, pr ? pr.icon + (ev.name === 'setRpm' ? ' ' + (ev.args?.rpm ?? '') : '') : ev.name, col);
    }
    if (ed.snapHit) { ctx.beginPath(); ctx.arc(mx(ed.snapHit.x), my(ed.snapHit.y), 11, 0, TAU); ctx.strokeStyle = T.ok; ctx.lineWidth = 2; ctx.stroke(); }
    if (ed.drag && liveDrag) {
      const px = mx(liveDrag.x), py = my(liveDrag.y), txt = liveDrag.h !== undefined ? `${deg(liveDrag.h).toFixed(0)} deg` : liveDrag.t !== undefined ? `t = ${liveDrag.t.toFixed(2)}` : `(${liveDrag.x.toFixed(1)}, ${liveDrag.y.toFixed(1)})`;
      tag(ctx, px + 14, py - 14, txt, T.heading);
    }
    if (ed.playPose) previewRobot(ctx, ed.playPose.x, ed.playPose.y, ed.playPose.h);   // preview robot
    ctx.restore();
  };

  // ---------- side panel: PathPlanner path editor (time chip, collapsible cards: waypoints, markers, rotation targets, point towards zones, states, constraints, optimizer) ----------
  const H = t => `<div class="pp-h">${t}</div>`;
  const btn = (id, label, extra = '', data = '') => `<button type="button" class="pp-btn${label ? '' : ' pp-icon'}" data-act="${id}" ${data} title="${esc(TIPS[id] || label)}" aria-label="${esc(TIPS[id] || label)}"${extra}>${ICONS[id] ? ico(id) : ''}${label ? `<span>${label}</span>` : ''}</button>`;
  const opts = (list, cur) => list.map(m => `<option${String(cur) === String(m) ? ' selected' : ''}>${m}</option>`).join('');
  const fld = (label, inner, cls = '') => `<label class="pp-f ${cls}"><span>${label}</span>${inner}</label>`;
  const num = (f, v, extra = '') => `<input data-f="${f}" value="${v}" inputmode="decimal" ${extra}>`;
  /** One accordion card: header = chevron, icon, title, optional summary text, optional + button (data-act), optional count. */
  const sec = (id, icon, title, body, { count, add, addTip, summary } = {}) => {
    const o = ed.open.has(id);
    const right = (summary ? `<span class="pp-sum">${summary}</span>` : '')
      + (add ? `<button type="button" class="pp-add" data-act="${add}" title="${esc(addTip || TIPS[add] || 'Add')}" aria-label="${esc(addTip || TIPS[add] || 'Add')}">${ico('plus', 16)}</button>` : '')
      + (count !== undefined ? `<span class="pp-count">${count}</span>` : '');
    return `<section class="pp-sec${o ? ' open' : ''}" data-secid="${id}"><div class="pp-sec-h"><button type="button" class="pp-sec-t" data-sec="${id}" aria-expanded="${o}">${ico('chev', 14)}${ico(icon, 18)}<span>${title}</span></button><span class="pp-sec-r">${right}</span></div>${o ? `<div class="pp-sec-b">${body}</div>` : ''}</section>`;
  };
  const OPEN_KEY = 'biobuzz.pathui.open.v1';
  ed.open = new Set(['waypoints']);
  try { const raw = hasLS ? localStorage.getItem(OPEN_KEY) : null; if (raw) ed.open = new Set(JSON.parse(raw)); } catch { /* defaults */ }
  const saveOpen = () => { try { if (hasLS) localStorage.setItem(OPEN_KEY, JSON.stringify([...ed.open])); } catch { /* ignore */ } };
  const segName = i => route.segments[i]?.label || 'Path ' + (i + 1);

  function topHtml() {
    return `<div class="pp-top"><span class="pp-time" data-timechip title="Estimated driving time of this path">~${timeEst().toFixed(2)}s</span><span class="pp-grow"></span>`
      + `${btn('undo', '', hist.canUndo ? '' : ' disabled')}${btn('redo', '', hist.canRedo ? '' : ' disabled')}${btn('mirror', '')}</div>`;
  }
  function toolsHtml() {
    const tb = ([id]) => `<button type="button" class="pp-btn pp-icon${ed.tool === id ? ' on' : ''}" data-tool="${id}" title="${esc(TIPS[id])}" aria-label="${esc(TIPS[id])}">${ico(id)}</button>`;
    const hint = ed.tool === 'select' ? 'Click a path or point to select.' : ed.tool === 'delete' ? 'Click a point to delete it.' : (ed.tool === 'curve' ? 'Click the field to append a waypoint with Bezier handles (Shift: straight). ' : 'Click the field to append a straight waypoint (Shift: curve). ') + 'Click a path to insert one. Drag anchors, handles and the rotation dot. Right-click or Delete removes.';
    return `<div class="pp-bar"><div class="pp-grp">${TOOLS.map(tb).join('')}</div></div>`
      + `<div class="pp-snap"><label title="Grid snapping. Hold Alt while dragging to disable.">Snap <select data-f="grid">${SNAPS.map(([v, l]) => `<option value="${v}"${ed.grid === v ? ' selected' : ''}>${l}</option>`).join('')}</select></label>`
      + `<label title="Snap to existing points"><input type="checkbox" data-f="magnet"${ed.magnet ? ' checked' : ''}> Magnet</label>`
      + `<label title="Draw faint grid lines at the snap step"><input type="checkbox" data-f="showgrid"${ed.showGrid ? ' checked' : ''}> Grid lines</label>`
      + `<label title="Rotation handle snap">Rotation snap <select data-f="hdgsnap">${[[0, 'free'], [5, '5 deg'], [15, '15 deg'], [45, '45 deg']].map(([v, l]) => `<option value="${v}"${ed.hdgSnap === v ? ' selected' : ''}>${l}</option>`).join('')}</select></label>`
      + `</div><div class="pp-hint">${hint}</div>`;
  }
  /** Floating playback bar (bottom-left of the field): play / pause, reset, slider. Event ticks sit on the slider. */
  function playbackHtml() {
    const est = timeEst();
    const ticks = P.routeEvents(route).map(ev => { const pr = P.EVENT_PRESETS[ev.name], t = timeAtFrac(ev.frac); return `<i class="pp-tick" title="${esc(pr ? pr.label : ev.name)} @ ${t.toFixed(2)} s" style="left:${est > 0 ? Math.min(100, t / est * 100) : 0}%;background:${pr ? pr.color : T.warn}"></i>`; }).join('');
    return `<div class="pp-play"><button type="button" class="pp-fb" data-act="toggle" title="${esc(TIPS.toggle)}" aria-label="${esc(TIPS.toggle)}">${ico('toggle', 20)}</button>`
      + `<button type="button" class="pp-fb" data-act="stop" title="${esc(TIPS.stop)}" aria-label="${esc(TIPS.stop)}">${ico('reset', 20)}</button>`
      + `<div class="pp-scrub"><input type="range" data-f="scrub" min="0" max="1000" value="${Math.round(ed.progress * 1000)}" style="--fill:${(ed.progress * 100).toFixed(1)}%" aria-label="Timeline scrubber"><div class="pp-ticks">${ticks}</div></div>`
      + `<span class="pp-ftime" data-time>${(ed.progress * est).toFixed(2)} / ${est.toFixed(2)} s</span>`
      + `<select data-f="pspeed" class="pp-fspeed" title="Playback speed" aria-label="Playback speed">${[0.1, 0.25, 0.5, 1, 1.5, 2, 4].map(v => `<option value="${v}"${ed.speed === v ? ' selected' : ''}>${v}x</option>`).join('')}</select></div>`;
  }
  /** Numeric anchor table: one row per waypoint (start first) with X, Y (in) and heading (deg) inputs, click-to-select, insert-after and delete. */
  function waypointTableHtml() {
    const rows = waypointRows(route), sel = ed.selPt;
    let h = `<div class="pp-wpt" role="table" aria-label="Waypoints"><div class="pp-wpt-row pp-wpt-hd" role="row"><span>#</span><span>X (in)</span><span>Y (in)</span><span>Heading (deg)</span><span></span></div>`;
    rows.forEach((r, k) => {
      const on = sel ? (sel.seg === r.seg && sel.idx === r.idx) : (!r.start && ed.sel === r.seg);
      const at = `data-wseg="${r.seg}" data-widx="${r.idx}" data-wstart="${r.start ? 1 : 0}"`;
      const lock = r.locked ? ' disabled' : '', hl = r.tangent || r.locked ? ' disabled' : '';
      h += `<div class="pp-wpt-row${on ? ' on' : ''}${r.locked ? ' locked' : ''}" data-wrow="1" ${at} role="row" tabindex="0" title="${r.start ? 'Start point' : 'Path ' + (r.seg + 1) + ' end anchor'}">`
        + `<b class="pp-wpn" style="background:${r.start ? T.ok : k === rows.length - 1 ? T.danger : '#e9edf5'}">${k + 1}</b>`
        + `<input data-f="wx" ${at} value="${r.x.toFixed(2)}" inputmode="decimal" aria-label="Waypoint ${k + 1} X (in)"${lock}>`
        + `<input data-f="wy" ${at} value="${r.y.toFixed(2)}" inputmode="decimal" aria-label="Waypoint ${k + 1} Y (in)"${lock}>`
        + `<input data-f="wh" ${at} value="${r.deg.toFixed(1)}" inputmode="decimal" aria-label="Waypoint ${k + 1} heading (deg)" title="${r.tangent ? 'Tangent heading follows the curve' : 'Waypoint heading (deg)'}"${hl}>`
        + `<span class="pp-wpt-a"><button type="button" class="pp-btn pp-icon pp-mini" data-wact="ins" ${at} title="Insert waypoint after" aria-label="Insert waypoint after ${k + 1}">${ico('cbadd', 12)}</button>`
        + `<button type="button" class="pp-btn pp-icon pp-mini" data-wact="del" ${at} title="Delete waypoint" aria-label="Delete waypoint ${k + 1}"${r.start || r.locked ? ' disabled' : ''}>${ico('del', 12)}</button></span></div>`;
    });
    return h + '</div>';
  }
  function waypointsHtml(issues, seg, r) {
    const wp = (sel, n, name, meta, attrs) => `<div class="pp-wp${sel ? ' on' : ''}" ${attrs} tabindex="0"><b class="pp-wpn" style="background:${n === 0 ? T.ok : n === route.segments.length ? T.danger : '#e9edf5'}">${n + 1}</b><span class="pp-wp-n">${esc(name)}</span><span class="pp-wp-m">${meta}</span></div>`;
    let h = waypointTableHtml() + '<details class="pp-adv"><summary>Paths between waypoints</summary><div class="pp-list">';
    route.segments.forEach((s, i) => {
      const on = (cond, act) => `<button type="button" class="pp-btn pp-icon pp-mini${cond ? ' on' : ''}" data-act="${act}" data-i="${i}" title="${esc(TIPS[act] || act)}" aria-label="${esc(TIPS[act] || act)}">${ico(act, 12)}</button>`;
      h += `<div class="pp-wp${i === ed.sel ? ' on' : ''}${s.hidden ? ' dim' : ''}" data-sel="${i}" tabindex="0"><b class="pp-wpn" style="background:${i + 1 === route.segments.length ? T.danger : '#e9edf5'}">${i + 2}</b>`
        + `<input class="pp-wp-in" data-f="rename" data-i="${i}" value="${esc(s.label || 'Path ' + (i + 1))}" aria-label="Path name"><span class="pp-wp-m">${s.type} ${P.segmentLength(s).toFixed(0)} in</span>`
        + `<span class="pp-wp-a">${on(!s.hidden, 'eye')}${on(s.locked, 'lock')}${on(false, 'up')}${on(false, 'down')}${on(false, 'del')}</span></div>`;
    });
    if (!route.segments.length) h += `<div class="pp-empty">Click on the field to add the first path.</div>`;
    h += '</div></details>';
    if (ed.selPt) {
      const s = route.segments[ed.selPt.seg], isStart = ed.selPt.idx === 0, p = isStart ? route.start : s?.points[ed.selPt.idx];
      if (p) h += `<div class="pp-sub"><div class="pp-card-h">${isStart ? 'Start point' : `Path ${ed.selPt.seg + 1}, ${ed.selPt.idx === s.points.length - 1 ? 'end anchor' : 'control point ' + ed.selPt.idx}`}</div><div class="pp-grid">${fld('X (in)', num('px', p.x.toFixed(2)))}${fld('Y (in)', num('py', p.y.toFixed(2)))}</div></div>`;
    }
    h += `<details class="pp-adv"><summary>Selected path settings</summary>${headingHtml(seg, r)}</details>`;
    if (issues.length) h += issues.map(i => `<div class="pp-issue ${i.severity === 'error' ? 'err' : 'warn'}" data-sel="${i.seg}">${i.severity === 'error' ? 'Error' : 'Warning'}: ${esc(i.msg)}</div>`).join('');
    return h;
  }
  function constraintsHtml() {
    return `<div class="pp-grid">${fld('Max velocity (in/s)', num('vmax', ed.vmax))}${fld('Max accel (in/s2)', num('amax', ed.aMax))}</div>`
      + `<div class="pp-note">Preview only. On the robot, speed and braking come from PathConstraints in your Constants.</div>`;
  }
  function startStateHtml() {
    return `<div class="pp-grid">${fld('X (in)', num('sx', route.start.x.toFixed(1)))}${fld('Y (in)', num('sy', route.start.y.toFixed(1)))}${fld('Rotation (deg)', num('sh', deg(route.start.h || 0).toFixed(1)))}</div>`;
  }
  function finalStateHtml() {
    const lastI = route.segments.length - 1, ls = route.segments[lastI];
    if (!ls) return `<div class="pp-empty">Add a waypoint to set the final state.</div>`;
    const rr = P.resolveHeadings(route)[lastI], e = ls.points.at(-1), tan = ls.heading?.mode === 'tangent';
    return `<div class="pp-grid">${fld('X (in)', `<input value="${e.x.toFixed(1)}" disabled>`)}${fld('Y (in)', `<input value="${e.y.toFixed(1)}" disabled>`)}${fld('Rotation (deg)', `<input data-f="eh" value="${deg(rr.h1).toFixed(1)}" inputmode="decimal"${tan || ls.locked ? ' disabled' : ''}>`)}</div>${tan ? '<div class="pp-note">The last path uses tangent heading; change its heading mode in Waypoints to edit this.</div>' : ''}`;
  }
  function headingHtml(seg, r) {
    if (!seg) return `<div class="pp-empty">Select a path in the list above to edit its type, heading and speed.</div>`;
    let h = `<div class="pp-card-h">${esc(seg.label || 'Path ' + (ed.sel + 1))}</div><div class="pp-grid">${fld('Type', `<select data-f="type">${opts(['line', 'curve'], seg.type)}</select>`)}${fld('Heading', `<select data-f="mode">${opts(['linear', 'constant', 'tangent'], seg.heading.mode)}</select>`)}`;
    if (seg.heading.mode !== 'tangent') h += fld('Start (deg)', num('hs', deg(r.h0).toFixed(1))) + (seg.heading.mode === 'linear' ? fld('End (deg)', num('he', deg(r.h1).toFixed(1))) : '');
    h += fld('Max speed (in/s)', num('speed', seg.speed ?? '', 'placeholder="auto"')) + fld('Colour', `<input type="color" data-f="color" value="${segColor(seg)}">`) + '</div>';
    h += `<label class="pp-check"><input type="checkbox" data-f="rev"${seg.heading.reversed ? ' checked' : ''}> Reversed (drive backwards)</label>`;
    h += `<div class="pp-bar2">${btn('split', 'Split 50%')}${btn('addctl', 'Add control')}${btn('delseg', 'Delete')}</div>`;
    return h;
  }
  const rotItems = () => route.segments.flatMap((s, i) => (s.rotations || []).map((c, k) => ({ i, k, c })));
  const zoneItems = () => route.segments.flatMap((s, i) => (s.zones || []).map((z, k) => ({ i, k, z })));
  const evItems = () => route.segments.reduce((n, s) => n + (s.callbacks || []).length, 0);
  function rotationsHtml() {
    const it = rotItems();
    return (it.length ? it.map(({ i, k, c }) => `<div class="pp-tgt"><span class="pp-dot" style="background:${T.select}"></span><span class="pp-grow">${esc(segName(i))}: t ${c.t.toFixed(2)}, ${deg(c.h).toFixed(0)} deg</span><button type="button" class="pp-x" data-rotdel="${k}" data-rseg="${i}" title="Remove rotation target" aria-label="Remove rotation target ${k + 1} of ${esc(segName(i))}">${ico('clear', 12)}</button></div>`).join('') : `<div class="pp-empty">No rotation targets. The + adds one halfway along the selected path.</div>`);
  }
  function zonesHtml() {
    const it = zoneItems();
    return (it.length ? it.map(({ i, k, z }) => `<div class="pp-tgt"><span class="pp-dot" style="background:${T.zone}"></span><span class="pp-grow">${esc(segName(i))}: t ${z.t0.toFixed(2)} to ${z.t1.toFixed(2)}, ${z.vmax} in/s, ${z.aMax} in/s2</span><button type="button" class="pp-x" data-zdel="${k}" data-zseg="${i}" title="Remove constraint zone" aria-label="Remove constraint zone ${k + 1} of ${esc(segName(i))}">${ico('clear', 12)}</button></div>`).join('') : `<div class="pp-empty">No constraint zones. The + adds one over the middle of the selected path.</div>`);
  }
  function eventsHtml(seg) {
    let h = '<div class="pp-chips">' + Object.entries(P.EVENT_PRESETS).map(([k, p]) => `<button type="button" class="pp-chip" data-evadd="${k}" title="Add ${esc(p.label)} at t=0.5 on this path (then drag the diamond)" style="--c:${p.color}"><b>${esc(p.icon)}</b>${esc(p.label)}</button>`).join('') + '</div>';
    if (!seg) h += `<div class="pp-empty">Select a path to add event markers to it.</div>`;
    h += (seg?.callbacks || []).map((c, i) => { const p = P.EVENT_PRESETS[c.name]; return `<div class="pp-ev"><span class="pp-dot" style="background:${p ? p.color : T.warn}"></span><span class="pp-grow">${esc(p ? p.label : c.name)}</span>${c.name === 'setRpm' ? `<input class="pp-sm" data-f="evrpm" data-i="${i}" value="${c.args?.rpm ?? 3000}" aria-label="rpm" title="rpm">` : ''}<input class="pp-sm" data-f="evt" data-i="${i}" value="${c.t.toFixed(2)}" aria-label="position" title="Position along the path (0..1)"><button type="button" class="pp-x" data-cbdel="${i}" title="Remove event" aria-label="Remove event">${ico('clear', 12)}</button></div>`; }).join('');
    return h + `<div class="pp-ev pp-custom"><span class="pp-grow"><input data-f="cbn" placeholder="custom event name" aria-label="Custom event name"></span><input class="pp-sm" data-f="cbt" value="0.5" aria-label="Position" title="Position along the path (0..1)">${btn('cbadd', '')}</div>`;
  }
  function exportHtml() {
    const si = ed.sourceInfo();
    let h = `<div class="pp-grid">${fld('Class name', `<input data-f="cls" value="${esc(ed.className)}">`)}</div>`
      + `<div class="pp-bar2">${btn('copy', 'Copy Java')}${btn('download', 'Download')}${btn('save', 'Save JSON')}${btn('load', 'Load JSON')}${btn('exportpp', 'Export .pp')}${btn('importpp', 'Import .pp')}${btn('clear', 'Clear path')}</div>`
      + `<input type="file" data-f="file" accept=".json,application/json" style="display:none"><input type="file" data-f="ppfile" accept=".pp,.json,application/json" style="display:none">`
      + `<div class="pp-note" data-badge style="color:${si.color}" title="${esc(si.error || 'Samples come from the real Pedro Pathing 2.1.0 core')}">Sampler: ${si.label}${si.busy ? ' ...' : ''}${si.error ? ' - ' + esc(si.error) : ''}</div>`;
    if (ed.ppWarnings.length) h += H('Import warnings') + ed.ppWarnings.map(w => `<div class="pp-issue warn">${esc(w)}</div>`).join('');
    return h;
  }
  ed.panelHtml = () => {
    ICONS.toggle = ed.playing ? ICONS.pause : '<path d="M7 4l13 8-13 8z"/>';
    ed.obstacles = obstacles();
    const seg = route.segments[ed.sel], r = seg ? P.resolveHeadings(route)[ed.sel] : null, issues = P.validateRoute(route, ed.obstacles);
    const rows = route.segments.length + 1, sh = deg(route.start.h || 0), fe = route.segments.length ? deg(P.resolveHeadings(route).at(-1).h1) : sh;
    let h = '<div class="pp-root"><div class="pp-scroll">';
    h += topHtml();
    h += sec('waypoints', 'waypoints', 'Waypoints', waypointsHtml(issues, seg, r), { count: rows });
    h += sec('events', 'marker', 'Event Markers', eventsHtml(seg), { count: evItems(), add: 'addev', addTip: 'Add a custom event marker halfway along the selected path' });
    h += sec('rotations', 'rot', 'Rotation Targets', rotationsHtml(), { count: rotItems().length, add: 'addrot' });
    h += '<div class="pp-gap"></div>';
    h += sec('startstate', 'start', 'Starting State', startStateHtml(), { summary: `${sh.toFixed(2)}&deg; starting` });
    h += sec('endstate', 'final', 'Final State', finalStateHtml(), { summary: `${fe.toFixed(2)}&deg; ending` });
    h += '<div class="pp-gap"></div>';
    h += sec('constraints', 'speedo', 'Global Constraints', constraintsHtml());
    h += sec('zones', 'zonei', 'Constraint Zones', zonesHtml(), { count: zoneItems().length, add: 'addzone' });
    h += '<div class="pp-gap"></div>';
    h += sec('settings', 'cog', 'Editor Settings', toolsHtml());
    h += sec('export', 'download', 'Export', exportHtml());
    return h + '</div>' + (pbRoot ? '' : playbackHtml()) + '</div>';
  };
  ed.playbackHtml = playbackHtml;

  function download(name, text, mime) {
    const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type: mime })); a.download = name; document.body.appendChild(a); a.click();
    setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 0);
  }
  function refreshPanel() {
    refreshPlayback();
    if (!root) return;
    const fe = root.querySelector(':focus'), keep = fe && (fe.getAttribute('data-f') || ''), keepI = fe && fe.getAttribute('data-i'), sc = root.querySelector('.pp-scroll'), top = sc ? sc.scrollTop : 0;
    root.innerHTML = ed.panelHtml(); wire();
    const ns = root.querySelector('.pp-scroll'); if (ns) ns.scrollTop = top;
    if (keep) root.querySelector(`[data-f="${keep}"]${keepI !== null ? `[data-i="${keepI}"]` : ''}`)?.focus();
  }
  ed.refresh = refreshPanel;
  function refreshPlayback() {
    if (!pbRoot) return;
    pbRoot.innerHTML = playbackHtml();
    pbRoot.querySelectorAll('[data-act]').forEach(b => { b.onclick = ev => { ev.stopPropagation(); act(b.dataset.act, pbRoot, b); }; });
    const sl = pbRoot.querySelector('[data-f="scrub"]'); if (sl) sl.oninput = () => field('scrub', sl);
    const sp = pbRoot.querySelector('[data-f="pspeed"]'); if (sp) sp.onchange = () => field('pspeed', sp);
  }
  /** Host puts the floating playback bar into el (bottom-left of the field). The side panel then omits its own copy. */
  ed.bindPlayback = el => { pbRoot = el; refreshPanel(); };
  /** Renames the open path (used by the editor's top bar). Same rules as the old name field: unique names, one undo entry. */
  ed.rename = nm => field('name', { type: 'text', value: String(nm ?? '') });

  function field(f, el) {
    const seg = route.segments[ed.sel], v = el.type === 'checkbox' ? el.checked : el.value, n = parseFloat(v);
    const res = seg ? P.resolveHeadings(route)[ed.sel] : null;
    switch (f) {
      case 'cls': ed.className = v.replace(/[^A-Za-z0-9_]/g, '') || 'MyAuto'; return refreshPanel();
      case 'name': {
        const nm = v.trim(); if (!nm || nm === route.name) return refreshPanel();
        selfWrite = true; let okRen = false; try { okRen = lib.rename(route.name, nm); } finally { selfWrite = false; }
        if (!okRen) { if (!lib.names().includes(route.name)) { route.name = nm; return commit(); } toast('A path named "' + nm + '" already exists'); return refreshPanel(); }
        route.name = nm; return commit();
      }
      case 'vmax': if (n > 0) { ed.vmax = n; bump(); } return refreshPanel();
      case 'amax': if (n > 0) { ed.aMax = n; bump(); } return refreshPanel();
      case 'pspeed': ed.speed = n > 0 ? n : 1; return;
      case 'loop': ed.loop = !!v; return;
      case 'scrub': return ed.scrub(n / 1000);
      case 'rename': { const s = route.segments[+el.dataset.i]; if (s) { s.label = v.trim() === 'Path ' + (+el.dataset.i + 1) ? '' : v.trim(); commit(); } return; }
      case 'px': case 'py': {
        if (!ed.selPt || !Number.isFinite(n)) return;
        const s = route.segments[ed.selPt.seg], p = ed.selPt.idx === 0 ? route.start : s?.points[ed.selPt.idx]; if (!p) return;
        if (isLocked(ed.selPt.seg)) return toast('Path is locked');
        setPt(ed.selPt.seg, ed.selPt.idx, f === 'px' ? clamp(n) : p.x, f === 'py' ? clamp(n) : p.y); return commit();
      }
      case 'sx': case 'sy': if (Number.isFinite(n)) setPt(0, 0, f === 'sx' ? clamp(n) : route.start.x, f === 'sy' ? clamp(n) : route.start.y); return commit();
      case 'sh': if (Number.isFinite(n)) route.start.h = rad(n); return commit();
      case 'eh': { const li = route.segments.length - 1; if (li >= 0 && Number.isFinite(n)) { const r2 = applyWaypointEdit(route, { seg: li, idx: route.segments[li].points.length - 1, start: false }, 'deg', n); if (!r2.ok) { toast(r2.reason); return refreshPanel(); } return commit(); } return refreshPanel(); }
      case 'wx': case 'wy': case 'wh': {   // waypoint table: one undoable edit per change; refused edits restore the table
        const row = { seg: +el.dataset.wseg, idx: +el.dataset.widx, start: el.dataset.wstart === '1' };
        const r = applyWaypointEdit(route, row, f === 'wx' ? 'x' : f === 'wy' ? 'y' : 'deg', v);
        if (!r.ok) { toast(r.reason); return refreshPanel(); }
        return commit();
      }
      case 'cbt': case 'cbn': return;
      case 'grid': ed.grid = Number.isFinite(n) ? n : 1; return refreshPanel();
      case 'magnet': ed.magnet = !!v; return refreshPanel();
      case 'showgrid': ed.showGrid = !!v; return refreshPanel();
      case 'hdgsnap': ed.hdgSnap = Number.isFinite(n) ? n : 15; return refreshPanel();
      case 'evt': { const c = seg?.callbacks?.[+el.dataset.i]; if (c && n >= 0 && n <= 1) { c.t = n; commit(); } return; }
      case 'evrpm': { const c = seg?.callbacks?.[+el.dataset.i]; if (c && n >= 0 && n <= 6000) { c.args = { ...(c.args || {}), rpm: n }; commit(); } return; }
    }
    if (!seg) return;
    if (f === 'type') P.setSegmentType(route, ed.sel, v);
    else if (f === 'mode') { seg.heading.mode = v; if (v !== 'tangent') { seg.heading.start = res.h0; if (v === 'linear') seg.heading.end = res.h1; else delete seg.heading.end; } }
    else if (f === 'hs' && Number.isFinite(n)) { seg.heading.start = rad(n); if (seg.heading.mode === 'linear' && seg.heading.end === undefined) seg.heading.end = res.h1; }
    else if (f === 'he' && Number.isFinite(n)) seg.heading.end = rad(n);
    else if (f === 'rev') seg.heading.reversed = !!v;
    else if (f === 'speed') { if (n > 0) seg.speed = n; else delete seg.speed; }
    else if (f === 'label') seg.label = v;
    else if (f === 'color') { if (/^#[0-9a-fA-F]{6}$/.test(v) && v.toLowerCase() !== P_DEFAULT) seg.color = v.toLowerCase(); else delete seg.color; }
    commit();
  }
  const P_DEFAULT = T.line;
  function act(a, root, el) {
    if ((a === 'addrot' || a === 'addzone' || a === 'addev') && !route.segments[ed.sel] && route.segments.length) ed.sel = route.segments.length - 1;
    const seg = route.segments[ed.sel], i = el ? +el.dataset.i : ed.sel, own = route.segments[i], pname = el && el.dataset.name;
    switch (a) {
      case 'noop': return;
      case 'addev': if (seg) { if (isLocked(ed.sel)) return toast('Path is locked'); if (P.addEvent(route, ed.sel, 0.5, 'callback') >= 0) commit(); } else toast('Add a waypoint first'); return;
      case 'undo': return ed.undo(); case 'redo': return ed.redo(); case 'mirror': return ed.mirror(); case 'clear': return ed.clear();
      case 'pnew': { libSave(); lib.create(); return; }
      case 'pdup': { libSave(); lib.duplicate(pname || route.name); return; }
      case 'pdel': {
        const n = pname || route.name;
        if (lib.names().length <= 1) { toast('The project needs at least one path; clearing it instead'); return ed.clear(); }
        lib.remove(n); toast('Deleted path "' + n + '"'); return;
      }
      case 'eye': if (own) { if (own.hidden) delete own.hidden; else own.hidden = true; commit(); } return;
      case 'lock': if (own) { if (own.locked) delete own.locked; else own.locked = true; commit(); } return;
      case 'up': case 'down': if (own && P.moveSegment(route, i, a === 'up' ? -1 : 1)) { ed.sel = i + (a === 'up' ? -1 : 1); ed.selPt = null; commit(); } return;
      case 'del': if (own) { if (own.locked) return toast('Path is locked'); P.deleteSegment(route, i); ed.selPt = null; clampSel(); commit(); } return;
      case 'addrot': {   // rotation target at the default t=0.5, heading = the interpolated heading there (no visual jump)
        if (!seg) return;
        if (isLocked(ed.sel)) return toast('Path is locked');
        if (P.addRotation(route, ed.sel, 0.5, P.rotationAt(route, ed.sel, 0.5) ?? 0) >= 0) commit();
        return;
      }
      case 'addzone': {  // constraint zone over t 0.25..0.75 at half the global vmax/aMax (UI default, not a robot constant)
        if (!seg) return;
        if (isLocked(ed.sel)) return toast('Path is locked');
        if (P.addZone(route, ed.sel, 0.25, 0.75, ed.vmax / 2, ed.aMax / 2) >= 0) commit();
        return;
      }
      case 'split': if (seg && P.splitSegment(route, ed.sel, 0.5)) commit(); return;
      case 'addctl': if (seg) { const m = P.bezierPoint(seg.points, 0.5); P.insertControl(route, ed.sel, m.x, m.y + 6); commit(); } return;
      case 'delseg': if (seg) { P.deleteSegment(route, ed.sel); ed.selPt = null; clampSel(); commit(); } return;
      case 'cbadd': if (seg) {
        const t = parseFloat(root.querySelector('[data-f="cbt"]').value), name = root.querySelector('[data-f="cbn"]').value.trim();
        if (!(t >= 0 && t <= 1)) return toast('Callback t must be within 0..1'); P.addEvent(route, ed.sel, t, name || 'callback'); commit();
      } return;
      case 'copy': { const txt = ed.javaText(); (navigator.clipboard ? navigator.clipboard.writeText(txt) : Promise.reject()).then(() => toast('Java copied to clipboard'), () => toast('Clipboard unavailable; use Download')); return; }
      case 'download': return download(ed.className + '.java', ed.javaText(), 'text/x-java-source');
      case 'save': return download((route.name || 'route').replace(/\W+/g, '_') + '.json', P.toJSON(route), 'application/json');
      case 'load': return root.querySelector('[data-f="file"]').click();
      case 'exportpp': return download((route.name || 'route').replace(/\W+/g, '_') + '.pp', exportPP(route), 'application/json');
      case 'importpp': return root.querySelector('[data-f="ppfile"]').click();
      case 'toggle': return ed.toggle(); case 'stop': return ed.stop();
      case 'first': return ed.seek(0); case 'last': return ed.seek(timeEst()); case 'back': return ed.stepBy(-0.05); case 'fwd': return ed.stepBy(0.05);
    }
  }
  ed.importPP = text => { const { route: r, warnings } = ppToRoute(text); ed.ppWarnings = warnings; replaceRoute(r); return warnings; };
  function wire() {
    root.querySelectorAll('[data-act]').forEach(b => { b.onclick = ev => { ev.stopPropagation(); act(b.dataset.act, root, b); }; });
    root.querySelectorAll('[data-tool]').forEach(b => { b.onclick = () => ed.setTool(b.dataset.tool); });
    root.querySelectorAll('[data-sec]').forEach(b => { b.onclick = () => { const k = b.dataset.sec; if (ed.open.has(k)) ed.open.delete(k); else ed.open.add(k); saveOpen(); refreshPanel(); }; });
    root.querySelectorAll('[data-popen]').forEach(d => { d.onclick = () => { const n = d.dataset.popen; if (n !== route.name) { libSave(); lib.setActive(n); } }; });
    root.querySelectorAll('[data-sel]').forEach(d => { d.onclick = e => { if (e.target.closest('input,button')) return; const i = +d.dataset.sel; if (i >= 0) { ed.sel = i; ed.selPt = null; refreshPanel(); } }; });
    root.querySelectorAll('[data-selstart]').forEach(d => { d.onclick = () => { ed.selPt = { seg: 0, idx: 0 }; refreshPanel(); }; });
    // waypoint table: clicking a row (not its inputs or buttons) selects that anchor
    root.querySelectorAll('[data-wrow]').forEach(d => { d.onclick = e => {
      if (e.target.closest('input,button')) return;
      const start = d.dataset.wstart === '1';
      if (start) ed.selPt = { seg: 0, idx: 0 }; else { ed.sel = +d.dataset.wseg; ed.selPt = { seg: +d.dataset.wseg, idx: +d.dataset.widx }; }
      refreshPanel();
    }; });
    root.querySelectorAll('[data-wact]').forEach(b => { b.onclick = e => {
      e.stopPropagation();
      const row = { seg: +b.dataset.wseg, idx: +b.dataset.widx, start: b.dataset.wstart === '1' };
      if (b.dataset.wact === 'ins') {
        const at = insertWaypointAfter(route, row);
        if (!at) return toast('Path is locked');
        ed.sel = at.seg; ed.selPt = at; return commit();
      }
      if (row.start) return toast('The start point cannot be deleted');
      if (!deleteWaypoint(route, row)) return toast('Path is locked');
      ed.selPt = null; clampSel(); commit();
    }; });
    root.querySelectorAll('[data-evadd]').forEach(b => { b.onclick = () => { if (ed.sel < 0 || !route.segments[ed.sel]) return toast('Select a path first'); if (P.addEvent(route, ed.sel, 0.5, b.dataset.evadd) >= 0) commit(); }; });
    root.querySelectorAll('[data-cbdel]').forEach(a => { a.onclick = () => { if (isLocked(ed.sel)) return toast('Path is locked'); route.segments[ed.sel].callbacks.splice(+a.dataset.cbdel, 1); ed.evSel = null; commit(); }; });
    root.querySelectorAll('[data-rotdel]').forEach(a => { a.onclick = () => { const sg = a.dataset.rseg !== undefined ? +a.dataset.rseg : ed.sel; if (isLocked(sg)) return toast('Path is locked'); if (P.removeRotation(route, sg, +a.dataset.rotdel)) { ed.rotSel = null; commit(); } }; });
    root.querySelectorAll('[data-zdel]').forEach(a => { a.onclick = () => { const sg = a.dataset.zseg !== undefined ? +a.dataset.zseg : ed.sel; if (isLocked(sg)) return toast('Path is locked'); if (P.removeZone(route, sg, +a.dataset.zdel)) commit(); }; });
    root.querySelectorAll('[data-f]').forEach(el => {
      const f = el.dataset.f;
      if (f === 'file' || f === 'ppfile') el.onchange = async () => {
        const file = el.files[0]; if (!file) return;
        try {
          const text = await file.text();
          if (f === 'file') replaceRoute(P.fromJSON(text)); else { const w = ed.importPP(text); if (w.length) toast(`Imported with ${w.length} warning(s)`); }
          toast('Loaded ' + file.name);
        } catch (e) { toast('Load failed: ' + e.message); }
        el.value = '';
      };
      else if (f === 'scrub') el.oninput = () => field(f, el);
      else el.onchange = () => field(f, el);
      if (f === 'rename') el.onfocus = () => { const i = +el.dataset.i; if (ed.sel !== i) { ed.sel = i; } };
    });
  }
  ed.bindPanel = el => { root = el; refreshPanel(); };  ed.panelRoot = () => root;
  P.PLAN_VIEW.editor = ed;
  ed.library = lib;

  // ---------- project library sync (Paths list shared with the auto planner) ----------
  libSave();
  lib.subscribe(o => {
    if (selfWrite) return;
    const a = o.active;
    if (a && a !== route.name && o.items[a]) { selfWrite = true; try { switchRoute(o.items[a]); } finally { selfWrite = false; } }
    // library emptied on purpose (delete): keep the editor route in memory only, do not write it back (that would resurrect the deleted path)
    else if (!ed.drag) refreshPanel();
  });
  resample();
  return ed;
}
