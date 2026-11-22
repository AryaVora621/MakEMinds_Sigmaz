// Import / export of the Pedro Pathing Visualizer project format (JSON, PROJECT_VERSION "2.0.0") <-> our route model.
// Keys (from the Visualizer's src/io/project.ts + types.ts): version, timestamp, startPoint{x,y,headingDeg},
// lines[{id,color,name,locked,kind:'atomic',endPoint,controlPoints,heading}], shapes, sequence[{kind:'path',lineId}|{kind:'wait',durationMs}],
// fieldPoints, robot, activePaths. Heading = linear{startDeg,endDeg} | constant{degrees} | tangential{reverse} | piecewise.
// NOT verified against a real exported file: the heading discriminator key name and the file extension. The importer is therefore
// tolerant (accepts type|mode|kind, ignores unknown keys, warns instead of failing). Our extras (callbacks, speed, reversed on
// non-tangent headings) travel in a per-line `biobuzz` object that the Visualizer should ignore.
import { newRoute, newSegment, fixContinuity, resolveHeadings, wrapAngle, FIELD } from './pathing.js';

export const PP_VERSION = '2.0.0';
export const DEFAULT_COLOR = '#ffc516';
const toRad = d => d * Math.PI / 180, toDeg = r => r * 180 / Math.PI;
const round = v => Math.round(v * 1e6) / 1e6;
const isNum = v => typeof v === 'number' && Number.isFinite(v);

/** autoAim callback args: keep only shootAfter, as a strict boolean. Returns undefined when nothing is left. */
function sanitizeAutoAimArgs(args) {
  if (!args || typeof args !== 'object' || Array.isArray(args) || !('shootAfter' in args)) return undefined;
  const v = args.shootAfter;
  return { shootAfter: v === true || v === 'true' || v === 1 };
}
/** Clone a callback for the .pp file: args are copied (booleans kept); autoAim args are sanitised and omitted when empty. */
function ppCallback(c) {
  const args = c.name === 'autoAim' ? sanitizeAutoAimArgs(c.args) : (c.args && typeof c.args === 'object' ? { ...c.args } : undefined);
  return args ? { t: c.t, name: c.name, args } : { t: c.t, name: c.name };
}

/** Route -> Visualizer project object. */
export function routeToPP(route, { timestamp = new Date().toISOString() } = {}) {
  const res = resolveHeadings(route);
  const lines = route.segments.map((s, i) => {
    const hd = s.heading || { mode: 'linear' }; let heading;
    if (hd.mode === 'tangent') heading = { type: 'tangential', reverse: !!hd.reversed };
    else if (hd.mode === 'constant') heading = { type: 'constant', degrees: round(toDeg(res[i].h0)) };
    else heading = { type: 'linear', startDeg: round(toDeg(res[i].h0)), endDeg: round(toDeg(res[i].h1)) };
    const e = s.points.at(-1), extra = {};
    if (s.callbacks?.length) extra.callbacks = s.callbacks.map(ppCallback);
    if (s.speed !== undefined) extra.speed = s.speed;
    if (hd.reversed && hd.mode !== 'tangent') extra.reversed = true;
    if (s.type) extra.type = s.type;
    if (s.hidden) extra.hidden = true;
    return {
      id: `line-${i + 1}`, name: s.label || `Path ${i + 1}`, color: s.color || DEFAULT_COLOR, locked: !!s.locked, kind: 'atomic',
      endPoint: { x: round(e.x), y: round(e.y) }, controlPoints: s.points.slice(1, -1).map(p => ({ x: round(p.x), y: round(p.y) })),
      heading, biobuzz: extra,
    };
  });
  return {
    version: PP_VERSION, timestamp, name: route.name, alliance: route.alliance,
    startPoint: { x: round(route.start.x), y: round(route.start.y), headingDeg: round(toDeg(route.start.h || 0)) },
    lines, shapes: [], sequence: lines.map(l => ({ kind: 'path', lineId: l.id })), fieldPoints: [], activePaths: lines.map(l => l.id),
  };
}
export const exportPP = (route, opts) => JSON.stringify(routeToPP(route, opts), null, 2);

function readPoint(p, where, warnings) {
  if (!p || !isNum(p.x) || !isNum(p.y)) { warnings.push(`${where}: invalid point, skipped`); return null; }
  if (p.x < 0 || p.x > FIELD || p.y < 0 || p.y > FIELD) warnings.push(`${where}: (${p.x}, ${p.y}) is outside the 0..${FIELD} field`);
  return { x: p.x, y: p.y };
}
function readHeading(h, where, warnings) {
  if (h === undefined || h === null) return { mode: 'tangent', reversed: false };
  const kind = String(h.type ?? h.mode ?? h.kind ?? '').toLowerCase();
  const num = (v, name) => { if (isNum(v)) return v; if (v !== undefined) warnings.push(`${where}: heading.${name} is not a finite number, using 0`); return 0; };
  if (kind === 'linear' || (!kind && ('startDeg' in h || 'endDeg' in h))) return { mode: 'linear', start: wrapAngle(toRad(num(h.startDeg, 'startDeg'))), end: wrapAngle(toRad(num(h.endDeg ?? h.startDeg, 'endDeg'))), reversed: false };
  if (kind === 'constant' || (!kind && 'degrees' in h)) return { mode: 'constant', start: wrapAngle(toRad(num(h.degrees, 'degrees'))), reversed: false };
  if (kind === 'tangential' || kind === 'tangent' || (!kind && 'reverse' in h)) return { mode: 'tangent', reversed: !!(h.reverse ?? h.reversed) };
  if (kind === 'piecewise') { warnings.push(`${where}: piecewise heading is not supported, imported as tangential`); return { mode: 'tangent', reversed: false }; }
  warnings.push(`${where}: unknown heading '${kind}', imported as tangential`); return { mode: 'tangent', reversed: false };
}

/** Visualizer project object (or JSON text) -> { route, warnings }. Throws only on unparsable / non-object input. */
export function ppToRoute(input) {
  let o = input;
  if (typeof input === 'string') { try { o = JSON.parse(input); } catch (e) { throw new Error('Invalid .pp JSON: ' + e.message); } }
  if (!o || typeof o !== 'object' || Array.isArray(o)) throw new Error('A .pp project must be a JSON object');
  const warnings = [];
  if (o.version !== undefined && o.version !== PP_VERSION) warnings.push(`Project version ${o.version} (expected ${PP_VERSION}); importing best-effort`);
  const sp = o.startPoint ?? o.start;
  let sx = 0, sy = 0, sh = 0;
  if (sp && isNum(sp.x) && isNum(sp.y)) {
    sx = sp.x; sy = sp.y;
    const hd = sp.headingDeg ?? sp.heading?.degrees ?? sp.heading?.startDeg ?? sp.heading;
    if (isNum(hd)) sh = toRad(hd); else if (hd !== undefined) warnings.push('startPoint: heading not understood, using 0');
    if (sx < 0 || sx > FIELD || sy < 0 || sy > FIELD) warnings.push(`startPoint: (${sx}, ${sy}) is outside the field`);
  } else warnings.push('startPoint missing or invalid, using (0, 0)');
  const route = newRoute({ name: typeof o.name === 'string' && o.name ? o.name : 'MyAuto', alliance: o.alliance === 'red' ? 'red' : 'blue', start: { x: sx, y: sy, h: wrapAngle(sh) } });

  const lines = Array.isArray(o.lines) ? o.lines : (o.lines === undefined ? [] : (warnings.push('lines: not an array, ignored'), []));
  const byId = new Map(); lines.forEach((l, i) => { if (l && l.id !== undefined) byId.set(l.id, l); });
  let order = lines;
  if (Array.isArray(o.sequence) && o.sequence.length) {
    order = [];
    for (const it of o.sequence) {
      if (it?.kind === 'path' || it?.lineId !== undefined) { const l = byId.get(it.lineId); if (l) order.push(l); else warnings.push(`sequence: unknown lineId ${it.lineId}`); }
      else if (it?.kind === 'wait') warnings.push('sequence: wait items are not supported, ignored');
    }
  }
  order.forEach((l, i) => {
    const w = `lines[${i}]`;
    if (!l || typeof l !== 'object') { warnings.push(`${w}: not an object, skipped`); return; }
    let ctrl = Array.isArray(l.controlPoints) ? l.controlPoints : [], end = l.endPoint, hd = l.heading;
    if (l.kind === 'compound' || Array.isArray(l.segments)) {
      const segs = (l.segments || []).filter(Boolean); warnings.push(`${w}: compound path flattened to its last segment`);
      const last = segs.at(-1); if (!last) { warnings.push(`${w}: empty compound path, skipped`); return; }
      ctrl = Array.isArray(last.controlPoints) ? last.controlPoints : []; end = last.endPoint; hd = l.heading ?? last.heading;
    }
    const e = readPoint(end, `${w}.endPoint`, warnings); if (!e) return;
    const cps = ctrl.map((p, j) => readPoint(p, `${w}.controlPoints[${j}]`, warnings)).filter(Boolean);
    const prev = route.segments.at(-1)?.points.at(-1) ?? { x: route.start.x, y: route.start.y };
    const isLine = cps.length === 0, seg = newSegment(isLine ? 'line' : 'curve', [prev, ...cps, e]);
    seg.heading = readHeading(hd, w, warnings);
    if (typeof l.name === 'string' && l.name && !/^Path \d+$/.test(l.name)) seg.label = l.name;
    if (typeof l.color === 'string' && /^#[0-9a-fA-F]{6}$/.test(l.color) && l.color.toLowerCase() !== DEFAULT_COLOR) seg.color = l.color.toLowerCase();
    if (l.locked === true) seg.locked = true;
    const x = l.biobuzz;
    if (x && typeof x === 'object') {
      if (Array.isArray(x.callbacks)) seg.callbacks = x.callbacks.filter(c => c && isNum(c.t) && c.t >= 0 && c.t <= 1).map(c => ppCallback({ t: c.t, name: String(c.name || ''), args: c.args }));
      if (isNum(x.speed) && x.speed > 0) seg.speed = x.speed;
      if (x.reversed === true && seg.heading.mode !== 'tangent') seg.heading.reversed = true;
      if (x.hidden === true) seg.hidden = true;
    }
    route.segments.push(seg);
  });
  fixContinuity(route);
  return { route, warnings };
}
