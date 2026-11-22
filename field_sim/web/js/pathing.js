// Pedro path designer model: pure (no DOM). Pedro frame: origin bottom-left, x right, y up the screen, 144 in field.
// A Route is { name, alliance, start:{x,y,h(rad)}, segments:[Segment] }.
// Segment: { type:'line'|'curve', points:[{x,y}...] (line = 2, curve >= 3; points[0] equals the previous end),
//            heading:{mode:'constant'|'linear'|'tangent', start?:rad, end?:rad, reversed:bool},
//            speed?: max in/s, callbacks:[{t:0..1, name}], label, zones?:[{t0,t1,vmax,aMax}] (constraint zones, see below) }
// Heading semantics: constant uses heading.start (default: the previous segment's end heading); linear goes
// start -> end the shortest way round (end defaults to start); tangent follows the path derivative (+pi if reversed).

export const FIELD = 144;
export const SCHEMA_VERSION = 1;
const TAU = Math.PI * 2;

export function wrapAngle(a) { a = ((a % TAU) + TAU) % TAU; return a > Math.PI ? a - TAU : a; }
export function angDiff(from, to) { return wrapAngle(to - from); }

// ---------- model ----------
export function newRoute(opts = {}) {
  return { name: opts.name || 'MyAuto', alliance: opts.alliance || 'blue', start: { x: (opts.alliance === 'red' ? 24 : 120), y: 24, h: 0, ...(opts.start || {}) }, segments: [] }; // blue default start at x=120 so it is inside the blue half (assumption)
}
export function newSegment(type, points) {
  return { type, points: points.map(p => ({ x: p.x, y: p.y })), heading: { mode: 'linear', reversed: false }, callbacks: [], label: '' };
}
export function cloneRoute(r) { return JSON.parse(JSON.stringify(r)); }
export function routeEnd(route) {
  const n = route.segments.length;
  return n ? { ...route.segments[n - 1].points.at(-1) } : { x: route.start.x, y: route.start.y };
}
export function addLine(route, x, y) {
  const e = routeEnd(route); route.segments.push(newSegment('line', [e, { x, y }])); return route.segments.length - 1;
}
/** Adds a curve ending at (x,y); with no control given it gets one at the midpoint nudged sideways. */
export function addCurve(route, x, y, cx, cy) {
  const e = routeEnd(route);
  if (cx === undefined) {
    const dx = x - e.x, dy = y - e.y, L = Math.hypot(dx, dy) || 1;
    cx = (e.x + x) / 2 - dy / L * L * 0.25; cy = (e.y + y) / 2 + dx / L * L * 0.25;
    cx = Math.max(0, Math.min(FIELD, cx)); cy = Math.max(0, Math.min(FIELD, cy));   // keep the auto control point on the field so JSON round-trips
  }
  route.segments.push(newSegment('curve', [e, { x: cx, y: cy }, { x, y }]));
  const s = route.segments.at(-1); s.heading = { mode: 'tangent', reversed: false };
  return route.segments.length - 1;
}
export function fixContinuity(route) {
  let p = { x: route.start.x, y: route.start.y };
  for (const s of route.segments) { s.points[0] = { x: p.x, y: p.y }; p = s.points.at(-1); }
  return route;
}
export function movePoint(route, seg, idx, x, y) {
  const s = route.segments[seg]; if (!s) return false;
  s.points[idx] = { x, y };
  if (idx === 0) { if (seg === 0) { route.start.x = x; route.start.y = y; } else route.segments[seg - 1].points[route.segments[seg - 1].points.length - 1] = { x, y }; }
  if (idx === s.points.length - 1 && route.segments[seg + 1]) route.segments[seg + 1].points[0] = { x, y };
  return true;
}
export function deletePoint(route, seg, idx) {
  const s = route.segments[seg]; if (!s) return false;
  const last = s.points.length - 1;
  if (idx === 0) return seg === 0 ? false : deletePoint(route, seg - 1, route.segments[seg - 1].points.length - 1);
  if (idx === last) { // remove this segment; the next one now starts where this one started
    const start = s.points[0]; route.segments.splice(seg, 1);
    if (route.segments[seg]) route.segments[seg].points[0] = { ...start };
    return true;
  }
  s.points.splice(idx, 1);
  if (s.points.length === 2) { s.type = 'line'; if (s.heading.mode === 'tangent') s.heading.mode = 'linear'; }
  return true;
}
/** Adds a control point to a segment (a line becomes a quadratic curve). */
export function insertControl(route, seg, x, y) {
  const s = route.segments[seg]; if (!s) return false;
  s.points.splice(s.points.length - 1, 0, { x, y }); s.type = 'curve'; return true;
}
/** Moves segment `seg` one place earlier (dir -1) or later (+1) in the sequence; joins are re-chained (fixContinuity) so starts move. */
export function moveSegment(route, seg, dir) {
  const j = seg + dir; if (!route.segments[seg] || !route.segments[j]) return false;
  [route.segments[seg], route.segments[j]] = [route.segments[j], route.segments[seg]]; fixContinuity(route); return true;
}
export function deleteSegment(route, seg) { return deletePoint(route, seg, route.segments[seg]?.points.length - 1); }
export function setSegmentType(route, seg, type) {
  const s = route.segments[seg]; if (!s) return false;
  const a = s.points[0], b = s.points.at(-1);
  if (type === 'line') { s.points = [a, b]; s.type = 'line'; }
  else if (s.points.length < 3) { s.points = [a, { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 + 6 }, b]; s.type = 'curve'; }
  else s.type = 'curve';
  return true;
}

// ---------- Bezier maths ----------
export function bezierPoint(pts, t) {
  let q = pts.map(p => [p.x, p.y]);
  for (let n = q.length - 1; n > 0; n--) for (let i = 0; i < n; i++) q[i] = [q[i][0] + (q[i + 1][0] - q[i][0]) * t, q[i][1] + (q[i + 1][1] - q[i][1]) * t];
  return { x: q[0][0], y: q[0][1] };
}
export function bezierDerivative(pts, t) {
  const n = pts.length - 1; if (n < 1) return { x: 0, y: 0 };
  const d = []; for (let i = 0; i < n; i++) d.push({ x: n * (pts[i + 1].x - pts[i].x), y: n * (pts[i + 1].y - pts[i].y) });
  return d.length === 1 ? d[0] : bezierPoint(d, t);
}
function splitPoints(pts, t) {
  const left = [], right = []; let q = pts.map(p => ({ x: p.x, y: p.y }));
  while (q.length) {
    left.push(q[0]); right.unshift(q.at(-1));
    q = q.slice(0, -1).map((p, i) => ({ x: p.x + (q[i + 1].x - p.x) * t, y: p.y + (q[i + 1].y - p.y) * t }));
  }
  return [left, right];
}

// ---------- headings ----------
/** Resolved {h0,h1} for every segment (continuation defaults filled in). */
export function resolveHeadings(route) {
  const out = []; let prev = route.start.h || 0;
  for (const s of route.segments) {
    const hd = s.heading || { mode: 'linear' }; let h0, h1;
    if (hd.mode === 'tangent') {
      const off = hd.reversed ? Math.PI : 0, a = bezierDerivative(s.points, 0), b = bezierDerivative(s.points, 1);
      h0 = Math.hypot(a.x, a.y) > 1e-9 ? Math.atan2(a.y, a.x) + off : prev;
      h1 = Math.hypot(b.x, b.y) > 1e-9 ? Math.atan2(b.y, b.x) + off : h0;
    } else if (hd.mode === 'constant') { h0 = h1 = hd.start ?? prev; }
    else { h0 = hd.start ?? prev; h1 = hd.end ?? h0; }
    h0 = wrapAngle(h0); h1 = wrapAngle(h1); out.push({ h0, h1 }); prev = h1;
  }
  return out;
}
function headingAt(seg, t, r) {
  const hd = seg.heading || { mode: 'linear' };
  if (hd.mode === 'tangent') {
    const d = bezierDerivative(seg.points, t);
    if (Math.hypot(d.x, d.y) > 1e-9) return wrapAngle(Math.atan2(d.y, d.x) + (hd.reversed ? Math.PI : 0));
    return t < 0.5 ? r.h0 : r.h1;
  }
  if (hd.mode === 'constant') return r.h0;
  return wrapAngle(r.h0 + angDiff(r.h0, r.h1) * t);
}

// ---------- sampling ----------
export function sampleSegment(seg, n = 50) {
  const out = []; for (let i = 0; i <= n; i++) { const t = i / n, p = bezierPoint(seg.points, t); out.push({ x: p.x, y: p.y, t }); }
  return out;
}
function segDense(seg) {
  const d = sampleSegment(seg, 200); let L = 0; d[0].l = 0;
  for (let i = 1; i < d.length; i++) { L += Math.hypot(d[i].x - d[i - 1].x, d[i].y - d[i - 1].y); d[i].l = L; }
  return { d, L };
}
export function segmentLength(seg) { return segDense(seg).L; }
export function routeLength(route) { return route.segments.reduce((s, g) => s + segmentLength(g), 0); }
/** Arc-length parameterised samples every `step` inches: [{x,y,h,t,seg,s}] (t is the Bezier parameter inside its segment). */
export function sampleRoute(route, step = 1) {
  const res = resolveHeadings(route), out = []; let base = 0;
  if (!route.segments.length) return [{ x: route.start.x, y: route.start.y, h: route.start.h || 0, t: 0, seg: -1, s: 0 }];
  route.segments.forEach((seg, i) => {
    const { d, L } = segDense(seg), push = (t, x, y, l) => out.push({ x, y, h: headingAt(seg, t, res[i]), t, seg: i, s: base + l });
    if (i === 0) push(0, d[0].x, d[0].y, 0);
    if (L > 1e-9) {
      let j = 1;
      for (let l = step; l < L - step * 1e-3; l += step) {
        while (j < d.length - 1 && d[j].l < l) j++;
        const a = d[j - 1], b = d[j], f = b.l > a.l ? (l - a.l) / (b.l - a.l) : 0;
        push(a.t + (b.t - a.t) * f, a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, l);
      }
    }
    push(1, d.at(-1).x, d.at(-1).y, L); base += L;
  });
  return out;
}

// ---------- mirror / split ----------
export function mirrorRoute(route) {
  const r = cloneRoute(route), mh = h => (h === undefined ? undefined : wrapAngle(Math.PI - h));
  r.alliance = route.alliance === 'red' ? 'blue' : route.alliance === 'blue' ? 'red' : route.alliance;
  r.start.x = FIELD - r.start.x; r.start.h = mh(r.start.h);
  for (const s of r.segments) {
    s.points.forEach(p => { p.x = FIELD - p.x; });
    for (const c of s.rotations || []) c.h = mh(c.h);
    if (s.heading.mode !== 'tangent') { s.heading.start = mh(s.heading.start); s.heading.end = mh(s.heading.end); if (s.heading.start === undefined) delete s.heading.start; if (s.heading.end === undefined) delete s.heading.end; }
  }
  return r;
}
/** Splits segment `seg` at Bezier parameter t into two segments (de Casteljau); returns true on success. */
export function splitSegment(route, seg, t = 0.5) {
  const s = route.segments[seg]; if (!s || t <= 0 || t >= 1) return false;
  const res = resolveHeadings(route)[seg], [lp, rp] = splitPoints(s.points, t);
  const a = { ...s, points: lp, heading: { ...s.heading }, callbacks: [], label: s.label }, b = { ...s, points: rp, heading: { ...s.heading }, callbacks: [], label: '' };
  if (s.heading.mode === 'linear') { const hm = wrapAngle(res.h0 + angDiff(res.h0, res.h1) * t); a.heading.start = res.h0; a.heading.end = hm; b.heading.start = hm; b.heading.end = res.h1; }
  else if (s.heading.mode === 'constant') { a.heading.start = b.heading.start = res.h0; }
  for (const c of s.callbacks || []) (c.t < t ? a : b).callbacks.push({ ...c, t: c.t < t ? c.t / t : (c.t - t) / (1 - t) });
  const rot = s.rotations || [];
  a.rotations = rot.filter(c => c.t <= t).map(c => ({ t: c.t / t, h: c.h }));
  b.rotations = rot.filter(c => c.t > t).map(c => ({ t: (c.t - t) / (1 - t), h: c.h }));
  // constraint zones are split with the segment (the spread above shares the array, so each half gets its own)
  const zn = zonesOf(s), az = zn.filter(z => z.t0 < t).map(z => ({ t0: z.t0 / t, t1: Math.min(z.t1, t) / t, vmax: z.vmax, aMax: z.aMax }));
  const bz = zn.filter(z => z.t1 > t).map(z => ({ t0: (Math.max(z.t0, t) - t) / (1 - t), t1: (z.t1 - t) / (1 - t), vmax: z.vmax, aMax: z.aMax }));
  delete a.zones; delete b.zones; if (az.length) a.zones = az; if (bz.length) b.zones = bz;
  route.segments.splice(seg, 1, a, b); return true;
}

// ---------- undo / redo (in-place snapshots) ----------
export function createHistory(route, limit = 200) {
  let stack = [JSON.stringify(route)], i = 0;
  const restore = () => { const o = JSON.parse(stack[i]); for (const k of Object.keys(route)) delete route[k]; Object.assign(route, o); return route; };
  return {
    commit() { const s = JSON.stringify(route); if (s === stack[i]) return false; stack = stack.slice(0, i + 1); stack.push(s); if (stack.length > limit) stack.shift(); i = stack.length - 1; return true; },
    undo() { if (i > 0) { i--; restore(); return true; } return false; },
    redo() { if (i < stack.length - 1) { i++; restore(); return true; } return false; },
    get canUndo() { return i > 0; }, get canRedo() { return i < stack.length - 1; },
  };
}

// ---------- real-Pedro bridge ----------
/** Request body for POST /api/pedro/sample (headings converted rad -> deg; points copied). */
export function routeToBridgeJson(route, step = 1) {
  const d = r => (r === undefined ? undefined : r * 180 / Math.PI);
  return {
    start: { x: route.start.x, y: route.start.y, h: d(route.start.h || 0) }, step,
    segments: route.segments.map(s => {
      const hd = s.heading || { mode: 'linear' }, heading = { mode: hd.mode, reversed: !!hd.reversed };
      if (hd.start !== undefined) heading.start = d(hd.start);
      if (hd.end !== undefined) heading.end = d(hd.end);
      return { type: s.type, points: s.points.map(p => ({ x: p.x, y: p.y })), heading };
    }),
  };
}

// ---------- JSON ----------
export function toJSON(route) { return JSON.stringify({ schema: SCHEMA_VERSION, name: route.name, alliance: route.alliance, start: route.start, segments: route.segments }, null, 2); }const MODES = ['constant', 'linear', 'tangent'];
function num(v, path, { field = true } = {}) {
  if (typeof v !== 'number' || !Number.isFinite(v)) throw new Error(`${path}: not a finite number (${v === null ? 'null/NaN' : String(v)})`);
  if (field && (v < 0 || v > FIELD)) throw new Error(`${path}: ${v} is outside the field (0..${FIELD})`);
  return v;
}
export function fromJSON(text) {
  let o; try { o = JSON.parse(text); } catch (e) { throw new Error('Invalid JSON: ' + e.message); }
  if (!o || typeof o !== 'object') throw new Error('Route must be a JSON object');
  if (o.schema !== SCHEMA_VERSION) throw new Error(`Unsupported schema version ${o.schema} (expected ${SCHEMA_VERSION})`);
  if (!o.start) throw new Error('start: missing');
  const r = newRoute({ name: String(o.name || 'MyAuto'), alliance: o.alliance === 'red' ? 'red' : 'blue', start: { x: num(o.start.x, 'start.x'), y: num(o.start.y, 'start.y'), h: num(o.start.h ?? 0, 'start.h', { field: false }) } });
  if (!Array.isArray(o.segments)) throw new Error('segments: must be an array');
  o.segments.forEach((s, i) => {
    const p = `segments[${i}]`;
    if (s.type !== 'line' && s.type !== 'curve') throw new Error(`${p}.type: must be 'line' or 'curve'`);
    if (!Array.isArray(s.points)) throw new Error(`${p}.points: missing`);
    if (s.type === 'line' && s.points.length !== 2) throw new Error(`${p}.points: a line needs exactly 2 points`);
    if (s.type === 'curve' && s.points.length < 3) throw new Error(`${p}.points: a curve needs at least 3 points`);
    const pts = s.points.map((q, j) => ({ x: num(q?.x, `${p}.points[${j}].x`), y: num(q?.y, `${p}.points[${j}].y`) }));
    const hd = s.heading || { mode: 'linear' };
    if (!MODES.includes(hd.mode)) throw new Error(`${p}.heading.mode: must be one of ${MODES.join(', ')}`);
    const seg = newSegment(s.type, pts); seg.heading = { mode: hd.mode, reversed: !!hd.reversed };
    if (hd.start !== undefined) seg.heading.start = num(hd.start, `${p}.heading.start`, { field: false });
    if (hd.end !== undefined) seg.heading.end = num(hd.end, `${p}.heading.end`, { field: false });
    if (s.speed !== undefined) { seg.speed = num(s.speed, `${p}.speed`, { field: false }); if (seg.speed <= 0) throw new Error(`${p}.speed: must be > 0`); }
    seg.label = String(s.label || '');
    if (s.rotations !== undefined && !Array.isArray(s.rotations)) throw new Error(`${p}.rotations: must be an array`);
    const rot = (s.rotations || []).map((c, k) => {
      const t = num(c?.t, `${p}.rotations[${k}].t`, { field: false });
      if (t < 0 || t > 1) throw new Error(`${p}.rotations[${k}].t: must be within 0..1`);
      return { t, h: wrapAngle(num(c.h, `${p}.rotations[${k}].h`, { field: false })) };
    }).sort((a, b) => a.t - b.t);
    if (rot.length) seg.rotations = rot; // absent when empty: old-format routes round-trip byte-identical (rotations read as [] via `|| []`)
    if (s.zones !== undefined && !Array.isArray(s.zones)) throw new Error(`${p}.zones: must be an array`);
    const zs = (s.zones || []).map((z, k) => {
      const t0 = num(z?.t0, `${p}.zones[${k}].t0`, { field: false }), t1 = num(z?.t1, `${p}.zones[${k}].t1`, { field: false });
      if (t0 < 0 || t0 > 1 || t1 < 0 || t1 > 1) throw new Error(`${p}.zones[${k}]: t0/t1 must be within 0..1`);
      const vmax = num(z.vmax, `${p}.zones[${k}].vmax`, { field: false }), aMax = num(z.aMax, `${p}.zones[${k}].aMax`, { field: false });
      if (!(vmax > 0) || !(aMax > 0)) throw new Error(`${p}.zones[${k}]: vmax and aMax must be > 0`);
      return { t0: Math.min(t0, t1), t1: Math.max(t0, t1), vmax, aMax };
    }).sort((a, b) => a.t0 - b.t0);
    if (zs.length) seg.zones = zs; // absent when empty, like rotations (reads as [] via zonesOf)
    if (typeof s.color === 'string' && /^#[0-9a-fA-F]{6}$/.test(s.color)) seg.color = s.color;
    if (s.hidden === true) seg.hidden = true;
    if (s.locked === true) seg.locked = true;
    seg.callbacks = (s.callbacks || []).map((c, k) => { const o = { t: num(c.t, `${p}.callbacks[${k}].t`, { field: false }), name: String(c.name || '') }; if (c.args && typeof c.args === 'object' && !Array.isArray(c.args)) o.args = { ...c.args }; return o; });
    for (const c of seg.callbacks) if (c.t < 0 || c.t > 1) throw new Error(`${p}.callbacks: t must be within 0..1`);
    r.segments.push(seg);
  });
  fixContinuity(r); return r;
}

// ---------- validation ----------
export function validateRoute(route, obstacles = []) {
  const issues = [], add = (severity, msg, seg = -1) => issues.push({ severity, msg, seg });
  const inb = p => p.x >= 0 && p.x <= FIELD && p.y >= 0 && p.y <= FIELD;
  if (!inb(route.start)) add('error', `Start (${route.start.x.toFixed(1)}, ${route.start.y.toFixed(1)}) is outside the field`);
  const res = resolveHeadings(route);
  route.segments.forEach((s, i) => {
    s.points.forEach((p, j) => { if (!inb(p)) add('error', `Segment ${i + 1} point ${j + 1} (${p.x.toFixed(1)}, ${p.y.toFixed(1)}) is outside the field`, i); });
    if (segmentLength(s) < 0.05) add('error', `Segment ${i + 1} has zero length`, i);
    const prev = i === 0 ? route.start : route.segments[i - 1].points.at(-1);
    if (Math.hypot(prev.x - s.points[0].x, prev.y - s.points[0].y) > 1e-6) add('error', `Segment ${i + 1} does not start where the previous one ends`, i);
    const prevH = i === 0 ? route.start.h || 0 : res[i - 1].h1;
    if (s.heading.mode !== 'constant' || s.heading.start !== undefined) {
      const j = Math.abs(angDiff(prevH, res[i].h0));
      if (j > Math.PI / 2 + 1e-6) add('warn', `Heading jumps ${(j * 180 / Math.PI).toFixed(0)} deg at the start of segment ${i + 1}`, i);
    }
    if (s.speed !== undefined && !(s.speed > 0)) add('error', `Segment ${i + 1} has a non-positive speed`, i);
  });
  if (obstacles.length) {
    const reported = new Set();
    for (const p of sampleRoute(route, 0.25)) for (const [oi, o] of obstacles.entries()) {
      if (p.x >= o.minX && p.x <= o.maxX && p.y >= o.minY && p.y <= o.maxY && !reported.has(p.seg + ':' + oi)) {
        reported.add(p.seg + ':' + oi); add('warn', `Segment ${p.seg + 1} passes through obstacle ${o.name || (o.kind ? o.kind.replace(/_/g, ' ') + ' ' : '') + (oi + 1)}`, p.seg);
      }
    }
  }
  return issues;
}

// ---------- constraint zones (PathPlanner-style speed/accel limits over a Bezier t-range of one segment) ----------
// segment.zones = [{t0, t1, vmax, aMax}] with 0 <= t0 <= t1 <= 1 (t is the segment's Bezier parameter). Kept sorted by t0.
// Absent when empty, so old JSON round-trips byte-identical; read as [] via zonesOf.
const zonesOf = s => s?.zones || [];
/** Adds a constraint zone on segment `seg` covering t0..t1 (clamped to 0..1 and ordered). Rejects non-finite values and
 *  non-positive speeds/accelerations. Returns the zone's sorted index or -1. */
export function addZone(route, seg, t0, t1, vmax, aMax) {
  const s = route.segments[seg];
  if (!s || ![t0, t1, vmax, aMax].every(Number.isFinite) || !(vmax > 0) || !(aMax > 0)) return -1;
  const a = Math.max(0, Math.min(1, t0)), b = Math.max(0, Math.min(1, t1));
  const z = { t0: Math.min(a, b), t1: Math.max(a, b), vmax, aMax };
  (s.zones ||= []).push(z); s.zones.sort((p, q) => p.t0 - q.t0);
  return s.zones.indexOf(z);
}
/** Removes zone i of segment seg; drops the zones field once it is empty. */
export function removeZone(route, seg, i) {
  const s = route.segments[seg], z = s?.zones; if (!z || !z[i]) return false;
  z.splice(i, 1); if (!z.length) delete s.zones; return true;
}
/** Speed/accel limits at Bezier parameter t of segment seg. If zones cover t, returns the smallest vmax and smallest aMax
 *  among them (zone values replace the globals); otherwise the globals passed in. */
export function constraintsAt(route, seg, t, { vmax = 60, aMax = 90 } = {}) {
  let best = null;
  for (const z of zonesOf(route.segments[seg])) {
    if (t < z.t0 || t > z.t1) continue;
    best = best ? { vmax: Math.min(best.vmax, z.vmax), aMax: Math.min(best.aMax, z.aMax) } : { vmax: z.vmax, aMax: z.aMax };
  }
  return best || { vmax, aMax };
}

// ---------- follower simulation ----------
/** Advances `state` ({s,v}, start with {}) by dt along the route with a trapezoidal speed profile. */
export function followSim(route, dt, state = {}, opts = {}) {
  const vmax = opts.vmax ?? 60, aMax = opts.aMax ?? 90;
  if (!state._pts || state._route !== route || opts.refresh) { state._route = route; state._pts = sampleRoute(route, 0.5); state._L = state._pts.at(-1).s; state.s = 0; state.v = 0; state.done = false; }
  const pts = state._pts, L = state._L;
  if (state.s === undefined) { state.s = 0; state.v = 0; }
  if (!state.done) {
    const i0 = Math.min(pts.length - 1, Math.max(0, Math.floor(state.s / 0.5))), seg = route.segments[pts[i0].seg];
    // zone limits at the current sample (no zones: the globals, so results are unchanged)
    const zc = constraintsAt(route, pts[i0].seg, pts[i0].t, { vmax, aMax });
    const cap = Math.min(zc.vmax, seg?.speed ?? Infinity), accel = zc.aMax, remaining = Math.max(0, L - state.s);
    let v = Math.min(state.v + accel * dt, cap, Math.sqrt(2 * accel * remaining));
    if (remaining > 1e-6) v = Math.max(v, Math.min(cap, 2));
    state.v = v; state.s = Math.min(L, state.s + v * dt);
    if (L - state.s < 1e-6) { state.s = L; state.v = 0; state.done = true; }
  }
  // locate the pose at arc length s
  let lo = 0, hi = pts.length - 1; while (lo < hi - 1) { const m = (lo + hi) >> 1; if (pts[m].s <= state.s) lo = m; else hi = m; }
  const a = pts[lo], b = pts[hi], f = b.s > a.s ? Math.min(1, Math.max(0, (state.s - a.s) / (b.s - a.s))) : 0;
  const out = { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, h: wrapAngle(a.h + angDiff(a.h, b.h) * f), done: !!state.done, progress: L > 0 ? state.s / L : 1, v: state.v, s: state.s };
  return out;
}
export function estimateTime(route, vmax = 60, aMax = 90) {
  const st = {}; let t = 0; const dt = 0.005;
  if (routeLength(route) < 1e-9) return 0;
  while (t < 600) { const r = followSim(route, dt, st, { vmax, aMax }); t += dt; if (r.done) break; }
  return t;
}

// ---------- event markers (callbacks that name a robot action) ----------
// A callback {t, name, args?} is an "event marker". If name is one of EVENT_PRESETS it is a real robot action: the Java export
// calls the subsystem and the auto planner (autos.js) runs the same-named command when the follower passes t.
export const EVENT_PRESETS = {
  intakeOn: { label: 'Intake on', color: '#3ba776', icon: 'I+', java: 'intake.on(); ' },
  intakeOff: { label: 'Intake off', color: '#3ba776', icon: 'I-', java: 'intake.off(); ' },
  shoot: { label: 'Shoot', color: '#c2427c', icon: 'S', java: 'shooter.shoot(1); ' },
  shootAll: { label: 'Shoot all', color: '#c2427c', icon: 'S*', java: 'shooter.shootAll(); ' },
  setRpm: { label: 'Set RPM', color: '#e0a030', icon: 'RPM', args: { rpm: 3000 }, java: c => `shooter.setRpm(${fmtNum(c.args?.rpm ?? 3000)}); ` },
  flowerDeploy: { label: 'Flower deploy', color: '#a78bfa', icon: 'F+', java: 'flower.deploy(); ' },
  flowerStow: { label: 'Flower stow', color: '#a78bfa', icon: 'F-', java: 'flower.stow(); ' },
  gateOpen: { label: 'Gate open', color: '#2fa8d8', icon: 'G+', java: 'gate.open(); ' },
  gateClose: { label: 'Gate close', color: '#2fa8d8', icon: 'G-', java: 'gate.close(); ' },
};
export const isEventPreset = name => Object.prototype.hasOwnProperty.call(EVENT_PRESETS, name);
function eventJava(c) { if (!isEventPreset(c.name)) return ''; const p = EVENT_PRESETS[c.name]; return typeof p.java === 'function' ? p.java(c) : p.java; }
/** All event markers of the route in travel order: [{seg, i, t, name, args?, x, y, frac (0..1 of total length), preset}]. */
export function routeEvents(route) {
  const lens = route.segments.map(segmentLength), total = lens.reduce((a, b) => a + b, 0) || 1, out = []; let base = 0;
  route.segments.forEach((s, si) => {
    (s.callbacks || []).forEach((c, i) => { const q = bezierPoint(s.points, c.t); out.push({ ...c, seg: si, i, x: q.x, y: q.y, frac: (base + lens[si] * c.t) / total, preset: isEventPreset(c.name) }); });
    base += lens[si];
  });
  return out.sort((a, b) => a.frac - b.frac);
}
export function addEvent(route, seg, t, name, args) {
  const s = route.segments[seg]; if (!s || !(t >= 0 && t <= 1)) return -1;
  const c = { t, name: String(name || 'callback') }; const a = args ?? EVENT_PRESETS[name]?.args; if (a) c.args = { ...a };
  (s.callbacks ||= []).push(c); return s.callbacks.length - 1;
}
export function moveEvent(route, seg, i, t) { const c = route.segments[seg]?.callbacks?.[i]; if (!c) return false; c.t = Math.max(0, Math.min(1, t)); return true; }
export function removeEvent(route, seg, i) { const cb = route.segments[seg]?.callbacks; if (!cb || !cb[i]) return false; cb.splice(i, 1); return true; }
/** Nearest Bezier parameter on segment `seg` to field point (x,y). */
export function nearestT(route, seg, x, y) {
  const s = route.segments[seg]; if (!s) return 0; let bt = 0, bd = Infinity;
  for (let i = 0; i <= 100; i++) { const t = i / 100, p = bezierPoint(s.points, t), d = (p.x - x) ** 2 + (p.y - y) ** 2; if (d < bd) { bd = d; bt = t; } }
  return bt;
}

// ---------- snapping ----------
export function snapAngle(rad, stepDeg = 15) { if (!(stepDeg > 0)) return wrapAngle(rad); const st = stepDeg * Math.PI / 180; return wrapAngle(Math.round(rad / st) * st); }
/** Snaps (x,y): to other route points within `radius` (magnet) first, else to a `grid` (inches; 0 = off). Clamped to the field. */
export function snapPoint(route, x, y, { grid = 1, magnet = true, radius = 2, exclude = null } = {}) {
  const cl = v => Math.max(0, Math.min(FIELD, v));
  if (magnet) {
    let best = null, bd = radius;
    const consider = (px, py, key) => { if (exclude && exclude.has(key)) return; const d = Math.hypot(px - x, py - y); if (d <= bd) { bd = d; best = { x: px, y: py }; } };
    consider(route.start.x, route.start.y, '0:0');
    route.segments.forEach((s, i) => s.points.forEach((p, j) => { if (j > 0) consider(p.x, p.y, i + ':' + j); }));
    if (best) return { x: cl(best.x), y: cl(best.y), snapped: 'point' };
  }
  if (grid > 0) return { x: cl(Math.round(x / grid) * grid), y: cl(Math.round(y / grid) * grid), snapped: 'grid' };
  return { x: cl(x), y: cl(y), snapped: null };
}

// ---------- waypoint heading ----------
/** Sets the heading the robot has at the END of segment `seg` (the waypoint heading). Returns false for tangent paths. */
export function setEndHeading(route, seg, h) {
  const s = route.segments[seg]; if (!s || s.heading.mode === 'tangent') return false;
  const res = resolveHeadings(route)[seg];
  if (s.heading.mode === 'constant') s.heading.start = wrapAngle(h);
  else { s.heading.start = res.h0; s.heading.end = wrapAngle(h); }
  return true;
}

// ---------- rotation targets (PathPlanner-style heading points along a segment) ----------
// segment.rotations = [{t:0..1, h:rad}], kept sorted by t. Only the heading between the segment's start and end
// is interpolated; the start/end waypoint headings are the anchors (rotationAt). Old JSON without the field loads as [].
const sortRot = s => { s.rotations = (s.rotations || []).sort((a, b) => a.t - b.t); return s.rotations; };
/** Adds a rotation target at Bezier parameter t with heading h (rad). Returns its index (sorted) or -1. */
export function addRotation(route, seg, t, h) {
  const s = route.segments[seg]; if (!s || !(t >= 0 && t <= 1) || !Number.isFinite(h)) return -1;
  const c = { t, h: wrapAngle(h) }; (s.rotations ||= []).push(c); sortRot(s); return s.rotations.indexOf(c);
}
/** Moves rotation target i to (t, h). Returns its new sorted index or -1. */
export function moveRotation(route, seg, i, t, h) {
  const s = route.segments[seg], c = s?.rotations?.[i]; if (!c || !(t >= 0 && t <= 1) || !Number.isFinite(h)) return -1;
  c.t = t; c.h = wrapAngle(h); sortRot(s); return s.rotations.indexOf(c);
}
export function removeRotation(route, seg, i) {
  const rot = route.segments[seg]?.rotations; if (!rot || !rot[i]) return false; rot.splice(i, 1); return true;
}
/** Heading at Bezier parameter t of segment `seg`: linear (shortest way round) between the neighbouring targets, with the
 *  segment start/end headings (resolveHeadings) as the outermost anchors. Null for an unknown segment. */
export function rotationAt(route, seg, t) {
  const s = route.segments[seg]; if (!s) return null;
  const r = resolveHeadings(route)[seg];
  const pts = [{ t: 0, h: r.h0 }, ...[...(s.rotations || [])].sort((a, b) => a.t - b.t), { t: 1, h: r.h1 }];
  if (t <= 0) return wrapAngle(r.h0);
  if (t >= 1) return wrapAngle(r.h1);
  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i], b = pts[i + 1];
    if (t >= a.t && t <= b.t) { const f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0; return wrapAngle(a.h + angDiff(a.h, b.h) * f); }
  }
  return wrapAngle(r.h1);
}

// ---------- time-parameterised trajectory ----------
/** Local fallback trajectory [{t,x,y,h,vx,vy}] every `dt` s from the trapezoidal follower model (same as the preview). */
export function localTrajectory(route, { dt = 0.02, vmax = 60, aMax = 90 } = {}) {
  const st = {}, out = []; let t = 0, prev = null;
  if (routeLength(route) < 1e-9) return [{ t: 0, x: route.start.x, y: route.start.y, h: route.start.h || 0, vx: 0, vy: 0 }];
  for (let i = 0; i < 60000; i++) {
    const r = followSim(route, i === 0 ? 0 : dt, st, { vmax, aMax }); if (i > 0) t += dt;
    out.push({ t, x: r.x, y: r.y, h: r.h, vx: prev ? (r.x - prev.x) / dt : 0, vy: prev ? (r.y - prev.y) / dt : 0 }); prev = r;
    if (r.done) break;
  }
  return out;
}
/** Best available trajectory: the real Pedro follower (pedrofollow.js, optional) else localTrajectory. Returns {traj, source}. */
export async function planTrajectory(route, opts = {}) {
  try {
    const mod = await import('./pedrofollow.js');
    if (mod && typeof mod.followRoute === 'function') {
      const traj = await mod.followRoute(route, { x: route.start.x, y: route.start.y, h: route.start.h || 0 });
      if (Array.isArray(traj) && traj.length > 1) return { traj, source: 'pedro-follower' };
    }
  } catch (_) { /* module missing or server down: fall back */ }
  return { traj: localTrajectory(route, opts), source: 'local' };
}
/** Interpolated pose of a trajectory at time t. */
export function trajectoryPoseAt(traj, t) {
  if (!traj || !traj.length) return null;
  if (t <= traj[0].t) return { ...traj[0] };
  const last = traj.at(-1); if (t >= last.t) return { ...last };
  let lo = 0, hi = traj.length - 1; while (lo < hi - 1) { const m = (lo + hi) >> 1; if (traj[m].t <= t) lo = m; else hi = m; }
  const a = traj[lo], b = traj[hi], f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
  return { t, x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, h: wrapAngle(a.h + angDiff(a.h, b.h) * f), vx: (a.vx || 0) + ((b.vx || 0) - (a.vx || 0)) * f, vy: (a.vy || 0) + ((b.vy || 0) - (a.vy || 0)) * f };
}

// ---------- Java export ----------
export function fmtNum(v) {
  let s = (Math.round(v * 100) / 100).toFixed(2).replace(/0+$/, '');
  if (s.endsWith('.')) s += '0'; return s === '-0.0' ? '0.0' : s;
}
export function fmtAngle(rad) {
  let deg = wrapAngle(rad) * 180 / Math.PI;
  const r5 = Math.round(deg / 5) * 5;
  deg = Math.abs(deg - r5) < 1e-6 ? r5 : Math.round(deg * 100) / 100;
  if (Object.is(deg, -0)) deg = 0;
  return `Math.toRadians(${Number.isInteger(deg) ? deg : String(deg)})`;
}
const javaId = (s, d) => { let id = String(s || d).replace(/[^A-Za-z0-9_]/g, '_'); if (!/^[A-Za-z_]/.test(id)) id = '_' + id; return id; };
const cmt = s => String(s || '').replace(/[^\w .,:()+-]/g, '');

function javaParts(route) {
  const res = resolveHeadings(route), fields = [], pathFields = [], body = [], events = [];
  const sp = route.start;
  fields.push(`private final Pose startPose = new Pose(${fmtNum(sp.x)}, ${fmtNum(sp.y)}, ${fmtAngle(sp.h || 0)});`);
  route.segments.forEach((s, i) => {
    const n = i + 1, e = s.points.at(-1);
    s.points.slice(1, -1).forEach((p, j) => fields.push(`private final Pose control${n}_${j + 1} = new Pose(${fmtNum(p.x)}, ${fmtNum(p.y)});`));
    fields.push(`private final Pose pose${n} = new Pose(${fmtNum(e.x)}, ${fmtNum(e.y)}, ${fmtAngle(res[i].h1)});`);
    pathFields.push(`path${n}`);
    const from = i === 0 ? 'startPose' : `pose${i}`, hd = s.heading;
    const geom = s.type === 'line' ? `new BezierLine(${from}, pose${n})` : `new BezierCurve(${[from, ...s.points.slice(1, -1).map((_, j) => `control${n}_${j + 1}`), `pose${n}`].join(', ')})`;
    const lines = [`        path${n} = follower.pathBuilder()`, `                .addPath(${geom})`];
    if (hd.mode === 'tangent') lines.push('                .setTangentHeadingInterpolation()');
    else if (hd.mode === 'constant') lines.push(`                .setConstantHeadingInterpolation(${fmtAngle(res[i].h0)})`);
    else lines.push(`                .setLinearHeadingInterpolation(${fmtAngle(res[i].h0)}, ${fmtAngle(res[i].h1)})`);
    if (hd.reversed) lines.push('                .setReversed()');
    // Pedro 2.1.0 chain callbacks never fire: markers are polled in the follow state (getCurrentTValue), see toJava.
    (s.callbacks || []).forEach((c, k) => { fields.push(`private boolean ev${n}_${k + 1};   // marker fired on path ${n}`); events.push({ path: n, flag: `ev${n}_${k + 1}`, t: c.t, code: eventJava(c), name: cmt(c.name) || 'callback' }); });
    lines.push('                .build();');
    const note = s.type === 'curve' && s.points.length > 4 ? `        // ${s.points.length}-point Bezier (degree ${s.points.length - 1}): BezierCurve accepts any number of control Poses\n` : '';
    body.push((s.label ? `        // ${cmt(s.label)}\n` : '') + note + lines.join('\n'));
  });
  return { fields, pathFields, body, events };
}
function usedImports(route) {
  const set = new Set(['com.pedropathing.geometry.Pose', 'com.pedropathing.paths.PathChain']);
  if (route.segments.some(s => s.type === 'line')) set.add('com.pedropathing.geometry.BezierLine');
  if (route.segments.some(s => s.type === 'curve')) set.add('com.pedropathing.geometry.BezierCurve');
  return set;
}
export function toJavaSnippet(route) {
  const { fields, pathFields, body } = javaParts(route);
  return [...fields, '', pathFields.length ? `private PathChain ${pathFields.join(', ')};` : '', '', 'public void buildPaths() {', body.join('\n\n'), '}'].join('\n');
}
export function toJava(route, opts = {}) {
  const cls = javaId(opts.className, 'MyAuto'), pkg = opts.package ?? 'commands.autonomous';
  const { fields, pathFields, body, events } = javaParts(route), n = route.segments.length;
  // Pedro 2.1.0 chain callbacks never fire, so each marker is polled while its path is being followed.
  const poll = p => events.filter(e => e.path === p).map(e => [
    `                if (!${e.flag} && follower.getCurrentTValue() >= ${fmtNum(e.t)}) {`, `                    ${e.flag} = true;`, `                    ${e.code}/* ${e.name} */`, '                }', ''].join('\n')).join('');
  const imps = [...usedImports(route), 'com.pedropathing.follower.Follower', 'com.qualcomm.robotcore.eventloop.opmode.Autonomous', 'com.qualcomm.robotcore.eventloop.opmode.OpMode', 'pedroPathing.Constants'].sort();
  const cases = [];
  for (let i = 0; i < n; i++) cases.push(i === 0
    ? `            case 0:\n                follower.followPath(path1, true);\n                setPathState(1);\n                break;`
    : `            case ${i}:\n${poll(i)}                if (!follower.isBusy()) {\n                    follower.followPath(path${i + 1}, true);\n                    setPathState(${i + 1});\n                }\n                break;`);
  cases.push(n === 0 ? `            case 0:\n                setPathState(-1);\n                break;` : `            case ${n}:\n${poll(n)}                if (!follower.isBusy()) {\n                    setPathState(-1);   // route finished\n                }\n                break;`);
  return [
    ...(pkg ? [`package ${pkg};`, ''] : []),
    ...imps.map(i => `import ${i};`), '',
    `/** ${cmt(route.name)} (${route.alliance} alliance), generated by the BIOBUZZ field sim path designer. */`,
    `@Autonomous(name = "${cmt(opts.opModeName || route.name || cls)}", group = "Pedro")`,
    `public class ${cls} extends OpMode {`,
    '    private Follower follower;',
    '    private int pathState;', '',
    ...fields.map(f => '    ' + f), '',
    pathFields.length ? `    private PathChain ${pathFields.join(', ')};` : '', '',
    '    public void buildPaths() {', body.length ? body.join('\n\n') : '        // no segments yet', '    }', '',
    '    private void setPathState(int state) {', '        pathState = state;', '    }', '',
    '    private void autonomousPathUpdate() {', '        switch (pathState) {', ...cases, '            default:', '                break;', '        }', '    }', '',
    '    @Override', '    public void init() {', '        follower = Constants.createFollower(hardwareMap);', '        follower.setStartingPose(startPose);', '        buildPaths();', '    }', '',
    '    @Override', '    public void start() {', '        setPathState(0);', '    }', '',
    '    @Override', '    public void loop() {', '        follower.update();', '        autonomousPathUpdate();', '',
    '        telemetry.addData("path state", pathState);', '        telemetry.addData("x", follower.getPose().getX());', '        telemetry.addData("y", follower.getPose().getY());',
    '        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));', '        telemetry.update();', '    }', '}', '',
  ].join('\n');
}

// ---------- project path library (PathPlanner-style "Paths" list; shared by the path editor and the auto planner) ----------
/** Named routes persisted in localStorage ({v, active, items:{name: route}}). Falls back to memory when storage is unavailable.
 *  subscribe(fn) is called after every write with the raw state. */
export function createPathLibrary(st, key = 'sim.paths.v1') {
  const subs = new Set(); let mem = { v: 1, active: null, items: {} };
  const store = () => { try { return st !== undefined ? st : (typeof localStorage !== 'undefined' ? localStorage : null); } catch (_) { return null; } };
  function read() {
    const s = store(); if (!s) return mem;
    try { const o = JSON.parse(s.getItem(key) || 'null'); if (o && o.items && typeof o.items === 'object') return o; } catch (_) { /* empty */ }
    return { v: 1, active: null, items: {} };
  }
  function write(o) {
    mem = o; const s = store(); if (s) { try { s.setItem(key, JSON.stringify(o)); } catch (_) { /* quota */ } }
    for (const f of [...subs]) { try { f(o); } catch (_) { /* subscriber error */ } }
    return true;
  }
  const lib = {
    all: () => read().items,
    names: () => Object.keys(read().items),
    get: n => { const r = read().items[n]; return r ? cloneRoute(r) : null; },
    active: () => { const o = read(); return o.active && o.items[o.active] ? o.active : null; },
    unique(base = 'New Path') { const it = read().items; if (!(base in it)) return base; for (let i = 2; ; i++) if (!((base + ' ' + i) in it)) return base + ' ' + i; },
    save(name, route) { const o = read(); o.items[name] = cloneRoute({ ...route, name }); if (!o.active) o.active = name; return write(o); },
    setActive(name) { const o = read(); if (!(name in o.items) || o.active === name) return false; o.active = name; return write(o); },
    create(name, route) { const o = read(), n = lib.unique(name || 'New Path'); o.items[n] = cloneRoute({ ...(route || newRoute({ name: n })), name: n }); o.active = n; write(o); return n; },
    duplicate(name) { const o = read(); if (!o.items[name]) return null; const n = lib.unique(name + ' copy'); o.items[n] = cloneRoute({ ...o.items[name], name: n }); o.active = n; write(o); return n; },
    rename(from, to) { const o = read(); to = String(to || '').trim(); if (!to || to === from || !o.items[from] || o.items[to]) return false; const items = {}; for (const [k, v] of Object.entries(o.items)) items[k === from ? to : k] = k === from ? { ...v, name: to } : v; o.items = items; if (o.active === from) o.active = to; return write(o); },
    remove(name) { const o = read(); if (!o.items[name]) return false; delete o.items[name]; if (o.active === name) o.active = Object.keys(o.items)[0] || null; return write(o); },
    /** Moves a path one place up (dir -1) or down (dir +1) in the list order. False at the ends or for an unknown name. */
    move(name, dir) { const o = read(), ks = Object.keys(o.items), i = ks.indexOf(name), j = i + (dir < 0 ? -1 : 1); if (i < 0 || j < 0 || j >= ks.length) return false; [ks[i], ks[j]] = [ks[j], ks[i]]; const items = {}; for (const k of ks) items[k] = o.items[k]; o.items = items; return write(o); },
    subscribe(fn) { subs.add(fn); return () => subs.delete(fn); },
  };
  return lib;
}
export const sharedPathLibrary = createPathLibrary();
/** Drawing hints published by the auto planner for the 2D map ({autoVisible, items:[{id, route, selected, markers:[{t,label}]}], start}). */
export const PLAN_VIEW = { viewQuarter: 0, autoVisible: false, items: [], start: null, alliance: "blue", fullField: false, openPath: null, bounds: viewBounds("blue", false) };

// ---------- PathPlanner-style waypoint editing ----------
/** Appends a connected waypoint ending at (x,y) as a cubic Bezier whose two handles sit at 1/3 and 2/3 (the path looks straight until a handle is dragged). Returns the new segment index. */
export function addWaypoint(route, x, y) {
  const e = routeEnd(route), c1 = { x: e.x + (x - e.x) / 3, y: e.y + (y - e.y) / 3 }, c2 = { x: e.x + (x - e.x) * 2 / 3, y: e.y + (y - e.y) * 2 / 3 };
  route.segments.push(newSegment('curve', [e, c1, c2, { x, y }]));
  return route.segments.length - 1;
}
/** Nearest point on the drawn route to (x,y): {seg, t, d, x, y} or null. Hidden segments are skipped. */
export function nearestOnRoute(route, x, y) {
  let best = null;
  route.segments.forEach((s, i) => {
    if (s.hidden) return;
    const t0 = nearestT(route, i, x, y), q = bezierPoint(s.points, t0), d = Math.hypot(q.x - x, q.y - y);
    if (!best || d < best.d) best = { seg: i, t: t0, d, x: q.x, y: q.y };
  });
  return best;
}
/** Inserts a waypoint on the route at the point nearest (x,y) when within maxDist; returns the index of the new anchor's segment (the first half) or -1. */
export function insertWaypointAt(route, x, y, maxDist = 4) {
  const n = nearestOnRoute(route, x, y);
  if (!n || n.d > maxDist || n.t <= 0.02 || n.t >= 0.98) return -1;
  return splitSegment(route, n.seg, n.t) ? n.seg : -1;
}
// ---------- alliance-limited view ----------
/** Visible field rectangle for an alliance. Red hive/alliance side is x < 72 and blue is x > 72 (docs/FIELD_CONSTANTS.md: "Red at low x", hives at x 59.25 / 84.75, midline 72).
 *  full = true shows the whole 144 x 144 field. Returns {x0,x1,y0,y1,full,alliance}. */
export function viewBounds(alliance, full = false) {
  if (full || (alliance !== 'red' && alliance !== 'blue')) return { x0: 0, x1: FIELD, y0: 0, y1: FIELD, full: true, alliance };
  return alliance === 'red' ? { x0: 0, x1: FIELD / 2, y0: 0, y1: FIELD, full: false, alliance } : { x0: FIELD / 2, x1: FIELD, y0: 0, y1: FIELD, full: false, alliance };
}
export const clampToView = (b, x, y) => ({ x: Math.max(b.x0, Math.min(b.x1, x)), y: Math.max(b.y0, Math.min(b.y1, y)) });
export const inView = (b, x, y) => x >= b.x0 && x <= b.x1 && y >= b.y0 && y <= b.y1;
/** Default start pose per alliance. ASSUMPTION (unverified against manual Fig 11-1, see docs/overnight/research_manual.md):
 *  red {x:24,y:24,h:PI/2} and blue {x:120,y:24,h:PI/2}. Blue is the mirror of red through x = FIELD/2 (h -> PI - h, PI/2 -> PI/2). */
export function defaultStart(alliance) {
  return alliance === 'red' ? { x: 24, y: 24, h: Math.PI / 2 } : { x: FIELD - 24, y: 24, h: Math.PI / 2 };
}
/** True when route.start lies inside viewBounds(route.alliance) (bounds inclusive). */
export function startInView(route) {
  return inView(viewBounds(route.alliance), route.start.x, route.start.y);
}
/** Sets route.alliance and, if route.start is outside that alliance's view, mirrors the start (x -> FIELD - x, h -> wrapAngle(PI - h)).
 *  Segments are left alone (use mirrorRoute for a full flip). Idempotent. Mutates and returns the route. */
export function retargetStart(route, alliance) {
  route.alliance = alliance;
  if (!startInView(route)) {
    route.start.x = FIELD - route.start.x;
    if (route.start.h !== undefined) route.start.h = wrapAngle(Math.PI - route.start.h);
  }
  return route;
}
/** Moves a point; moving an anchor also carries the Bezier handles attached to it (like PathPlanner), control points move alone. */
export function moveAnchor(route, seg, idx, x, y) {
  const s = route.segments[seg]; if (!s || !s.points[idx]) return false;
  const last = s.points.length - 1, old = s.points[idx], dx = x - old.x, dy = y - old.y, shift = p => { if (p) { p.x += dx; p.y += dy; } };
  if (idx === 0) { if (last > 1) shift(s.points[1]); const pr = route.segments[seg - 1]; if (pr && pr.points.length > 2) shift(pr.points[pr.points.length - 2]); }
  else if (idx === last) { if (last > 1) shift(s.points[last - 1]); const nx = route.segments[seg + 1]; if (nx && nx.points.length > 2) shift(nx.points[1]); }
  return movePoint(route, seg, idx, x, y);
}
// ---------- flip and view transforms (pure) ----------
/** Mirror (x -> FIELD-x) or rotate 180 about field centre (x -> FIELD-x, y -> FIELD-y). Unknown mode returns a copy. */
export function flipPoint(x, y, mode) {
  if (mode === 'mirror') return { x: FIELD - x, y };
  if (mode === 'rotate') return { x: FIELD - x, y: FIELD - y };
  return { x, y };
}
/** Heading under the same flip: mirror -> wrapAngle(PI - h), rotate -> wrapAngle(h + PI). */
export function flipHeading(h, mode) {
  if (mode === 'mirror') return wrapAngle(Math.PI - h);
  if (mode === 'rotate') return wrapAngle(h + Math.PI);
  return h;
}
/** How to draw the field from an alliance wall. ASSUMPTION (research 3, no verified source): blue is shown rotated 180
 *  so its own wall is on the left; red is shown unrotated. flipX is false for both (no mirror view is used). */
export function allianceViewTransform(alliance) {
  return { rotate180: alliance === 'blue', flipX: false };
}
// Screen mapping: field inches -> pixels. The view frame (u right, v up) is the field frame, optionally turned: rotate180 (about field centre),
// or rot90 = +1 (quarter turn counter-clockwise: screen right = +y, screen up = -x) / -1 (clockwise: right = -y, up = +x). rot90 wins over rotate180.
// Then the crop bounds {x0,x1,y0,y1} (field space) are fitted and centred into a w x h box with uniform scale. Screen y grows downward.
function viewUV(view, x, y) {
  if (view.rot90 === 1) return { u: y, v: FIELD - x };
  if (view.rot90 === -1) return { u: FIELD - y, v: x };
  return view.rotate180 ? { u: FIELD - x, v: FIELD - y } : { u: x, v: y };
}
function viewXY(view, u, v) {
  if (view.rot90 === 1) return { x: FIELD - v, y: u };
  if (view.rot90 === -1) return { x: v, y: FIELD - u };
  return view.rotate180 ? { x: FIELD - u, y: FIELD - v } : { x: u, y: v };
}
function viewFit(view, w, h) {
  const b = view.bounds || { x0: 0, x1: FIELD, y0: 0, y1: FIELD };
  const p = viewUV(view, b.x0, b.y0), q = viewUV(view, b.x1, b.y1);
  const rb = { x0: Math.min(p.u, q.u), x1: Math.max(p.u, q.u), y0: Math.min(p.v, q.v), y1: Math.max(p.v, q.v) };
  const bw = rb.x1 - rb.x0, bh = rb.y1 - rb.y0;
  if (!(bw > 0 && bh > 0 && w > 0 && h > 0)) throw new RangeError('toScreen: view and canvas must have positive size');
  const k = Math.min(w / bw, h / bh), ox = (w - bw * k) / 2, oy = (h - bh * k) / 2;
  return { rb, k, ox, oy };
}
/** Uniform pixels-per-inch of the fitted view in a w x h box (same maths as toScreen). */
export function viewScale(view, w, h) { return viewFit(view, w, h).k; }
export function toScreen(view, x, y, w, h) {
  const f = viewFit(view, w, h), { u, v } = viewUV(view, x, y);
  return { x: f.ox + (u - f.rb.x0) * f.k, y: f.oy + (f.rb.y1 - v) * f.k };
}
/** Exact inverse of toScreen for the same view and canvas size. */
export function fromScreen(view, px, py, w, h) {
  const f = viewFit(view, w, h);
  return viewXY(view, f.rb.x0 + (px - f.ox) / f.k, f.rb.y1 - (py - f.oy) / f.k);
}
