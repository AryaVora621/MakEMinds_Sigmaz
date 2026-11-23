// Importers that turn real robot logs into the shared Log shape:
//   Log = {source:string, meta:object, series:{[name]:{t:number[] (s), v:any[], type:string}}}
// Supported inputs: CSV (comma, semicolon or tab), FTC Dashboard CSV (pose converted into sim coordinates), WPILOG, and a JSON Log.
// Units follow the sim: inches, seconds, radians. Constants come from params.js (DASH).
// The WPILOG path inherits the unverified layout warning in wpilog.js.
import { DASH } from './params.js';
import { readWpilog } from './wpilog.js';

const DEFAULT_ALIASES = {
  t: 't', time: 't', timestamp: 't',
  x: 'x', pose_x: 'x', posex: 'x',
  y: 'y', pose_y: 'y',
  h: 'heading', heading: 'heading', theta: 'heading', pose_heading: 'heading',
  power: 'power', u: 'power', voltage: 'power',
  v: 'velocity', vel: 'velocity', velocity: 'velocity',
  fwd: 'fwd', strafe: 'strafe', turn: 'turn',
};

function splitLine(line, delim) {
  const out = [];
  let cur = '', q = false;
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (q) {
      if (c === '"') { if (line[i + 1] === '"') { cur += '"'; i++; } else q = false; }
      else cur += c;
    } else if (c === '"') q = true;
    else if (c === delim) { out.push(cur); cur = ''; }
    else cur += c;
  }
  out.push(cur);
  return out;
}

function detectDelimiter(headerLine) {
  let best = ',', bestN = -1;
  for (const d of [',', ';', '\t']) {
    const n = headerLine.split(d).length - 1;
    if (n > bestN) { best = d; bestN = n; }
  }
  return best;
}

/**
 * Parse delimited text. First non-empty line is the header; the rest are numeric rows (non-numeric cell -> NaN).
 * @returns {{columns:string[], rows:number[][]}}
 */
export function parseCsv(text, opts = {}) {
  const lines = String(text).replace(/^﻿/, '').split(/\r\n|\n|\r/).filter(l => l.trim() !== '');
  if (lines.length === 0) throw new Error('parseCsv: empty input');
  const delim = opts.delimiter ?? detectDelimiter(lines[0]);
  const columns = splitLine(lines[0], delim).map(c => c.trim());
  const rows = [];
  for (let i = 1; i < lines.length; i++) {
    const cells = splitLine(lines[i], delim);
    const row = new Array(columns.length);
    for (let j = 0; j < columns.length; j++) {
      const s = (cells[j] ?? '').trim();
      row[j] = s === '' ? NaN : Number(s);
    }
    rows.push(row);
  }
  return { columns, rows };
}

function median(a) {
  const s = a.slice().sort((x, y) => x - y), n = s.length;
  return n === 0 ? NaN : n % 2 ? s[(n - 1) / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
}

function pickTimeScale(tRaw, unit) {
  if (unit === 's') return { unit: 's', scale: 1 };
  if (unit === 'ms') return { unit: 'ms', scale: 1e-3 };
  if (unit === 'us') return { unit: 'us', scale: 1e-6 };
  if (unit !== undefined && unit !== 'auto') throw new Error('importCsvLog: bad timeUnit ' + unit);
  const dts = [];
  for (let i = 1; i < tRaw.length; i++) dts.push(tRaw[i] - tRaw[i - 1]);
  const md = median(dts);
  if (md > 1000) return { unit: 'us', scale: 1e-6 };
  if (md > 1) return { unit: 'ms', scale: 1e-3 };
  return { unit: 's', scale: 1 };
}

/**
 * CSV to Log. Header aliases are case-insensitive (see DEFAULT_ALIASES). Requires a time column. Time is converted to
 * seconds ('auto': median dt > 1000 means us, > 1 means ms, else s). Each other column becomes a series of type 'double';
 * samples whose value is not finite are dropped from that series.
 */
export function importCsvLog(text, opts = {}) {
  const { columns, rows } = parseCsv(text, opts);
  const aliases = { ...DEFAULT_ALIASES };
  for (const [k, v] of Object.entries(opts.aliases ?? {})) aliases[k.toLowerCase()] = v;
  const names = columns.map(c => aliases[c.toLowerCase()] ?? c);
  const ti = names.indexOf('t');
  if (ti < 0) throw new Error('importCsvLog: no time column (t, time or timestamp)');
  const good = rows.filter(r => Number.isFinite(r[ti]));
  const tScale = pickTimeScale(good.map(r => r[ti]), opts.timeUnit);
  const series = {};
  for (let j = 0; j < names.length; j++) {
    if (j === ti) continue;
    const t = [], v = [];
    for (const r of good) if (Number.isFinite(r[j])) { t.push(r[ti] * tScale.scale); v.push(r[j]); }
    series[names[j]] = { t, v, type: 'double' };
  }
  return { source: opts.source ?? 'csv', meta: { format: 'csv', timeUnit: tScale.unit, columns: names, rows: good.length }, series };
}

/**
 * FTC Dashboard field frame to sim frame. Dashboard origin is the mat centre in inches (SOURCED
 * acmerobotics.github.io/ftc-dashboard/fieldview); the rotation between Dashboard and Pedro axes is ASSUMED 0 in params.js
 * (DASH.rotRad) and the user must check it. [x,y] = R(rotRad)*[x,y] + offsetIn; h = h + rotRad.
 * @param {number[]} pose [x,y] or [x,y,h]
 * @returns {number[]} same length as pose
 */
export function dashToSim(pose, frame = DASH) {
  const c = Math.cos(frame.rotRad), s = Math.sin(frame.rotRad);
  const x = c * pose[0] - s * pose[1] + frame.offsetIn[0];
  const y = s * pose[0] + c * pose[1] + frame.offsetIn[1];
  return pose.length > 2 ? [x, y, pose[2] + frame.rotRad] : [x, y];
}

/** Dashboard CSV: importCsvLog then convert x, y, heading (when present, same sample count) from Dashboard to sim frame. */
export function importDashboardCsv(text, opts = {}) {
  const frame = opts.frame ?? DASH;
  const log = importCsvLog(text, { ...opts, source: opts.source ?? 'dashboard-csv' });
  const sx = log.series.x, sy = log.series.y, sh = log.series.heading;
  if (sx && sy && sx.v.length === sy.v.length) {
    for (let i = 0; i < sx.v.length; i++) {
      const p = dashToSim([sx.v[i], sy.v[i]], frame);
      sx.v[i] = p[0]; sy.v[i] = p[1];
    }
  }
  if (sh) for (let i = 0; i < sh.v.length; i++) sh.v[i] += frame.rotRad;
  log.meta.frame = { offsetIn: [...frame.offsetIn], rotRad: frame.rotRad };
  return log;
}

/** WPILOG to Log: one series per entry name. struct:Pose2d entries also get derived scalar series name/x, name/y, name/heading. */
export function importWpilog(buf) {
  const w = readWpilog(buf);
  const series = {};
  for (const r of w.records) {
    const name = r.name || ('entry' + r.entry);
    const s = series[name] ?? (series[name] = { t: [], v: [], type: r.type });
    s.t.push(r.t); s.v.push(r.value);
    if (r.type === 'struct:Pose2d') {
      for (const [suffix, val] of [['x', r.value.x], ['y', r.value.y], ['heading', r.value.h]]) {
        const key = name + '/' + suffix;
        const d = series[key] ?? (series[key] = { t: [], v: [], type: 'double' });
        d.t.push(r.t); d.v.push(val);
      }
    }
  }
  return { source: 'wpilog', meta: { format: 'wpilog', version: w.version, extraHeader: w.extraHeader, entries: w.entries.size }, series };
}

function asBytes(input) {
  if (input instanceof Uint8Array) return input;
  if (input instanceof ArrayBuffer) return new Uint8Array(input);
  if (ArrayBuffer.isView(input)) return new Uint8Array(input.buffer, input.byteOffset, input.byteLength);
  return null;
}

/** Sniff the format: WPILOG magic, JSON (leading { or [), CSV (delimiter in the first line), else 'unknown'. filename extension is a hint. */
export function detectFormat(input, filename = '') {
  const ext = (String(filename).match(/\.([A-Za-z0-9]+)$/) ?? [])[1]?.toLowerCase();
  const bytes = asBytes(input);
  if (bytes) {
    const head = new TextDecoder().decode(bytes.subarray(0, 6));
    if (head === 'WPILOG') return 'wpilog';
    if (ext === 'wpilog') return 'unknown';
    input = new TextDecoder().decode(bytes);
  }
  if (typeof input !== 'string') return 'unknown';
  if (ext === 'json') return 'json';
  if (ext === 'csv' || ext === 'tsv' || ext === 'txt') return 'csv';
  const t = input.replace(/^﻿/, '').trimStart();
  if (t.startsWith('{') || t.startsWith('[')) return 'json';
  const first = t.split(/\r\n|\n|\r/)[0] ?? '';
  if (/[,;\t]/.test(first) && /\n/.test(t)) return 'csv';
  return 'unknown';
}

/** Import any supported input to a Log. A JSON input must already be a Log ({series:{...}}). Throws Error on unknown formats. */
export function importAny(input, filename = '') {
  const fmt = detectFormat(input, filename);
  const bytes = asBytes(input);
  if (fmt === 'wpilog') return importWpilog(input);
  const text = bytes ? new TextDecoder().decode(bytes) : input;
  if (fmt === 'csv') return importCsvLog(text, { source: filename || 'csv' });
  if (fmt === 'json') {
    const obj = JSON.parse(text);
    if (!obj || typeof obj !== 'object' || typeof obj.series !== 'object') throw new Error('importAny: JSON is not a Log (missing series)');
    return { source: obj.source ?? (filename || 'json'), meta: obj.meta ?? {}, series: obj.series };
  }
  throw new Error('importAny: unrecognised log format');
}

function interpAt(ts, vs, t) {
  const n = ts.length;
  if (t <= ts[0]) return vs[0];
  if (t >= ts[n - 1]) return vs[n - 1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (ts[m] <= t) lo = m; else hi = m; }
  const f = (t - ts[lo]) / (ts[hi] - ts[lo]);
  return vs[lo] + f * (vs[hi] - vs[lo]);
}

function need(log, name) {
  const s = log.series?.[name];
  if (!s || !s.t.length) throw new Error('log has no series "' + name + '"');
  return s;
}

/**
 * Pose track from a Log: series named x, y and (optionally) heading. If x and y are absent and the log holds one
 * struct:Pose2d series, that is used. y and h are linearly interpolated onto x's times when their times differ.
 * @returns {{t:number[],x:number[],y:number[],h?:number[]}} inches, seconds, radians (h omitted if absent)
 */
export function toPoseTrack(log, names = {}) {
  let nx = names.x ?? 'x', ny = names.y ?? 'y', nh = names.h ?? 'heading';
  if (!log.series[nx] && !names.x) {
    const pose = Object.keys(log.series).find(k => log.series[k].type === 'struct:Pose2d');
    if (pose) { nx = pose + '/x'; ny = pose + '/y'; nh = pose + '/heading'; }
  }
  const sx = need(log, nx), sy = need(log, ny);
  const sh = log.series[nh];
  const t = sx.t.slice(), x = sx.v.slice();
  const same = (a, b) => a.t.length === b.t.length && a.t.every((v, i) => v === b.t[i]);
  const y = same(sx, sy) ? sy.v.slice() : t.map(tt => interpAt(sy.t, sy.v, tt));
  const out = { t, x, y };
  if (sh && sh.t.length) out.h = same(sx, sh) ? sh.v.slice() : t.map(tt => interpAt(sh.t, sh.v, tt));
  return out;
}

/** Feedforward samples {t,u,v} from series power (u) and velocity (v), v's times as the base. */
export function toFeedforwardSamples(log, names = {}) {
  const sv = need(log, names.v ?? 'velocity'), su = need(log, names.u ?? 'power');
  const same = su.t.length === sv.t.length && su.t.every((v, i) => v === sv.t[i]);
  const u = same ? su.v.slice() : sv.t.map(tt => interpAt(su.t, su.v, tt));
  return { t: sv.t.slice(), u, v: sv.v.slice() };
}
