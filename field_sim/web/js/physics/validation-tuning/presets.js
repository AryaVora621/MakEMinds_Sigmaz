// Tuning presets: a named snapshot of the sim's ball and drive parameters, with validation, JSON round-trip and
// injectable-storage CRUD. Ranges and keys come from params.js PARAM_SCHEMA; the storage key and version from PRESET.
//   Preset = {version, name, created (ISO string), ball:{field:value}, drive:{field:value}, source:{[key]:string}}
// Every storage access is wrapped in try/catch and fails soft (false / null / []), because localStorage can be missing or
// throw in private windows and during thumbnail capture.
import { PRESET, PARAM_SCHEMA, schemaEntry } from './params.js';
import { BALL } from '../../balls.js';
import { MecanumDrive } from '../../driver.js';
import { feedforwardToDrive } from './fit.js';

export const PRESET_VERSION = PRESET.version;

const ballEntries = () => PARAM_SCHEMA.filter(e => e.group === 'ball');
const driveEntries = () => PARAM_SCHEMA.filter(e => e.group === 'drive');

function readFields(entries, obj) {
  const out = {};
  for (const e of entries) out[e.field] = obj[e.field];
  return out;
}

function deepFreeze(o) {
  for (const v of Object.values(o)) if (v && typeof v === 'object') deepFreeze(v);
  return Object.freeze(o);
}

function buildDefault() {
  const drive = new MecanumDrive();
  const ball = readFields(ballEntries(), BALL);
  const dr = readFields(driveEntries(), drive);
  const source = {};
  for (const e of PARAM_SCHEMA) source[e.key] = 'default';
  return deepFreeze({ version: PRESET_VERSION, name: 'default', created: new Date(0).toISOString(), ball, drive: dr, source });
}

/** Sim defaults captured from BALL and a fresh MecanumDrive when this module loads. Frozen. */
export const DEFAULT_PRESET = buildDefault();

/** Snapshot the current BALL values and the given drive instance (or the default drive values when none is given). */
export function capturePreset(name, opts = {}) {
  const ball = readFields(ballEntries(), BALL);
  const drive = opts.drive ? readFields(driveEntries(), opts.drive) : { ...DEFAULT_PRESET.drive };
  const source = {};
  for (const e of PARAM_SCHEMA) source[e.key] = 'captured';
  return { version: PRESET_VERSION, name: String(name), created: new Date().toISOString(), ball, drive, source };
}

/**
 * Check a preset. Rejects: not an object, wrong version, missing name, unknown key, non-finite value, value outside the schema range.
 * Missing keys are allowed (partial presets apply only what they hold).
 */
export function validatePreset(obj) {
  const errors = [];
  if (!obj || typeof obj !== 'object' || Array.isArray(obj)) return { ok: false, errors: ['preset is not an object'] };
  if (obj.version !== PRESET_VERSION) errors.push('unsupported version ' + obj.version + ' (expected ' + PRESET_VERSION + ')');
  if (typeof obj.name !== 'string' || obj.name === '') errors.push('name must be a non-empty string');
  for (const group of ['ball', 'drive']) {
    const g = obj[group];
    if (!g || typeof g !== 'object' || Array.isArray(g)) { errors.push(group + ' must be an object'); continue; }
    for (const [field, value] of Object.entries(g)) {
      const key = group + '.' + field;
      const e = schemaEntry(key);
      if (!e) { errors.push('unknown key ' + key); continue; }
      if (typeof value !== 'number' || !Number.isFinite(value)) { errors.push(key + ' is not a finite number'); continue; }
      if (value < e.min || value > e.max) errors.push(key + ' = ' + value + ' outside [' + e.min + ', ' + e.max + ']');
    }
  }
  if (obj.source !== undefined && (typeof obj.source !== 'object' || obj.source === null || Array.isArray(obj.source))) errors.push('source must be an object');
  return { ok: errors.length === 0, errors };
}

/**
 * Write a preset into BALL and, when opts.drive is given, into that drive instance (Object.assign). Throws on an invalid preset.
 * @returns {() => void} restore, which puts back the values that were overwritten
 */
export function applyPreset(p, opts = {}) {
  const v = validatePreset(p);
  if (!v.ok) throw new Error('applyPreset: ' + v.errors.join('; '));
  const prevBall = {};
  for (const k of Object.keys(p.ball)) prevBall[k] = BALL[k];
  const drive = opts.drive ?? null;
  const prevDrive = {};
  if (drive) for (const k of Object.keys(p.drive)) prevDrive[k] = drive[k];
  Object.assign(BALL, p.ball);
  if (drive) Object.assign(drive, p.drive);
  let done = false;
  return () => {
    if (done) return;
    done = true;
    Object.assign(BALL, prevBall);
    if (drive) Object.assign(drive, prevDrive);
  };
}

export function serializePreset(p) { return JSON.stringify(p, null, 2); }

/** Parse and validate a preset JSON string. Throws Error with the validation messages. */
export function parsePreset(s) {
  const obj = JSON.parse(s);
  const v = validatePreset(obj);
  if (!v.ok) throw new Error('parsePreset: ' + v.errors.join('; '));
  return obj;
}

function getStore(storage) {
  try { return storage ?? globalThis.localStorage ?? null; } catch (_) { return null; }
}

function readMap(storage) {
  try {
    const st = getStore(storage);
    if (!st) return null;
    const raw = st.getItem(PRESET.storageKey);
    if (raw === null || raw === undefined || raw === '') return {};
    const m = JSON.parse(raw);
    return m && typeof m === 'object' && !Array.isArray(m) ? m : {};
  } catch (_) { return null; }
}

function writeMap(storage, map) {
  try {
    const st = getStore(storage);
    if (!st) return false;
    st.setItem(PRESET.storageKey, JSON.stringify(map));
    return true;
  } catch (_) { return false; }
}

/** Names of saved presets, sorted. [] on any failure. */
export function listPresets(storage) {
  const m = readMap(storage);
  return m ? Object.keys(m).sort() : [];
}

/** Save under p.name. Returns false for an invalid preset or any storage failure. */
export function savePreset(p, storage) {
  if (!validatePreset(p).ok) return false;
  const m = readMap(storage);
  if (!m) return false;
  m[p.name] = p;
  return writeMap(storage, m);
}

/** Load by name; null when missing, invalid or on storage failure. */
export function loadPreset(name, storage) {
  const m = readMap(storage);
  if (!m || !Object.prototype.hasOwnProperty.call(m, name)) return null;
  return validatePreset(m[name]).ok ? m[name] : null;
}

/** Delete by name; true only if it existed and the write succeeded. */
export function deletePreset(name, storage) {
  const m = readMap(storage);
  if (!m || !Object.prototype.hasOwnProperty.call(m, name)) return false;
  delete m[name];
  return writeMap(storage, m);
}

/**
 * Build a new preset from fitted results. fits: {ff:{kS,kV,kA}, decel:{decel}, restitution:{field,value}, drag:{k}}.
 * ff sets drive.maxSpeed and drive.maxAccel (via feedforwardToDrive); decel sets drive.maxDecel; restitution sets ball[field];
 * drag sets ball.dragK. Values are clamped into the schema range (and the source note says so) so the result always validates.
 */
export function fitToPreset(base, fits, name) {
  const p = JSON.parse(JSON.stringify(base));
  p.name = name ?? (base.name + '-fit');
  p.created = new Date().toISOString();
  p.source = p.source ?? {};
  const set = (key, value, note) => {
    const e = schemaEntry(key);
    if (!e) throw new Error('fitToPreset: unknown key ' + key);
    if (!Number.isFinite(value)) throw new RangeError('fitToPreset: non-finite value for ' + key);
    const c = Math.min(e.max, Math.max(e.min, value));
    p[e.group][e.field] = c;
    p.source[key] = c === value ? note : note + ' (clamped from ' + value + ')';
  };
  if (fits?.ff) {
    const d = feedforwardToDrive(fits.ff);
    set('drive.maxSpeed', d.maxSpeed, 'fit:feedforward');
    set('drive.maxAccel', d.maxAccel, 'fit:feedforward');
  }
  if (fits?.decel) set('drive.maxDecel', fits.decel.decel, 'fit:zero-power-decel');
  if (fits?.restitution) set('ball.' + fits.restitution.field, fits.restitution.value, 'fit:restitution');
  if (fits?.drag) set('ball.dragK', fits.drag.k, 'fit:drag');
  return p;
}
