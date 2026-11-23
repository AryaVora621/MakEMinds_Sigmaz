// WPILOG reader and writer.
//
// WARNING: the byte layout below is PRIOR knowledge (the lead's recollection of the WPILib data log format). The
// researcher could NOT fetch datalog.adoc this session, so this layout has NOT been verified against the spec.
// Check the reader against a real .wpilog file from a robot before trusting any number it produces.
// The writer exists only so tests can round-trip.
//
// Layout (PRIOR):
//   header: 'WPILOG' (6 ASCII bytes), uint16 LE version (0x0100), uint32 LE extra-header length, UTF-8 extra header.
//   record: 1 header byte: bits 0-1 = entryId byte length - 1, bits 2-3 = payload-size byte length - 1,
//           bits 4-6 = timestamp byte length - 1. Then entryId (LE), payload size (LE), timestamp (LE, integer us), payload.
//   entry 0 is the control entry. Payload byte 0: 0=Start (u32 entryId, u32 len + name, u32 len + type, u32 len + metadata),
//           1=Finish (u32 entryId), 2=SetMetadata (u32 entryId, u32 len + metadata).
//   types: boolean 1 byte; int64 8 LE; float 4 LE; double 8 LE; string = whole payload UTF-8; boolean[]/int64[]/float[]/double[]
//          concatenated; string[] = u32 count then (u32 len + bytes)*; raw = bytes.
//   struct:Pose2d = 24 bytes: x, y, rotation as float64 LE, metres and radians (SOURCED
//          github.wpilib.org/allwpilib/docs/release/cpp/_pose2d_struct_8h_source.html). Decoded to inches:
//          {x (in), y (in), h (rad)}. Unknown types come back as raw Uint8Array.
// Times: record timestamps are integer microseconds in the file; the reader returns seconds.
import { WPILOG, M_PER_IN, IN_PER_M } from './params.js';

const TD = new TextDecoder('utf-8');
const TE = new TextEncoder();

function toBytes(buf) {
  if (buf instanceof Uint8Array) return buf;
  if (buf instanceof ArrayBuffer) return new Uint8Array(buf);
  if (ArrayBuffer.isView(buf)) return new Uint8Array(buf.buffer, buf.byteOffset, buf.byteLength);
  throw new TypeError('wpilog: expected ArrayBuffer or Uint8Array');
}

function readUintLE(b, off, len) {
  let v = 0, mul = 1;
  for (let i = 0; i < len; i++) { v += b[off + i] * mul; mul *= 256; }
  return v;
}

function writeUintLE(arr, v, len) {
  let x = v;
  for (let i = 0; i < len; i++) { arr.push(x % 256); x = Math.floor(x / 256); }
}

function minLen(v) {
  let len = 1, lim = 256;
  while (v >= lim && len < 8) { len++; lim *= 256; }
  return len;
}

function dv(bytes) { return new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength); }

/** Decode one payload to a JS value for the given WPILOG type string. */
export function decodeValue(type, bytes) {
  const b = toBytes(bytes);
  const d = dv(b);
  switch (type) {
    case 'boolean': return b[0] !== 0;
    case 'int64': return Number(d.getBigInt64(0, true));
    case 'float': return d.getFloat32(0, true);
    case 'double': return d.getFloat64(0, true);
    case 'string': case 'json': return TD.decode(b);
    case 'boolean[]': return Array.from(b, v => v !== 0);
    case 'int64[]': { const o = []; for (let i = 0; i + 8 <= b.length; i += 8) o.push(Number(d.getBigInt64(i, true))); return o; }
    case 'float[]': { const o = []; for (let i = 0; i + 4 <= b.length; i += 4) o.push(d.getFloat32(i, true)); return o; }
    case 'double[]': { const o = []; for (let i = 0; i + 8 <= b.length; i += 8) o.push(d.getFloat64(i, true)); return o; }
    case 'string[]': {
      const n = d.getUint32(0, true), o = [];
      let p = 4;
      for (let i = 0; i < n; i++) {
        const l = d.getUint32(p, true); p += 4;
        o.push(TD.decode(b.subarray(p, p + l))); p += l;
      }
      return o;
    }
    case 'struct:Pose2d':
      if (b.length < 24) throw new Error('wpilog: struct:Pose2d needs 24 bytes');
      return { x: d.getFloat64(0, true) * IN_PER_M, y: d.getFloat64(8, true) * IN_PER_M, h: d.getFloat64(16, true) };
    default: return Uint8Array.from(b);   // raw and unknown types
  }
}

/** Encode a JS value to a payload. struct:Pose2d accepts {x,y,h|rot|rotation} (inches, rad) or [x,y,h]. */
export function encodeValue(type, value) {
  switch (type) {
    case 'boolean': return Uint8Array.of(value ? 1 : 0);
    case 'int64': { const o = new Uint8Array(8); dv(o).setBigInt64(0, BigInt(Math.round(value)), true); return o; }
    case 'float': { const o = new Uint8Array(4); dv(o).setFloat32(0, value, true); return o; }
    case 'double': { const o = new Uint8Array(8); dv(o).setFloat64(0, value, true); return o; }
    case 'string': case 'json': return TE.encode(String(value));
    case 'boolean[]': return Uint8Array.from(value, v => (v ? 1 : 0));
    case 'int64[]': { const o = new Uint8Array(8 * value.length); const d = dv(o); value.forEach((v, i) => d.setBigInt64(8 * i, BigInt(Math.round(v)), true)); return o; }
    case 'float[]': { const o = new Uint8Array(4 * value.length); const d = dv(o); value.forEach((v, i) => d.setFloat32(4 * i, v, true)); return o; }
    case 'double[]': { const o = new Uint8Array(8 * value.length); const d = dv(o); value.forEach((v, i) => d.setFloat64(8 * i, v, true)); return o; }
    case 'string[]': {
      const parts = value.map(s => TE.encode(String(s)));
      const o = new Uint8Array(4 + parts.reduce((a, p) => a + 4 + p.length, 0));
      const d = dv(o);
      d.setUint32(0, parts.length, true);
      let p = 4;
      for (const s of parts) { d.setUint32(p, s.length, true); p += 4; o.set(s, p); p += s.length; }
      return o;
    }
    case 'struct:Pose2d': {
      const x = Array.isArray(value) ? value[0] : value.x;
      const y = Array.isArray(value) ? value[1] : value.y;
      const h = Array.isArray(value) ? value[2] : (value.h ?? value.rot ?? value.rotation ?? 0);
      const o = new Uint8Array(24); const d = dv(o);
      d.setFloat64(0, x * M_PER_IN, true); d.setFloat64(8, y * M_PER_IN, true); d.setFloat64(16, h, true);
      return o;
    }
    default:
      if (value instanceof Uint8Array) return value;
      if (ArrayBuffer.isView(value)) return new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
      throw new TypeError('wpilog: cannot encode unknown type ' + type);
  }
}

function readControlString(d, b, p) {
  const l = d.getUint32(p, true); p += 4;
  return [TD.decode(b.subarray(p, p + l)), p + l];
}

/**
 * Parse a .wpilog buffer. Throws Error on bad magic. A truncated final record is ignored.
 * @returns {{version:number, extraHeader:string, entries:Map<number,{id:number,name:string,type:string,metadata:string}>,
 *   records:{entry:number,name:string,type:string,t:number,value:any}[]}} t in seconds.
 */
export function readWpilog(buf) {
  const b = toBytes(buf);
  const magic = WPILOG.magic;
  if (b.length < 12 || TD.decode(b.subarray(0, magic.length)) !== magic) throw new Error('wpilog: bad magic, not a WPILOG file');
  const d = dv(b);
  const version = d.getUint16(6, true);
  const extraLen = d.getUint32(8, true);
  if (12 + extraLen > b.length) throw new Error('wpilog: truncated header');
  const extraHeader = TD.decode(b.subarray(12, 12 + extraLen));
  const entries = new Map();
  const records = [];
  let p = 12 + extraLen;
  while (p < b.length) {
    const hb = b[p];
    const idLen = (hb & 3) + 1, szLen = ((hb >> 2) & 3) + 1, tsLen = ((hb >> 4) & 7) + 1;
    const hdr = 1 + idLen + szLen + tsLen;
    if (p + hdr > b.length) break;
    const id = readUintLE(b, p + 1, idLen);
    const size = readUintLE(b, p + 1 + idLen, szLen);
    const tUs = readUintLE(b, p + 1 + idLen + szLen, tsLen);
    const start = p + hdr;
    if (start + size > b.length) break;
    const payload = b.subarray(start, start + size);
    p = start + size;
    if (id === 0) {
      if (size < 1) continue;
      const pd = dv(payload);
      const kind = payload[0];
      if (kind === 0 && size >= 5) {
        const eid = pd.getUint32(1, true);
        let q = 5, name, type, metadata;
        [name, q] = readControlString(pd, payload, q);
        [type, q] = readControlString(pd, payload, q);
        [metadata, q] = readControlString(pd, payload, q);
        entries.set(eid, { id: eid, name, type, metadata });
      } else if (kind === 2 && size >= 5) {
        const eid = pd.getUint32(1, true);
        const e = entries.get(eid);
        if (e) e.metadata = readControlString(pd, payload, 5)[0];
      }
      continue;   // Finish (1) leaves the entry known: earlier records stay valid
    }
    const e = entries.get(id);
    const type = e ? e.type : 'raw';
    let value;
    try { value = decodeValue(type, payload); } catch (_) { value = Uint8Array.from(payload); }
    records.push({ entry: id, name: e ? e.name : '', type, t: tUs / 1e6, value });
  }
  return { version, extraHeader, entries, records };
}

function pushRecord(out, id, tUs, payload) {
  const idLen = minLen(id), szLen = Math.min(minLen(payload.length), 4), tsLen = minLen(tUs);
  out.push((idLen - 1) | ((szLen - 1) << 2) | ((tsLen - 1) << 4));
  writeUintLE(out, id, idLen);
  writeUintLE(out, payload.length, szLen);
  writeUintLE(out, tUs, tsLen);
  for (let i = 0; i < payload.length; i++) out.push(payload[i]);
}

/**
 * Write a .wpilog. Entries get ids 1.. in order; each gets a Start control record at t=0. Records name an entry by name.
 * @returns {Uint8Array}
 */
export function writeWpilog(entries, records, opts = {}) {
  const extra = TE.encode(opts.extraHeader ?? '');
  const out = [];
  for (const c of WPILOG.magic) out.push(c.charCodeAt(0));
  writeUintLE(out, WPILOG.version, 2);
  writeUintLE(out, extra.length, 4);
  for (const c of extra) out.push(c);
  const ids = new Map(), types = new Map();
  entries.forEach((e, i) => {
    const id = i + 1;
    ids.set(e.name, id); types.set(e.name, e.type);
    const parts = [TE.encode(e.name), TE.encode(e.type), TE.encode(e.metadata ?? '')];
    const pl = [0];
    writeUintLE(pl, id, 4);
    for (const s of parts) { writeUintLE(pl, s.length, 4); for (const c of s) pl.push(c); }
    pushRecord(out, 0, 0, pl);
  });
  for (const r of records) {
    if (!ids.has(r.name)) throw new Error('wpilog: record for unknown entry ' + r.name);
    pushRecord(out, ids.get(r.name), Math.round(r.tUs), encodeValue(types.get(r.name), r.value));
  }
  return Uint8Array.from(out);
}
