// statehash.js - canonical FNV-1a hash of a state snapshot over Float64 little-endian bytes.
// Objects: keys sorted, each key hashed as string code units. Arrays in order. Booleans 0/1.
// Strings as UTF-16 code units. -0 -> 0, NaN -> canonical 0x7ff8000000000000.
import { TS } from './params.js';

const TAG_NULL = 0, TAG_NUM = 1, TAG_STR = 2, TAG_ARR = 3, TAG_OBJ = 4, TAG_BOOL = 5;

function canon(v, quant) {
  if (Number.isNaN(v)) return NaN;
  if (quant > 0 && Number.isFinite(v)) v = Math.round(v / quant);
  return v === 0 ? 0 : v; // -0 -> 0
}

function pushStr(s, out) {
  out.push(TAG_STR, s.length);
  for (let i = 0; i < s.length; i++) out.push(s.charCodeAt(i));
}

function walk(x, out, quant) {
  if (x === null || x === undefined) { out.push(TAG_NULL); return; }
  switch (typeof x) {
    case 'number': out.push(TAG_NUM, canon(x, quant)); return;
    case 'boolean': out.push(TAG_BOOL, x ? 1 : 0); return;
    case 'bigint': out.push(TAG_NUM, canon(Number(x), quant)); return;
    case 'string': pushStr(x, out); return;
    case 'object': {
      if (ArrayBuffer.isView(x) && !(x instanceof DataView)) {
        out.push(TAG_ARR, x.length);
        for (let i = 0; i < x.length; i++) walk(Number(x[i]), out, quant);
        return;
      }
      if (Array.isArray(x)) {
        out.push(TAG_ARR, x.length);
        for (let i = 0; i < x.length; i++) walk(x[i], out, quant);
        return;
      }
      const keys = Object.keys(x).sort();
      out.push(TAG_OBJ, keys.length);
      for (const k of keys) {
        pushStr(k, out);
        walk(x[k], out, quant);
      }
      return;
    }
    default: out.push(TAG_NULL); // functions, symbols
  }
}

// Canonical traversal as Float64Array (debugging/tests). quant>0 applies Math.round(v/quant) to numeric values.
export function canonicalNumbers(obj, quant = 0) {
  const out = [];
  walk(obj, out, quant);
  return Float64Array.from(out);
}

function fnv1aF64(f64) {
  const buf = new ArrayBuffer(8);
  const dv = new DataView(buf);
  const u8 = new Uint8Array(buf);
  let h = 0x811c9dc5;
  for (let i = 0; i < f64.length; i++) {
    const v = f64[i];
    if (Number.isNaN(v)) { dv.setUint32(0, 0, true); dv.setUint32(4, 0x7ff80000, true); }
    else dv.setFloat64(0, v, true);
    for (let j = 0; j < 8; j++) {
      h ^= u8[j];
      h = Math.imul(h, 0x01000193);
    }
  }
  return h >>> 0;
}

const hex8 = (n) => n.toString(16).padStart(8, '0');

export function hashState(obj, { quant = 0 } = {}) {
  return hex8(fnv1aF64(canonicalNumbers(obj, quant)));
}

export function hashBoth(obj, quant = TS.HASH_QUANT) {
  return { exact: hashState(obj), quant: hashState(obj, { quant }) };
}
