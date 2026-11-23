import test from 'node:test';
import assert from 'node:assert/strict';
import { hashState, hashBoth, canonicalNumbers } from '../../web/js/physics/timestep-determinism/statehash.js';

test('hash is independent of key order (nested too)', () => {
  const a = { x: 1, y: { p: 2, q: [3, 4] }, z: true };
  const b = { z: true, y: { q: [3, 4], p: 2 }, x: 1 };
  assert.equal(hashState(a), hashState(b));
});

test('array order matters', () => {
  assert.notEqual(hashState({ a: [1, 2] }), hashState({ a: [2, 1] }));
});

test('-0 hashes like 0', () => {
  assert.equal(hashState({ v: -0 }), hashState({ v: 0 }));
  assert.equal(hashState([-0]), hashState([0]));
});

test('1e-7 perturbation changes exact hash but not quantised hash', () => {
  const a = { v: 1.0 }, b = { v: 1.0 + 1e-7 };
  assert.notEqual(hashState(a), hashState(b));
  assert.equal(hashState(a, { quant: 1e-6 }), hashState(b, { quant: 1e-6 }));
  const ha = hashBoth(a), hb = hashBoth(b);
  assert.notEqual(ha.exact, hb.exact);
  assert.equal(ha.quant, hb.quant);
});

test('NaN with any payload hashes canonically', () => {
  const dv = new DataView(new ArrayBuffer(8));
  dv.setUint32(0, 1, true); dv.setUint32(4, 0x7ff80000, true); // NaN with payload 1
  const odd = dv.getFloat64(0, true);
  assert.ok(Number.isNaN(odd));
  assert.equal(hashState({ v: odd }), hashState({ v: NaN }));
  assert.notEqual(hashState({ v: NaN }), hashState({ v: 0 }));
});

test('hash is 8 lowercase hex chars', () => {
  for (const o of [{}, { a: 1 }, [1, 2, 3], 'str', { s: 'hello', n: null }]) {
    assert.match(hashState(o), /^[0-9a-f]{8}$/);
  }
  const h = hashBoth({ a: 1 });
  assert.match(h.exact, /^[0-9a-f]{8}$/);
  assert.match(h.quant, /^[0-9a-f]{8}$/);
});

test('canonicalNumbers returns Float64Array and is key-order independent', () => {
  const n1 = canonicalNumbers({ a: 1, b: [2, 3] });
  const n2 = canonicalNumbers({ b: [2, 3], a: 1 });
  assert.ok(n1 instanceof Float64Array);
  assert.deepEqual(Array.from(n1), Array.from(n2));
});

test('booleans and strings are distinguished', () => {
  assert.notEqual(hashState({ a: true }), hashState({ a: false }));
  assert.notEqual(hashState({ a: 'x' }), hashState({ a: 'y' }));
});
