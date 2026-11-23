// The WPILOG byte layout in wpilog.js is PRIOR knowledge, not verified against datalog.adoc.
// These tests prove writer/reader self-consistency, not conformance with real robot logs.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readWpilog, writeWpilog, encodeValue, decodeValue } from '../../web/js/physics/validation-tuning/wpilog.js';
import { M_PER_IN } from '../../web/js/physics/validation-tuning/params.js';

const poseOf = (v) => ({ x: v.x ?? v[0], y: v.y ?? v[1], h: v.h ?? v.rotation ?? v.heading ?? v.theta ?? v[2] });

test('bad magic throws', () => {
  const bytes = new Uint8Array(20); bytes.set(Array.from('NOTLOG', (c) => c.charCodeAt(0)));
  assert.throws(() => readWpilog(bytes), Error);
});
test('header carries version and extra header', () => {
  const buf = writeWpilog([], [], { extraHeader: 'hello' });
  assert.equal(String.fromCharCode(...buf.slice(0, 6)), 'WPILOG');
  const r = readWpilog(buf);
  assert.equal(r.version, 0x0100);
  assert.equal(r.extraHeader, 'hello');
});
test('round trip of every scalar and array type', () => {
  const cases = [
    ['boolean', true], ['boolean', false],
    ['int64', 123456789012], ['int64', -5],
    ['float', 1.5], ['double', Math.PI],
    ['string', 'héllo wörld'],
    ['boolean[]', [true, false, true]],
    ['int64[]', [1, -2, 300000]],
    ['float[]', [0.5, -2.25]],
    ['double[]', [Math.E, -1e-9]],
    ['string[]', ['a', '', 'ccc']],
  ];
  const entries = cases.map(([type], i) => ({ name: `/e${i}`, type, metadata: '' }));
  const records = cases.map(([, value], i) => ({ name: `/e${i}`, tUs: 1000 * (i + 1), value }));
  const r = readWpilog(writeWpilog(entries, records));
  assert.equal(r.records.length, cases.length);
  cases.forEach(([type, value], i) => {
    const rec = r.records.find((x) => x.name === `/e${i}`);
    assert.equal(rec.type, type);
    if (type === 'float') assert.ok(Math.abs(rec.value - value) < 1e-6);
    else assert.deepEqual(rec.value, value, type);
  });
});
test('raw bytes round-trip as Uint8Array', () => {
  const raw = new Uint8Array([0, 1, 2, 250, 255]);
  const r = readWpilog(writeWpilog([{ name: '/r', type: 'raw' }], [{ name: '/r', tUs: 5, value: raw }]));
  assert.deepEqual(Array.from(r.records[0].value), Array.from(raw));
});
test('timestamps are converted from microseconds to seconds', () => {
  const r = readWpilog(writeWpilog([{ name: '/d', type: 'double' }], [{ name: '/d', tUs: 1500000, value: 1 }, { name: '/d', tUs: 2500000, value: 2 }]));
  assert.deepEqual(r.records.map((x) => x.t), [1.5, 2.5]);
});
test('large timestamps and entry ids use wider byte fields', () => {
  const entries = Array.from({ length: 300 }, (_, i) => ({ name: `/n${i}`, type: 'int64' }));
  const records = [{ name: '/n299', tUs: 5e9, value: 7 }];
  const r = readWpilog(writeWpilog(entries, records));
  assert.equal(r.records[0].name, '/n299');
  assert.equal(r.records[0].t, 5000);
  assert.equal(r.records[0].value, 7);
  assert.equal(r.entries.size, 300);
});
test('struct:Pose2d is 24 bytes in metres and radians', () => {
  const bytes = encodeValue('struct:Pose2d', { x: 10, y: -20, h: 0.5, rotation: 0.5, heading: 0.5 });
  assert.equal(bytes.length, 24);
  const dv = new DataView(bytes.buffer, bytes.byteOffset, 24);
  assert.ok(Math.abs(dv.getFloat64(0, true) - 10 * M_PER_IN) < 1e-12);
  assert.ok(Math.abs(dv.getFloat64(8, true) - -20 * M_PER_IN) < 1e-12);
  assert.ok(Math.abs(dv.getFloat64(16, true) - 0.5) < 1e-12);
});
test('struct:Pose2d round-trips through the file in inches', () => {
  const val = { x: 36, y: 72, h: -1.2, rotation: -1.2, heading: -1.2 };
  const r = readWpilog(writeWpilog([{ name: '/pose', type: 'struct:Pose2d' }], [{ name: '/pose', tUs: 10, value: val }]));
  const p = poseOf(r.records[0].value);
  assert.ok(Math.abs(p.x - 36) < 1e-9 && Math.abs(p.y - 72) < 1e-9 && Math.abs(p.h + 1.2) < 1e-9, JSON.stringify(r.records[0].value));
  const d = poseOf(decodeValue('struct:Pose2d', encodeValue('struct:Pose2d', val)));
  assert.ok(Math.abs(d.x - 36) < 1e-9);
});
test('unknown types decode as raw', () => {
  const v = decodeValue('struct:Mystery', new Uint8Array([1, 2, 3]));
  assert.deepEqual(Array.from(v), [1, 2, 3]);
});
test('entry metadata and names survive', () => {
  const r = readWpilog(writeWpilog([{ name: '/m', type: 'double', metadata: '{"unit":"in"}' }], []));
  const e = [...r.entries.values()][0];
  assert.equal(e.name, '/m'); assert.equal(e.type, 'double'); assert.equal(e.metadata, '{"unit":"in"}');
});
