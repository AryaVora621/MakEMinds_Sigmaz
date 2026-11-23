import test from 'node:test';
import assert from 'node:assert/strict';
import { TS, makeParams } from '../../web/js/physics/timestep-determinism/params.js';
import { splitmix32, fnv1a32, createRng, installRandomTrap } from '../../web/js/physics/timestep-determinism/prng.js';

test('TS is frozen and rejects writes', () => {
  assert.ok(Object.isFrozen(TS));
  assert.throws(() => { TS.PHYSICS_HZ = 1; }, TypeError);
});

test('TS rates are mutually consistent', () => {
  assert.ok(Math.abs(TS.CONTROL_DT * TS.CONTROL_HZ - 1) < 1e-12);
  assert.equal(TS.PHYSICS_HZ / TS.CONTROL_HZ, TS.CONTROL_DIVIDER);
  assert.ok(Math.abs(TS.PHYSICS_DT * TS.PHYSICS_HZ - 1) < 1e-12);
  assert.equal(TS.PHYSICS_HZ, 240);
  assert.equal(TS.CONTROL_HZ, 60);
});

test('makeParams derives values and rejects non-multiples', () => {
  const p = makeParams({ PHYSICS_HZ: 480 });
  assert.ok(Object.isFrozen(p));
  assert.equal(p.CONTROL_DIVIDER, 8);
  assert.ok(Math.abs(p.PHYSICS_DT - 1 / 480) < 1e-15);
  assert.throws(() => makeParams({ PHYSICS_HZ: 250, CONTROL_HZ: 60 }));
});

test('same seed gives identical 1000-value sequence; different seeds differ', () => {
  const a = createRng(123), b = createRng(123), c = createRng(124);
  const sa = [], sb = [], sc = [];
  for (let i = 0; i < 1000; i++) { sa.push(a.nextU32()); sb.push(b.nextU32()); sc.push(c.nextU32()); }
  assert.deepEqual(sa, sb);
  assert.notDeepEqual(sa, sc);
});

test('splitmix32 and fnv1a32 are deterministic uint32', () => {
  const s1 = splitmix32(5), s2 = splitmix32(5);
  for (let i = 0; i < 10; i++) {
    const v = s1();
    assert.equal(v, s2());
    assert.ok(Number.isInteger(v) && v >= 0 && v < 2 ** 32);
  }
  assert.equal(fnv1a32(''), 0x811c9dc5);
  assert.equal(fnv1a32('a'), 0xe40c292c); // known FNV-1a 32-bit test vector
});

test('stream isolation from siblings and parent', () => {
  const ref = createRng(99).stream('a');
  const refSeq = Array.from({ length: 50 }, () => ref.nextU32());
  const parent = createRng(99);
  const a = parent.stream('a');
  const b = parent.stream('b');
  for (let i = 0; i < 37; i++) { b.nextU32(); parent.nextU32(); }
  const seq = Array.from({ length: 50 }, () => a.nextU32());
  assert.deepEqual(seq, refSeq);
  assert.notEqual(createRng(99).stream('b').nextU32(), refSeq[0]);
});

test('getState/setState round-trips', () => {
  const r = createRng(7);
  for (let i = 0; i < 20; i++) r.next();
  const s = r.getState();
  assert.equal(s.length, 4);
  const x = Array.from({ length: 30 }, () => r.nextU32());
  r.setState(s);
  const y = Array.from({ length: 30 }, () => r.nextU32());
  assert.deepEqual(x, y);
});

test('next() stays in [0,1)', () => {
  const r = createRng(11);
  for (let i = 0; i < 10000; i++) {
    const v = r.next();
    assert.ok(v >= 0 && v < 1);
  }
});

test('gaussian has mean ~0 and sd ~1', () => {
  const r = createRng(2024);
  const n = 20000;
  let s = 0, s2 = 0;
  for (let i = 0; i < n; i++) { const g = r.gaussian(); s += g; s2 += g * g; }
  const mean = s / n;
  const sd = Math.sqrt(s2 / n - mean * mean);
  assert.ok(Math.abs(mean) < 0.05, `mean ${mean}`);
  assert.ok(Math.abs(sd - 1) < 0.05, `sd ${sd}`);
});

test('installRandomTrap counts calls and restore() restores Math.random', () => {
  const original = Math.random;
  const trap = installRandomTrap();
  try {
    assert.notEqual(Math.random, original);
    Math.random(); Math.random(); Math.random();
    assert.equal(trap.count(), 3);
  } finally { trap.restore(); }
  assert.equal(Math.random, original);
  const t2 = installRandomTrap({ throwOnCall: true });
  try { assert.throws(() => Math.random()); } finally { t2.restore(); }
  assert.equal(Math.random, original);
});
