import test from 'node:test';
import assert from 'node:assert/strict';
import { mergeParams } from '../../web/js/physics/velocity-pidf/params.js';
import { wrapInt16, QuadratureEncoder } from '../../web/js/physics/velocity-pidf/encoder.js';
import { mulberry32, gaussian, LoopClock, CommandQueue, quantizePower } from '../../web/js/physics/velocity-pidf/timing.js';

const P = mergeParams();
const mkEnc = (over = {}) => {
  const e = new QuadratureEncoder({ cpr: 28, velWindowS: 0.05, historyS: 0.25, int16Wrap: true, direction: 1, ...over });
  e.reset(0, 0);
  return e;
};

test('wrapInt16', () => {
  assert.equal(wrapInt16(40000), -25536);
  assert.equal(wrapInt16(-40000), 25536);
  assert.equal(wrapInt16(100), 100);
  assert.equal(wrapInt16(-100), -100);
  assert.equal(wrapInt16(32767), 32767);
  assert.equal(wrapInt16(32768), -32768);
});

test('encoder counts are integers (floor) and velocity tracks a constant speed in 20 ticks/s steps', () => {
  const e = mkEnc();
  const w = 2 * Math.PI * 50;            // 50 rev/s = 1400 ticks/s
  const h = 0.0005;
  for (let i = 1; i <= 4000; i++) {
    const t = i * h;
    e.update(t, w * t);
    assert.ok(Number.isInteger(e.position));
    const v = e.velocityAt(t);
    assert.ok(Number.isInteger(v));
    assert.equal(v % 20, 0, `v=${v}`);
    if (t > 0.2) assert.ok(Math.abs(v - 1400) <= 45, `v=${v} t=${t}`);
  }
  assert.equal(e.position, Math.floor(w * 2 * 28 / (2 * Math.PI)));
});

test('encoder direction -1 flips the sign', () => {
  const e = mkEnc({ direction: -1 });
  for (let i = 1; i <= 1000; i++) e.update(i * 0.0005, 2 * Math.PI * 10 * i * 0.0005);
  assert.ok(e.velocityAt(0.5) < 0);
});

test('encoder velocity wraps to int16 when enabled', () => {
  const on = mkEnc(), off = mkEnc({ int16Wrap: false });
  const w = 2 * Math.PI * 2000;    // 56000 ticks/s, outside int16
  for (let i = 1; i <= 800; i++) { on.update(i * 0.0005, w * i * 0.0005); off.update(i * 0.0005, w * i * 0.0005); }
  assert.ok(off.velocityAt(0.4) > 32767);
  assert.equal(on.velocityAt(0.4), wrapInt16(off.velocityAt(0.4)));
});

test('encoder history is trimmed and reset reseeds', () => {
  const e = mkEnc();
  for (let i = 1; i <= 4000; i++) e.update(i * 0.0005, 5 * i * 0.0005);
  assert.ok(Number.isFinite(e.velocityAt(2.0)));
  e.reset(0, 0);
  assert.equal(e.position, 0);
  assert.equal(e.velocityAt(0), 0);
});

test('mulberry32 is deterministic and in [0,1)', () => {
  const a = mulberry32(12345), b = mulberry32(12345), c = mulberry32(99);
  const sa = [], sb = [], sc = [];
  for (let i = 0; i < 100; i++) { sa.push(a()); sb.push(b()); sc.push(c()); }
  assert.deepEqual(sa, sb);
  assert.notDeepEqual(sa, sc);
  assert.ok(sa.every((x) => x >= 0 && x < 1));
});

test('gaussian has roughly zero mean and unit sd', () => {
  const r = mulberry32(1);
  let s = 0, s2 = 0; const n = 20000;
  for (let i = 0; i < n; i++) { const g = gaussian(r); s += g; s2 += g * g; }
  const m = s / n, sd = Math.sqrt(s2 / n - m * m);
  assert.ok(Math.abs(m) < 0.05, `mean ${m}`);
  assert.ok(Math.abs(sd - 1) < 0.05, `sd ${sd}`);
});

test('LoopClock periods stay within [min,max] and vary', () => {
  const clk = new LoopClock(P.timing, mulberry32(P.timing.seed));
  let t = 0; const periods = new Set();
  for (let i = 0; i < 2000; i++) {
    const n = clk.next(t);
    const d = n - t;
    assert.ok(d >= P.timing.loopPeriodMinS - 1e-12 && d <= P.timing.loopPeriodMaxS + 1e-12, `d=${d}`);
    periods.add(d.toFixed(6));
    t = n;
  }
  assert.ok(periods.size > 100);
  const mean = t / 2000;
  assert.ok(Math.abs(mean - P.timing.loopPeriodMeanS) < 0.002, `mean ${mean}`);
});

test('CommandQueue pops due commands in time order, stable for ties', () => {
  const q = new CommandQueue();
  q.push(0.3, { id: 'c' });
  q.push(0.1, { id: 'a1' });
  q.push(0.1, { id: 'a2' });
  q.push(0.2, { id: 'b' });
  assert.equal(q.size, 4);
  assert.deepEqual(q.popDue(0.05), []);
  assert.deepEqual(q.popDue(0.1).map((c) => c.id), ['a1', 'a2']);
  assert.deepEqual(q.popDue(0.25).map((c) => c.id), ['b']);
  assert.equal(q.size, 1);
  assert.deepEqual(q.popDue(0.3 - 1e-13).map((c) => c.id), ['c']);   // within 1e-12 tolerance
  q.push(1, { id: 'z' });
  q.clear();
  assert.equal(q.size, 0);
});

test('quantizePower', () => {
  assert.equal(quantizePower(NaN, 32767), 0);
  assert.equal(quantizePower(2, 32767), 1);
  assert.equal(quantizePower(-2, 32767), -1);
  assert.equal(quantizePower(0, 32767), 0);
  assert.ok(Math.abs(quantizePower(0.5, 32767) - 0.5) < 1 / 32767);
  assert.equal(quantizePower(0.3, 10), 0.3);
  assert.equal(quantizePower(0.34, 10), 0.3);
});
