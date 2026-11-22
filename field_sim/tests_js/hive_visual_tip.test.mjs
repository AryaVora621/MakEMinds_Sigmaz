import { test } from 'node:test';
import assert from 'node:assert/strict';
import { dampedEase, hiveTipVisual, dropSchedule, dropsDue, parseHiveCollider, TriMesh, hiveTipLabel } from '../web/js/hivemesh.js';

const base = { tipStartT: 1, fromDeg: -30, toDeg: 30, tipDurationS: 0.6 };
test('tip visual endpoints and window', () => {
  assert.deepEqual(hiveTipVisual({ ...base, t: 0.5 }), { angleDeg: -30, progress: 0, releasing: false });
  assert.equal(hiveTipVisual({ ...base, t: 1.0 }).releasing, true);
  assert.deepEqual(hiveTipVisual({ ...base, t: 1.6 }), { angleDeg: 30, progress: 1, releasing: false });
  assert.ok(Math.abs(hiveTipVisual({ ...base, t: 1.3 }).progress - 0.5) < 1e-9);
  assert.equal(hiveTipVisual({ ...base, tipDurationS: 0, t: 1 }).angleDeg, 30);
});
test('damped ease is monotone, bounded, 0..1', () => {
  let prev = -1;
  for (let i = 0; i <= 100; i++) { const e = dampedEase(i / 100); assert.ok(e >= prev && e <= 1 + 1e-12); prev = e; }
  assert.equal(dampedEase(0), 0); assert.ok(Math.abs(dampedEase(1) - 1) < 1e-12);
  assert.ok(Math.abs(dampedEase(0.25) - 0.44998) < 1e-4);
});
test('angle never overshoots either way', () => {
  for (const [a, b] of [[-30, 30], [30, -30]]) for (let i = 0; i <= 200; i++) {
    const g = hiveTipVisual({ t: i / 200 * 0.6, tipStartT: 0, fromDeg: a, toDeg: b, tipDurationS: 0.6 }).angleDeg;
    assert.ok(g >= Math.min(a, b) - 1e-9 && g <= Math.max(a, b) + 1e-9);
  }
});
test('drop schedule and due-detection', () => {
  const s = dropSchedule({ nBalls: 3, tipDurationS: 0.6 });
  assert.deepEqual(s.map(x => +x.toFixed(6)), [0.3, 0.4, 0.5]);
  assert.deepEqual(dropSchedule({ nBalls: 0, tipDurationS: 0.6 }), []);
  assert.deepEqual(dropsDue(s, 0, 0.35), [0]);
  assert.deepEqual(dropsDue(s, 0.35, 0.6), [1, 2]);
  assert.deepEqual(dropsDue(s, 0.6, 1), []);
});
test('parser rejects truncated, short and bad-index data; TriMesh rejects bad index', () => {
  assert.throws(() => parseHiveCollider(new ArrayBuffer(4)));
  const b = new ArrayBuffer(8 + 8 + 12 + 8 + 48 + 12), dv = new DataView(b);
  'HVC1'.split('').forEach((c, i) => dv.setUint8(i, c.charCodeAt(0))); dv.setUint32(4, 1, true);
  dv.setUint32(8 + 8 + 12, 4, true); dv.setUint32(8 + 8 + 12 + 4, 1, true);
  dv.setUint32(8 + 8 + 12 + 8 + 48, 0, true); dv.setUint32(8 + 8 + 12 + 8 + 48 + 4, 1, true); dv.setUint32(8 + 8 + 12 + 8 + 48 + 8, 9, true);
  assert.throws(() => parseHiveCollider(b), /tri index/);
  assert.throws(() => parseHiveCollider(b.slice(0, 60)), /truncated/);
  assert.throws(() => new TriMesh(new Float32Array(9), new Uint32Array([0, 1, 5])));
});
test('degenerate triangle yields no NaN contact', () => {
  const m = new TriMesh(new Float32Array([0, 0, 0, 0, 0, 0, 1, 0, 0]), new Uint32Array([0, 1, 2]));
  const r = m.sphereVsMesh([0.5, 0.5, 0.3], 1.4);
  assert.ok(!r.hit || Number.isFinite(r.depth));
});
test('hiveTipLabel null-safe', () => assert.equal(hiveTipLabel(null), 'pollen 0 + nectar 0 = 0.0/8'));
