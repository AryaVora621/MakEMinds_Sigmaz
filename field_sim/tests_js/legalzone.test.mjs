import test from 'node:test';
import assert from 'node:assert/strict';
import { LegalZone, pointInPolygon, distanceToHive } from '../web/js/legalzone.js';

test('presets list has all ids and assumption wording', () => {
  const ps = LegalZone.presets();
  for (const id of ['open', 'own_half', 'audience_side', 'far_side', 'near_hive_ring', 'outside_hive_footprint', 'custom']) {
    const p = ps.find((q) => q.id === id);
    assert.ok(p, id);
    assert.equal(p.assumption, true);
    assert.match(p.description, /assum/i);
  }
});
test('open: inside field only', () => {
  const z = new LegalZone();
  assert.ok(z.contains(10, 10, 'red') && z.contains(140, 140, 'blue'));
  assert.ok(!z.contains(-1, 10, 'red') && !z.contains(10, 145, 'red'));
});
test('own_half flips per alliance', () => {
  const z = new LegalZone({ preset: 'own_half' });
  assert.ok(z.contains(20, 50, 'red')); assert.ok(!z.contains(100, 50, 'red'));
  assert.ok(z.contains(100, 50, 'blue')); assert.ok(!z.contains(20, 50, 'blue'));
});
test('own_half uses constructor alliance when none passed', () => {
  const z = new LegalZone({ preset: 'own_half', alliance: 'blue' });
  assert.ok(z.contains(100, 50)); assert.ok(!z.contains(20, 50));
});
test('audience_side', () => {
  const z = new LegalZone({ preset: 'audience_side' });
  for (const a of ['red', 'blue']) { assert.ok(z.contains(70, 30, a)); assert.ok(!z.contains(70, 100, a)); }
});
test('far_side', () => {
  const z = new LegalZone({ preset: 'far_side' });
  for (const a of ['red', 'blue']) { assert.ok(z.contains(70, 100, a)); assert.ok(!z.contains(70, 30, a)); }
});
test('boundary y=72 legal for both halves', () => {
  assert.ok(new LegalZone({ preset: 'audience_side' }).contains(50, 72, 'red'));
  assert.ok(new LegalZone({ preset: 'far_side' }).contains(50, 72, 'red'));
  assert.ok(new LegalZone({ preset: 'own_half' }).contains(72, 10, 'blue'));
});
test('annulus near_hive_ring per alliance', () => {
  const z = new LegalZone({ preset: 'near_hive_ring' });
  assert.ok(!z.contains(59.25, 72, 'red'));
  assert.ok(z.contains(59.25 + 50, 72, 'red'));
  assert.ok(!z.contains(59.25 + 100, 72, 'red'));
  assert.ok(z.contains(84.75 - 50, 72, 'blue'));
  assert.ok(!z.contains(84.75, 72, 'blue'));
  assert.ok(z.contains(59.25 + 30, 72, 'red'));
  const z2 = new LegalZone({ preset: 'near_hive_ring', minR: 10, maxR: 20 });
  assert.ok(z2.contains(59.25 + 15, 72, 'red')); assert.ok(!z2.contains(59.25 + 25, 72, 'red'));
});
test('outside_hive_footprint', () => {
  const z = new LegalZone({ preset: 'outside_hive_footprint' });
  assert.ok(!z.contains(59.25, 72, 'red')); assert.ok(!z.contains(84.75, 72, 'blue'));
  assert.ok(z.contains(10, 10, 'red')); assert.ok(z.contains(130, 130, 'blue'));
  assert.ok(z.contains(59.25 + 10.02, 72, 'red'));
});
test('custom polygons', () => {
  const z = new LegalZone({ preset: 'custom', polygons: [[[0, 0], [20, 0], [20, 20], [0, 20]], [[100, 100], [120, 100], [110, 120]]] });
  assert.ok(z.contains(10, 10, 'red')); assert.ok(z.contains(110, 105, 'blue'));
  assert.ok(!z.contains(50, 50, 'red')); assert.ok(z.contains(20, 10, 'red'));
});
test('pointInPolygon concave, vertex, bad poly', () => {
  const L = [[0, 0], [10, 0], [10, 4], [4, 4], [4, 10], [0, 10]];
  assert.ok(pointInPolygon(2, 8, L)); assert.ok(!pointInPolygon(8, 8, L));
  assert.ok(pointInPolygon(0, 0, L)); assert.ok(!pointInPolygon(1, 1, []));
});
test('enabled=false allows everything', () => {
  const z = new LegalZone({ preset: 'audience_side', enabled: false });
  assert.ok(z.contains(70, 140, 'red')); assert.ok(z.contains(-50, 500, 'blue'));
});
test('JSON round trip', () => {
  const z = new LegalZone({ preset: 'custom', polygons: [[[0, 0], [5, 0], [5, 5]]], alliance: 'red', enabled: false });
  const r = LegalZone.fromJSON(JSON.parse(JSON.stringify(z)));
  assert.deepEqual(r.toJSON(), z.toJSON());
  const a = LegalZone.fromJSON({ preset: 'near_hive_ring', minR: 5, maxR: 15 });
  assert.equal(LegalZone.fromJSON(a.toJSON()).maxR, 15);
});
test('rejects bad input', () => {
  assert.throws(() => new LegalZone({ preset: 'nope' }));
  assert.throws(() => new LegalZone({ alliance: 'green' }));
  assert.throws(() => new LegalZone({ preset: 'custom' }));
  assert.throws(() => new LegalZone({ polygons: [[[0, 0], [1, 1]]] }));
  assert.throws(() => new LegalZone({ polygons: [[[0, 0], [1, NaN], [2, 2]]] }));
  assert.throws(() => new LegalZone({ preset: 'near_hive_ring', minR: 50, maxR: 10 }));
  assert.throws(() => LegalZone.fromJSON(null));
  assert.throws(() => LegalZone.fromJSON('x'));
});
test('describe is non-empty and mentions assumption', () => {
  for (const p of LegalZone.presets()) {
    const z = new LegalZone({ preset: p.id, polygons: [[[0, 0], [5, 0], [5, 5]]] });
    assert.match(z.describe(), /assum/i);
  }
  assert.match(new LegalZone({ preset: 'audience_side' }).describe(), /y < 72/);
});
test('polygonsFor shapes and distanceToHive', () => {
  assert.equal(new LegalZone({ preset: 'audience_side' }).polygonsFor('red')[0].length, 4);
  const ring = new LegalZone({ preset: 'near_hive_ring' }).polygonsFor('blue')[0];
  assert.ok(ring.outer.length > 8 && ring.inner.length > 8);
  assert.ok(Math.abs(ring.outer[0][0] - (84.75 + 90)) < 1e-9);
  assert.equal(new LegalZone({ preset: 'own_half' }).polygonsFor('red')[0][2][0], 72);
  assert.equal(new LegalZone({ preset: 'own_half' }).polygonsFor('blue')[0][0][0], 72);
  assert.ok(Math.abs(distanceToHive(59.25, 82, 'red') - 10) < 1e-9);
  assert.equal(distanceToHive(0, 0, 'red', { x: 3, y: 4 }), 5);
});
