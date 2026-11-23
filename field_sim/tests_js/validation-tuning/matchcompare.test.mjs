import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { loadReference, compareCycles, compareScoring, shotHitRate } from '../../web/js/physics/validation-tuning/matchcompare.js';
import { loadFieldEnvNode, FIELD_BOUNDS } from '../../web/js/physics/validation-tuning/node_env.js';
import { mulberry32, gaussian } from '../../web/js/physics/validation-tuning/rng.js';

const templateUrl = new URL('../../web/js/physics/validation-tuning/reference_template.json', import.meta.url);

test('reference template loads and carries no invented numbers', () => {
  const raw = JSON.parse(readFileSync(templateUrl, 'utf8'));
  const ref = loadReference(raw);
  assert.equal(typeof ref.source, 'string');
  assert.deepEqual(ref.cycles, []);
  assert.ok(ref.shots === null || ref.shots.attempted === 0 || ref.shots.attempted === undefined);
  assert.deepEqual(loadReference(readFileSync(templateUrl, 'utf8')).cycles, []); // string input also accepted
});
test('loadReference keeps supplied data and rejects non-numeric cycles', () => {
  const r = loadReference({ source: 'scout', cycles: [10, 12.5], shots: { made: 3, attempted: 5 } });
  assert.deepEqual(r.cycles, [10, 12.5]);
  assert.deepEqual(r.shots, { made: 3, attempted: 5 });
  assert.throws(() => loadReference({ source: 's', cycles: ['x'] }));
});
test('node_env exposes the field bounds and hives', async () => {
  assert.deepEqual(FIELD_BOUNDS, { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 });
  const env = await loadFieldEnvNode({ withHives: true });
  assert.deepEqual(env.bounds, FIELD_BOUNDS);
  assert.ok(env.hives && Object.keys(env.hives).length >= 1);
  const bare = await loadFieldEnvNode();
  assert.deepEqual(bare.bounds, FIELD_BOUNDS);
});
test('compareCycles summary, Welch and KS', () => {
  const g = gaussian(mulberry32(8));
  const a = Array.from({ length: 40 }, () => 20 + 2 * g());
  const b = Array.from({ length: 40 }, () => 26 + 2 * g());
  const r = compareCycles(a, b);
  assert.equal(r.sim.n, 40); assert.equal(r.real.n, 40);
  assert.ok(Math.abs(r.sim.mean - 20) < 1.5 && Math.abs(r.real.mean - 26) < 1.5);
  assert.ok(r.welch.p < 1e-6 && r.ks.p < 1e-4 && r.ks.d > 0.5);
  const same = compareCycles(a, a);
  assert.ok(same.welch.p > 0.99 && same.ks.d === 0);
});
test('compareScoring reports z, p and Wilson for each side', () => {
  const r = compareScoring({ k: 50, n: 100 }, { k: 60, n: 100 });
  assert.ok(r.p > 0.05 && r.p < 0.3);
  assert.ok(r.sim.lo < 0.5 && r.sim.hi > 0.5 && r.real.lo < 0.6 && r.real.hi > 0.6);
  assert.ok(compareScoring({ k: 20, n: 100 }, { k: 80, n: 100 }).p < 1e-6);
});
test('shotHitRate is deterministic per seed with 0 <= p <= 1', async () => {
  const env = await loadFieldEnvNode({ withHives: true });
  const a = shotHitRate(env, { nTrials: 8, seed: 11 });
  const b = shotHitRate(env, { nTrials: 8, seed: 11 });
  assert.deepEqual(a, b);
  assert.equal(a.n, 8);
  assert.ok(a.k >= 0 && a.k <= a.n);
  assert.ok(a.p >= 0 && a.p <= 1);
  assert.ok(a.ci.lo <= a.p && a.p <= a.ci.hi);
});
