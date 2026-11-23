import test from 'node:test';
import assert from 'node:assert/strict';
import { gen, forAll, assertProperty } from '../../web/js/physics/validation-tuning/proptest.js';
import { mulberry32 } from '../../web/js/physics/validation-tuning/rng.js';
import { PROP } from '../../web/js/physics/validation-tuning/params.js';

test('generators produce values in range', () => {
  const rng = mulberry32(1);
  const d = gen.double({ min: -2, max: 5 }), i = gen.integer({ min: 3, max: 9 });
  for (let k = 0; k < 200; k++) {
    const a = d.generate(rng), b = i.generate(rng);
    assert.ok(a >= -2 && a <= 5);
    assert.ok(Number.isInteger(b) && b >= 3 && b <= 9);
  }
  assert.equal(gen.constant(7).generate(rng), 7);
  assert.ok([1, 2, 3].includes(gen.oneOf(1, 2, 3).generate(rng)));
  const rec = gen.record({ a: gen.integer({ min: 0, max: 1 }), b: gen.constant('x') }).generate(rng);
  assert.equal(rec.b, 'x');
  const arr = gen.array(gen.integer({ min: 0, max: 5 }), { minLength: 2, maxLength: 4 }).generate(rng);
  assert.ok(arr.length >= 2 && arr.length <= 4);
});
test('a passing property reports numRuns runs', () => {
  const r = forAll(gen.double({ min: 0, max: 1 }), (v) => v >= 0 && v <= 1, { numRuns: 25, seed: 3 });
  assert.equal(r.ok, true); assert.equal(r.runs, 25); assert.equal(r.seed, 3);
});
test('default numRuns comes from PROP', () => {
  const r = forAll(gen.constant(1), () => true, { seed: 1 });
  assert.ok([PROP.numRuns, PROP.numRunsCI, Number(process.env[PROP.runsEnv])].includes(r.runs), 'runs ' + r.runs); // default is numRuns, or numRunsCI / PROP_RUNS when set
});
test('a failing double property shrinks toward the failure boundary and reports the seed', () => {
  const r = forAll(gen.double({ min: 0, max: 100 }), (v) => v < 10, { seed: 42, numRuns: 100 });
  assert.equal(r.ok, false);
  assert.equal(r.seed, 42);
  assert.ok(Number.isInteger(r.failure.run));
  assert.ok(r.failure.value >= 10);
  assert.ok(r.failure.shrunk >= 10, 'shrunk value must still fail');
  assert.ok(r.failure.shrunk <= r.failure.value);
  assert.ok(r.failure.shrunk < 12, `shrunk ${r.failure.shrunk} should be near the bound 10`);
});
test('shrinking moves toward the bound nearest zero (negative range)', () => {
  const r = forAll(gen.double({ min: -100, max: -1 }), (v) => v > -20, { seed: 9, numRuns: 100 });
  assert.equal(r.ok, false);
  assert.ok(r.failure.shrunk <= -20 && r.failure.shrunk >= r.failure.value);
  assert.ok(r.failure.shrunk > -22);
});
test('array failures shrink by dropping elements', () => {
  const arb = gen.array(gen.integer({ min: 0, max: 50 }), { minLength: 0, maxLength: 20 });
  const r = forAll(arb, (a) => a.length < 3, { seed: 5, numRuns: 100 });
  assert.equal(r.ok, false);
  assert.equal(r.failure.shrunk.length, 3);
});
test('record failures shrink one field at a time', () => {
  const arb = gen.record({ a: gen.integer({ min: 0, max: 100 }), b: gen.integer({ min: 0, max: 100 }) });
  const r = forAll(arb, (v) => v.a < 10, { seed: 6, numRuns: 100 });
  assert.equal(r.ok, false);
  assert.ok(r.failure.shrunk.a >= 10 && r.failure.shrunk.a <= r.failure.value.a);
  assert.equal(r.failure.shrunk.b, 0);
});
test('a throwing predicate counts as a failure and carries the error', () => {
  const r = forAll(gen.integer({ min: 0, max: 10 }), (v) => { if (v >= 0) throw new Error('boom'); return true; }, { seed: 1, numRuns: 5 });
  assert.equal(r.ok, false);
  assert.match(String(r.failure.error), /boom/);
});
test('same seed gives the same failing run', () => {
  const run = () => forAll(gen.double({ min: 0, max: 100 }), (v) => v < 10, { seed: 77 });
  const a = run(), b = run();
  assert.equal(a.failure.run, b.failure.run);
  assert.equal(a.failure.value, b.failure.value);
  assert.equal(a.failure.shrunk, b.failure.shrunk);
});
test('PROP_SEED environment variable sets the default seed', () => {
  const old = process.env[PROP.seedEnv];
  process.env[PROP.seedEnv] = '1234';
  try { assert.equal(forAll(gen.constant(1), () => true, { numRuns: 1 }).seed, 1234); }
  finally { if (old === undefined) delete process.env[PROP.seedEnv]; else process.env[PROP.seedEnv] = old; }
});
test('assertProperty throws with the seed in the message and passes silently otherwise', () => {
  assertProperty(gen.constant(1), () => true, { numRuns: 3, seed: 1 });
  assert.throws(() => assertProperty(gen.double({ min: 0, max: 100 }), (v) => v < 10, { seed: 321 }), /321/);
});
