import test from 'node:test';
import assert from 'node:assert/strict';
import { mean, variance, std, median, quantile, normalCdf, normalQuantile, wilson, agrestiCoull, twoProportionZ,
  sampleSizeTwoProportions, regIncBeta, studentTCdf, welchT, ksTwoSample, acceptRate } from '../../web/js/physics/validation-tuning/stats.js';
import { mulberry32, gaussian } from '../../web/js/physics/validation-tuning/rng.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b} (tol ${tol})`);

test('basic moments', () => {
  assert.equal(mean([1, 2, 3, 4]), 2.5);
  near(variance([1, 2, 3, 4]), 5 / 3, 1e-12); // n-1
  near(std([1, 2, 3, 4]), Math.sqrt(5 / 3), 1e-12);
  assert.equal(median([3, 1, 2]), 2);
  assert.equal(median([4, 1, 2, 3]), 2.5);
});
test('quantile type 7', () => {
  near(quantile([1, 2, 3, 4], 0.5), 2.5, 1e-12);
  near(quantile([1, 2, 3, 4, 5], 0.25), 2, 1e-12);
  assert.equal(quantile([1, 2, 3], 0), 1);
  assert.equal(quantile([1, 2, 3], 1), 3);
});
test('empty input throws TypeError', () => {
  assert.throws(() => mean([]), TypeError);
  assert.throws(() => median([]), TypeError);
  assert.throws(() => wilson(0, 0), TypeError);
});
test('normalCdf and normalQuantile', () => {
  near(normalCdf(0), 0.5, 1e-7);
  near(normalCdf(1.959963984540054), 0.975, 2e-7);
  near(normalQuantile(0.975), 1.959964, 1e-6);
  near(normalQuantile(0.5), 0, 1e-9);
  near(normalQuantile(0.8), 0.8416212, 1e-5);
  for (const p of [0.01, 0.2, 0.7, 0.99]) near(normalCdf(normalQuantile(p)), p, 1e-6);
});
test('wilson(0,10): lo 0 and hi 0.27754', () => {
  const w = wilson(0, 10);
  near(w.lo, 0, 1e-9);
  near(w.hi, 0.27754, 1e-4);
  assert.equal(w.p, 0);
});
test('wilson interval brackets p and stays in [0,1]', () => {
  for (const [k, n] of [[1, 10], [5, 10], [10, 10], [37, 100]]) {
    const w = wilson(k, n);
    assert.ok(w.lo >= 0 && w.hi <= 1 && w.lo <= w.p && w.p <= w.hi);
  }
});
test('agrestiCoull is close to wilson for large n', () => {
  const a = agrestiCoull(500, 1000), w = wilson(500, 1000);
  near(a.lo, w.lo, 2e-3); near(a.hi, w.hi, 2e-3);
});
test('sampleSizeTwoProportions(0.5, 0.6) = 385', () => {
  assert.equal(sampleSizeTwoProportions(0.5, 0.6), 385);
});
test('twoProportionZ: equal proportions give z 0 and p 1; different give small p', () => {
  const same = twoProportionZ(50, 100, 50, 100);
  near(same.z, 0, 1e-12); near(same.p, 1, 1e-6);
  const diff = twoProportionZ(20, 100, 60, 100);
  assert.ok(diff.p < 1e-5 && diff.z < 0);
  near(twoProportionZ(50, 100, 60, 100).p, 0.1552, 1e-3);
});
test('regIncBeta known values and studentTCdf symmetry', () => {
  near(regIncBeta(0.5, 2, 2), 0.5, 1e-9);
  near(regIncBeta(0, 2, 3), 0, 1e-12);
  near(regIncBeta(1, 2, 3), 1, 1e-12);
  near(studentTCdf(0, 7), 0.5, 1e-9);
  near(studentTCdf(2.0, 10) + studentTCdf(-2.0, 10), 1, 1e-9);
  near(studentTCdf(2.228, 10), 0.975, 1e-3); // t table, df 10
});
function sample(n, mu, sd, seed) { const g = gaussian(mulberry32(seed)); return Array.from({ length: n }, () => mu + sd * g()); }
test('welchT: same distribution is not significant, shifted is', () => {
  const a = sample(60, 10, 2, 1), b = sample(60, 10, 2, 2), c = sample(60, 13, 2, 3);
  assert.ok(welchT(a, b).p > 0.01);
  const w = welchT(a, c);
  assert.ok(w.p < 1e-6 && w.t < 0 && w.df > 50);
});
test('welchT matches a hand-computed case', () => {
  const r = welchT([1, 2, 3, 4, 5], [2, 3, 4, 5, 6]);
  near(r.t, -1, 1e-9); near(r.df, 8, 1e-9);
  near(r.p, 0.3466, 2e-3);
});
test('ksTwoSample: identical gives d 0 and p 1; separated gives d 1 and tiny p', () => {
  const a = sample(50, 0, 1, 4);
  const same = ksTwoSample(a, a);
  assert.equal(same.d, 0); near(same.p, 1, 1e-9);
  const far = ksTwoSample(a, a.map((x) => x + 100));
  assert.equal(far.d, 1); assert.ok(far.p < 1e-6);
});
test('acceptRate uses the wilson bounds', () => {
  assert.equal(acceptRate(95, 100, { pMin: 0.8 }).pass, true);
  assert.equal(acceptRate(5, 10, { pMin: 0.8 }).pass, false);
  assert.equal(acceptRate(2, 100, { pMax: 0.1 }).pass, true);
  assert.equal(acceptRate(30, 100, { pMax: 0.1 }).pass, false);
  assert.equal(acceptRate(5, 10, {}).pass, true);
  const r = acceptRate(95, 100, { pMin: 0.8 });
  assert.ok(r.ci.lo < 0.95 && r.ci.hi > 0.95);
});
