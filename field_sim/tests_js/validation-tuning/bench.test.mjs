// Timing numbers are machine dependent, so the regression gate runs only when BENCH_GATE=1.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BENCH_SCENES, runBenchmark, runAll, toCsv, compareBaseline } from '../../web/js/physics/validation-tuning/bench.js';
import { BENCH } from '../../web/js/physics/validation-tuning/params.js';

const gate = globalThis.process?.env?.[BENCH.gateEnv] === '1';
const baselineUrl = new URL('../../web/js/physics/validation-tuning/bench_baseline.json', import.meta.url);

test('scene list matches params', () => assert.deepEqual([...BENCH_SCENES].sort(), Object.keys(BENCH.scenes).sort()));

test('runBenchmark returns a finite, well-ordered result', () => {
  const r = runBenchmark('drop', { steps: 20, runCount: 2 });
  assert.equal(r.scene, 'drop');
  assert.equal(r.balls, BENCH.scenes.drop.balls);
  assert.equal(r.steps, 20);
  for (const k of ['medianMs', 'p95Ms', 'maxMs', 'minMedianMs']) assert.ok(Number.isFinite(r[k]) && r[k] >= 0, k);
  assert.ok(r.medianMs <= r.p95Ms + 1e-12 && r.p95Ms <= r.maxMs + 1e-12);
  assert.ok(r.minMedianMs <= r.medianMs + 1e-12 || r.runs.length === 2);
  assert.equal(r.runs.length, 2);
});
test('an injected clock makes the timing exact', () => {
  let t = 0;
  const r = runBenchmark('drop', { steps: 10, runCount: 1, now: () => (t += 2) });
  assert.equal(r.medianMs, 2); assert.equal(r.maxMs, 2);
});
test('toCsv has the documented header and one row per result', () => {
  const rows = [runBenchmark('drop', { steps: 5, runCount: 1 }), runBenchmark('push', { steps: 5, runCount: 1 })];
  const lines = toCsv(rows).trim().split('\n');
  assert.equal(lines[0], 'scene,balls,steps,median_ms,p95_ms,max_ms');
  assert.equal(lines.length, 3);
  assert.equal(lines[1].split(',').length, 6);
  assert.ok(lines[1].startsWith('drop,'));
});
test('compareBaseline flags regressions beyond the factor only', () => {
  const base = { results: [{ scene: 'drop', medianMs: 1 }] };
  const f = BENCH.regressFactor;
  assert.equal(compareBaseline([{ scene: 'drop', medianMs: 1 * f * 0.99 }], base).pass, true);
  const bad = compareBaseline([{ scene: 'drop', medianMs: 1 * f * 1.5 }], base);
  assert.equal(bad.pass, false);
  assert.equal(bad.regressions[0].scene, 'drop');
});
test('the shipped baseline file has the documented shape', () => {
  const b = JSON.parse(readFileSync(baselineUrl, 'utf8'));
  assert.ok(typeof b.node === 'string' || typeof b.nodeVersion === 'string' || Object.keys(b).some((k) => /node/i.test(k)));
  assert.ok(Object.keys(b).some((k) => /machine/i.test(k)));
  assert.ok(Array.isArray(b.results) && b.results.length >= 1);
});
test(`regression gate against the baseline (runs only when ${BENCH.gateEnv}=1)`, { skip: !gate && `set ${BENCH.gateEnv}=1` }, () => {
  const b = JSON.parse(readFileSync(baselineUrl, 'utf8'));
  const res = compareBaseline(runAll(), b);
  assert.equal(res.pass, true, JSON.stringify(res.regressions));
});
