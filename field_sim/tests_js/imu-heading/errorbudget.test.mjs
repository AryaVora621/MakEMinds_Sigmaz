// Error-budget + Monte Carlo tests (imu-heading). Heading in rad (stats) or deg (Monte Carlo), position in in.
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  headingErrorStats, lateralErrorSigma, PROFILES, PROFILE_DURATIONS, runMonteCarlo, percentile,
} from '../../web/js/physics/imu-heading/errorbudget.js';
import { resolveImuConfig } from '../../web/js/physics/imu-heading/params.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (tol ${e})`);
const DEG = Math.PI / 180;

test('percentile basics', () => {
  assert.equal(percentile([1, 2, 3, 4, 5], 50), 3);
  assert.equal(percentile([5, 1, 3], 0), 1);
  assert.equal(percentile([5, 1, 3], 100), 5);
  const p95 = percentile(Array.from({ length: 101 }, (_, i) => i), 95);
  near(p95, 95, 1);
});

test('analytic heading sigma matches the spec sanity numbers (pinpoint)', () => {
  const cfg = resolveImuConfig('pinpoint');
  const s30 = headingErrorStats(cfg, 30).sigma / DEG;
  const s120 = headingErrorStats(cfg, 120).sigma / DEG;
  assert.ok(s30 > 0.11 && s30 < 0.14, `30 s sigma ${s30} deg`);
  assert.ok(s120 > 0.46 && s120 < 0.52, `120 s sigma ${s120} deg`);
  for (const k of ['bias', 'arw', 'rw', 'gm', 'scale']) assert.ok(k in headingErrorStats(cfg, 30).terms, k);
  // linear-in-time bias dominates: 4x time gives ~4x sigma
  assert.ok(Math.abs(s120 / s30 - 4) < 0.5);
  // scale-error term grows with rotated angle
  assert.ok(headingErrorStats(cfg, 30, 8 * 2 * Math.PI).sigma > headingErrorStats(cfg, 30, 0).sigma);
});

test('analytic lateral error: ~1.9 in bias term at 60 in/s over 30 s, quadratic growth', () => {
  const cfg = resolveImuConfig('pinpoint');
  const l = lateralErrorSigma(cfg, 60, 30);
  assert.ok(l.sigma > 1.85 && l.sigma < 2.2, `sigma ${l.sigma} in`);
  const l60 = lateralErrorSigma(cfg, 60, 60).sigma;
  assert.ok(l60 / l.sigma > 3.2 && l60 / l.sigma < 4.4, `ratio ${l60 / l.sigma}`);
  assert.equal(lateralErrorSigma(resolveImuConfig('ideal'), 60, 30).sigma, 0);
  assert.equal(headingErrorStats(resolveImuConfig('ideal'), 30, 10).sigma, 0);
});

test('PROFILES are deterministic and well formed', () => {
  assert.equal(PROFILE_DURATIONS.auto30, 30);
  assert.equal(PROFILE_DURATIONS.teleop120, 120);
  let rot = 0;
  const hits = [];
  for (let i = 0; i < 12000; i++) {
    const t = i * 0.01;
    const a = PROFILES.auto30(t), a2 = PROFILES.auto30(t);
    assert.deepEqual(a, a2);
    for (const k of ['vf', 'vl', 'omega']) assert.ok(Number.isFinite(a[k]), `${k} @ ${t}`);
    if (t < 30) rot += Math.abs(a.omega) * 0.01;
    const b = PROFILES.teleop120(t);
    assert.ok(Number.isFinite(b.vf) && Number.isFinite(b.vl) && Number.isFinite(b.omega));
    if (b.collision) { assert.ok(b.collision.impactSpeedInS > 0); hits.push(t); }
  }
  near(rot, 2 * Math.PI, 0.2 * 2 * Math.PI); // four 90-degree turns
  assert.ok(hits.length > 0);
  for (const t of hits) assert.ok([35, 70, 100].some((c) => Math.abs(t - c) < 0.2), `collision at ${t}`);
  for (const c of [35, 70, 100]) assert.ok(hits.some((t) => Math.abs(t - c) < 0.2), `no collision near ${c}`);
});

test('Monte Carlo is reproducible for a given seed', () => {
  const a = runMonteCarlo({ preset: 'pinpoint', runs: 6, seed: 99, profile: 'auto30' });
  const b = runMonteCarlo({ preset: 'pinpoint', runs: 6, seed: 99, profile: 'auto30' });
  assert.deepEqual(a.final, b.final);
  assert.equal(a.final.length, 6);
  assert.ok(a.series.length > 5);
  for (const k of ['mean', 'p50', 'p95', 'max']) { assert.ok(Number.isFinite(a.headingErrDeg[k])); assert.ok(Number.isFinite(a.posErrIn[k])); }
  assert.ok(a.headingErrDeg.p50 <= a.headingErrDeg.p95 + 1e-12 && a.headingErrDeg.p95 <= a.headingErrDeg.max + 1e-12);
});

test('Monte Carlo pinpoint auto30 heading sigma is within 35% of the analytic sigma', () => {
  const cfg = resolveImuConfig('pinpoint');
  const analytic = headingErrorStats(cfg, 30, 2 * Math.PI).sigma / DEG; // deg
  const mc = runMonteCarlo({ preset: 'pinpoint', runs: 60, seed: 12345, profile: 'auto30' });
  const rms = Math.sqrt(mc.final.reduce((s, r) => s + r.headingErrDeg ** 2, 0) / mc.final.length);
  assert.ok(Math.abs(rms - analytic) / analytic < 0.35, `MC rms ${rms} deg vs analytic ${analytic} deg`);
});

test('teleop120 p95 heading error exceeds auto30 p95', () => {
  const auto = runMonteCarlo({ preset: 'pinpoint', runs: 40, seed: 5, profile: 'auto30' });
  const tele = runMonteCarlo({ preset: 'pinpoint', runs: 40, seed: 5, profile: 'teleop120' });
  assert.ok(tele.headingErrDeg.p95 > auto.headingErrDeg.p95, `${tele.headingErrDeg.p95} !> ${auto.headingErrDeg.p95}`);
  assert.ok(tele.posErrIn.p95 > 0);
});

test('Monte Carlo with the ideal preset has ~zero error', () => {
  const mc = runMonteCarlo({ preset: 'ideal', runs: 3, seed: 1, profile: 'auto30' });
  assert.ok(mc.headingErrDeg.max < 1e-9, `heading ${mc.headingErrDeg.max}`);
  assert.ok(mc.posErrIn.max < 1e-9, `pos ${mc.posErrIn.max}`);
});

test('a worse gyro preset has a larger Monte Carlo heading error', () => {
  const good = runMonteCarlo({ preset: 'pinpoint', runs: 30, seed: 3, profile: 'auto30' });
  const bad = runMonteCarlo({ preset: 'pinpoint', runs: 30, seed: 3, profile: 'auto30', overrides: { biasInitSigmaDps: 0.05 } });
  assert.ok(bad.headingErrDeg.p95 > 2 * good.headingErrDeg.p95, `${bad.headingErrDeg.p95} vs ${good.headingErrDeg.p95}`);
});
