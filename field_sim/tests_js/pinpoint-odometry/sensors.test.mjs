// Tests for podwheel.js and imu.js (P5). Units: inches, rad, seconds.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeOdoParams, MM_PER_IN } from '../../web/js/physics/pinpoint-odometry/params.js';
import { makeRng } from '../../web/js/physics/pinpoint-odometry/rng.js';
import { PodWheel } from '../../web/js/physics/pinpoint-odometry/podwheel.js';
import { Imu } from '../../web/js/physics/pinpoint-odometry/imu.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} got ${a} want ${b}`);
const NOSLIP = { slip: { enabled: false } };

function mkPod(kind, overrides = {}) {
  const params = makeOdoParams('nominal', overrides);
  return { params, pod: new PodWheel(kind, params, makeRng(params.seed)) };
}
function drive(pod, vf, vl, om, T, dt = 1 / 60, sub = 3) {
  const n = Math.round(T / dt);
  for (let i = 0; i < n; i++) {
    pod.beginStep(dt, vf, vl, om, {});
    for (let k = 0; k < sub; k++) pod.advance(dt / sub, vf, vl, om);
  }
}
function std(a) {
  const m = a.reduce((s, x) => s + x, 0) / a.length;
  return Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / (a.length - 1));
}

test('counts equal floor(travel * 25.4 * ticksPerMmTruth)', () => {
  const { params, pod } = mkPod('forward', NOSLIP);
  drive(pod, 30, 0, 0, 1);
  near(pod.travelIn, 30, 1e-6);
  const expectTicks = pod.travelIn * MM_PER_IN * params.ticksPerMmTruth;
  near(pod.ticksF, expectTicks, 1e-6);
  assert.equal(pod.count, Math.floor(pod.ticksF));
  assert.ok(Number.isInteger(pod.count));
});

test('quantize=false gives float count', () => {
  const { pod } = mkPod('forward', { slip: { enabled: false }, counter: { quantize: false } });
  drive(pod, 7.3, 0, 0, 0.5);
  assert.equal(pod.count, pod.ticksF);
});

test('strafe pod reads vl', () => {
  const { pod } = mkPod('strafe', NOSLIP);
  drive(pod, 10, 20, 0, 1);
  near(pod.groundTravelIn, 20, 1e-6);
});

test('diameterScale 1.02 under-reads by 1/1.02', () => {
  const base = mkPod('forward', NOSLIP).pod;
  const big = mkPod('forward', { slip: { enabled: false }, truth: { forwardPod: { diameterScale: 1.02 } } }).pod;
  drive(base, 30, 0, 0, 1); drive(big, 30, 0, 0, 1);
  near(big.ticksF / base.ticksF, 1 / 1.02, 1e-6);
});

test('slip.enabled=false means no slip even with collision and accel', () => {
  const { pod } = mkPod('forward', NOSLIP);
  pod.beginStep(1 / 60, 0, 0, 0);
  pod.beginStep(1 / 60, 100, 0, 0, { collision: 1, pushing: true });
  assert.equal(pod.slipFraction, 0);
  assert.equal(pod.inLiftoff, false);
  for (let i = 0; i < 10; i++) pod.advance(0.005, 100, 0, 0);
  near(pod.travelIn, pod.groundTravelIn, 1e-12);
});

test('liftoff after severity-1 collision under-reads, then recovers the rate', () => {
  const { pod } = mkPod('forward');
  const vf = 40;
  drive(pod, vf, 0, 0, 0.2); // establish steady motion
  const t0 = pod.travelIn, g0 = pod.groundTravelIn;
  near(t0, g0, 1e-6);
  pod.beginStep(1 / 60, vf, 0, 0, { collision: 1 });
  assert.equal(pod.inLiftoff, true);
  for (let i = 0; i < 10; i++) pod.advance(0.002, vf, 0, 0); // 20 ms, inside liftoff window
  const lost = (pod.groundTravelIn - g0) - (pod.travelIn - t0);
  assert.ok(lost > 0.05, `expected under-read, lost=${lost}`);
  for (let i = 0; i < 40; i++) pod.advance(0.002, vf, 0, 0); // run out the timer
  assert.equal(pod.inLiftoff, false);
  pod.beginStep(1 / 60, vf, 0, 0, {});
  const t1 = pod.travelIn, g1 = pod.groundTravelIn;
  pod.advance(0.01, vf, 0, 0);
  near(pod.travelIn - t1, pod.groundTravelIn - g1, 1e-9, 'rate recovered');
  assert.ok(pod.travelIn < pod.groundTravelIn); // net deficit persists
});

test('hard acceleration causes slip fraction > 0; steady motion none', () => {
  const { pod } = mkPod('forward');
  pod.beginStep(1 / 60, 0, 0, 0);
  pod.beginStep(1 / 60, 50, 0, 0); // 3000 in/s^2
  assert.ok(pod.slipFraction > 0);
  const { pod: p2 } = mkPod('forward');
  p2.beginStep(1 / 60, 50, 0, 0); p2.beginStep(1 / 60, 50, 0, 0);
  assert.equal(p2.slipFraction, 0);
});

test('reset zeros counters', () => {
  const { pod } = mkPod('forward', NOSLIP);
  drive(pod, 30, 0, 0, 0.2);
  pod.reset();
  assert.equal(pod.travelIn, 0); assert.equal(pod.groundTravelIn, 0); assert.equal(pod.count, 0);
});

test('Imu scaleError 0.01 gives heading 1.01x (unwrapped)', () => {
  const params = makeOdoParams('nominal', { imu: { scaleError: 0.01, noiseRadPerSqrtS: 0, biasWalkRadSPerSqrtS: 0 } });
  const imu = new Imu(params, makeRng(1));
  for (let i = 0; i < 600; i++) imu.advance(2, 1 / 60);
  near(imu.headingRad, 20 * 1.01, 1e-6);
});

test('Imu constant bias integrates; reset zeroes heading', () => {
  const params = makeOdoParams('nominal', { imu: { biasRadS: 0.01, noiseRadPerSqrtS: 0, biasWalkRadSPerSqrtS: 0 } });
  const imu = new Imu(params, makeRng(1));
  for (let i = 0; i < 100; i++) imu.advance(0, 0.01);
  near(imu.headingRad, 0.01, 1e-9);
  imu.reset(0);
  assert.equal(imu.headingRad, 0);
});

test('Imu quantRad quantizes heading', () => {
  const params = makeOdoParams('nominal', { imu: { quantRad: 0.01, noiseRadPerSqrtS: 0, biasWalkRadSPerSqrtS: 0 } });
  const imu = new Imu(params, makeRng(1));
  imu.advance(0.537, 0.1);
  const q = imu.headingRad / 0.01;
  near(q, Math.round(q), 1e-6);
});

test('Imu white noise: heading std grows ~ sqrt(t)', () => {
  const params = makeOdoParams('nominal', { imu: { noiseRadPerSqrtS: 0.01, biasWalkRadSPerSqrtS: 0 } });
  const h1 = [], h4 = [];
  for (let s = 1; s <= 200; s++) {
    const imu = new Imu(params, makeRng(s));
    for (let i = 0; i < 100; i++) imu.advance(0, 0.01);
    h1.push(imu.headingRad);
    for (let i = 0; i < 300; i++) imu.advance(0, 0.01);
    h4.push(imu.headingRad);
  }
  const r = std(h4) / std(h1);
  assert.ok(r > 1.6 && r < 2.5, `ratio ${r}`);
  near(std(h1), 0.01, 0.003);
});

test('pod travel noise: error std grows ~ sqrt(distance)', () => {
  const e1 = [], e4 = [];
  for (let s = 1; s <= 200; s++) {
    const params = makeOdoParams('nominal', { slip: { enabled: false }, noise: { travelNoiseInPerSqrtIn: 0.01 }, seed: s });
    const pod = new PodWheel('forward', params, makeRng(s));
    pod.beginStep(0.1, 10, 0, 0);
    for (let i = 0; i < 100; i++) pod.advance(0.01, 10, 0, 0);
    e1.push(pod.travelIn - pod.groundTravelIn);
    for (let i = 0; i < 300; i++) pod.advance(0.01, 10, 0, 0);
    e4.push(pod.travelIn - pod.groundTravelIn);
  }
  const r = std(e4) / std(e1);
  assert.ok(r > 1.6 && r < 2.5, `ratio ${r}`);
});

test('noise is deterministic for a given seed', () => {
  const run = () => {
    const params = makeOdoParams('nominal', { imu: { noiseRadPerSqrtS: 0.01 } });
    const imu = new Imu(params, makeRng(99));
    for (let i = 0; i < 50; i++) imu.advance(1, 0.01);
    return imu.headingRad;
  };
  assert.equal(run(), run());
});
