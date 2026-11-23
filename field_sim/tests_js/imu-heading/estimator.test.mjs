// OdometryModel, HeadingKF, sampleVision and PoseEstimator tests (imu-heading). Units: in, s, rad.
import test from 'node:test';
import assert from 'node:assert/strict';
import { OdometryModel, HeadingKF, makeHeadingKF, sampleVision, PoseEstimator } from '../../web/js/physics/imu-heading/estimator.js';
import {
  resolveImuConfig, resolveOdometryConfig, resolveVisionConfig, resolveEstimatorConfig, DEG,
} from '../../web/js/physics/imu-heading/params.js';
import { createRng } from '../../web/js/physics/imu-heading/rng.js';
import { wrapPi } from '../../web/js/physics/imu-heading/angles.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (tol ${e})`);

const NO_ODO_ERR = { podScaleSigma: 0, podNoiseInPerRtIn: 0, podSlipSigmaIn: 0, ticksPerIn: 0 };

function makeEst({ imu = {}, imuPreset = 'ideal', est = {}, odo = NO_ODO_ERR, vision = {}, seed = 1 } = {}) {
  return new PoseEstimator({
    imuCfg: resolveImuConfig(imuPreset, imu),
    estimatorCfg: resolveEstimatorConfig(est),
    odometryCfg: resolveOdometryConfig(odo),
    visionCfg: resolveVisionConfig(vision),
    rng: createRng(seed).fork('odo'),
  });
}
const ready = (total, t = 0) => ({ yaw: wrapPi(total), total, rate: 0, t, status: 'READY' });

test('truth mode is a pass-through of setTruePose', () => {
  const e = makeEst({ est: { mode: 'truth' } });
  e.reset({ x: 0, y: 0, h: 0 }, 0);
  e.setTruePose({ x: 12, y: -7, h: 2.5 });
  e.step(0.01, 0.01, { f: 100, l: 100 }, ready(99), { collision: false, measureOdometry: true });
  const p = e.getPose();
  assert.equal(p.x, 12); assert.equal(p.y, -7); near(p.h, 2.5, 1e-15);
  e.setTruePose({ x: 1, y: 2, h: -3 });
  assert.deepEqual(e.getPose(), { x: 1, y: 2, h: -3 });
});

test('null reading is also a pass-through (pose held)', () => {
  const e = makeEst();
  e.reset({ x: 5, y: 6, h: 0.3 }, 0);
  e.step(0.01, 0.01, { f: 3, l: 0 }, null);
  near(e.getPose().x, 5, 1e-15); near(e.getPose().y, 6, 1e-15);
});

test('ideal gyro + noiseless odometry: a 1 m square returns to the start within 1e-6', () => {
  const e = makeEst();
  e.reset({ x: 0, y: 0, h: 0 }, 0);
  const side = 39.37; // in (1 m)
  const N = 100;
  let t = 0, hTrue = 0;
  e.step(0.01, t, { f: 0, l: 0 }, ready(0, t)); // anchor
  for (let k = 0; k < 4; k++) {
    for (let i = 0; i < N; i++) { t += 0.01; e.step(0.01, t, { f: side / N, l: 0 }, ready(hTrue, t)); }
    for (let i = 0; i < N; i++) { hTrue += Math.PI / 2 / N; t += 0.01; e.step(0.01, t, { f: 0, l: 0 }, ready(hTrue, t)); }
  }
  const p = e.getPose();
  near(p.x, 0, 1e-6); near(p.y, 0, 1e-6);
  near(wrapPi(p.h), 0, 1e-9);
  near(e.getHeadingUnwrapped(), 2 * Math.PI, 1e-9);
});

test('straight drive at a non-zero heading goes the right way', () => {
  const e = makeEst();
  e.reset({ x: 0, y: 0, h: Math.PI / 2 }, 0);
  const total0 = Math.PI / 2;
  e.step(0.01, 0, { f: 0, l: 0 }, ready(total0, 0));
  for (let i = 1; i <= 100; i++) e.step(0.01, i * 0.01, { f: 0.5, l: 0 }, ready(total0, i * 0.01));
  near(e.getPose().x, 0, 1e-9); near(e.getPose().y, 50, 1e-9);
});

test('CALIBRATING status holds the pose and does not predict', () => {
  const e = makeEst();
  e.reset({ x: 1, y: 2, h: 0 }, 0);
  e.step(0.01, 0.01, { f: 0, l: 0 }, ready(0, 0.01));
  e.step(0.01, 0.02, { f: 10, l: 5 }, { yaw: 0, total: 0, rate: 0, t: 0.02, status: 'CALIBRATING' });
  assert.deepEqual(e.getPose(), { x: 1, y: 2, h: 0 });
  e.step(0.01, 0.03, { f: 10, l: 0 }, { yaw: 0, total: 0, rate: 0, t: 0.03, status: 'NOT_READY' });
  assert.deepEqual(e.getPose(), { x: 1, y: 2, h: 0 });
  e.step(0.01, 0.04, { f: 10, l: 0 }, ready(0, 0.04));
  near(e.getPose().x, 11, 1e-9);
});

test('heading error after 30 s is smaller with vision than without (biased gyro)', () => {
  const imu = { biasInitSigmaDps: 0.5, noiseDensityDpsRtHz: 0.005 };
  const biasRad = 0.5 * DEG; // rad/s, injected into the synthetic reading
  const run = (withVision) => {
    const e = makeEst({ imu, vision: { enabled: withVision } });
    const vc = resolveVisionConfig({ enabled: true });
    const vr = createRng(77);
    e.reset({ x: 0, y: 0, h: 0 }, 0);
    e.step(0.01, 0, { f: 0, l: 0 }, ready(0, 0));
    for (let i = 1; i <= 3000; i++) {
      const t = i * 0.01;
      e.step(0.01, t, { f: 0, l: 0 }, ready(biasRad * t, t)); // true heading is 0, gyro drifts
      if (withVision && i % 10 === 0) e.addVisionPose(sampleVision({ x: 0, y: 0, h: 0 }, vc, vr, t));
    }
    return Math.abs(wrapPi(e.getPose().h));
  };
  const noVis = run(false), vis = run(true);
  near(noVis, 0.5 * DEG * 30, 1e-6); // 15 deg of uncorrected drift
  assert.ok(vis < noVis, `${vis} !< ${noVis}`);
  assert.ok(vis < 0.2 * noVis && vis < 2 * DEG, `vision heading error ${vis / DEG} deg`);
});

test('latency-compensated vision uses the history, not the current pose', () => {
  const e = makeEst({ odo: { ...NO_ODO_ERR, podNoiseInPerRtIn: 0.003 } }); // pod noise > 0 gives Pxy > 0
  e.reset({ x: 0, y: 0, h: 0 }, 0);
  e.step(0.01, 0, { f: 0, l: 0 }, ready(0, 0));
  let stamped = null;
  for (let i = 1; i <= 200; i++) {
    const t = i * 0.01;
    e.step(0.01, t, { f: 0.1, l: 0 }, ready(0, t)); // ~10 in/s
    if (i === 100) stamped = { t, x: e.getPose().x, y: e.getPose().y };
  }
  const before = e.getPose();
  const pxxBefore = e.getCovariance().pxx;
  assert.ok(before.x - stamped.x > 5, 'robot moved on since the measurement time');
  // measurement taken 1 s ago, equal to what the estimator believed at that time, tiny sigma -> alpha ~ 1
  const ok = e.addVisionPose({ x: stamped.x, y: stamped.y, h: 0, t: stamped.t, sigmaXY: 1e-4, sigmaH: 0.05 });
  assert.equal(ok, true);
  const after = e.getPose();
  near(after.x, before.x, 1e-6); // a naive (uncompensated) update would have pulled x back ~10 in
  near(after.y, before.y, 1e-6);
  assert.ok(e.getCovariance().pxx < pxxBefore, 'position variance shrinks after a fused fix');
  // a measurement outside the history ring is rejected
  assert.equal(e.addVisionPose({ x: 0, y: 0, h: 0, t: -50, sigmaXY: 1, sigmaH: 0.05 }), false);
});

test('vision position fix pulls an offset pose toward the measurement', () => {
  const e = makeEst({ odo: { ...NO_ODO_ERR, podNoiseInPerRtIn: 0.05 } });
  e.reset({ x: 10, y: 10, h: 0 }, 0);
  e.step(0.01, 0, { f: 0, l: 0 }, ready(0, 0));
  for (let i = 1; i <= 100; i++) e.step(0.01, i * 0.01, { f: 0.1, l: 0 }, ready(0, i * 0.01));
  const p0 = e.getPose();
  assert.equal(e.addVisionPose({ x: p0.x + 4, y: p0.y, h: 0, t: 1.0, sigmaXY: 0.5, sigmaH: 0.05 }), true);
  const p1 = e.getPose();
  assert.ok(p1.x > p0.x && p1.x <= p0.x + 4 + 1e-9, `${p0.x} -> ${p1.x}`);
});

test('HeadingKF: predict applies bias, update shrinks variance and learns the bias', () => {
  const kf = new HeadingKF({ qc: 0, qb: 0, p0b: 1e-4 });
  assert.equal(kf.correction, 0);
  kf.predict(1);
  near(kf.P[1][1], 1e-4, 1e-15);
  near(kf.P[0][0], 1e-4, 1e-15); // F P F^T = p00 - 2*dt*p01 + dt^2*p11
  // gyro reads +b too high; measurement says heading error accumulates -> the filter should estimate beta ~ b
  const b = 0.01; // rad/s
  const sig = 0.002;
  let t = 0;
  for (let i = 0; i < 200; i++) {
    kf.predict(0.1); t += 0.1;
    const hGyro = b * t, hTrue = 0;
    kf.update(hTrue - (hGyro + kf.correction), sig);
  }
  near(kf.biasEst, b, 0.1 * b);
  assert.ok(kf.P[0][0] < 1e-4);
});

test('makeHeadingKF seeds the filter from the gyro config', () => {
  const c = resolveImuConfig('pinpoint');
  const kf = makeHeadingKF(c);
  near(kf.P[1][1], c.biasInitSigmaRad ** 2, 1e-18);
  near(kf.P[0][0], 0, 1e-18);
  kf.predict(1);
  assert.ok(kf.P[0][0] > 0);
});

test('OdometryModel: noiseless is exact in total; scale error and tick quantization carry the remainder', () => {
  const m = new OdometryModel(resolveOdometryConfig({ ...NO_ODO_ERR, ticksPerIn: 505.3 }), createRng(1));
  let sf = 0;
  for (let i = 0; i < 1000; i++) sf += m.measure(0.1, 0).df;
  near(sf, 100, 1 / 505.3 + 1e-9); // within one tick: remainder is carried, not dropped
  const sc = new OdometryModel(resolveOdometryConfig({ ...NO_ODO_ERR, podScaleSigma: 0.05 }), createRng(3));
  const { sF } = sc.scale;
  assert.notEqual(sF, 0);
  near(sc.measure(10, 0).df, 10 * (1 + sF), 1e-9);
  const m1 = new OdometryModel(resolveOdometryConfig(), createRng(9)), m2 = new OdometryModel(resolveOdometryConfig(), createRng(9));
  assert.deepEqual(m1.measure(1, 0.5), m2.measure(1, 0.5));
});

test('OdometryModel: collision adds slip noise', () => {
  const cfg = resolveOdometryConfig({ ...NO_ODO_ERR, podSlipSigmaIn: 0.5 });
  const m = new OdometryModel(cfg, createRng(4));
  let s = 0;
  for (let i = 0; i < 200; i++) { const r = m.measure(0, 0, { collision: true }); s += Math.abs(r.df) + Math.abs(r.dl); }
  assert.ok(s > 10, `slip total ${s}`);
  const q = new OdometryModel(cfg, createRng(4));
  for (let i = 0; i < 200; i++) { const r = q.measure(0, 0, { collision: false }); assert.equal(r.df, 0); assert.equal(r.dl, 0); }
});

test('sampleVision: unbiased around truth with configured sigma', () => {
  const vc = resolveVisionConfig({ enabled: true });
  const r = createRng(5);
  let sx = 0, sh = 0, sx2 = 0;
  const n = 4000;
  for (let i = 0; i < n; i++) {
    const m = sampleVision({ x: 10, y: 20, h: 0.5 }, vc, r, 1.25);
    sx += m.x - 10; sx2 += (m.x - 10) ** 2; sh += m.h - 0.5;
    assert.equal(m.t, 1.25);
    assert.equal(m.sigmaXY, vc.posSigmaIn);
    assert.equal(m.sigmaH, vc.headingSigmaRad);
  }
  assert.ok(Math.abs(sx / n) < 0.1 && Math.abs(sh / n) < 0.01);
  near(Math.sqrt(sx2 / n), vc.posSigmaIn, 0.1 * vc.posSigmaIn);
});

test('estimator never needs the true heading: same readings give the same pose', () => {
  const run = () => {
    const e = makeEst({ odo: undefined, seed: 3 });
    e.reset({ x: 0, y: 0, h: 0 }, 0);
    for (let i = 0; i <= 100; i++) e.step(0.01, i * 0.01, { f: 0.3, l: 0.1 }, ready(0.01 * i, i * 0.01));
    return e.getPose();
  };
  assert.deepEqual(run(), run());
});
