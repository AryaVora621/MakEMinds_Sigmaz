// Tests for pinpoint.js (P5), driven by stub sensors. Device units: mm, rad, s.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeOdoParams } from '../../web/js/physics/pinpoint-odometry/params.js';
import { PinpointDevice } from '../../web/js/physics/pinpoint-odometry/pinpoint.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} got ${a} want ${b}`);

function mkDev(preset = 'nominal', overrides = {}, init = {}) {
  const params = makeOdoParams(preset, overrides);
  const fwd = { count: init.fwd ?? 0 }, str = { count: init.str ?? 0 }, imu = { headingRad: 0 };
  const dev = new PinpointDevice(params, { fwdPod: fwd, strPod: str, imu });
  return { params, fwd, str, imu, dev };
}
const ZERO_OFFS = { config: { forwardPodYIn: 0, strafePodXIn: 0 } };

test('integrates a 4-leg square back to the origin', () => {
  const { params, fwd, imu, dev } = mkDev('nominal', ZERO_OFFS);
  const tpm = params.config.ticksPerMm;
  let t = 0, ticks = 0;
  const leg = 500; // mm
  for (let i = 0; i < 4; i++) {
    ticks += Math.round(leg * tpm);
    fwd.count = ticks;
    t += 0.1; dev.tick(t);
    imu.headingRad += Math.PI / 2;
    t += 0.1; dev.tick(t);
    if (i === 0) { const s = dev.snapshot(); near(s.xMm, leg, 0.2); near(s.yMm, 0, 0.2); }
  }
  const s = dev.snapshot();
  near(s.xMm, 0, 0.5); near(s.yMm, 0, 0.5);
  near(Math.abs(s.hRad), 0, 1e-9);
});

test('encoder offsets: spin in place with wrong offsets shows translation', () => {
  // zero offsets configured while the pods are physically offset: spurious translation per radian
  const { params, fwd, imu, dev } = mkDev('nominal', ZERO_OFFS);
  const tpm = params.config.ticksPerMm;
  // forward pod at y=-0.63 in spins: dF = -om*py*dt  -> +0.63 in per rad
  const rad = 0.5, pyMm = -0.63 * 25.4;
  fwd.count = Math.round(-pyMm * rad * tpm);
  imu.headingRad = rad;
  dev.tick(0.01);
  assert.ok(dev.snapshot().xMm > 0); // translation appears (that is the error)
});

test('int32 wraparound count delta works', () => {
  const start = 2147483600;
  const { params, fwd, dev } = mkDev('nominal', ZERO_OFFS, { fwd: start });
  fwd.count = (start + 100) | 0; // wraps negative
  assert.ok(fwd.count < 0);
  dev.tick(0.01);
  near(dev.snapshot().xMm, 100 / params.config.ticksPerMm, 1e-9);
});

test('setPosition rebases pose, heading and encoder baselines', () => {
  const { fwd, imu, dev } = mkDev('nominal', ZERO_OFFS);
  fwd.count = 12345; imu.headingRad = 5;
  dev.setPosition({ xMm: 100, yMm: 200, hRad: 1 });
  dev.tick(0.01);
  const s = dev.snapshot();
  near(s.xMm, 100, 1e-9); near(s.yMm, 200, 1e-9); near(s.hRad, 1, 1e-9);
  assert.equal(s.vxMm, 0);
});

test('velocity at rest is exactly 0', () => {
  const { dev } = mkDev('nominal', ZERO_OFFS);
  for (let i = 1; i <= 100; i++) dev.tick(i / 1500);
  const s = dev.snapshot();
  assert.equal(s.vxMm, 0); assert.equal(s.vyMm, 0); assert.equal(s.wRadS, 0);
});

test('reports loop time and frequency', () => {
  const { dev } = mkDev('nominal', ZERO_OFFS);
  dev.tick(1 / 1500); dev.tick(2 / 1500);
  const s = dev.snapshot();
  near(s.frequencyHz, 1500, 1);
  near(s.loopTimeUs, 1e6 / 1500, 1);
});

function velStd(n) {
  const { params, fwd, dev } = mkDev('nominal', { ...ZERO_OFFS, firmware: { velWindowSamples: n } });
  const vMmS = 30 * 25.4; // 30 in/s
  const perTick = vMmS * params.config.ticksPerMm / 1500;
  let acc = 0;
  const vs = [];
  for (let i = 1; i <= 3000; i++) {
    acc += perTick; fwd.count = Math.floor(acc);
    dev.tick(i / 1500);
    if (i > 100) vs.push(dev.snapshot().vxMm);
  }
  const m = vs.reduce((s, x) => s + x, 0) / vs.length;
  return { sd: Math.sqrt(vs.reduce((s, x) => s + (x - m) ** 2, 0) / vs.length), mean: m, vMmS };
}

test('velocity std under quantization shrinks as velWindowSamples grows', () => {
  const a = velStd(1), b = velStd(4), c = velStd(15);
  assert.ok(a.sd > b.sd && b.sd > c.sd, `${a.sd} ${b.sd} ${c.sd}`);
  near(c.mean, c.vMmS, 0.02 * c.vMmS);
});

test('resetPosAndIMU sets NOT_READY for resetDurationS', () => {
  const { params, dev } = mkDev('nominal', ZERO_OFFS);
  dev.tick(0.01);
  dev.resetPosAndIMU(1.0);
  assert.equal(dev.status, 'NOT_READY');
  dev.tick(1.0 + params.firmware.resetDurationS * 0.5);
  assert.equal(dev.status, 'NOT_READY');
  dev.tick(1.0 + params.firmware.resetDurationS + 0.01);
  assert.equal(dev.status, 'READY');
  near(dev.snapshot().xMm, 0, 1e-9);
});

test('history is oldest-first, capped, and a copy', () => {
  const { dev } = mkDev('nominal', { ...ZERO_OFFS, firmware: { historyLen: 10 } });
  for (let i = 1; i <= 25; i++) dev.tick(i * 0.001);
  const h = dev.history();
  assert.equal(h.length, 10);
  assert.ok(h[0].tS < h[9].tS);
  h.length = 0;
  assert.equal(dev.history().length, 10);
  dev.clearHistory();
  assert.equal(dev.history().length, 0);
});
