// GyroModel tests (imu-heading). Units: rad, rad/s, s. Error terms are isolated by starting from the 'ideal'
// preset and overriding one human-unit field at a time.
import test from 'node:test';
import assert from 'node:assert/strict';
import { GyroModel } from '../../web/js/physics/imu-heading/gyro.js';
import { resolveImuConfig, DEG } from '../../web/js/physics/imu-heading/params.js';
import { createRng } from '../../web/js/physics/imu-heading/rng.js';
import { wrapPi } from '../../web/js/physics/imu-heading/angles.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (tol ${e})`);

function mk(over = {}, seed = 1, preset = 'ideal') {
  const g = new GyroModel(resolveImuConfig(preset, over), createRng(seed));
  g.reset(0);
  return g;
}
// advance with constant true rate omega (rad/s) for seconds at frame dt
function spin(g, seconds, omega, dt = 0.01, ctx) {
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) g.advance(dt, omega, omega, ctx);
}

test('ideal preset: error is exactly zero and yaw equals wrapped true heading over random motion', () => {
  const g = mk();
  const r = createRng(11);
  const h0 = 0.4;
  g.setYaw(h0);
  let w0 = 0, h = h0;
  for (let i = 0; i < 2000; i++) {
    const w1 = (r.next() - 0.5) * 12; // rad/s
    g.advance(0.01, w0, w1);
    h += 0.5 * (w0 + w1) * 0.01;
    w0 = w1;
    const rd = g.read(g.time);
    assert.equal(rd.status, 'READY');
    near(rd.yaw, wrapPi(h), 1e-9);
    assert.ok(rd.yaw > -Math.PI && rd.yaw <= Math.PI);
  }
  assert.equal(g.getDiagnostics().errorRad, 0);
  const d = g.getDiagnostics();
  near(g.read(g.time).total, h0 + d.totalTrueRad, 1e-12);
  near(g.read(g.time).yaw, wrapPi(h0 + d.totalTrueRad), 1e-12);
});

test('status flow: NOT_READY before reset, READY after reset with no calibration', () => {
  const g = new GyroModel(resolveImuConfig('ideal'), createRng(1));
  assert.equal(g.status, 'NOT_READY');
  assert.equal(g.read(0).status, 'NOT_READY');
  g.reset(0);
  assert.equal(g.status, 'READY');
});

test('stationary with only an initial bias: error == b0 * t', () => {
  const g = mk({ biasInitSigmaDps: 0.5 }, 3);
  spin(g, 10, 0);
  const d = g.getDiagnostics();
  assert.ok(Math.abs(d.b0) > 1e-6, 'b0 should be drawn non-zero');
  near(d.errorRad, d.b0 * 10, 1e-9);
  near(g.time, 10, 1e-9);
});

test('ARW: std of the final error over 300 seeds ~ noiseDensity * sqrt(t)', () => {
  const t = 2; // s
  const over = { noiseDensityDpsRtHz: 0.05 };
  const cfg = resolveImuConfig('ideal', over);
  const errs = [];
  for (let s = 1; s <= 300; s++) {
    const g = mk(over, s);
    spin(g, t, 0);
    errs.push(g.getDiagnostics().errorRad);
  }
  const mean = errs.reduce((a, b) => a + b, 0) / errs.length;
  const sd = Math.sqrt(errs.reduce((a, b) => a + (b - mean) ** 2, 0) / (errs.length - 1));
  const expect = cfg.noiseDensityRad * Math.sqrt(t);
  assert.ok(Math.abs(sd - expect) / expect < 0.2, `sd ${sd} vs ${expect}`);
});

test('flywheel vibration multiplies the white-noise std', () => {
  const over = { noiseDensityDpsRtHz: 0.05, vibrationNoiseMult: 3 };
  const sdOf = (ctx) => {
    const errs = [];
    for (let s = 1; s <= 150; s++) {
      const g = mk(over, s);
      spin(g, 1, 0, 0.01, ctx);
      errs.push(g.getDiagnostics().errorRad);
    }
    const m = errs.reduce((a, b) => a + b, 0) / errs.length;
    return Math.sqrt(errs.reduce((a, b) => a + (b - m) ** 2, 0) / (errs.length - 1));
  };
  const ratio = sdOf({ linAccelG: 0, flywheelOn: true }) / sdOf({ linAccelG: 0, flywheelOn: false });
  assert.ok(Math.abs(ratio - 3) / 3 < 0.25, `ratio ${ratio}`);
});

test('scale error: 10 turns accumulate ~ scaleErr * 20*pi', () => {
  const g = mk({ scaleErrSigma: 0.01 }, 5);
  const eps = g.getDiagnostics().scaleErr;
  assert.ok(Math.abs(eps) > 1e-4);
  const omega = 2; // rad/s
  spin(g, (20 * Math.PI) / omega, omega); // 10 full turns
  const d = g.getDiagnostics();
  near(d.totalTrueRad, 20 * Math.PI, 0.05);
  near(d.errorRad, eps * d.totalTrueRad, 1e-6);
  near(d.errorRad, eps * 20 * Math.PI, 0.01 * Math.abs(eps) * 20 * Math.PI + 1e-3);
});

test('temperature drift follows tempCoeff * dT ramp', () => {
  const g = mk({ tempCoeffDpsPerK: 0.005, tempRiseK: 3, tempTauS: 600 });
  const t = 60, tau = 600;
  spin(g, t, 0);
  const expect = 0.005 * DEG * 3 * (t - tau * (1 - Math.exp(-t / tau)));
  const got = g.getDiagnostics().errorRad;
  assert.ok(Math.abs(got - expect) / expect < 0.02, `${got} vs ${expect}`);
});

test('saturation: rate above full scale clips, counts events and loses angle', () => {
  const g = mk({ fullScaleDps: 100 });
  const fs = 100 * DEG;
  spin(g, 1, 5); // 5 rad/s >> 1.745 rad/s
  const d = g.getDiagnostics();
  assert.ok(d.satEvents > 0);
  near(d.totalMeasRad, fs * 1, 1e-6);
  near(d.errorRad, -(5 - fs) * 1, 1e-6);
});

test('shock with a peak rate above full scale subtracts the clipped angle', () => {
  const g = mk({ fullScaleDps: 100 });
  const fs = 100 * DEG;
  g.shock({ impactSpeedInS: 40, peakRateRad: 10, durationS: 0.1 });
  const d = g.getDiagnostics();
  assert.equal(d.satEvents, 1);
  near(d.errorRad, -(10 - fs) * 0.1, 1e-9); // ideal shockSigma = 0, so only the clip loss remains
  const g2 = mk({ fullScaleDps: 100 });
  g2.shock({ impactSpeedInS: 40, peakRateRad: -10, durationS: 0.1 });
  near(g2.getDiagnostics().errorRad, +(10 - fs) * 0.1, 1e-9);
});

test('shock heading jump scales with impact speed', () => {
  const g = mk({ shockSigmaDegPerMps: 5 }, 8);
  g.shock({ impactSpeedInS: 39.37 }); // 1 m/s
  const e1 = Math.abs(g.getDiagnostics().errorRad);
  assert.ok(e1 > 0 && e1 < 5 * DEG * 6, `jump ${e1}`);
  const g0 = mk({ shockSigmaDegPerMps: 5 }, 8);
  g0.shock({ impactSpeedInS: 0 });
  assert.equal(g0.getDiagnostics().errorRad, 0);
});

test('latency: at constant omega the reported yaw lags by omega * latency', () => {
  const lat = 0.05, omega = 1.5;
  const g = mk({ latencyS: lat });
  g.setYaw(0);
  spin(g, 2, omega);
  const rd = g.read(g.time);
  near(rd.total, omega * (2 - lat), 1e-9);
  near(rd.rate, omega, 1e-9);
  near(rd.t, 2 - lat, 1e-9);
  // reading further in the past (but within the buffer) interpolates linearly
  near(g.read(g.time - 0.5).total, omega * (2 - 0.5 - lat), 1e-9);
});

test('heading LSB quantization (revBno055 style 0.0625 deg)', () => {
  const lsb = 0.0625 * DEG;
  const g = mk({ headingLsbDeg: 0.0625 });
  g.setYaw(0.3333);
  const rd = g.read(g.time);
  assert.ok(Math.abs(rd.yaw - rd.total) <= lsb / 2 + 1e-12);
  near(rd.yaw / lsb, Math.round(rd.yaw / lsb), 1e-6);
  const real = new GyroModel(resolveImuConfig('revBno055'), createRng(2));
  real.reset(0);
  spin(real, 0.5, 0.2);
  const r2 = real.read(real.time);
  if (r2.status === 'READY') near(r2.yaw / lsb, Math.round(r2.yaw / lsb), 1e-6);
});

test('resetYaw gives yaw 0 and keeps integrating; setYaw sets the reported heading', () => {
  const g = mk();
  spin(g, 1, 1);
  g.resetYaw();
  near(g.read(g.time).yaw, 0, 1e-12);
  spin(g, 0.5, 1);
  near(g.read(g.time).yaw, 0.5, 1e-9);
  g.setYaw(2.0);
  near(g.read(g.time).yaw, 2.0, 1e-12);
  spin(g, 1, 1);
  near(g.read(g.time).yaw, wrapPi(3.0), 1e-9);
  g.setYaw(3.0);
  spin(g, 1, 1); // 4.0 rad wraps past pi
  const rd = g.read(g.time);
  near(rd.total, 4.0, 1e-9);
  near(rd.yaw, wrapPi(4.0), 1e-9);
});

test('calibration while spinning bakes the mean true rate in as bias', () => {
  const omega = 1; // rad/s
  const g = mk({ calibDurationS: 0.5 });
  assert.equal(g.status, 'CALIBRATING');
  g.setYaw(0);
  const frozen = g.read(g.time);
  assert.equal(frozen.status, 'CALIBRATING');
  assert.equal(frozen.rate, 0);
  spin(g, 0.6, omega);
  assert.equal(g.status, 'READY');
  near(Math.abs(g.getDiagnostics().b0), omega, 1e-6);
});

test('calibration while still leaves only the small residual bias', () => {
  const g = mk({ calibDurationS: 0.25, biasInitSigmaDps: 0.004 }, 4);
  spin(g, 0.5, 0);
  assert.equal(g.status, 'READY');
  assert.ok(Math.abs(g.getDiagnostics().b0) < 6 * 0.004 * DEG);
});

test('startCalibration re-enters CALIBRATING then READY', () => {
  const g = mk({ calibDurationS: 0.2 });
  spin(g, 0.5, 0);
  assert.equal(g.status, 'READY');
  g.startCalibration();
  assert.equal(g.status, 'CALIBRATING');
  spin(g, 0.5, 0);
  assert.equal(g.status, 'READY');
});

test('same seed gives identical diagnostics; different seeds differ (pinpoint)', () => {
  const run = (seed) => { const g = mk({}, seed, 'pinpoint'); spin(g, 3, 1.2, 0.01, { linAccelG: 0.1, flywheelOn: true }); return g.getDiagnostics(); };
  assert.deepEqual(run(5), run(5));
  assert.notDeepEqual(run(5), run(6));
});

test('substep count is independent of dt split for the deterministic terms', () => {
  const a = mk({ scaleErrSigma: 0.01 }, 2), b = mk({ scaleErrSigma: 0.01 }, 2);
  spin(a, 1, 2, 0.01);
  spin(b, 1, 2, 0.05);
  near(a.getDiagnostics().errorRad, b.getDiagnostics().errorRad, 1e-9);
});
