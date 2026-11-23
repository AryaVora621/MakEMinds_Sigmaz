// ImuHeadingSystem + adapters tests (imu-heading). Units: in, s, rad.
import test from 'node:test';
import assert from 'node:assert/strict';
import { ImuHeadingSystem } from '../../web/js/physics/imu-heading/system.js';
import {
  fieldCentricExtraOffset, turretAimError, toPedroPose, fromPedroPose,
} from '../../web/js/physics/imu-heading/adapters.js';
import { se2Exp, wrapPi, angDiff } from '../../web/js/physics/imu-heading/angles.js';
import { MecanumDrive } from '../../web/js/driver.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (tol ${e})`);

const ZERO_ODO = { podScaleSigma: 0, podNoiseInPerRtIn: 0, podSlipSigmaIn: 0, ticksPerIn: 0 };

// Drives the TRUE pose with se2Exp and feeds the system once per frame.
function makeRig(opts, start = { x: 20, y: 30, h: 0.3 }) {
  const sys = new ImuHeadingSystem(opts);
  const st = { x: start.x, y: start.y, h: wrapPi(start.h) };
  sys.reset(st, 0);
  const rig = {
    sys, st, t: 0,
    run(seconds, vf, vl, om, { dt = 0.01, events = {}, onFrame } = {}) {
      const n = Math.round(seconds / dt);
      for (let i = 0; i < n; i++) {
        const p = se2Exp(st, vf * dt, vl * dt, om * dt);
        st.x = p.x; st.y = p.y; st.h = wrapPi(p.h);
        rig.t += dt;
        const ev = typeof events === 'function' ? events(i) : events;
        sys.step(dt, st, { collisions: [], flywheelOn: false, visionVisible: false, ...ev });
        if (onFrame) onFrame(i);
      }
    },
  };
  return rig;
}

test('enabled=false: the estimate equals the truth exactly', () => {
  const rig = makeRig({ preset: 'pinpoint', seed: 3, enabled: false });
  rig.run(2, 40, 10, 1.3);
  rig.run(1, 0, 0, 0);
  const e = rig.sys.getEstimatedPose(), tr = rig.st;
  assert.equal(e.x, tr.x); assert.equal(e.y, tr.y); assert.equal(e.h, tr.h);
  assert.equal(rig.sys.getHeadingError(), 0);
  assert.equal(rig.sys.getPositionError(), 0);
});

test("estimator mode 'truth' also returns the truth", () => {
  const rig = makeRig({ preset: 'pinpoint', seed: 3, estimatorOverrides: { mode: 'truth' } });
  rig.run(2, 40, 0, 2);
  const e = rig.sys.getEstimatedPose();
  assert.equal(e.x, rig.st.x); assert.equal(e.y, rig.st.y); near(e.h, rig.st.h, 1e-15);
});

test('ideal preset (no pod error): estimate tracks the truth', () => {
  const rig = makeRig({ preset: 'ideal', seed: 1, odometryOverrides: ZERO_ODO });
  rig.run(3, 40, 15, 1.7);
  rig.run(0.2, 0, 0, 0);
  assert.ok(Math.abs(rig.sys.getHeadingError()) < 1e-6, `heading err ${rig.sys.getHeadingError()}`);
  assert.ok(rig.sys.getPositionError() < 0.05, `pos err ${rig.sys.getPositionError()} in`);
});

test('ideal preset: heading is exact DURING a rate step too (no frame-ramp or anchor loss)', () => {
  // Regression: a 0 -> 1.7 rad/s step at dt = 0.01 s used to show -0.0085 rad (half-frame ramp) in the gyro
  // and -0.0085..-0.017 rad (lost first-frame increment) in the estimate while still turning.
  const rig = makeRig({ preset: 'ideal', seed: 1, odometryOverrides: ZERO_ODO });
  let worst = 0;
  rig.run(1, 40, 15, 1.7, { onFrame: () => { worst = Math.max(worst, Math.abs(rig.sys.getHeadingError())); } });
  assert.ok(worst < 1e-9, `worst heading err while turning ${worst} rad`);
  near(angDiff(rig.sys.getGyroReading().yaw, rig.st.h), 0, 1e-9);
});

test('non-ideal preset: gyro true-rate integral equals the true rotation exactly', () => {
  // The sensor errors (scale, bias, noise, LSB, ...) must be the only gyro error; the true-rate integral the
  // model uses (totalTrueRad) must equal the actual unwrapped truth rotation, sum of angDiff per frame.
  const rig = makeRig({ preset: 'pinpoint', seed: 9, odometryOverrides: ZERO_ODO });
  const h0 = rig.st.h;
  let unwrapped = 0, prev = h0;
  const om = (i) => (i % 50 < 25 ? 2.5 : -1.0); // rad/s, square wave rate steps every 0.25 s
  for (let i = 0; i < 300; i++) {
    rig.run(0.01, 0, 0, om(i));
    unwrapped += angDiff(rig.st.h, prev); prev = rig.st.h;
  }
  const d = rig.sys.getTelemetry().gyro;
  near(d.totalTrueRad, unwrapped, 1e-9);
  assert.ok(Math.abs(d.errorRad) > 0, 'pinpoint sensor errors still present');
  assert.ok(Math.abs(d.errorRad) < 2 * Math.PI / 180, `pinpoint gyro error ${d.errorRad} rad over 3 s`);
});

test('pinpoint preset: gyro bias makes the heading estimate drift, slowly', () => {
  const rig = makeRig({ preset: 'pinpoint', seed: 12, odometryOverrides: ZERO_ODO });
  rig.run(30, 0, 0, 0);
  const err = Math.abs(rig.sys.getHeadingError());
  assert.ok(err > 0, 'some error is expected');
  assert.ok(err < 3 * Math.PI / 180, `heading error ${err} rad is implausibly large for a Pinpoint-class gyro over 30 s`);
});

test('same seed gives identical telemetry; a different seed differs', () => {
  const run = (seed) => {
    const rig = makeRig({ preset: 'pinpoint', seed });
    rig.run(5, 30, 5, 1.1, { events: { flywheelOn: true } });
    return JSON.stringify(rig.sys.getTelemetry());
  };
  assert.equal(run(4), run(4));
  assert.notEqual(run(4), run(5));
});

test('host gating: the estimate only changes at 1/hostHz', () => {
  const hostHz = 10;
  const rig = makeRig({ preset: 'pinpoint', seed: 2, hostHz, odometryOverrides: { podNoiseInPerRtIn: 0, podSlipSigmaIn: 0, podScaleSigma: 0 } });
  let last = rig.sys.getEstimatedPose();
  let changes = 0;
  const sec = 2;
  rig.run(sec, 30, 0, 0, {
    onFrame: () => {
      const p = rig.sys.getEstimatedPose();
      if (p.x !== last.x || p.y !== last.y || p.h !== last.h) changes++;
      last = p;
    },
  });
  const expected = sec * hostHz;
  assert.ok(changes >= expected - 2 && changes <= expected + 1, `changes ${changes}, expected ~${expected}`);
  // and at hostHz = 100 with a 100 Hz frame loop it changes (nearly) every frame
  const fast = makeRig({ preset: 'pinpoint', seed: 2, hostHz: 100, odometryOverrides: ZERO_ODO });
  let l2 = fast.sys.getEstimatedPose(), c2 = 0;
  fast.run(1, 30, 0, 0, { onFrame: () => { const p = fast.sys.getEstimatedPose(); if (p.x !== l2.x) c2++; l2 = p; } });
  assert.ok(c2 >= 90, `fast changes ${c2}`);
});

test('a collision raises satEvents or changes the gyro error', () => {
  const base = { preset: 'pinpoint', seed: 6, imuOverrides: { shockSigmaDegPerMps: 5 } };
  const a = makeRig(base), b = makeRig(base);
  a.run(1, 0, 0, 0);
  b.run(1, 0, 0, 0);
  b.run(0.02, 0, 0, 0, { events: { collisions: [{ impactSpeedInS: 40, peakRateRad: 100, durationS: 0.05 }] } });
  a.run(0.02, 0, 0, 0);
  const ga = a.sys.getTelemetry().gyro, gb = b.sys.getTelemetry().gyro;
  assert.ok(gb.satEvents > ga.satEvents || Math.abs(gb.errorRad - ga.errorRad) > 1e-6,
    `sat ${ga.satEvents}->${gb.satEvents}, err ${ga.errorRad}->${gb.errorRad}`);
});

test('telemetry shape and consistency', () => {
  const rig = makeRig({ preset: 'pinpoint', seed: 8 });
  rig.run(1, 20, 0, 0.5);
  const T = rig.sys.getTelemetry();
  for (const k of ['t', 'truePose', 'estPose', 'headingErrDeg', 'posErrIn', 'gyro', 'cov', 'status']) assert.ok(k in T, k);
  near(T.headingErrDeg, rig.sys.getHeadingError() * 180 / Math.PI, 1e-9);
  near(T.posErrIn, rig.sys.getPositionError(), 1e-9);
  near(rig.sys.getHeadingError(), angDiff(rig.sys.getEstimatedPose().h, rig.sys.getTruePose().h), 1e-12);
  assert.equal(rig.sys.getGyroReading().status, 'READY');
  assert.ok(rig.sys.config && rig.sys.gyro && rig.sys.estimator);
});

test('reset with skipCalibration leaves the gyro READY at the true heading', () => {
  const sys = new ImuHeadingSystem({ preset: 'otos', seed: 1 }); // 0.612 s calibration if not skipped
  sys.reset({ x: 0, y: 0, h: 1.0 }, 0, { skipCalibration: true });
  assert.equal(sys.getGyroReading().status, 'READY');
  near(sys.getGyroReading().yaw, 1.0, 1e-3); // small residual: gyro ran through the skipped calibration window
  const s2 = new ImuHeadingSystem({ preset: 'otos', seed: 1 });
  s2.reset({ x: 0, y: 0, h: 1.0 }, 0, { skipCalibration: false });
  assert.equal(s2.getGyroReading().status, 'CALIBRATING');
});

test('field-centric: true heading + extra offset equals driving with the estimated heading', () => {
  const hTrue = 0.9, hEst = 0.75; // rad
  const extra = fieldCentricExtraOffset(hTrue, hEst);
  near(extra, -angDiff(hEst, hTrue), 1e-15);
  near(extra, hTrue - hEst, 1e-12);
  for (const cmd of [{ fwd: 1, strafe: 0, turn: 0 }, { fwd: 0.4, strafe: -0.8, turn: 0.2 }, { fwd: -0.3, strafe: 0.6, turn: -0.5 }]) {
    const d1 = new MecanumDrive(), d2 = new MecanumDrive();
    const s1 = { x: 0, y: 0, h: hTrue, bodyV: [0, 0, 0] };
    const s2 = { x: 0, y: 0, h: hEst, bodyV: [0, 0, 0] };
    d1.step(cmd, 0.02, s1, { fieldCentric: true, headingOffset: extra });
    d2.step(cmd, 0.02, s2, { fieldCentric: true, headingOffset: 0 });
    for (let i = 0; i < 3; i++) near(s1.bodyV[i], s2.bodyV[i], 1e-9);
  }
  // also across the +-pi seam
  const e2 = fieldCentricExtraOffset(Math.PI - 0.05, -Math.PI + 0.05);
  near(e2, -0.1, 1e-12); // estimate is 0.1 rad ccw of truth across the seam -> offset -0.1
});

test('system getDriverHeadingOffset matches the adapter', () => {
  const rig = makeRig({ preset: 'pinpoint', seed: 21, imuOverrides: { biasInitSigmaDps: 0.5 } });
  rig.run(20, 0, 0, 0);
  near(rig.sys.getDriverHeadingOffset(), fieldCentricExtraOffset(rig.st.h, rig.sys.getEstimatedPose().h), 1e-12);
});

test('turretAimError is zero when est == true, and equals the heading error otherwise', () => {
  const p = { x: 30, y: 40, h: 0.6 };
  const tgt = { x: 120, y: 120 };
  const z = turretAimError({ truePose: p, estPose: { ...p }, target: tgt });
  near(z.aimErrRad, 0, 1e-12); near(z.missLateralIn, 0, 1e-9);
  near(z.rangeIn, Math.hypot(90, 80), 1e-9);
  const d = 0.1;
  const e = turretAimError({ truePose: p, estPose: { ...p, h: p.h + d }, target: tgt });
  near(Math.abs(e.aimErrRad), d, 1e-9);
  assert.ok(Math.abs(Math.abs(e.missLateralIn) - e.rangeIn * Math.sin(d)) < 0.02 * e.rangeIn * d + 1e-6, `miss ${e.missLateralIn}`);
});

test('Pedro pose conversion: heading in degrees, round trip', () => {
  const p = toPedroPose({ x: 10, y: 20, h: Math.PI / 2 });
  assert.equal(p.x, 10); assert.equal(p.y, 20); near(p.h, 90, 1e-9);
  const q = fromPedroPose({ x: 1, y: 2, h: 180 });
  near(q.h, Math.PI, 1e-12);
  const r = fromPedroPose(toPedroPose({ x: 3, y: 4, h: -1.2 }));
  near(r.x, 3, 1e-12); near(r.h, -1.2, 1e-12);
});

test('vision, when visible and enabled, reduces the long-run heading error', () => {
  const imu = { biasInitSigmaDps: 0.5 };
  const mk = (vis) => makeRig({ preset: 'pinpoint', seed: 31, imuOverrides: imu, odometryOverrides: ZERO_ODO,
    visionOverrides: { enabled: vis } });
  const a = mk(false), b = mk(true);
  a.run(30, 0, 0, 0);
  b.run(30, 0, 0, 0, { events: { visionVisible: true } });
  assert.ok(Math.abs(b.sys.getHeadingError()) <= Math.abs(a.sys.getHeadingError()),
    `${b.sys.getHeadingError()} vs ${a.sys.getHeadingError()}`);
});
