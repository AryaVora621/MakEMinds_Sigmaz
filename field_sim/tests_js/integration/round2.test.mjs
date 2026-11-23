// Round-2 integration tests for web/js/physics_glue.js: new drive models, robot-vs-field collision models, imu-heading in
// the Pinpoint, battery bus -> flywheel plants, electrical tick on every motion source, render interpolation,
// pedro-fidelity params from the spec, balls.js model switches.
// Run: node --test "tests_js/integration/*.test.mjs"
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { PhysicsRuntime, driveGearingFromSpec, pedroParamsFromSpec, GyroImuAdapter, stickToBody, frictionDriveParamsFromSpec } from '../../web/js/physics_glue.js';
import { MecanumDrive } from '../../web/js/driver.js';
import { ShooterPlant } from '../../web/js/aimcontrol.js';
import { deriveConstants } from '../../web/js/robotspec.js';
import { BALL_MODELS } from '../../web/js/balls.js';
import { transformPolygon } from '../../web/js/physics/wall-collisions/geometry.js';
import { collideRobot, maxPenetration } from '../../web/js/physics/wall-collisions/manifold.js';
import { resolveParams } from '../../web/js/physics/pedro-fidelity/params.js';
import { simulateBridgeRequest } from '../../web/js/physics/pedro-fidelity/closedloop.js';
import { MOTORS } from '../../web/js/physics/motor-model/params.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const spec = JSON.parse(rd('../../web/data/robot_spec.json'));
const col = JSON.parse(rd('../../web/data/colliders.json'));
const fieldData = { ...col, boxes: col.boxes.filter((b) => b.zMax > 3) };   // app.js RAIL_MAX_Z = 3 (flower trim omitted)
const D = deriveConstants(spec);
const START = { x: 72, y: 24, h: Math.PI / 2 };
const DT = 1 / 120;

function make(settings = {}, withField = false) {
  const kin = new MecanumDrive({ maxSpeed: D.DRIVE.maxSpeed, maxAccel: D.DRIVE.maxAccel, maxTurn: D.DRIVE.maxTurn });
  const plant = new ShooterPlant({ turretRateDegS: 487, turretKp: 14, turretMinDeg: -150, turretMaxDeg: 150, hoodMinDeg: 25, hoodMaxDeg: 65, flywheelTauS: 0.35, maxRpm: 6000, rpmDipFrac: 0.06 });
  const rt = new PhysicsRuntime({ spec, settings, kinematicDrive: kin, plant, startPose: START, colliders: withField ? fieldData : null });
  return { rt, kin, plant };
}
const newS = () => ({ ...START, vx: 0, vy: 0, om: 0, bodyV: [0, 0, 0] });
function lcg(s) { return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }
const finite = (S) => [S.x, S.y, S.h, S.vx, S.vy, S.om, ...S.bodyV].every(Number.isFinite);

// ---------- drive gearing + new drive models ----------
test('spec gearing: 5203 6000 rpm / (57/11 x 2) = 10.364:1, 579 wheel rpm, 124.1 in/s free (vault: 575 wheel rpm at 1150 rated)', () => {
  const g = driveGearingFromSpec(spec);
  assert.ok(Math.abs(g.ratio - 57 / 11 * 2) < 1e-12);
  assert.ok(Math.abs(g.wheelFreeRpm - 578.95) < 0.01, String(g.wheelFreeRpm));
  assert.ok(Math.abs(g.freeSpeedInS - 124.10) < 0.01, String(g.freeSpeedInS));
  const f = frictionDriveParamsFromSpec(spec);
  assert.ok(Math.abs(f.MOTOR.stallTorqueNm - MOTORS.gobilda5203.stallTorqueNm * g.ratio) < 1e-12);
  // spec reconciliation: maxSpeedInS sits below free speed and within 10 % of the floor-loss models (114.5..114.6 in/s)
  assert.ok(spec.drive.maxSpeedInS < g.freeSpeedInS && spec.drive.maxSpeedInS > 0.9 * g.freeSpeedInS);
});

for (const driveModel of ['dynamics', 'friction']) {
  test(`${driveModel} drive: forward/strafe/turn all move the right way, a dead battery is slower than a fresh one`, () => {
    const top = {};
    for (const preset of ['fresh', 'dead']) {
      const { rt } = make({ driveModel, batterySag: true, batteryPreset: preset, weightTransfer: false, odometry: 'off' });
      const S = newS(); S.h = 0;
      for (let i = 0; i < 240; i++) { const v = rt.driveStep({ fwd: 1 }, DT, S, {}); S.vx = v.vx; S.vy = v.vy; S.om = v.om; }
      top[preset] = S.bodyV[0];
      assert.ok(finite(S));
      if (preset === 'fresh') {
        assert.ok(rt.telemetry(S).drive.model === driveModel);
        // fresh runtime per command: three back-to-back 2 s full-power starts on ONE battery heat the sourced 20 A ATO
        // fuse model (battery-sag params.js FUSE, 32.8 A peak = 1.64 x rated, 3.5 s open time) past damage 1 (measured:
        // 0.45 after the first start, 0.84 after the second, blown in the third), which is a real effect, not a drive bug.
        for (const [c, k, lo] of [[{ strafe: 1 }, 1, 30], [{ turn: 1 }, 2, 2]]) {
          const r2 = make({ driveModel, batterySag: true, weightTransfer: false, odometry: 'off' }).rt;
          const S2 = newS(); S2.h = 0;
          for (let i = 0; i < 240; i++) r2.driveStep(c, DT, S2, {});
          assert.ok(S2.bodyV[k] > lo && Math.abs(S2.bodyV[0 === k ? 1 : 0]) < 2, `${JSON.stringify(c)} -> ${S2.bodyV}`);
        }
      }
    }
    assert.ok(top.fresh > 60 && top.dead < top.fresh - 3, JSON.stringify(top));
  });
}

test('stick mapping (field-centric too) is identical to the motor-model copy of driver.js MecanumDrive', () => {
  const m = make().rt.motorDrive;
  const r = lcg(9);
  for (let i = 0; i < 200; i++) {
    const cmd = { fwd: r() * 2 - 1, strafe: r() * 2 - 1, turn: r() * 2 - 1 }, h = r() * 6 - 3, off = r() - 0.5;
    for (const [fc, al] of [[false, null], [true, 'blue'], [true, 'red'], [true, null]]) {
      const a = stickToBody(cmd, h, { fieldCentric: fc, headingOffset: off, alliance: al });
      const b = m._frame(cmd, { h }, fc, off, al);
      for (let k = 0; k < 3; k++) assert.ok(Math.abs(a[k] - b[k]) < 1e-12, `${a} vs ${b}`);
    }
  }
});

// ---------- robot-vs-field collision models ----------
// 60 s of random full-power driving at 120 Hz (seeded), 3 seeds, motor drive at the vault gearing (up to 124 in/s).
// Measured on 2026-11-22: max penetration legacy 0.000 in, contact 0.007..0.054 in, rigid 0.010..0.016 in; no NaN in any
// mode; no tip-over in any mode. 'contact' and 'rigid' are NOT the default: both leave a small (< 0.1 in) speculative-
// solver penetration that the legacy pose correction never shows, so they are not a strict no-regression replacement.
function soak(mode, seed) {
  const { rt } = make({ robotCollision: mode, odometry: 'off' }, true);
  const S = newS(); const r = lcg(seed); let cmd = {}, maxPen = 0, dist = 0, fallen = 0;
  const w = rt.wallCtx;
  for (let i = 0; i < 120 * 60; i++) {
    if (i % 90 === 0) cmd = { fwd: r() * 2 - 1, strafe: r() * 2 - 1, turn: (r() * 2 - 1) * 0.5 };
    const v = rt.driveStep(cmd, DT, S, {}); S.vx = v.vx; S.vy = v.vy; S.om = v.om;
    const px = S.x, py = S.y;
    const c = rt.moveRobot(S, DT); rt.afterMotion(S, DT, c);
    if (!finite(S)) return { nan: true };
    dist += Math.hypot(S.x - px, S.y - py);
    maxPen = Math.max(maxPen, maxPenetration(collideRobot(transformPolygon(w.poly, S.x, S.y, S.h), w.prepared, 0, w.P)));
    if (rt.wt.snapshot.tip.mode === 'fallen') fallen++;
  }
  return { nan: false, maxPen, dist, fallen };
}
for (const mode of ['legacy', 'contact', 'rigid']) {
  test(`robotCollision ${mode}: 3 x 60 s random driving, no NaN, penetration < 0.1 in, robot keeps moving, never falls`, () => {
    for (const seed of [1, 2, 3]) {
      const r = soak(mode, seed);
      assert.equal(r.nan, false, `NaN, seed ${seed}`);
      assert.ok(r.maxPen < 0.1, `penetration ${r.maxPen} in, seed ${seed}`);
      if (mode === 'legacy') assert.ok(r.maxPen < 1e-6, 'legacy pose correction never penetrates');
      assert.ok(r.dist > 300, `distance ${r.dist} in, seed ${seed}`);
      assert.equal(r.fallen, 0, `tip-over steps ${r.fallen}, seed ${seed}`);
    }
  });
}

test('moveRobot: a head-on wall hit stops the robot at the wall in every mode; the removed velocity is the impact', () => {
  for (const mode of ['legacy', 'contact', 'rigid']) {
    const { rt } = make({ robotCollision: mode, odometry: 'off', weightTransfer: false }, true);
    const B = rt.obstacles.bounds;
    const S = { x: 72, y: B.minY + 9 + 0.4, h: Math.PI / 2, vx: 0, vy: -100, om: 0, bodyV: [-100, 0, 0] };
    let first = null;
    for (let i = 0; i < 6; i++) { S.vy = -100; S.vx = 0; S.om = 0; const c = rt.moveRobot(S, DT); if (c && !first) first = c; }
    assert.ok(first && first.hit, mode);
    assert.ok(S.y > B.minY + 9 - 0.1, `${mode}: y ${S.y} vs wall ${B.minY + 9}`);
    const n = first.normals[0], vn = first.vBefore.vx * n[0] + first.vBefore.vy * n[1];
    // removed = approach speed (100) plus the rebound: restitution <= 0.3 in both solvers (wall-collisions/rigid params)
    assert.ok(vn < -50 && vn >= -100 * 1.3 - 1e-9, `${mode}: removed normal speed ${vn}`);
  }
});

// ---------- imu-heading inside the Pinpoint ----------
test('imuModel gyro: the Pinpoint reads the imu-heading GyroModel; ideal preset is bit-exact, nominal stays within the basic budget', () => {
  const err = {};
  for (const imuModel of ['basic', 'gyro']) for (const odometry of ['ideal', 'nominal']) {
    const { rt } = make({ imuModel, odometry, weightTransfer: false, batterySag: false, driveModel: 'kinematic' });
    if (imuModel === 'gyro') assert.ok(rt.odo.imu instanceof GyroImuAdapter && rt.odo.device._imu === rt.odo.imu);
    const S = newS();
    for (let i = 0; i < 120 * 20; i++) {
      const v = rt.driveStep({ fwd: 0.3, turn: 0.6 }, DT, S, {}); S.vx = v.vx; S.vy = v.vy; S.om = v.om;
      S.x += S.vx * DT; S.y += S.vy * DT; S.h = Math.atan2(Math.sin(S.h + S.om * DT), Math.cos(S.h + S.om * DT));
      rt.afterMotion(S, DT, null);
    }
    const t = rt.telemetry(S).odometry;
    err[imuModel + '/' + odometry] = Math.abs(t.errDeg);
  }
  assert.ok(err['gyro/ideal'] < 0.05, JSON.stringify(err));
  // Budget for the imu-heading 'pinpoint' preset (imu-heading/params.js): the dominant term over 20 s of turning is the
  // scale error, 1-sigma 0.002 + asymmetry 0.0005, times the total rotation. Total rotation = 0.6 x maxTurnRadS 9.4 x 20 s
  // = 112.8 rad, so 3 sigma = 3 x 0.0025 x 112.8 rad = 0.846 rad = 48.5 deg; bias terms add < 0.1 deg in 20 s. Measured
  // 4.5 deg (seed 1). The basic Pinpoint IMU measured 1.1 deg. The gyro is worse because it models scale error, so it stays
  // opt-in (imuModel default 'basic').
  const rotRad = 0.6 * D.DRIVE.maxTurn * 20;
  assert.ok(err['gyro/nominal'] < 3 * 0.0025 * rotRad * 180 / Math.PI, JSON.stringify(err));
  assert.ok(err['gyro/nominal'] > 0, 'the nominal gyro really adds error');
});

// ---------- battery-sag bus -> flywheel plants; electrical tick in auto / link / playback ----------
for (const flywheelModel of ['dynamics', 'pidf']) {
  test(`flywheel ${flywheelModel} reads the battery-sag bus voltage: a dead pack gives a lower plant voltage and slower spin-up`, () => {
    const res = {};
    for (const preset of ['fresh', 'dead']) {
      const { rt, plant } = make({ flywheelModel, batteryPreset: preset, batterySag: true });
      const S = newS();
      for (let i = 0; i < 120; i++) { rt.passiveStep(DT, S, {}); rt.plantStep(DT, { turretDeg: 0, hoodDeg: 40, rpm: 5000 }); }
      const busPlant = flywheelModel === 'dynamics' ? rt.fwDyn.params.battery.voltageV : rt.fwPidf.battery.p.openCircuitV;
      assert.ok(Math.abs(busPlant - rt.elec.last.vBus) < 1e-9, `plant bus ${busPlant} vs elec ${rt.elec.last.vBus}`);
      res[preset] = { rpm: plant.state.rpm, v: rt.elec.last.vBus };
    }
    assert.ok(res.dead.v < res.fresh.v - 0.5, JSON.stringify(res));
    assert.ok(res.dead.rpm < res.fresh.rpm, JSON.stringify(res));
    // sag off: the plants get their own default batteries back
    const { rt } = make({ flywheelModel, batterySag: true });
    rt.passiveStep(DT, newS(), {}); rt.set('batterySag', false);
    assert.equal(rt.fwDyn.params.battery.voltageV, 12);
  });
}

test('passiveStep: the electrical system runs when the robot is moved by auto/link/playback (no driveStep)', () => {
  const { rt } = make({ batterySag: true });
  const S = newS();
  rt.passiveStep(DT, S, {});
  const idle = rt.elec.last.iTotal, t0 = rt.elec.t;
  S.vx = 0; S.vy = 80; S.om = 0;
  for (let i = 0; i < 60; i++) rt.passiveStep(DT, S, { intakeOn: true });
  assert.ok(rt.elec.t > t0 + 0.49, 'battery clock advanced');
  assert.ok(rt.elec.last.iTotal > idle, `current ${rt.elec.last.iTotal} A vs idle ${idle} A`);
  assert.ok(Number.isFinite(rt.elec.last.vBus) && rt.elec.last.vBus > 9);
});

// ---------- render interpolation (FixedClock alpha) ----------
test('renderPose interpolates between the last two fixed steps by FixedClock.alpha; teleports and renderInterp=false draw the current pose', () => {
  const { rt } = make({ fixedStep: true, fixedHz: 120 });
  const S = newS();
  const fs = rt.frameSteps(1.5 / 120);           // one step and a half-step remainder
  assert.equal(fs.steps, 1);
  rt.beginStep(S); S.x += 1; rt.onStep();
  assert.ok(Math.abs(rt.renderAlpha() - 0.5) < 1e-9);
  assert.ok(Math.abs(rt.renderPose(S).x - (START.x + 0.5)) < 1e-9);
  rt.beginStep(S); S.x += 50; assert.equal(rt.renderPose(S).x, S.x, 'teleport');
  rt.set('renderInterp', false); rt.beginStep(S); S.x += 1; assert.equal(rt.renderPose(S).x, S.x);
});

// ---------- pedro-fidelity from the Pedro constants + spec ----------
test('pedroPlant spec: geometry from robot_spec.json, Pedro library constants unchanged, a 48 in line still finishes', () => {
  const ov = pedroParamsFromSpec(spec);
  const P = resolveParams(ov), P0 = resolveParams({});
  assert.equal(P.plant.trackWidthIn, spec.chassis.track);
  assert.equal(P.plant.wheelBaseIn, spec.chassis.wheelbase);
  assert.deepEqual(P.pedro, P0.pedro);
  assert.deepEqual(P.mecanum, P0.mecanum);
  const line = { type: 'line', points: [{ x: 0, y: 0 }, { x: 48, y: 0 }], heading: { mode: 'constant', start: 0 } };
  const tr = simulateBridgeRequest({ start: { x: 0, y: 0, h: 0 }, segments: [line] }, ov);
  assert.equal(tr.finished, true);
  const last = tr.at(-1);
  assert.ok(Math.hypot(last.x - 48, last.y) < 1, `${last.x},${last.y}`);
  const { rt } = make({ autoFollower: 'pedro', pedroPlant: 'spec' });
  const runner = { followerModel: 'auto', pedroParams: {}, _svc: null };
  rt.configureRunner(runner);
  assert.equal(runner.followerModel, 'pedro');
  assert.equal(runner.pedroParams.plant.trackWidthIn, spec.chassis.track);
  rt.set('pedroPlant', 'library'); rt.configureRunner(runner); assert.deepEqual(runner.pedroParams, {});
});

// ---------- balls.js model switches ----------
test('ballAero / ballBall switches write balls.js BALL_MODELS (defaults legacy)', () => {
  const { rt } = make();
  assert.deepEqual({ ...BALL_MODELS }, { aero: 'legacy', ballBall: 'legacy' });
  rt.set('ballBall', 'solver'); rt.set('ballAero', 'physical');
  assert.deepEqual({ ...BALL_MODELS }, { aero: 'physical', ballBall: 'solver' });
  rt.set('ballBall', 'legacy'); rt.set('ballAero', 'legacy');
  assert.deepEqual({ ...BALL_MODELS }, { aero: 'legacy', ballBall: 'legacy' });
});
