// Integration tests for web/js/physics_glue.js (the glue between app.js and the physics area modules).
// Run: node --test "tests_js/integration/*.test.mjs"   (quoted glob; a bare directory fails on Node 24 / Windows)
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { PhysicsRuntime, PHYS_DEFAULTS, PHYS_OPTIONS, mergeSettings, motorDriveParamsFromSpec, formatPhysicsTelemetry, driveGearingFromSpec } from '../../web/js/physics_glue.js';
import { MecanumDrive } from '../../web/js/driver.js';
import { ShooterPlant } from '../../web/js/aimcontrol.js';
import { deriveConstants, derivePhysics, validateSpec } from '../../web/js/robotspec.js';
import * as SR from '../../web/js/shotrules.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const spec = JSON.parse(rd('../../web/data/robot_spec.json'));
const root = {}; vm.runInNewContext(rd('../../web/js/shot_core.js'), { window: root });
const Core = root.Core; SR.setCore(Core);
const D = deriveConstants(spec);
const START = { x: 72, y: 24, h: Math.PI / 2 };

function make(settings = {}) {
  const kin = new MecanumDrive({ maxSpeed: D.DRIVE.maxSpeed, maxAccel: D.DRIVE.maxAccel, maxTurn: D.DRIVE.maxTurn });
  const plant = new ShooterPlant({ turretRateDegS: 487, turretKp: 14, turretMinDeg: -150, turretMaxDeg: 150, hoodMinDeg: 25, hoodMaxDeg: 65, flywheelTauS: 0.35, maxRpm: 6000, rpmDipFrac: 0.06 });
  const rt = new PhysicsRuntime({ spec, settings, kinematicDrive: kin, plant, startPose: START });
  return { rt, kin, plant };
}
function drive(rt, S, cmd, seconds, dt = 1 / 120) {
  for (let i = 0; i < Math.round(seconds / dt); i++) {
    const v = rt.driveStep(cmd, dt, S, {}, { intakeOn: true });
    S.vx = v.vx; S.vy = v.vy; S.om = v.om;
    S.x += v.vx * dt; S.y += v.vy * dt; S.h += v.om * dt;
    rt.afterMotion(S, dt, null);
  }
}
const newS = () => ({ ...START, vx: 0, vy: 0, om: 0, bodyV: [0, 0, 0] });

test('robot_spec.json still validates and carries the physics block', () => {
  const v = validateSpec(spec);
  assert.deepEqual(v.errors, []);
  const P = derivePhysics(spec);
  assert.equal(mergeSettings(PHYS_DEFAULTS, P.defaults).rejected.length, 0, 'every spec physics default is a legal value');
});

test('settings: bad values are rejected, legal ones applied', () => {
  const m = mergeSettings(PHYS_DEFAULTS, { driveModel: 'warp', fixedHz: 120, batterySag: 'yes', nope: 1 });
  assert.deepEqual([...m.rejected].sort(), ['batterySag', 'driveModel', 'nope']);
  assert.equal(m.driveModel, PHYS_DEFAULTS.driveModel);
  const { rt } = make();
  assert.equal(rt.set('driveModel', 'warp'), rt.cfg.driveModel);
  for (const [k, vals] of Object.entries(PHYS_OPTIONS)) for (const v of vals) assert.equal(rt.set(k, v), v, k + '=' + v);
});

test('motor drive params come from the spec (mass, wheel radius, half spacings)', () => {
  const p = motorDriveParamsFromSpec(spec);
  assert.ok(Math.abs(p.massKg - 31.4 * 0.45359237) < 1e-9);
  assert.ok(Math.abs(p.wheelRadiusM - 2.047 * 0.0254) < 1e-9);
  assert.ok(Math.abs(p.lxM - 5.04 * 0.0254) < 1e-9 && Math.abs(p.lyM - 6.35 * 0.0254) < 1e-9);
});

// Speed band. Round 1 used 30..70 in/s, which encoded the old 60 in/s drive.maxSpeedInS placeholder and the motor-model's
// assumed 13.7:1 x 2 gearing (46.8 in/s free). Round 2 puts the vault gearing in robot_spec.json drive.gearing
// (5203 6000 rpm / (57/11 x 2.0) = 579 wheel rpm -> 579 * 2*pi*2.047 / 60 = 124.1 in/s free at 12 V), so the old upper
// bound is physically wrong: measured at 2 s full stick the models give 114 (kinematic, = maxSpeedInS), 122.2 (motor,
// 12 V), 125.9 / 135.5 in/s (kinematic / motor on a fresh pack, bus up to the 14.2 V bound asserted below, battery-sag
// scales speed with bus voltage). New band from the analytic values: lower = half of drive.maxSpeedInS (any model that
// is not stalled), upper = free speed x 14.2 V / 12 V = 146.9 in/s (no model can beat free speed at the bus voltage).
const G = driveGearingFromSpec(spec);
const SPEED_LO = 0.5 * spec.drive.maxSpeedInS, SPEED_HI = G.freeSpeedInS * 14.2 / 12;
for (const driveModel of ['kinematic', 'motor', 'dynamics', 'friction']) for (const wt of [false, true]) for (const sag of [false, true]) {
  test(`drive ${driveModel}, weightTransfer ${wt}, batterySag ${sag}: forward 2 s reaches a sane speed`, () => {
    const { rt } = make({ driveModel, weightTransfer: wt, batterySag: sag, odometry: 'nominal' });
    const S = newS();
    drive(rt, S, { fwd: 1, strafe: 0, turn: 0 }, 2);
    const sp = Math.hypot(S.vx, S.vy);
    assert.ok(Number.isFinite(sp) && sp > SPEED_LO && sp < SPEED_HI, `speed ${sp} in/s (band ${SPEED_LO}..${SPEED_HI.toFixed(1)})`);
    assert.ok(S.y > START.y + 40, `moved ${S.y - START.y} in`);
    const t = rt.telemetry(S);
    assert.ok(t.odometry.errIn < 1, `odometry error ${t.odometry.errIn} in after 2 s`);
    // fresh 12 V NiMH pack (10 cells) rests near 14 V, 1.40 V/cell at full charge (battery-sag params.js ocvTable)
    if (sag) assert.ok(t.battery.vBus > 9 && t.battery.vBus < 14.2, `bus ${t.battery.vBus} V`);
    assert.ok(formatPhysicsTelemetry(t).length >= 3);
  });
}

test('weight transfer caps a step command to a load-aware acceleration', () => {
  const { rt } = make({ driveModel: 'kinematic', weightTransfer: true, batterySag: false });
  const S = newS();
  rt.kin.maxAccel = 5000; rt.kin.maxDecel = 8000;    // the slew limit no longer bounds the step
  const dt = 1 / 120;
  rt.driveStep({ fwd: 1 }, dt, S, {});
  const lim = rt.wt.pathLimits();
  assert.ok(S.bodyV[0] / dt <= lim.maxAccelFwd * 1.05, `accel ${S.bodyV[0] / dt} vs ${lim.maxAccelFwd}`);
  assert.equal(rt.kin.maxAccel, 5000, 'the kinematic drive settings are restored after the battery/traction step');
});

test('battery sag limits the kinematic drive and restores its settings', () => {
  const { rt, kin } = make({ driveModel: 'kinematic', batterySag: true, batteryPreset: 'dead', weightTransfer: false });
  const S = newS();
  drive(rt, S, { fwd: 1 }, 2);
  assert.equal(kin.maxSpeed, D.DRIVE.maxSpeed);
  assert.ok(rt.telemetry(S).battery.speedScale < 1);
});

for (const flywheelModel of ['legacy', 'dynamics', 'pidf']) {
  test(`flywheel ${flywheelModel}: spins up to 3000 rpm and a shot dips it`, () => {
    const { rt, plant } = make({ flywheelModel });
    const dt = 1 / 120;
    for (let i = 0; i < 4 / dt; i++) rt.plantStep(dt, { turretDeg: 30, hoodDeg: 40, rpm: 3000 });
    assert.ok(Math.abs(plant.state.rpm - 3000) < 150, `rpm ${plant.state.rpm}`);
    assert.ok(Math.abs(plant.state.turretDeg - 30) < 1 && Math.abs(plant.state.hoodDeg - 40) < 1, 'turret and hood still driven');
    const before = plant.state.rpm;
    rt.kick('pollen');
    for (let i = 0; i < 6; i++) rt.plantStep(dt, { turretDeg: 30, hoodDeg: 40, rpm: 3000 });
    assert.ok(plant.state.rpm < before - 20, `dip ${before} -> ${plant.state.rpm}`);
  });
}

test('fixed step: frame times are split into whole fixed steps, variable mode is one clamped step', () => {
  const { rt } = make({ fixedStep: true, fixedHz: 120 });
  let n = 0;
  for (let i = 0; i < 60; i++) { const r = rt.frameSteps(1 / 60); assert.equal(r.h, 1 / 120); for (let k = 0; k < r.steps; k++) { n++; rt.onStep(); } }
  assert.ok(n >= 119 && n <= 120, `steps ${n}`);
  rt.set('fixedStep', false);
  assert.deepEqual(rt.frameSteps(0.2), { steps: 1, h: 0.05 });
});

test('seeded shot noise is reproducible and within +-1.5 %', () => {
  const a = make().rt, b = make().rt;
  for (let i = 0; i < 50; i++) { const x = a.shotNoise(); assert.equal(x, b.shotNoise()); assert.ok(x >= 0.985 && x <= 1.015); }
});

test('teleports relocalise the odometry instead of counting as motion', () => {
  const { rt } = make({ odometry: 'nominal' });
  const S = newS();
  S.x = 120; S.y = 100; rt.afterMotion(S, 1 / 120, null);
  const p = rt.aimPose(S);
  assert.ok(Math.hypot(p.x - 120, p.y - 100) < 1e-6);
});

test('physical exit model: the app aim path (solveAutoShot + plant setpoints + forwardShot) lands in the opening', () => {
  const SH = { ...D.SHOOTER, exitModel: 'physical' };
  const TG = { x: 100, y: 72, z: 34, normal: [-1, 0, 0] };
  for (const pose of [{ x: 40, y: 70, h: 0 }, { x: 55, y: 60, h: 0.4 }, { x: 30, y: 80, h: -0.3 }]) {
    const p = { ...pose, vx: 0, vy: 0, om: 0 };
    const s = SR.solveAutoShot(p, SH, TG, 'pollen', Core);
    assert.ok(s.ok, JSON.stringify(pose));
    const ev = SR.evaluateShot({ mode: 'assist', kind: 'pollen', pose: p, shooter: SH, target: TG, rpm: s.rpm, turretDeg: s.turretDeg, hoodDeg: s.hoodDeg, core: Core });
    assert.equal(ev.forward.lands, true, JSON.stringify(ev.forward.miss));
  }
});
