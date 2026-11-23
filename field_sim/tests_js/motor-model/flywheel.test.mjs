import test from 'node:test';
import assert from 'node:assert/strict';
import { FLYWHEEL } from '../../web/js/physics/motor-model/params.js';
import { makeGearmotor } from '../../web/js/physics/motor-model/derive.js';
import { FlywheelPlant } from '../../web/js/physics/motor-model/flywheel.js';
import { PowerSystem } from '../../web/js/physics/motor-model/hub.js';
import { DcTurretPlant } from '../../web/js/physics/motor-model/turret.js';

const near = (a, b, e) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (+-${e})`);

test('reaches 5000 rpm and ready() within 2 s', () => {
  const f = new FlywheelPlant();
  f.setTargetRpm(5000);
  let tReady = null;
  for (let i = 0; i < 100; i++) { f.step(0.02); if (tReady === null && f.ready()) tReady = (i + 1) * 0.02; }
  assert.ok(tReady !== null && tReady <= 2, `ready at ${tReady}`);
  near(f.rpm, 5000, 5000 * 0.06); // I-term windup after spin-up decays slowly (kI 5e-4); 28-CPR velocity quantum is ~107 rpm
});

test('targets clamp to [0, maxRpm]', () => {
  const f = new FlywheelPlant();
  f.setTargetRpm(9000); assert.equal(f.targetRpm, FLYWHEEL.maxRpm);
  f.setTargetRpm(-10); assert.equal(f.targetRpm, 0);
});

test('shoot dip matches the closed form', () => {
  const f = new FlywheelPlant();
  f.setTargetRpm(4000);
  for (let i = 0; i < 150; i++) f.step(0.02);
  const before = f.rpm;
  const r = f.shoot({ ballMassKg: 0.025, exitSpeedMps: 8 });
  const E = 0.5 * 0.025 * 64 * (1 + FLYWHEEL.ballRotFactor) / FLYWHEEL.shotEff;
  near(r.energyJ, E, 1e-9);
  near(E, 1.905, 0.01);
  const Jtot = FLYWHEEL.wheelJ + makeGearmotor('gobilda5203', 'none', { numMotors: 2 }).rotorJReflected;
  const w = before * 2 * Math.PI / 60;
  const wAfter = Math.sqrt(w * w - 2 * E / Jtot);
  near(r.rpmBefore, before, 1e-6);
  near(r.rpmAfter, wAfter * 60 / (2 * Math.PI), 1e-6);
  assert.ok(r.rpmAfter < r.rpmBefore && r.rpmAfter > 0.85 * r.rpmBefore);
});

test('recovers after a shot and reports telemetry', () => {
  const f = new FlywheelPlant();
  f.setTargetRpm(4000);
  for (let i = 0; i < 150; i++) f.step(0.02);
  f.shoot({ exitSpeedMps: 8 });
  assert.ok(!f.ready());
  for (let i = 0; i < 100; i++) f.step(0.02);
  assert.ok(f.ready());
  const t = f.telemetry();
  for (const k of ['rpm', 'targetRpm', 'duty', 'statorA', 'windingC', 'encoderTps', 'busV']) assert.ok(Number.isFinite(t[k]), k);
  near(f.surfaceSpeedMps(), f.omega * 0.048, 1e-9);
});

test('works on a shared PowerSystem', () => {
  const ps = new PowerSystem();
  const f = new FlywheelPlant({ power: ps });
  f.setTargetRpm(3000);
  for (let i = 0; i < 20; i++) { f.step(0.02); ps.endFrame(0.02); }
  assert.ok(f.rpm > 500);
});

test('DC turret reaches target within hard stops', () => {
  const t = new DcTurretPlant();
  t.setTargetDeg(45);
  for (let i = 0; i < 200; i++) t.step(0.02);
  assert.ok(t.atTarget(2), `deg ${t.deg}`);
  t.setTargetDeg(500);
  for (let i = 0; i < 400; i++) t.step(0.02);
  assert.ok(t.deg <= 180 + 1e-9);
  t.reset(0);
  assert.equal(t.deg, 0);
});
