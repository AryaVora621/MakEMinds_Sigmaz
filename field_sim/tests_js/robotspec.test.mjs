import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from '../web/vendor/three.module.js';
import { validateSpec, deriveConstants, provenanceReport, footprintCorners, footprintBoxCorners } from '../web/js/robotspec.js';
import { buildRobot, shooterGeometry } from '../web/js/robotmodel.js';
import { robotCorners } from '../web/js/collision.js';

const load = () => JSON.parse(readFileSync(new URL('../web/data/robot_spec.json', import.meta.url), 'utf8'));
const near = (a, b, t = 1e-9) => assert.ok(Math.abs(a - b) <= t, `${a} vs ${b}`);

test('shipped robot_spec.json validates with no errors', () => {
  const r = validateSpec(load());
  assert.deepEqual(r.errors, []);
  assert.equal(r.ok, true);
  assert.deepEqual(r.warnings, []);
});

test('deriveConstants SHOOTER matches today app.js', () => {
  const { SHOOTER } = deriveConstants(load());
  assert.deepEqual(SHOOTER, {
    exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1.0, maxRpm: 6000, entryDeg: 50, fixedElevDeg: 45,
    hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -150, turretMaxDeg: 150,
  });
});

test('deriveConstants CAMERA, PLANT, DRIVE match today app.js', () => {
  const d = deriveConstants(load());
  assert.deepEqual(d.CAMERA, { x: 4.2, y: 0, z: 15.9, pitchDeg: 25, hfovDeg: 54.5, vfovDeg: 42, rangeIn: 150 });
  assert.deepEqual(d.PLANT, { turretRateDegS: 487, turretKp: 14, flywheelTauS: 0.35, rpmDipFrac: 0.06 });
  // DRIVE pin changed in round 2 (integrator): robot_spec.json drive.gearing now holds the vault drive (5203 at 5.2:1 into
  // a 24:48 belt = 10.364:1, 124.1 in/s free at 2.047 in), and drive.reconciliation records the measured full-stick steady
  // state of the two drive models with floor losses: mecanum-dynamics 114.6 in/s / 9.42 rad/s, wheel-friction 114.5 in/s /
  // 10.80 rad/s. The old 60 in/s / 3.2 rad/s were placeholders (prov "placeholder") at about half the gearing's capability.
  // maxAccel 140 is unchanged (still a placeholder). app.js fallback DRIVE was updated to match.
  assert.deepEqual(d.DRIVE, { maxSpeed: 114, maxAccel: 140, maxTurn: 9.4 });
});

test('deriveConstants footprint is the MaxBot 381 x 375 mm frame', () => {
  const d = deriveConstants(load());
  near(d.halfX, 7.5); near(d.halfY, 7.38);
  assert.equal(d.FOOTPRINT.length, 4);
});

test('validateSpec rejects a negative wheel radius', () => {
  const s = load(); s.wheels[2].radius = -1;
  const r = validateSpec(s); assert.equal(r.ok, false);
  assert.ok(r.errors.some(e => /wheels\[2\]\.radius/.test(e)));
});

test('validateSpec rejects an insane flywheel radius', () => {
  const s = load(); s.shooter.flywheelRadius = 40;
  assert.equal(validateSpec(s).ok, false);
});

test('validateSpec rejects hood min > max', () => {
  const s = load(); s.shooter.hood.minDeg = 70;
  const r = validateSpec(s); assert.equal(r.ok, false);
  assert.ok(r.errors.some(e => /hood/.test(e)));
});

test('validateSpec rejects NaN and non-positive exit height', () => {
  const s = load(); s.camera.pitchDeg = NaN; s.shooter.exitHeightAboveFloor = 0;
  const r = validateSpec(s); assert.equal(r.ok, false);
  assert.ok(r.errors.some(e => /camera\.pitchDeg/.test(e)));
  assert.ok(r.errors.some(e => /exitHeightAboveFloor/.test(e)));
});

test('validateSpec rejects an open or degenerate polygon', () => {
  const s = load(); s.chassis.footprint = [[0, 0], [10, 0]];
  const r = validateSpec(s); assert.equal(r.ok, false);
  assert.ok(r.errors.some(e => /open|degenerate/.test(e)));
});

test('validateSpec rejects a self-intersecting polygon', () => {
  const s = load(); s.chassis.footprint = [[-9, -9], [9, 9], [9, -9], [-9, 9]];
  const r = validateSpec(s); assert.equal(r.ok, false);
  assert.ok(r.errors.some(e => /self-intersecting/.test(e)));
});

test('validateSpec accepts a closed ring (duplicate last vertex) and a non-convex outline', () => {
  const s = load(); s.chassis.footprint = [[-9, -9], [9, -9], [9, 9], [0, 3], [-9, 9], [-9, -9]];
  const r = validateSpec(s); assert.equal(r.ok, true, r.errors.join('; '));
});

test('oversize footprint is a warning, not an error', () => {
  const s = load(); s.chassis.footprint = [[-10, -9], [10, -9], [10, 9], [-10, 9]];
  const r = validateSpec(s);
  assert.equal(r.ok, true);
  assert.ok(r.warnings.some(w => /sizing rule/.test(w)));
});

test('wrong units or schema version is an error', () => {
  const s = load(); s.units = 'mm'; s.schema_version = 2;
  assert.equal(validateSpec(s).errors.length >= 2, true);
});

test('provenanceReport counts every numeric leaf by tag', () => {
  const rep = provenanceReport(load());
  assert.equal(rep.cad, 0);
  assert.ok(rep.repo > 5 && rep.vault > 20 && rep.placeholder > rep.repo);
  assert.equal(rep.cad + rep.repo + rep.vault + rep.placeholder, rep.items.length);
  const get = p => rep.items.find(i => i.path === p).provenance;
  assert.equal(get('shooter.exitHeightAboveFloor'), 'vault');
  assert.equal(get('shooter.pivot.x'), 'repo');
  assert.equal(get('shooter.pivot.z'), 'vault');
  assert.equal(get('wheels[0].radius'), 'vault');
  assert.equal(get('wheels[0].y'), 'placeholder');
  assert.equal(get('chassis.footprint'), 'vault');
  assert.equal(get('shooter.maxRpm'), 'repo');
  assert.equal(get('intake.capacity'), 'repo');
});

test('provenanceReport reflects a cad-tagged value', () => {
  const s = load(); s.camera.prov.pitchDeg = 'cad';
  const rep = provenanceReport(s);
  assert.equal(rep.cad, 1);
  assert.equal(rep.items.find(i => i.path === 'camera.pitchDeg').provenance, 'cad');
});

test('footprintBoxCorners equals collision.js robotCorners', () => {
  const a = footprintBoxCorners(10, 20, 0.7, 9, 8), b = robotCorners(10, 20, 0.7, 9, 8);
  a.forEach((p, i) => { near(p[0], b[i][0]); near(p[1], b[i][1]); });
  const c = footprintCorners([[1, 0], [0, 1], [-1, 0]], 5, 5, Math.PI / 2);
  near(c[0][0], 5); near(c[0][1], 6);
});

test('buildRobot builds 4 wheels, turret group, hood pivot, flywheel and parts', () => {
  const spec = load();
  const R = buildRobot(THREE, spec);
  assert.equal(R.parts.wheels.length, 4);
  assert.ok(R.turretGroup && R.hoodPivot && R.flywheelMesh);
  assert.equal(R.parts.sensors.length, 2);
  assert.ok(R.parts.intake && R.parts.intakeRear && R.parts.flower && R.parts.gate && R.parts.camera && R.parts.chassis);
  near(R.turretGroup.position.z, 9.5);
  assert.equal(R.turretGroup.children.includes(R.hoodPivot), true);
  R.setPose({ turretDeg: 90, hoodDeg: 25 });
  near(R.turretGroup.rotation.z, Math.PI / 2); near(R.hoodPivot.rotation.y, -25 * Math.PI / 180);
});

test('buildRobot chassis bounding box matches the footprint; whole robot fits the 18 in start envelope', () => {
  const spec = load();
  const R = buildRobot(THREE, spec);
  R.group.updateMatrixWorld(true);
  const c = new THREE.Box3().setFromObject(R.parts.chassis), tol = 0.3;
  assert.ok(Math.abs(c.min.x + 7.5) < 1 && Math.abs(c.max.x - 7.5) < 1, `x ${c.min.x}..${c.max.x}`);
  assert.ok(c.max.y < 7.38 + tol && c.min.y > -7.38 - tol, `y ${c.min.y}..${c.max.y}`);
  assert.ok(c.max.y > 6.5, 'cassette caps reach the frame edge');
  const b = new THREE.Box3().setFromObject(R.group);
  assert.ok(b.min.z > -1e-6, `z ${b.min.z}`);
  assert.ok(b.max.x < 9.3 && b.min.x > -9.3 && b.max.y < 7.9, `folded intakes stay inside 18 in: ${b.min.x}..${b.max.x}`);
});

test('wheel meshes sit at the spec x/y/radius', () => {
  const spec = load();
  const R = buildRobot(THREE, spec);
  spec.wheels.forEach((w, i) => {
    near(R.parts.wheels[i].position.x, w.x); near(R.parts.wheels[i].position.y, w.y); near(R.parts.wheels[i].position.z, w.radius);
  });
});

test('shooterGeometry gives exit point, height and launch direction', () => {
  const spec = load();
  const g0 = shooterGeometry(spec, 0, 45);
  near(g0.exitHeight, 18.4); near(g0.flywheelCenterRobot[2], 21.4); near(g0.exitPointRobot[0], 3.2);
  near(g0.launchDir[0], Math.SQRT1_2); near(g0.launchDir[2], Math.SQRT1_2);
  const g90 = shooterGeometry(spec, 90, 25);
  near(g90.exitPointRobot[0], 0, 1e-9); near(g90.exitPointRobot[1], 3.2, 1e-9);
  near(Math.hypot(...g90.launchDir), 1);
});
