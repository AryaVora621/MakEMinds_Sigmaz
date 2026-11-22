import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from '../web/vendor/three.module.js';
import { buildRobot } from '../web/js/robotmodel.js';
import { loadRobotGLB, parseGLB, findMovingParts, readGLBContainer, orientationMatrix } from '../web/js/robotglb.js';

const spec = () => JSON.parse(readFileSync(new URL('../web/data/robot_spec.json', import.meta.url), 'utf8'));
const near = (a, b, t = 1e-3) => assert.ok(Math.abs(a - b) <= t, `${a} vs ${b}`);

function triCount(root) {
  let n = 0;
  root.traverse(o => { if (o.isMesh) { const g = o.geometry; n += (g.index ? g.index.count : g.attributes.position.count) / 3; } });
  return n;
}

// ---------------- procedural robot ----------------
test('procedural robot: 4 wheels, each with >= 8 roller meshes', () => {
  const R = buildRobot(THREE, spec());
  assert.equal(R.parts.wheels.length, 4);
  for (const w of R.parts.wheels) {
    const rollers = w.children.filter(c => c.isMesh && c.name === 'roller');
    assert.ok(rollers.length >= 8, `${w.name}: ${rollers.length} rollers`);
  }
});

test('procedural robot: mecanum rollers use the diagonal pattern (FL/BR mirror FR/BL)', () => {
  const R = buildRobot(THREE, spec());
  const axis = w => { const r = w.children.find(c => c.name === 'roller'); return new THREE.Vector3(0, 1, 0).applyQuaternion(r.quaternion); };
  const [fl, fr, bl, br] = R.parts.wheels;
  // roller 0 sits at theta 0 on every wheel, so axes compare directly: x*y>0 wheels share a handedness
  near(axis(fl).z, axis(br).z); near(axis(fr).z, axis(bl).z); near(axis(fl).z, -axis(fr).z);
  assert.ok(Math.abs(axis(fl).y) > 0.5 && Math.abs(axis(fl).z) > 0.5, 'rollers are neither parallel nor perpendicular to the axle');
});

test('procedural robot: turret / hood / flywheel exist, rotate and setPose works', () => {
  const R = buildRobot(THREE, spec());
  assert.ok(R.turretGroup && R.hoodPivot && R.flywheelMesh);
  assert.ok(R.turretGroup.children.includes(R.hoodPivot));
  assert.ok(R.hoodPivot.children.includes(R.flywheelMesh));
  R.setPose({ turretDeg: 90, hoodDeg: 30, flywheelRad: 1.25, wheelRad: [0.5, 0.5, 0.5, 0.5], gateOpen: 1, flowerRad: 0.3 });
  near(R.turretGroup.rotation.z, Math.PI / 2); near(R.hoodPivot.rotation.y, -30 * Math.PI / 180);
  near(R.flywheelMesh.rotation.y, 1.25); near(R.parts.wheels[2].rotation.y, 0.5);
  R.group.updateMatrixWorld(true);
  // the camera is on the turret, so a 90 deg turn moves it to +y
  const p = new THREE.Vector3(); R.parts.camera.getWorldPosition(p);
  assert.ok(p.y > 3, `camera y ${p.y}`);
  // rotating the flywheel must move geometry (it has off-centre parts)
  assert.ok(R.flywheelMesh.children.length >= 2);
});

test('procedural robot: folded start state stays within 18 x 18 in (plus rollers), above the floor', () => {
  for (const ally of [undefined, 'red', 'blue']) {
    const s = spec(), R = buildRobot(THREE, s, { allianceColor: ally });
    R.group.updateMatrixWorld(true);
    const b = new THREE.Box3().setFromObject(R.group), tol = 0.3;
    assert.ok(b.min.x > -9 - tol && b.max.x < 9 + tol, `x ${b.min.x}..${b.max.x}`);
    assert.ok(b.min.y > -7.5 - tol && b.max.y < 7.5 + tol, `y ${b.min.y}..${b.max.y}`);
    assert.ok(b.min.z > -1e-6, `z ${b.min.z}`);
    assert.equal(!!R.parts.bumper, !!ally);
  }
});

test('procedural robot: dimensions come from the spec (frame, wheels, deck, exit height)', () => {
  const s = spec(), R = buildRobot(THREE, s);
  R.group.updateMatrixWorld(true);
  s.wheels.forEach((w, i) => { near(R.parts.wheels[i].position.x, w.x); near(R.parts.wheels[i].position.y, w.y); near(R.parts.wheels[i].position.z, w.radius); });
  near(s.wheels[0].radius * 2 * 25.4, 104, 0.1);                       // 104 mm GripForce
  near(Math.abs(s.wheels[0].x) * 25.4, 128, 0.2);                      // wheels at x +-128 mm
  near((s.chassis.footprint[1][0] - s.chassis.footprint[0][0]) * 25.4, 381, 0.1);
  near(s.chassis.deckZ * 25.4, 224, 0.3);                               // deck at z 224 mm
  near(s.shooter.exitHeightAboveFloor, 18.4); near(s.shooter.flywheelRadius * 2 * 25.4, 96, 0.2);
  near(s.shooter.turret.boreRadius * 2 * 25.4, 105, 0.2);
  near(R.turretGroup.position.z, s.shooter.pivot.z);
  // the flywheel sits above the ball exit point
  const p = new THREE.Vector3(); R.flywheelMesh.getWorldPosition(p);
  near(p.z, s.shooter.pivot.z + s.shooter.exitPoint.z + s.shooter.flywheelAbove + 0, 3.0);
});

test('procedural robot: dual intakes mirror each other, fold 38 mm -> 106 mm, rollers spin', () => {
  const s = spec(), R = buildRobot(THREE, s);
  assert.ok(R.parts.intake && R.parts.intakeRear && R.parts.gate && R.parts.gateRear);
  assert.equal(R.parts.intakeRollers.length, 4);
  const reach = g => { g.updateMatrixWorld(true); const b = new THREE.Box3().setFromObject(g); return [b.min.x, b.max.x]; };
  const fx = s.chassis.footprint[2][0];
  near(reach(R.parts.intake)[1] - fx, 1.5, 0.15); near(-reach(R.parts.intakeRear)[0] - fx, 1.5, 0.15);
  R.setPose({ intakeDeploy: 1 });
  near(reach(R.parts.intake)[1] - fx, 4.2, 0.15); near(-reach(R.parts.intakeRear)[0] - fx, 4.2, 0.15);
  R.setPose({ intakeRad: 1.1 });
  near(R.parts.intakeRollers[0].rotation.y, 1.1);
  assert.ok(R.parts.intakeRollers[0].children.length >= 3);
});

test('procedural robot: animate() spins wheels from velocity, rollers/flywheel, folds intakes, drives the flower arm', () => {
  const R = buildRobot(THREE, spec());
  R.animate(0.05, { vx: 20, vy: 0, om: 0 });
  const r = R.parts.wheels.map(w => w.rotation.y);
  assert.ok(r.every(v => v > 0.4), 'forward drive spins all wheels forward');
  near(r[0], r[3]); near(r[1], r[2]);
  const before = R.parts.wheels[0].rotation.y;
  R.animate(0.05, { vx: 0, vy: 20 });                                    // strafe left: FL back, FR forward
  assert.ok(R.parts.wheels[0].rotation.y < before && R.parts.wheels[1].rotation.y > r[1]);
  const fw0 = R.flywheelMesh.rotation.y; R.animate(0.05, { rpm: 3000, turretDeg: 40, hoodDeg: 35 });
  assert.ok(R.flywheelMesh.rotation.y !== fw0); near(R.turretGroup.rotation.z, 40 * Math.PI / 180); near(R.hoodPivot.rotation.y, -35 * Math.PI / 180);
  const rr0 = R.parts.intakeRollers[0].rotation.y;
  for (let i = 0; i < 40; i++) R.animate(0.02, { intakeOn: true });
  assert.ok(R.parts.intakeRollers[0].rotation.y !== rr0); near(R.state.deploy, 1);
  for (let i = 0; i < 40; i++) R.animate(0.02, { intakeOn: false });
  near(R.state.deploy, 0);
  R.animate(0.02, { flowerArmDeg: -35 }); near(R.parts.flowerArm.hinge.rotation.y, 35 * Math.PI / 180);
});

test('procedural robot: flower arm is stowed upright inside the envelope and drops down in front when deployed', () => {
  const R = buildRobot(THREE, spec(), { flowerArmCfg: JSON.parse(readFileSync(new URL('../web/data/flower_arm.json', import.meta.url), 'utf8')) });
  const A = R.parts.flowerArm, pad = new THREE.Vector3();
  R.setPose({ flowerArmDeg: 90 }); R.group.updateMatrixWorld(true);
  A.parts.paddle.getWorldPosition(pad);
  const top = new THREE.Box3().setFromObject(A.group); assert.ok(top.max.z <= 18, `stowed top ${top.max.z}`); assert.ok(top.max.x < 9.3);
  R.setPose({ flowerArmDeg: -35 }); R.group.updateMatrixWorld(true);
  const p2 = new THREE.Vector3(); A.parts.paddle.getWorldPosition(p2);
  assert.ok(p2.x > 12 && p2.z < 5 && p2.z > 1, `paddle at ${p2.toArray()}`);
  const bb = new THREE.Box3().setFromObject(A.group); assert.ok(bb.min.z > -0.1, `arm below floor ${bb.min.z}`);
  // the paddle hangs straight down at every angle
  for (const a of [90, 40, 0, -35]) { A.setAngle(a); R.group.updateMatrixWorld(true); const d = new THREE.Vector3(0, 0, -1).transformDirection(A.parts.paddle.matrixWorld); near(d.z, -1, 1e-6); }
});

test('procedural robot: bumper takes the alliance colour; spec.alliance also works', () => {
  const a = buildRobot(THREE, spec(), { allianceColor: 'red' });
  const m = a.parts.bumper.children.find(c => c.isMesh && c.material.color.getHex() === 0xd62828);
  assert.ok(m);
  const s = spec(); s.alliance = 'blue';
  assert.ok(buildRobot(THREE, s).parts.bumper);
});

test('procedural robot: triangle count is modest', () => {
  const t = triCount(buildRobot(THREE, spec(), { allianceColor: 'red' }).group);
  assert.ok(t < 70000, `${t} triangles`);
  assert.ok(t > 2000, `${t} triangles (suspiciously empty)`);
});

test('procedural robot: parts the host and spec expect', () => {
  const R = buildRobot(THREE, spec());
  assert.equal(R.parts.sensors.length, 2);
  assert.ok(R.parts.intake && R.parts.intakeRear && R.parts.transfer && R.parts.battery && R.parts.flower && R.parts.gate && R.parts.camera && R.parts.chassis && R.parts.electronics && R.parts.odometry);
  assert.equal(R.parts.pods.length, 2); assert.equal(R.parts.transferRollers.length, 3);
  assert.ok(R.parts.intake.children.length >= 1);
});

// ---------------- GLB parser ----------------
/** Builds a .glb: one box mesh (8 verts, 12 tris, no normals) with a material; `nodes` supplied by the caller. */
function makeGLB({ size = [457, 300, 457], nodes, extra = {}, corruptBin = false } = {}) {
  const [sx, sy, sz] = size.map(v => v / 2);
  const pos = new Float32Array([-sx, -sy, -sz, sx, -sy, -sz, sx, sy, -sz, -sx, sy, -sz, -sx, -sy, sz, sx, -sy, sz, sx, sy, sz, -sx, sy, sz]);
  const idx = new Uint16Array([0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6, 0, 4, 5, 0, 5, 1, 1, 5, 6, 1, 6, 2, 2, 6, 7, 2, 7, 3, 3, 7, 4, 3, 4, 0]);
  if (corruptBin) idx[3] = 200;
  const bin = new Uint8Array(pos.byteLength + idx.byteLength + 4);
  bin.set(new Uint8Array(pos.buffer), 0); bin.set(new Uint8Array(idx.buffer), pos.byteLength);
  const json = {
    asset: { version: '2.0' }, scene: 0, scenes: [{ nodes: [0] }],
    nodes: nodes || [{ name: 'Chassis', mesh: 0, translation: [10, 20, 30] }],
    meshes: [{ primitives: [{ attributes: { POSITION: 0 }, indices: 1, material: 0 }] }],
    materials: [{ name: 'alu', pbrMetallicRoughness: { baseColorFactor: [1, 0, 0, 1], metallicFactor: 0.2, roughnessFactor: 0.7 } }],
    accessors: [
      { bufferView: 0, componentType: 5126, count: 8, type: 'VEC3', min: [-sx, -sy, -sz], max: [sx, sy, sz] },
      { bufferView: 1, componentType: 5123, count: 36, type: 'SCALAR' },
    ],
    bufferViews: [{ buffer: 0, byteOffset: 0, byteLength: pos.byteLength }, { buffer: 0, byteOffset: pos.byteLength, byteLength: idx.byteLength }],
    buffers: [{ byteLength: bin.byteLength }], ...extra,
  };
  return packGLB(json, bin);
}
function packGLB(json, bin) {
  let js = new TextEncoder().encode(JSON.stringify(json));
  const jp = (4 - (js.length % 4)) % 4; const jb = new Uint8Array(js.length + jp).fill(0x20); jb.set(js);
  const total = 12 + 8 + jb.length + 8 + bin.length, out = new Uint8Array(total), dv = new DataView(out.buffer);
  dv.setUint32(0, 0x46546C67, true); dv.setUint32(4, 2, true); dv.setUint32(8, total, true);
  dv.setUint32(12, jb.length, true); dv.setUint32(16, 0x4E4F534A, true); out.set(jb, 20);
  const o = 20 + jb.length; dv.setUint32(o, bin.length, true); dv.setUint32(o + 4, 0x004E4942, true); out.set(bin, o + 8);
  return out;
}

test('GLB: container reads back JSON and BIN', () => {
  const { json, bin } = readGLBContainer(makeGLB());
  assert.equal(json.asset.version, '2.0'); assert.ok(bin.length > 0);
});

test('GLB: mm Y-up file is scaled to inches, recentred, floor at z=0, x forward', async () => {
  // 457 x 300 x 457 mm box, Y up. Forward +X: x extent 457mm=18in, y(left) extent is file Z = 18in, height = file Y = 11.8in
  const m = await loadRobotGLB(THREE, makeGLB(), spec(), { orientation: { upAxis: 'Y', forwardAxis: '+X' } });
  assert.ok(m, loadRobotGLB.lastReason);
  near(m.scale, 1 / 25.4, 1e-6); assert.equal(m.units, 'mm');
  near(m.bounds.size[0], 457 / 25.4); near(m.bounds.size[1], 457 / 25.4); near(m.bounds.size[2], 300 / 25.4);
  near(m.bounds.min[2], 0, 1e-4);
  near((m.bounds.min[0] + m.bounds.max[0]) / 2, 0, 1e-4); near((m.bounds.min[1] + m.bounds.max[1]) / 2, 0, 1e-4);
  assert.deepEqual(m.warnings, []);
  let mesh = null; m.group.traverse(o => { if (o.isMesh) mesh = o; });
  assert.ok(mesh.castShadow && mesh.receiveShadow);
  assert.equal(mesh.material.color.r, 1); near(mesh.material.metalness, 0.2); near(mesh.material.roughness, 0.7);
});

test('GLB: metres are detected, and Z-up / forward axis options re-map the box', async () => {
  // 0.4572 x 0.3 x 0.2 m: Z-up with forward +Y: robot x = file Y (0.3 m = 11.8 in), robot y(left) = up x fwd = z x y = -x ... extent 18 in, z = file Z = 7.9 in
  const g = makeGLB({ size: [0.4572, 0.3, 0.2] });
  const m = await loadRobotGLB(THREE, g, spec(), { orientation: { upAxis: 'Z', forwardAxis: '+Y' } });
  assert.ok(m, loadRobotGLB.lastReason);
  assert.equal(m.units, 'm'); near(m.scale, 1 / 0.0254, 1e-6);
  near(m.bounds.size[0], 0.3 / 0.0254); near(m.bounds.size[1], 0.4572 / 0.0254); near(m.bounds.size[2], 0.2 / 0.0254);
  assert.ok(Math.abs(m.bounds.min[2]) < 1e-4);
});

test('GLB: orientationMatrix is a proper rotation; bad axes throw', () => {
  const M = orientationMatrix(THREE, { upAxis: 'Z', forwardAxis: '-Y' });
  near(M.determinant(), 1);
  const f = new THREE.Vector3(0, -1, 0).applyMatrix4(M); near(f.x, 1); near(f.y, 0); near(f.z, 0);
  assert.throws(() => orientationMatrix(THREE, { upAxis: 'Y', forwardAxis: '+Y' }));
});

test('GLB: implausible size gives a warning instead of failing; explicit units override', async () => {
  const g = makeGLB({ size: [2000, 2000, 2000] });
  const m = await loadRobotGLB(THREE, g, spec());
  assert.ok(m); assert.ok(m.warnings.length >= 1);
  const m2 = await loadRobotGLB(THREE, makeGLB({ size: [18, 11, 18] }), spec(), { units: 'in' });
  near(m2.scale, 1); assert.deepEqual(m2.warnings, []);
});

test('GLB: node hierarchy and TRS/matrix transforms are applied', async () => {
  // child node carries the mesh, parent scales by 2 and moves; box becomes 2x bigger
  const nodes = [{ name: 'Root', scale: [2, 2, 2], children: [1] }, { name: 'Child', mesh: 0, matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 5, 0, 0, 1] }];
  const m = await loadRobotGLB(THREE, makeGLB({ size: [4.5, 3, 4.5], nodes }), spec(), { units: 'in' });
  near(m.bounds.size[0], 9); near(m.bounds.size[2], 6);
});

test('GLB: findMovingParts finds turret / hood / flywheel by name, case-insensitive, topmost wins', async () => {
  const nodes = [
    { name: 'MaxBot', children: [1, 3, 4, 5] },
    { name: 'TURRET Assembly', children: [2], translation: [0, 1, 0] },
    { name: 'turret plate', mesh: 0 },
    { name: 'Shooter hood v2', mesh: 0 },
    { name: 'FlyWheel_L', mesh: 0 },
    { name: 'Drivetrain', mesh: 0 },
  ];
  const m = await loadRobotGLB(THREE, makeGLB({ size: [18, 11, 18], nodes, extra: { scenes: [{ nodes: [0] }] } }), spec(), { units: 'in' });
  assert.ok(m, loadRobotGLB.lastReason);
  const p = findMovingParts(m.group);
  assert.equal(p.turret.name, 'TURRET Assembly'); assert.equal(p.hood.name, 'Shooter hood v2'); assert.equal(p.flywheel.name, 'FlyWheel_L');
  assert.deepEqual(p.missing, []);
  const q = findMovingParts(m.group, { turret: ['yaw'], hood: [/hood/i] });
  assert.equal(q.turret, null); assert.deepEqual(q.missing, ['turret']); assert.equal(q.hood.name, 'Shooter hood v2');
  assert.equal(m.parts.turret.name, 'TURRET Assembly');
});

test('GLB: corrupt, truncated and bogus files return null with a reason (never throw)', async () => {
  const good = makeGLB();
  const cases = {
    badMagic: (() => { const g = good.slice(); g[0] = 0; return g; })(),
    truncated: good.slice(0, good.length - 40),
    tiny: new Uint8Array(5),
    badIndex: makeGLB({ corruptBin: true }),
    notJson: new TextEncoder().encode('hello world'),
    badVersion: (() => { const g = good.slice(); g[4] = 1; return g; })(),
  };
  for (const [k, v] of Object.entries(cases)) {
    const m = await loadRobotGLB(THREE, v, spec());
    assert.equal(m, null, k); assert.ok(loadRobotGLB.lastReason.length > 0, k);
  }
  await assert.rejects(parseGLB(THREE, cases.badMagic));
  await assert.rejects(parseGLB(THREE, cases.truncated));
  // draco is refused cleanly
  const dr = packGLB({ asset: { version: '2.0' }, extensionsRequired: ['KHR_draco_mesh_compression'], scenes: [{ nodes: [] }] }, new Uint8Array(4));
  assert.equal(await loadRobotGLB(THREE, dr, spec()), null);
  assert.match(loadRobotGLB.lastReason, /draco/i);
});

test('GLB: missing file (404 / network error) returns null so the host falls back', async () => {
  const f404 = async () => ({ ok: false, status: 404 });
  assert.equal(await loadRobotGLB(THREE, 'data/robot.glb', spec(), { fetchImpl: f404 }), null);
  assert.match(loadRobotGLB.lastReason, /404/);
  const fErr = async () => { throw new Error('offline'); };
  assert.equal(await loadRobotGLB(THREE, 'data/robot.glb', spec(), { fetchImpl: fErr }), null);
  assert.match(loadRobotGLB.lastReason, /offline/);
  const glb = makeGLB();
  const fOk = async () => ({ ok: true, status: 200, arrayBuffer: async () => glb.buffer.slice(glb.byteOffset, glb.byteOffset + glb.byteLength) });
  const m = await loadRobotGLB(THREE, 'data/robot.glb', spec(), { fetchImpl: fOk });
  assert.ok(m);
});

test('GLB: embedded data-URI .gltf works', async () => {
  const g = makeGLB(); const { json, bin } = readGLBContainer(g);
  json.buffers[0].uri = 'data:application/octet-stream;base64,' + Buffer.from(bin).toString('base64');
  const { scene } = await parseGLB(THREE, json);
  let tris = 0; scene.traverse(o => { if (o.isMesh) tris += o.geometry.index.count / 3; });
  assert.equal(tris, 12);
});
