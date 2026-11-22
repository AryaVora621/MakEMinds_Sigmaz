import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from '../web/vendor/three.module.js';
import { BallWorld, ballsInBox } from '../web/js/balls.js';
import { FlowerArm, findFlowerContact, feedFromFlower, ease } from '../web/js/flowerarm.js';
import { buildFlowerArm } from '../web/js/flowerarm3d.js';

const rd = p => JSON.parse(readFileSync(new URL(p, import.meta.url), 'utf8'));
const cfg = () => rd('../web/data/flower_arm.json');
const scene = rd('../web/data/scene.json');
const flowers = scene.constants.flowers;
const near = (a, b, t = 1e-6) => assert.ok(Math.abs(a - b) <= t, `${a} vs ${b}`);
const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 };
const newWorld = () => { const w = new BallWorld({ bounds }); w.reset(scene.balls.balls); return w; };
const mouth = p => ({ x: p.x + 11 * Math.cos(p.h), y: p.y + 11 * Math.sin(p.h), h: p.h });
const deployed = () => { const a = new FlowerArm(cfg()); a.deploy(); for (let i = 0; i < 40; i++) a.update(0.02); return a; };
// robot centre so that the tip sits on the flower opening, facing it
function poseFor(arm, fl) {
  const oc = fl.opening_center, tx = arm.tipRobot()[0];
  const dir = { far: [0, 1], audience: [0, -1], red_side: [-1, 0], blue_side: [1, 0] }[fl.wall];
  const h = Math.atan2(dir[1], dir[0]);
  return { x: oc[0] - dir[0] * tx, y: oc[1] - dir[1] * tx, h };
}

test('starts stowed at stowed angle', () => {
  const a = new FlowerArm(cfg());
  assert.equal(a.state, 'stowed'); assert.equal(a.angleDeg, 90); assert.equal(a.progress, 0);
});
test('ease endpoints and shape', () => {
  near(ease(0), 0); near(ease(1), 1); near(ease(0.5), 0.5);
  assert.ok(ease(0.25) < 0.25 && ease(0.75) > 0.75);
});
test('deploy transitions deploying -> deployed in deployS', () => {
  const a = new FlowerArm(cfg()); a.deploy(); assert.equal(a.state, 'deploying');
  a.update(0.2); assert.equal(a.state, 'deploying'); assert.ok(a.progress > 0.4 && a.progress < 0.5);
  a.update(0.26); assert.equal(a.state, 'deployed'); near(a.angleDeg, -35);
});
test('stow transitions stowing -> stowed', () => {
  const a = deployed(); a.stow(); assert.equal(a.state, 'stowing');
  a.update(0.2); assert.equal(a.state, 'stowing'); a.update(0.3); assert.equal(a.state, 'stowed'); near(a.angleDeg, 90);
});
test('toggle flips and mid-motion reverses continuously', () => {
  const a = new FlowerArm(cfg()); a.toggle(); assert.equal(a.state, 'deploying');
  a.update(0.2); const p = a.progress; a.toggle(); assert.equal(a.state, 'stowing'); assert.equal(a.progress, p);
  a.update(0.1); assert.ok(a.progress < p);
});
test('redundant deploy/stow are no-ops; reset works', () => {
  const a = new FlowerArm(cfg()); a.stow(); assert.equal(a.state, 'stowed');
  a.deploy(); a.deploy(); assert.equal(a.state, 'deploying');
  a.update(1); a.reset(); assert.equal(a.state, 'stowed'); assert.equal(a.progress, 0);
});
test('onState callbacks fire', () => {
  const a = new FlowerArm(cfg()), log = []; a.onState((s, o) => log.push(o + '>' + s));
  a.deploy(); a.update(1); a.stow(); a.update(1);
  assert.deepEqual(log, ['stowed>deploying', 'deploying>deployed', 'deployed>stowing', 'stowing>stowed']);
});
test('tipWorld for several headings (deployed)', () => {
  const a = deployed(), [lx, , lz] = a.tipRobot(); const c0 = cfg(), drop = c0.paddle.height + c0.paddle.rubberThickness - 0.4; near(lz, c0.hinge.z + c0.arm.length * Math.sin(-35 * Math.PI / 180) - drop);
  for (const h of [0, Math.PI / 2, Math.PI, -Math.PI / 2, 0.7]) {
    const t = a.tipWorld({ x: 50, y: 60, h });
    near(t[0], 50 + Math.cos(h) * lx); near(t[1], 60 + Math.sin(h) * lx); near(t[2], lz);
  }
});
test('stowed tip is upright over the hinge (inside the 18 in envelope); deployed tip is in front and low', () => {
  const a = new FlowerArm(cfg()), t = a.tipRobot(), c = cfg();
  assert.ok(t[2] > c.hinge.z && t[2] < 18 && Math.abs(t[0] - c.hinge.x) < 0.5);
  const d = deployed().tipRobot(); assert.ok(d[0] > 13 && d[2] >= c.reach.minTipZ && d[2] < 3);
});
test('rear arm mirrors the tip and the alignment', () => {
  const c = { ...cfg(), side: 'rear' }, a = new FlowerArm(c); a.deploy(); for (let i = 0; i < 40; i++) a.update(0.02);
  const f = deployed(); near(a.tipRobot()[0], -f.tipRobot()[0]); near(a.tipRobot()[2], f.tipRobot()[2]);
  const fl = flowers[0], dir = { far: [0, 1], audience: [0, -1], red_side: [-1, 0], blue_side: [1, 0] }[fl.wall], tx = f.tipRobot()[0];
  const p = { x: fl.opening_center[0] - dir[0] * tx, y: fl.opening_center[1] - dir[1] * tx, h: Math.atan2(-dir[1], -dir[0]) };
  assert.equal(findFlowerContact(a, p, flowers).flower.id, fl.id);
});
test('contact: aligned deployed arm finds the flower', () => {
  const a = deployed(), fl = flowers[0], p = poseFor(a, fl);
  const c = findFlowerContact(a, p, flowers);
  assert.equal(c.flower.id, fl.id); assert.ok(c.distance < 0.01); assert.ok(c.alignmentDeg < 0.01);
});
test('contact: misaligned heading, too far, not deployed all null', () => {
  const a = deployed(), fl = flowers[0], p = poseFor(a, fl);
  assert.equal(findFlowerContact(a, { ...p, h: p.h + 0.5 }, flowers), null);
  assert.equal(findFlowerContact(a, { ...p, y: p.y - 6 }, flowers), null);
  const s = new FlowerArm(cfg()); assert.equal(findFlowerContact(s, p, flowers), null);
  const m = new FlowerArm(cfg()); m.deploy(); m.update(0.2); assert.equal(findFlowerContact(m, p, flowers), null);
});
test('feed releases one ball per interval, top ball first, into the intake mouth', () => {
  const a = deployed(), w = newWorld(), fl = flowers[0], p = poseFor(a, fl), hop = { count: 0, cap: 4 };
  const c = findFlowerContact(a, p, flowers);
  const inFl = () => w.balls.filter(b => /flower 1 /.test(scene.balls.balls.find(s => s.id === b.id)?.where || ''));
  assert.equal(inFl().length, 4);
  const topId = inFl().sort((x, y) => y.z - x.z)[0].id;
  assert.equal(feedFromFlower(a, w, c, 0.2, hop, p).released, 0);
  const r = feedFromFlower(a, w, c, 0.06, hop, p); assert.equal(r.released, 1);
  const rel = w.balls.find(b => b.id === topId); assert.ok(!rel.sleeping && rel.state === 'free');
  let reached = false;
  for (let i = 0; i < 90 && !reached; i++) { w.step(1 / 120); if (ballsInBox(w, mouth(p), 4, 8.5).includes(rel)) reached = true; }
  assert.ok(reached, 'ball reached intake box');
});
test('feed respects per-deploy cap and hopper capacity', () => {
  const a = deployed(), w = newWorld(), fl = flowers[1], p = poseFor(a, fl), c = findFlowerContact(a, p, flowers);
  let n = 0; for (let i = 0; i < 100; i++) { n += feedFromFlower(a, w, c, 0.05, { count: 0, cap: 99 }, p).released; w.step(0.05); }
  assert.equal(n, 4);
  const a2 = deployed(), w2 = newWorld(), p2 = poseFor(a2, fl); let m = 0;
  for (let i = 0; i < 20; i++) m += feedFromFlower(a2, w2, findFlowerContact(a2, p2, flowers), 0.05, { count: 3, cap: 4 }, p2).released;
  assert.equal(m, 1);
  const a3 = deployed(); assert.equal(feedFromFlower(a3, newWorld(), findFlowerContact(a3, p2, flowers), 1, { count: 4, cap: 4 }, p2).released, 0);
});
test('new deploy resets the per-deploy cap', () => {
  const a = deployed(), w = newWorld(), p = poseFor(a, flowers[2]);
  for (let i = 0; i < 30; i++) feedFromFlower(a, w, findFlowerContact(a, p, flowers), 0.1, { count: 0, cap: 99 }, p);
  assert.equal(a.fedCount, 4);
  a.stow(); a.update(1); a.deploy(); a.update(1); assert.equal(a.fedCount, 0);
});
test('feed works on every flower and balls reach the mouth', () => {
  for (const fl of flowers) {
    const a = deployed(), w = newWorld(), p = poseFor(a, fl), hop = { count: 0, cap: 4 };
    const c = findFlowerContact(a, p, flowers); assert.equal(c.flower.id, fl.id);
    const got = new Set();
    for (let i = 0; i < 360; i++) {
      feedFromFlower(a, w, c, 1 / 120, hop, p); w.step(1 / 120);
      for (const b of ballsInBox(w, mouth(p), 4, 8.5)) got.add(b.id);
    }
    assert.ok(got.size >= 3, `flower ${fl.id}: ${got.size} balls reached the mouth`);
  }
});
test('3D builder builds, setAngle rotates, wrist keeps the paddle hanging, bbox sane', () => {
  const c = cfg(), A = buildFlowerArm(THREE, c);
  assert.ok(A.group.isObject3D && A.hinge && A.wrist && typeof A.setAngle === 'function');
  A.setAngle(0); near(A.hinge.rotation.y, 0);
  A.setAngle(90); near(A.hinge.rotation.y, -Math.PI / 2); near(A.wrist.rotation.y, Math.PI / 2);
  A.group.updateMatrixWorld(true);
  const bb = new THREE.Box3().setFromObject(A.group), sz = bb.getSize(new THREE.Vector3());
  assert.ok(sz.y > 10 && sz.y < 16, sz.toArray().join());
  assert.ok(bb.max.z <= 18 && bb.max.z > c.hinge.z + c.arm.length - 1, 'upright arm stays inside the 18 in envelope');
  let tris = 0; A.group.traverse(o => { if (o.isMesh) tris += o.geometry.index.count / 3; });
  assert.ok(tris < 1500);
  A.setAngle(-35); A.group.updateMatrixWorld(true);
  const lo = new THREE.Box3().setFromObject(A.group);
  assert.ok(lo.max.x > c.hinge.x + c.arm.length * Math.cos(35 * Math.PI / 180) - 1 && lo.min.z >= -0.1 && lo.min.z < 2);
  // the model's paddle bottom agrees with the logic's tip height
  const arm = new FlowerArm(c); arm.deploy(); for (let i = 0; i < 40; i++) arm.update(0.02); A.sync(arm); A.group.updateMatrixWorld(true);
  const rub = new THREE.Vector3(); A.parts.rubber.getWorldPosition(rub);
  near(rub.z - c.paddle.rubberThickness / 2, arm.tipRobot()[2], 1e-6); near(rub.x, arm.tipRobot()[0], 1e-6);
  A.setFraction(0); near(A.hinge.rotation.y, -Math.PI / 2);
  const r = buildFlowerArm(THREE, { ...c, side: 'rear' }); r.setAngle(-35); r.group.updateMatrixWorld(true);
  assert.ok(new THREE.Box3().setFromObject(r.group).min.x < -c.hinge.x);
});
test('flower_arm.json has provenance tags and cad_todo', () => {
  const c = cfg(); assert.ok(c.cad_todo.length > 0); assert.ok(c.hinge.prov['*']); assert.equal(c.feed.feedIntervalS, 0.25);
});

test('regression: FlowerArm without a config (failed json fetch) still deploys and feeds', async () => {
  const { DEFAULT_FLOWER_CFG } = await import('../web/js/flowerarm.js');
  const a = new FlowerArm(undefined); assert.equal(a.state, 'stowed');
  a.toggle(); for (let i = 0; i < 40; i++) a.update(0.02);
  assert.equal(a.state, 'deployed'); near(a.angleDeg, DEFAULT_FLOWER_CFG.angles.deployedDeg);
  const b = new FlowerArm({ angles: { deployedDeg: -50 } }); b.deploy(); b.update(5); near(b.angleDeg, -50); assert.equal(b.cfg.angles.stowedDeg, 90);
});
test('regression: logic angle drives the 3D hinge and the paddle drops below the stowed height', () => {
  const a = new FlowerArm(cfg()), m = buildFlowerArm(THREE, cfg());
  m.group.updateMatrixWorld(true); const z0 = new THREE.Vector3(); m.parts.paddle.getWorldPosition(z0);
  const r0 = m.hinge.rotation.y; a.deploy(); for (let i = 0; i < 40; i++) a.update(0.02); m.sync(a);
  m.group.updateMatrixWorld(true); const z1 = new THREE.Vector3(); m.parts.paddle.getWorldPosition(z1);
  assert.ok(m.hinge.rotation.y !== r0); assert.ok(z1.z < z0.z - 8, `paddle z ${z0.z} -> ${z1.z}`);
  assert.ok(z1.x > z0.x + 4);
});
