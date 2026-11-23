import test from 'node:test';
import assert from 'node:assert/strict';
import { probeContacts, createOverlayState, buildDebugGeometry, energyReadout } from '../../web/js/physics/validation-tuning/overlay.js';
import { createThreeOverlay } from '../../web/js/physics/validation-tuning/overlay_three.js';
import { installValidationTools } from '../../web/js/physics/validation-tuning/install.js';
import { LAYERS, OVERLAY } from '../../web/js/physics/validation-tuning/params.js';
import { mulberry32 } from '../../web/js/physics/validation-tuning/rng.js';
import { BallWorld, BALL } from '../../web/js/balls.js';
import { fakeThree, fakeDoc } from './_fixtures.mjs';

const bounds = { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 };

function restingWorld() {
  const w = new BallWorld({ bounds }, mulberry32(1));
  w.addBall('pollen', [70, 70, BALL.r], [0, 0, 0]);
  for (let i = 0; i < 60; i++) w.step(1 / 60);
  return w;
}
function pileWorld(seed = 3, n = 12) {
  const rng = mulberry32(seed);
  const w = new BallWorld({ bounds }, rng);
  for (let i = 0; i < n; i++) w.addBall('pollen', [60 + rng() * 20, 60 + rng() * 20, 6 + rng() * 30], [0, 0, 0]);
  return w;
}
function fakeWin() {
  const l = {};
  return { addEventListener: (t, f) => (l[t] ||= []).push(f), removeEventListener: (t, f) => { l[t] = (l[t] || []).filter((x) => x !== f); },
    fire: (t, ev) => (l[t] || []).forEach((f) => f(ev)), listeners: l };
}

test('a ball resting on the floor gives a floor contact with n = [0,0,1]', () => {
  const w = restingWorld();
  const c = probeContacts(w).filter((x) => x.kind === 'floor');
  assert.equal(c.length, 1);
  assert.deepEqual(c[0].n, [0, 0, 1]);
  assert.ok(c[0].depth >= -OVERLAY.contactSlopIn && c[0].depth < 0.5);
  assert.ok(Math.abs(c[0].p[2]) < 1e-9);
});
test('a ball touching a wall gives a wall contact with an inward normal', () => {
  const w = new BallWorld({ bounds }, mulberry32(1));
  const b = w.addBall('pollen', [bounds.minX + BALL.r - 0.01, 70, 30], [0, 0, 0]);
  const c = probeContacts(w).filter((x) => x.kind === 'wall');
  assert.ok(c.length >= 1);
  assert.ok(c[0].n[0] > 0.99);
  assert.ok(b);
});
test('two overlapping balls give a ball-ball contact, normal from b to a', () => {
  const w = new BallWorld({ bounds }, mulberry32(1));
  const a = w.addBall('pollen', [70, 70, 30], [0, 0, 0]);
  w.addBall('pollen', [70 + BALL.r, 70, 30], [0, 0, 0]); // closer than 2r
  const c = probeContacts(w).filter((x) => x.kind === 'ball');
  assert.equal(c.length, 1);
  assert.ok(c[0].n[0] < -0.99, JSON.stringify(c[0].n)); // a is at smaller x, so b -> a points toward -x
  assert.ok(a);
});
test('probeContacts does not mutate the world', () => {
  const w = pileWorld();
  for (let i = 0; i < 30; i++) w.step(1 / 60);
  const before = JSON.stringify(w.balls.map((b) => [b.x, b.y, b.z, b.vx, b.vy, b.vz]));
  probeContacts(w); buildDebugGeometry(w, { layers: 0xffff, state: createOverlayState(), dt: 1 / 60 });
  assert.equal(JSON.stringify(w.balls.map((b) => [b.x, b.y, b.z, b.vx, b.vy, b.vz])), before);
});
test('positions.length = segCount * 6 and colors match', () => {
  const w = pileWorld();
  for (let i = 0; i < 20; i++) w.step(1 / 60);
  const g = buildDebugGeometry(w, { layers: LAYERS.DEFAULT, state: createOverlayState(), dt: 1 / 60 });
  assert.ok(g.segCount > 0);
  assert.equal(g.positions.length, g.segCount * 6);
  assert.equal(g.colors.length, g.segCount * 6);
  assert.ok(g.positions instanceof Float32Array && g.colors instanceof Float32Array);
  for (const v of g.positions) assert.ok(Number.isFinite(v));
  for (const v of g.colors) assert.ok(v >= 0 && v <= 1);
});
test('layer masking removes segments', () => {
  const w = pileWorld();
  for (let i = 0; i < 60; i++) w.step(1 / 60);
  const st = () => createOverlayState();
  const count = (layers) => buildDebugGeometry(w, { layers, state: st(), dt: 1 / 60 }).segCount;
  assert.equal(count(0), 0);
  const col = count(LAYERS.COLLIDERS), vel = count(LAYERS.VELOCITY), axes = count(LAYERS.BODY_AXES), both = count(LAYERS.COLLIDERS | LAYERS.VELOCITY);
  assert.ok(col > 0 && axes > 0);
  assert.equal(both, col + vel);
  assert.ok(count(LAYERS.DEFAULT) > col);
  assert.equal(count(LAYERS.BODY_AXES), axes);
  assert.equal(axes, 3 * w.balls.length);
});
test('AABB layer draws 12 segments per ball', () => {
  const w = pileWorld(2, 5);
  assert.equal(buildDebugGeometry(w, { layers: LAYERS.AABBS, state: createOverlayState(), dt: 1 / 60 }).segCount, 12 * 5);
});
test('energy is always returned in joules', () => {
  const w = restingWorld();
  const g = buildDebugGeometry(w, { layers: 0, state: createOverlayState(), dt: 1 / 60 });
  assert.ok(g.energy.totalJ >= 0);
  assert.ok(Math.abs(g.energy.totalJ - (g.energy.kinJ + g.energy.potJ)) < 1e-12);
  assert.deepEqual(energyReadout(w), g.energy);
});
test('trails accumulate in state and are capped at trailLen', () => {
  const w = pileWorld(4, 3);
  const st = createOverlayState();
  for (let i = 0; i < OVERLAY.trailLen + 30; i++) { w.step(1 / 60); buildDebugGeometry(w, { layers: LAYERS.TRAILS, state: st, dt: 1 / 60 }); }
  for (const tr of st.trails.values()) assert.ok(tr.length <= OVERLAY.trailLen && tr.length > 10);
});
test('three adapter works with a fake THREE: growable buffers, draw range, visibility, dispose', () => {
  const THREE = fakeThree();
  const parent = new THREE.Scene();
  const ov = createThreeOverlay(THREE, parent);
  assert.ok(parent.children.includes(ov.object3d));
  assert.equal(ov.object3d.renderOrder, 999);
  const w = pileWorld(5, 20);
  const g = buildDebugGeometry(w, { layers: LAYERS.COLLIDERS | LAYERS.AABBS, state: createOverlayState(), dt: 1 / 60 });
  ov.update(g);
  const geo = ov.object3d.geometry;
  assert.equal(geo.drawRange.count, g.segCount * 2);
  assert.ok(geo.attributes.position.array.length >= g.positions.length);
  assert.deepEqual(Array.from(geo.attributes.position.array.slice(0, 12)), Array.from(g.positions.slice(0, 12)));
  const small = buildDebugGeometry(w, { layers: LAYERS.BODY_AXES, state: createOverlayState(), dt: 1 / 60 });
  ov.update(small);
  assert.equal(geo.drawRange.count, small.segCount * 2);
  ov.setVisible(false); assert.equal(ov.visible, false); assert.equal(ov.object3d.visible, false);
  ov.setVisible(true); assert.equal(ov.visible, true);
  ov.dispose();
  assert.ok(!parent.children.includes(ov.object3d));
});
test('determinism: 600 steps with install tick on the wrapped step equal 600 plain steps on x..vz', () => {
  const plain = pileWorld(9, 25), wrapped = pileWorld(9, 25);
  const THREE = fakeThree(), scene3d = new THREE.Scene(), win = fakeWin();
  const stats = [];
  const tools = installValidationTools({ THREE, scene3d, world: wrapped, getDrive: () => null, diag: { setPhysicsStats: (s) => stats.push(s) }, doc: fakeDoc(), win });
  tools.toggleOverlay(); // overlay visible so geometry is built every frame
  for (let i = 0; i < 600; i++) { plain.step(1 / 60); wrapped.step(1 / 60); tools.tick(16); }
  assert.equal(plain.balls.length, wrapped.balls.length);
  for (let i = 0; i < plain.balls.length; i++) {
    for (const k of ['x', 'y', 'z', 'vx', 'vy', 'vz']) assert.equal(wrapped.balls[i][k], plain.balls[i][k], `ball ${i} ${k}`);
  }
  assert.ok(stats.length >= 600 / OVERLAY.hudEveryFrames - 1, `HUD updates ${stats.length}`);
  const s = tools.stats();
  assert.ok(Number.isFinite(s.stepMs) && s.balls === 25);
  tools.dispose();
  assert.equal(wrapped.step, BallWorld.prototype.step);
});
test('stepFixed also goes through the wrapped step and still matches', () => {
  const plain = pileWorld(10, 10), wrapped = pileWorld(10, 10);
  const tools = installValidationTools({ world: wrapped, doc: fakeDoc(), win: fakeWin() });
  for (let i = 0; i < 60; i++) { plain.stepFixed(1 / 60); wrapped.stepFixed(1 / 60); tools.tick(16); }
  for (let i = 0; i < plain.balls.length; i++) for (const k of ['x', 'y', 'z', 'vx', 'vy', 'vz']) assert.equal(wrapped.balls[i][k], plain.balls[i][k]);
  tools.dispose();
});
test('F8 toggles the overlay and F9 the panel; __vt is exposed; dispose removes listeners', () => {
  const w = pileWorld(1, 3), THREE = fakeThree(), win = fakeWin(), doc = fakeDoc();
  const tools = installValidationTools({ THREE, scene3d: new THREE.Scene(), world: w, getDrive: () => null, doc, win });
  assert.ok(win.__vt);
  const was = tools.overlay.visible;
  const press = (key) => { win.fire('keydown', { key, preventDefault() {} }); doc.fire('keydown', { key, preventDefault() {} }); };
  press(OVERLAY.keyOverlay);
  assert.notEqual(tools.overlay.visible, was);
  tools.dispose();
});
