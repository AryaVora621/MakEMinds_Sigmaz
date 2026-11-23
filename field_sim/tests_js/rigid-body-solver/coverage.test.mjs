// Round 2 coverage: restitution symmetry, kinematic robots in every position mode, polygon colliders, removeBody.
// All units SI (m, kg, s, rad).
import test, { beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { resetBodyIds, World, createBody, makePolygon, makeBox } from '../../web/js/physics/rigid-body-solver/index.js';
import { runFor, makeWall, makeRobot, penetrationOf, ROBOT_HALF_M as H } from './helpers.mjs';

beforeEach(() => resetBodyIds());

const MODES = ['soft', 'baumgarte', 'ngs'];

// ---------- restitution ----------

// Square-on impact, e = 0.5, 2 m/s. Analytic rebound = e*v = 1.0 m/s, times linear damping
// exp(-0.1 1/s * ~0.38 s) = 0.963 over the rest of the 0.4 s run -> about 0.96 m/s. Spin must stay 0
// by symmetry (the single-pass solver gave 1.28 rad/s here).
for (const mode of MODES) {
  test(`square-on bounce is symmetric: rebound near e*v and no spin (${mode})`, () => {
    const w = new World({ solver: { positionMode: mode } });
    makeWall(w, 1, 0, 0.5, 5, { restitution: 0.5 });
    const r = makeRobot(w, { x: 0.2, y: 0, drive: false, vx: 2, restitution: 0.5 });
    runFor(w, 0.4);
    assert.ok(-r.linearVelocity.x > 0.93 && -r.linearVelocity.x < 1.01, 'rebound ' + -r.linearVelocity.x);
    assert.ok(Math.abs(r.angularVelocity) < 1e-3, 'spin ' + r.angularVelocity);
    // 1e-3 m/s: friction sees the residual left by restitutionTolerance (1e-4 m/s), measured |vy| <= 1.1e-4
    assert.ok(Math.abs(r.linearVelocity.y) < 1e-3, 'vy ' + r.linearVelocity.y);
  });
}

test('off-centre bounce mirrored in y gives mirrored spin', () => {
  // Wall box covers only the top (or bottom) 0.08 m of the robot face: one-sided impact that spins the body.
  // Symmetry check of the off-centre path. This is a 1-point contact, so it passes with 1 or 8 restitution passes;
  // the 2-point order dependence is caught by the square-on tests above (1 pass: 0.68 m/s, 1.24 rad/s).
  const run = (sign) => {
    resetBodyIds();
    const w = new World();
    makeWall(w, 1, 0.35 * sign, 0.5, 0.2, { restitution: 0.5 });
    const r = makeRobot(w, { x: 0.2, y: 0, drive: false, vx: 2, restitution: 0.5 });
    runFor(w, 0.4);
    return r;
  };
  const p = run(1), m = run(-1);
  assert.ok(Math.abs(p.angularVelocity) > 0.1, 'no spin from off-centre hit: ' + p.angularVelocity);
  assert.ok(Math.abs(p.linearVelocity.x - m.linearVelocity.x) < 5e-3, `vx ${p.linearVelocity.x} vs ${m.linearVelocity.x}`);
  assert.ok(Math.abs(p.linearVelocity.y + m.linearVelocity.y) < 5e-3, `vy ${p.linearVelocity.y} vs ${m.linearVelocity.y}`);
  assert.ok(Math.abs(p.angularVelocity + m.angularVelocity) < 1e-2, `w ${p.angularVelocity} vs ${m.angularVelocity}`);
});

// ---------- kinematic robots ----------

for (const mode of MODES) {
  test(`kinematic robot pushes a dynamic one, keeps its own pose path exactly (${mode})`, () => {
    const w = new World({ solver: { positionMode: mode } });
    const k = makeRobot(w, { x: 0, y: 0, type: 'kinematic', target: { vx: 1, vy: 0, om: 0 } });
    const d = makeRobot(w, { x: 2 * H + 0.2, y: 0, drive: false });
    let maxPen = 0;
    runFor(w, 2, 1 / 60, (i, t) => { if (t > 0.5) maxPen = Math.max(maxPen, penetrationOf(w, d)); });
    // kinematic: infinite mass, so NGS / soft / baumgarte corrections must never move it
    assert.ok(Math.abs(k.linearVelocity.x - 1) < 1e-9, 'kinematic vx ' + k.linearVelocity.x);
    assert.ok(Math.abs(k.position.x - 2) < 1e-9, 'kinematic x ' + k.position.x);
    assert.ok(Math.abs(k.position.y) < 1e-12 && Math.abs(k.angle) < 1e-12, 'kinematic drifted');
    assert.ok(d.linearVelocity.x > 0.9, 'pushed body vx ' + d.linearVelocity.x);
    assert.ok(d.position.x > k.position.x + 2 * H - 0.012, 'pushed body inside kinematic: ' + (d.position.x - k.position.x));
    assert.ok(maxPen < 0.012, 'penetration ' + maxPen);
  });

  test(`kinematic robot ignores static walls (no kinematic-static pair) (${mode})`, () => {
    const w = new World({ solver: { positionMode: mode } });
    makeWall(w, 1, 0, 0.5, 5);
    const k = makeRobot(w, { x: 0, y: 0, type: 'kinematic', target: { vx: 1, vy: 0, om: 0 } });
    runFor(w, 1);
    assert.equal(w.arbiters.size, 0);
    assert.ok(Math.abs(k.position.x - 1) < 1e-9, 'kinematic x ' + k.position.x);
  });

  test(`rotating kinematic robot spins a resting dynamic neighbour (${mode})`, () => {
    const w = new World({ solver: { positionMode: mode } });
    const k = makeRobot(w, { x: 0, y: 0, type: 'kinematic', target: { vx: 0, vy: 0, om: 3 } });
    const d = makeRobot(w, { x: 2 * H + 0.005, y: 0, drive: false });
    runFor(w, 0.5);
    assert.ok(Math.abs(k.angle - 1.5) < 1e-9, 'kinematic angle ' + k.angle);
    assert.ok(Math.hypot(d.position.x - 2 * H - 0.005, d.position.y) > 0.05, 'neighbour not moved');
  });
}

// ---------- polygon colliders ----------

function hexagon(r) {
  const pts = [];
  for (let i = 0; i < 6; i++) pts.push([r * Math.cos(i * Math.PI / 3 + Math.PI / 6), r * Math.sin(i * Math.PI / 3 + Math.PI / 6)]);
  return makePolygon(pts); // flat faces at +-x (vertices at +-30 deg)
}

for (const mode of MODES) {
  test(`force-pushed hexagon body rests against a wall with small penetration and no jitter (${mode})`, () => {
    const w = new World({ solver: { positionMode: mode } });
    makeWall(w, 1, 0, 0.5, 5); // face at x = 0.5
    // mass properties come from the polygon (shapeUnitInertia)
    const hx = createBody({ shape: hexagon(0.25), x: 0, y: 0, mass: 15 }, w.params);
    w.addBody(hx);
    const flat = 0.25 * Math.cos(Math.PI / 6); // m, centre-to-face distance
    const xs = [];
    let maxPen = 0;
    runFor(w, 3, 1 / 60, (i, t) => {
      hx.force.x = 60; // N, constant push into the wall
      if (t > 2) xs.push(hx.position.x);
      if (t > 1) maxPen = Math.max(maxPen, penetrationOf(w, hx));
    });
    assert.ok(maxPen < 0.012, 'penetration ' + maxPen);
    assert.ok(Math.max(...xs) - Math.min(...xs) < 0.002, 'jitter');
    assert.ok(Math.abs(hx.position.x - (0.5 - flat)) < 0.012, 'rest x ' + hx.position.x);
    assert.ok(Math.abs(hx.angle) < 0.01, 'tipped to angle ' + hx.angle);
  });
}

test('hexagon bounce off a flat face rebounds at about e*v with no spin', () => {
  const w = new World();
  makeWall(w, 1, 0, 0.5, 5, { restitution: 0.5 });
  const hx = w.addBody(createBody({ shape: hexagon(0.25), x: 0.2, y: 0, mass: 15, vx: 2, restitution: 0.5 }, w.params));
  runFor(w, 0.4);
  assert.ok(-hx.linearVelocity.x > 0.93 && -hx.linearVelocity.x < 1.01, 'rebound ' + -hx.linearVelocity.x);
  assert.ok(Math.abs(hx.angularVelocity) < 1e-3, 'spin ' + hx.angularVelocity);
});

test('static wedge polygon deflects a robot sliding into its slope', () => {
  const w = new World();
  // right triangle, hypotenuse facing -x/+y at 45 deg
  w.addBody(createBody({ type: 'static', shape: makePolygon([[0.5, -1], [1.5, -1], [1.5, 0]]), x: 0, y: 0 }, w.params));
  const r = makeRobot(w, { x: 0, y: -0.9 + H, drive: false, vx: 1.5 });
  // start just clear of the slope
  r.position.x = 0.5 - H - 0.05;
  let hit = false;
  // contactsOf only holds the last substep, so watch the arbiter map instead
  runFor(w, 1, 1 / 60, () => { if (w.arbiters.size) hit = true; });
  assert.ok(hit, 'never touched the wedge');
  assert.ok(r.linearVelocity.y > 0.1, 'not deflected upward, vy ' + r.linearVelocity.y);
});

test('polygon vs polygon dynamic pair conserves linear momentum when free', () => {
  const w = new World({ motion: { linearDamping: 0, angularDamping: 0 } });
  const a = w.addBody(createBody({ shape: hexagon(0.2), x: 0, y: 0, mass: 10, vx: 1, linearDamping: 0, angularDamping: 0 }, w.params));
  const b = w.addBody(createBody({ shape: makeBox(0.15, 0.3), x: 0.6, y: 0.05, mass: 5, linearDamping: 0, angularDamping: 0 }, w.params));
  const p0 = 10 * 1;
  runFor(w, 1);
  const px = 10 * a.linearVelocity.x + 5 * b.linearVelocity.x;
  const py = 10 * a.linearVelocity.y + 5 * b.linearVelocity.y;
  assert.ok(b.linearVelocity.x > 0.1, 'no collision happened');
  assert.ok(Math.abs(px - p0) < 1e-9 && Math.abs(py) < 1e-9, `momentum (${px}, ${py})`);
});

// ---------- removeBody ----------

test('removeBody by object drops the body and its arbiters, later steps ignore it', () => {
  const w = new World();
  const wall = makeWall(w, 1, 0, 0.5, 5);
  const r = makeRobot(w, { x: 0.1, y: 0, target: { vx: 1, vy: 0, om: 0 } });
  runFor(w, 1);
  assert.ok(w.arbiters.size > 0, 'expected contact before removal');
  w.removeBody(wall);
  assert.equal(w.bodies.length, 1);
  assert.equal(w.arbiters.size, 0);
  assert.equal(w.getBody(wall.id), undefined);
  runFor(w, 1);
  assert.ok(r.position.x > 0.5, 'robot still blocked at x ' + r.position.x);
  assert.equal(w.arbiters.size, 0);
});

test('removeBody by id works and an unknown id is a no-op', () => {
  const w = new World();
  const a = makeRobot(w, { x: 0, y: 0, drive: false });
  const b = makeRobot(w, { x: 2 * H - 0.005, y: 0, drive: false });
  w.step(1 / 60);
  assert.equal(w.arbiters.size, 1);
  w.removeBody(9999);
  assert.equal(w.bodies.length, 2);
  assert.equal(w.arbiters.size, 1);
  w.removeBody(b.id);
  assert.deepEqual(w.bodies.map((x) => x.id), [a.id]);
  assert.equal(w.arbiters.size, 0);
  w.step(1 / 60);
  assert.deepEqual(w.contactsOf(a), []);
});
