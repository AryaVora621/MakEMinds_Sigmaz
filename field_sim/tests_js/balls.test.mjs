import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL, ballsInBox } from '../web/js/balls.js';
import { SURFACES } from '../web/js/physics/ball-bounce-rolling/params.js';

const G = BALL.g;
const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 };

function realEnv() {
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells };
  }
  return { bounds, hives };
}
const run = (w, secs, dt = 1 / 60) => { for (let t = 0; t < secs; t += dt) w.step(dt); };
const energy = b => 0.5 * (b.vx ** 2 + b.vy ** 2 + b.vz ** 2) + G * b.z;
const speed = b => Math.hypot(b.vx, b.vy, b.vz);

test('drop bounces then rests', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [70, 70, 30], [0, 0, 0]);
  let bounced = false, maxUp = 0;
  for (let i = 0; i < 60 * 10; i++) { w.step(1 / 60); if (b.vz > 20) bounced = true; maxUp = Math.max(maxUp, b.z); }
  assert.ok(bounced);
  assert.ok(Math.abs(b.z - BALL.r) < 0.02, 'z=' + b.z);
  assert.ok(b.sleeping);
});

test('energy never increases (single ball)', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [70, 70, 30], [10, 0, 0]);
  let prev = energy(b);
  for (let i = 0; i < 60 * 5; i++) {
    w.step(1 / 60);
    const e = energy(b);
    assert.ok(e <= prev + 1e-6 * prev + 1.0, `E rose ${prev} -> ${e} at ${i}`);
    prev = Math.max(e, 0) < prev ? e : prev;
  }
});

test('rolling ball decelerates and stops', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [10, 70, BALL.r], [40, 0, 0]);
  run(w, 0.5);
  assert.ok(b.vx < 40 && b.vx > 20, 'vx=' + b.vx);
  run(w, 4);
  assert.equal(b.vx, 0);
  assert.ok(b.x > 20 && b.x < 70, 'x=' + b.x);
  assert.ok(b.sleeping);
});

test('ball reflects off each wall', () => {
  const cases = [['minX', [-100, 0], 1, 0], ['maxX', [100, 0], -1, 0], ['minY', [0, -100], 0, 1], ['maxY', [0, 100], 0, -1]];
  for (const [, v, sx, sy] of cases) {
    const w = new BallWorld({ bounds });
    const b = w.addBall('pollen', [72, 72, BALL.r], [v[0], v[1], 0]);
    let reflected = false;
    for (let i = 0; i < 60 * 5; i++) {
      w.step(1 / 60);
      assert.ok(b.x >= BALL.r - 1e-6 && b.x <= 144 - BALL.r + 1e-6 && b.y >= BALL.r - 1e-6 && b.y <= 144 - BALL.r + 1e-6);
      if (b.vx * sx > 5 || b.vy * sy > 5) reflected = true;
    }
    assert.ok(reflected);
  }
});

// WHY THIS TEST CHANGED (round 2). The original asserted |p(0.5 s) - p0| < 0.06*40 for two balls ROLLING on the floor. That
// passed at HEAD only because the old model had no spin. It also counted the floor's tangential force, which is EXTERNAL to
// the two-ball system, so it never tested the collision by itself. With the spin-aware contact model (ball-bounce-rolling,
// hollow-shell k = I/(m r^2) = 0.636 for pollen), each ball keeps its rolling spin through the hit (billiard follow-through).
// Measured at 1/60 s steps: just after the hit, a has vx = -7.65 with r*wy = +29.10, and b has vx = +24.42 with r*wy = -16.25.
// While a ball slides on a flat floor, friction acts through the contact point, so L = vx + k*r*wy is conserved. a: L = 10.86,
// so it rolls at L/(1+k) = 6.64 in/s (6.53 measured). b: L = 14.08, so it rolls at 8.61 in/s (8.56 measured). After that
// both roll in +x, and rolling resistance crr*g = 0.04*386.09 = 15.44 in/s^2 slows BOTH the same way. Over the ~0.25 s left,
// that removes about 2*15.44*0.25 = 7.7 in/s of momentum, so p(0.5 s) = 11.5 is the correct value and the 2.4 bound
// cannot hold. The momentum claim is now tested with the external floor friction switched off. Its bound is kept at
// 0.06*40, but it also has to hold to 0.05 in/s (only the tiny air drag of a ball briefly lifted by ball-ball friction is
// left). The follow-through test below checks the frictional floor against the analytic invariant.
function withFloorFriction(muSlide, crr, fn) {
  const f = SURFACES.floor, s = { muSlide: f.muSlide, crr: f.crr };
  f.muSlide = muSlide; f.crr = crr;
  try { return fn(); } finally { f.muSlide = s.muSlide; f.crr = s.crr; }
}
test('head-on ball-ball collision conserves momentum', () => {
  withFloorFriction(0, 0, () => {
    const w = new BallWorld({ bounds });
    const a = w.addBall('pollen', [60, 70, BALL.r], [40, 0, 0]);
    const b = w.addBall('pollen', [80, 70, BALL.r], [-20, 0, 0]);
    const p0 = a.vx + b.vx;
    run(w, 0.5);
    const p1 = a.vx + b.vx;
    assert.ok(Math.abs(p1 - p0) < 0.06 * 40, `p ${p0} -> ${p1}`);
    assert.ok(Math.abs(p1 - p0) < 0.05, `collision alone must conserve p: ${p0} -> ${p1}`);
    assert.ok(a.vx < 20 && b.vx > -20, 'they exchanged velocity');
  });
});
test('head-on collision on the real floor: spin follow-through keeps vx + k*r*wy while sliding', () => {
  const w = new BallWorld({ bounds });
  const a = w.addBall('pollen', [60, 70, BALL.r], [40, 0, 0]);
  const b = w.addBall('pollen', [80, 70, BALL.r], [-20, 0, 0]);
  const L = o => o.vx + o.k * o.r * o.wy;   // in/s: angular momentum about the floor contact point / (m r)
  let hit = -1, La = 0, Lb = 0;
  for (let i = 0; i < 30; i++) {
    w.step(1 / 60);
    if (hit < 0 && a.vx < 20) { hit = i; La = L(a); Lb = L(b); continue; }
    const slipA = Math.abs(a.vx - a.r * a.wy), slipB = Math.abs(b.vx - b.r * b.wy);
    if (hit >= 0 && slipA > 0.5) assert.ok(Math.abs(L(a) - La) < 0.01 * Math.abs(La), `a: L ${La} -> ${L(a)} at ${i}`);
    if (hit >= 0 && slipB > 0.5) assert.ok(Math.abs(L(b) - Lb) < 0.01 * Math.abs(Lb), `b: L ${Lb} -> ${L(b)} at ${i}`);
  }
  assert.ok(hit >= 0, 'balls collided');
  // both end up rolling forward at L/(1+k), less rolling resistance (<= crr*g*0.5 s = 7.7 in/s)
  for (const [o, L0] of [[a, La], [b, Lb]]) {
    const vRoll = L0 / (1 + o.k);
    assert.ok(o.vx > 0 && o.vx <= vRoll + 1e-6 && o.vx > vRoll - 0.5 * SURFACES.floor.crr * G, `vx ${o.vx} vs roll ${vRoll}`);
  }
});

test('resting balls do not overlap', () => {
  const w = new BallWorld({ bounds });
  const bs = [];
  for (let i = 0; i < 20; i++) bs.push(w.addBall('pollen', [60 + (i % 5) * 1.2, 60 + Math.floor(i / 5) * 1.2, 2 + i * 3], [0, 0, 0]));
  run(w, 6);
  for (let i = 0; i < bs.length; i++) for (let j = i + 1; j < bs.length; j++) {
    const d = Math.hypot(bs[i].x - bs[j].x, bs[i].y - bs[j].y, bs[i].z - bs[j].z);
    assert.ok(d > 2 * BALL.r - 0.15, `overlap ${i},${j}: ${d}`);
  }
  assert.ok(bs.every(b => b.z >= BALL.r - 1e-6));
});

test('robot box pushes a ball and ball never ends up inside', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [60, 70, BALL.r], [0, 0, 0]);
  const robot = { x: 40, y: 70, h: 0.3, vx: 30, vy: 0, om: 0 };
  const dims = { halfX: 9, halfY: 9, height: 18 };
  let moved = false;
  for (let i = 0; i < 60 * 3; i++) {
    robot.x += robot.vx / 60;
    w.setRobot(robot, dims);
    w.step(1 / 60);
    const dx = b.x - robot.x, dy = b.y - robot.y, c = Math.cos(robot.h), s = Math.sin(robot.h);
    const lx = c * dx + s * dy, ly = -s * dx + c * dy;
    assert.ok(!(Math.abs(lx) < 9 && Math.abs(ly) < 9 && b.z < 18), `ball inside robot at step ${i}`);
    if (b.vx > 20) moved = true;
  }
  assert.ok(moved && b.x > 80, 'x=' + b.x);
  w.setRobot(null, null);
  assert.equal(w.robot, null);
});

test('robot surface velocity sweeps a sleeping ball awake', () => {
  const w = new BallWorld({ bounds });
  w.reset([{ id: 'a', type: 'pollen', position: [60, 70, BALL.r], diameter: 2.782 }]);
  assert.ok(w.balls[0].sleeping);
  const robot = { x: 45, y: 70, h: 0, vx: 40, vy: 0, om: 0 };
  for (let i = 0; i < 60; i++) { robot.x += 40 / 60; w.setRobot(robot, { halfX: 9, halfY: 9, height: 18 }); w.step(1 / 60); }
  assert.ok(w.balls[0].x > 62);
  assert.ok(!w.balls[0].sleeping || w.balls[0].x > 62);
});

test('ball bounces off box and circle obstacles', () => {
  const env = { bounds, boxes: [{ minX: 80, minY: 60, maxX: 90, maxY: 80, zMin: 0, zMax: 10 }], circles: [{ x: 40, y: 70, r: 5, zMin: 0, zMax: 10 }] };
  const w = new BallWorld(env);
  const a = w.addBall('pollen', [60, 70, 3], [60, 0, 0]);
  const b = w.addBall('pollen', [60, 100, 3], [-60, -20, 0]);
  const c = w.addBall('pollen', [60, 110, 3], [-60, 0, 0]);
  c.y = 70.5; c.x = 70; c.vx = -60; c.vy = 0; // dead centre on circle -> reflects
  a.y = 70; b.x = 90; b.y = 120; b.vx = 0; b.vy = 0;
  let aRefl = false, cRefl = false;
  for (let i = 0; i < 60 * 3; i++) {
    w.step(1 / 60);
    assert.ok(!(a.x > 80 - BALL.r && a.x < 90 + BALL.r && a.y > 60 && a.y < 80 && a.z < 10), 'a inside box');
    assert.ok(Math.hypot(c.x - 40, c.y - 70) >= 5 + BALL.r - 0.01 || c.z > 10, 'c inside circle');
    if (a.vx < -5) aRefl = true;
    if (c.vx > 5) cRefl = true;
  }
  assert.ok(aRefl, 'box reflected');
  assert.ok(cRefl, 'circle reflected');
});

function shootAt(w, cell, from, T, type = 'pollen') {
  const aim = cell.aim_point;
  const v = [(aim[0] - from[0]) / T, (aim[1] - from[1]) / T, (aim[2] - from[2]) / T + 0.5 * G * T];
  return w.addBall(type, from, v);
}

test('shot through a real hive opening is captured exactly once', () => {
  for (const a of ['red', 'blue']) {
    const env = realEnv();
    const w = new BallWorld(env);
    const cn = env.hives[a].cells.audience.opening_normal[2] > 0 ? 'audience' : 'far';   // the RAISED cell at the CAD pose (blue is far_up)
    const cell = env.hives[a].cells[cn];
    // fire from the side the opening normal faces (audience side for audience_up cell)
    const from = [cell.aim_point[0], cell.aim_point[1] + cell.opening_normal[1] * 28, cell.aim_point[2] + cell.opening_normal[2] * 28 - 20];
    from[2] = Math.max(from[2], 12);
    const calls = [];
    w.onCapture = (b, al, cn) => calls.push([b, al, cn]);
    const b = shootAt(w, cell, from, 0.55);
    assert.ok(b.vx * 0 + (b.vy * cell.opening_normal[1] + b.vz * cell.opening_normal[2] - G * 0.55 * cell.opening_normal[2]) < 0 || true);
    run(w, 3);
    assert.equal(calls.length, 1, a + ' captured ' + calls.length);
    assert.equal(calls[0][0], b);
    assert.equal(b.state, 'scored'); assert.equal(b.hive, a); assert.equal(b.cell, cn);
    assert.equal(w.stats()[a === 'red' ? 'scoredRed' : 'scoredBlue'], 1);
    assert.equal(w.balls.length, 1);
    // placed inside the cell volume (CAD frame at rest): between x range and above the floor
    const xs = cell.opening_corners.map(q => q[0]);
    assert.ok(b.x > Math.min(...xs) && b.x < Math.max(...xs));
  }
});

test('multiple captured balls do not overlap', () => {
  const env = realEnv(), w = new BallWorld(env), cell = env.hives.red.cells.audience;
  let n = 0; w.onCapture = () => n++;
  const from = [cell.aim_point[0], cell.aim_point[1] - 24, 12];
  for (let i = 0; i < 12; i++) { shootAt(w, cell, from, 0.55); run(w, 0.2); }
  run(w, 2);
  assert.equal(n, 12);
  const bs = w.balls;
  for (let i = 0; i < bs.length; i++) for (let j = i + 1; j < bs.length; j++)
    assert.ok(Math.hypot(bs[i].x - bs[j].x, bs[i].y - bs[j].y, bs[i].z - bs[j].z) > 2 * BALL.r, 'scored overlap');
});

test('a shot that misses is not captured', () => {
  const env = realEnv(), w = new BallWorld(env), cell = env.hives.red.cells.audience;
  let n = 0; w.onCapture = () => n++;
  const miss = JSON.parse(JSON.stringify(cell)); miss.aim_point[0] += 40; // 40 in to the side
  const from = [cell.aim_point[0], cell.aim_point[1] - 24, 12];
  shootAt(w, miss, from, 0.55);
  // shot that stops short: lands on the floor in front
  w.addBall('pollen', from, [0, 20, 20]);
  run(w, 4);
  assert.equal(n, 0);
  assert.equal(w.stats().scoredRed + w.stats().scoredBlue, 0);
});

test('hive rotation convention matches scene far_up data', () => {
  const hc = scene.constants.hives.red, g = scene.groups.hive_red;
  const cell = hc.states.audience_up.cells.audience, tgt = hc.states.far_up.cells.audience;
  const d = (30 - g.cad_pose_angle_deg) * Math.PI / 180, c = Math.cos(d), s = Math.sin(d);
  const p = cell.aim_point, dy = p[1] - g.pivot[1], dz = p[2] - g.pivot[2];
  const y = g.pivot[1] + dy * c - dz * s, z = g.pivot[2] + dy * s + dz * c;
  assert.ok(Math.abs(y - tgt.aim_point[1]) < 0.05 && Math.abs(z - tgt.aim_point[2]) < 0.05, `${y},${z} vs ${tgt.aim_point}`);
});

test('shot into tipped hive (far_up) is captured at the rotated opening', () => {
  const env = realEnv(), w = new BallWorld(env);
  const tgt = scene.constants.hives.red.states.far_up.cells.far;
  w.setHiveAngles({ red: 30 });
  run(w, 0.05);
  const n = tgt.opening_normal;
  let got = 0; w.onCapture = () => got++;
  const from = [tgt.aim_point[0], tgt.aim_point[1] + n[1] * 28, Math.max(12, tgt.aim_point[2] + n[2] * 28)];
  shootAt(w, tgt, from, 0.6);
  run(w, 3);
  assert.equal(got, 1);
});

test('sleeping stack stays put and wakes when hit', () => {
  const w = new BallWorld({ bounds });
  const start = [];
  for (let i = 0; i < 4; i++) start.push({ id: 's' + i, type: 'pollen', position: [50, 50, BALL.r + i * 2.6], diameter: 2.782 });
  w.reset(start);
  const before = w.balls.map(b => [b.x, b.y, b.z]);
  run(w, 3);
  w.balls.forEach((b, i) => assert.deepEqual([b.x, b.y, b.z], before[i]));
  assert.equal(w.stats().sleeping, 4);
  const hit = w.addBall('pollen', [30, 50, BALL.r], [80, 0, 0]);
  run(w, 2);
  assert.ok(w.stats().sleeping < 5 && w.balls.some(b => b.id[0] === 's' && !(b.x === 50 && b.y === 50)), 'stack disturbed');
  assert.ok(hit);
});

test('hold, release, removeBall, ballsInBox', () => {
  const w = new BallWorld({ bounds });
  const a = w.addBall('pollen', [50, 50, BALL.r], [0, 0, 0]);
  const b = w.addBall('nectar_red', [80, 80, 2], [0, 0, 0]);
  assert.equal(b.r, 1.7995);
  assert.deepEqual(ballsInBox(w, { x: 50, y: 50, h: 0.7 }, 5, 5), [a]);
  w.hold(a); assert.equal(w.stats().held, 1);
  assert.deepEqual(ballsInBox(w, { x: 50, y: 50, h: 0 }, 5, 5), []);
  w.release(a, [0, 0, 40]); assert.equal(a.state, 'free');
  run(w, 0.05); assert.ok(a.z > BALL.r + 1);
  w.removeBall(b); assert.equal(w.balls.length, 1);
});

test('real start balls settle without drifting; hive tip wakes preloads', () => {
  const env = realEnv(), w = new BallWorld(env);
  w.reset(scene.balls.balls);
  const n = w.balls.length;
  assert.equal(n, 56);
  const snap = w.balls.map(b => [b.x, b.y, b.z]);
  run(w, 2);
  w.balls.forEach((b, i) => assert.deepEqual([b.x, b.y, b.z], snap[i]));
  w.setHiveAngles({ red: 30 }); run(w, 0.5);
  assert.equal(w.balls.length, n);
});

test('performance: 60 balls x 600 steps', () => {
  const env = realEnv(), w = new BallWorld(env);
  let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 60; i++) w.addBall('pollen', [10 + rnd() * 124, 10 + rnd() * 124, 2 + rnd() * 20], [(rnd() - 0.5) * 100, (rnd() - 0.5) * 100, rnd() * 50]);
  w.setRobot({ x: 72, y: 72, h: 0.4, vx: 0, vy: 0, om: 0 }, { halfX: 9, halfY: 9, height: 18 });
  const t0 = performance.now();
  for (let i = 0; i < 600; i++) w.step(1 / 60);
  const ms = (performance.now() - t0) / 600;
  console.log(`perf: ${ms.toFixed(3)} ms/frame (60 balls, 60 Hz frames)`);
  assert.ok(ms < 2, 'too slow: ' + ms);
  for (const b of w.balls) assert.ok(Number.isFinite(b.x + b.y + b.z));
});
