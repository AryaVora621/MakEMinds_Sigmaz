// Simulator physics regression tests (node:test, ESM).
// Scope: deterministic behaviour of the ball world. New physics tests append below the
// determinism test in this file. Helpers mirror the setup used in tests_js/physics3d.test.mjs.
import test from 'node:test';
import assert from 'node:assert/strict';
import { BallWorld, BALL, INTAKE, MAX_HELD } from '../web/js/balls.js';

const bounds = { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 };

// Seeded PRNG (mulberry32). Same seed gives the same sequence on every run.
function seededRng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function buildWorld(seed) {
  const rng = seededRng(seed);
  const w = new BallWorld({ bounds }, rng);
  // Drop 12 balls from varied heights at positions drawn from the same seeded stream.
  for (let i = 0; i < 12; i++) {
    const x = 20 + rng() * 104;
    const y = 20 + rng() * 104;
    const z = 6 + rng() * 60;
    w.addBall('pollen', [x, y, z], [(rng() - 0.5) * 20, (rng() - 0.5) * 20, 0]);
  }
  return w;
}

const STEPS = 600;
const DT = 1 / 60;

test('identical inputs give identical ball state', () => {
  const a = buildWorld(12345);
  const b = buildWorld(12345);
  assert.equal(a.balls.length, 12);
  assert.equal(b.balls.length, 12);
  for (let s = 0; s < STEPS; s++) { a.step(DT); b.step(DT); }
  assert.equal(a.balls.length, b.balls.length);
  const keys = ['x', 'y', 'z', 'vx', 'vy', 'vz'];
  for (let i = 0; i < a.balls.length; i++) {
    for (const k of keys) {
      assert.equal(a.balls[i][k], b.balls[i][k], `ball ${i} ${k} differs`);
    }
  }
});

// ---------------------------------------------------------------- HIVE STACKING
// Four pollen balls are fed one after another through the raised (audience) cell opening of the red hive at the CAD
// pose. They must end scored, settle without jitter, and never overlap deeply.
test('balls stacked in a hive cell settle without jitter', async () => {
  const { readFileSync } = await import('node:fs');
  const { parseHiveCollider } = await import('../web/js/hivemesh.js');
  const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
  const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
  const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells, mesh: meshes[a] };
  }
  const w = new BallWorld({ bounds, hives });
  const cell = hives.red.cells.audience, n = cell.opening_normal, ap = cell.aim_point;
  const DT2 = 1 / 60;
  const balls = [];
  for (let i = 0; i < 4; i++) {
    // start just outside the opening and feed the ball into the cell along -normal
    balls.push(w.addBall('pollen', [ap[0] + n[0] * 4, ap[1] + n[1] * 4, ap[2] + n[2] * 4], [-n[0] * 60, -n[1] * 60, -n[2] * 60]));
    for (let t = 0; t < 0.8; t += DT2) w.step(DT2);
  }
  for (let t = 0; t < 5; t += DT2) w.step(DT2);
  const zs = balls.map(() => []);
  for (let t = 0; t < 2; t += DT2) { w.step(DT2); balls.forEach((b, i) => zs[i].push(b.z)); }
  for (const b of balls) {
    const sp = Math.hypot(b.vx, b.vy, b.vz);
    assert.ok(sp < 2, `ball speed ${sp} in/s`);
  }
  zs.forEach((z, i) => {
    const m = z.reduce((s, v) => s + v, 0) / z.length;
    const sd = Math.sqrt(z.reduce((s, v) => s + (v - m) ** 2, 0) / z.length);
    assert.ok(sd < 0.05, `ball ${i} z std-dev ${sd} in`);
  });
  for (let i = 0; i < balls.length; i++) {
    for (let j = i + 1; j < balls.length; j++) {
      const a = balls[i], c = balls[j];
      const d = Math.hypot(a.x - c.x, a.y - c.y, a.z - c.z);
      assert.ok(a.r + c.r - d < 0.15, `balls ${i},${j} overlap ${(a.r + c.r - d).toFixed(3)} in`);
    }
  }
  const st = w.stats();
  assert.equal(st.scoredRed + st.scoredBlue, 4, 'scored count');
});

// ---------------------------------------------------------------- NO TUNNELLING AT MAX SPEED
// Fast-ball regression (research task "no tunnelling"): the substep count must cover the pair's relative speed, not just one ball.
const TEN_F = 1 / 60;
const inBounds = (b) => b.x >= bounds.minX && b.x <= bounds.maxX && b.y >= bounds.minY && b.y <= bounds.maxY;
const finiteVel = (b) => Number.isFinite(b.vx) && Number.isFinite(b.vy) && Number.isFinite(b.vz);

test('no tunnelling: a 1800 in/s ball fired at the perimeter wall from 30 in away stays inside the bounds', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [bounds.maxX - 30, 72, BALL.r], [BALL.maxSpeed, 0, 0]);
  for (let t = 0; t < 0.5; t += TEN_F) w.step(TEN_F);
  assert.ok(inBounds(b), `ball escaped to x=${b.x}`);
  assert.ok(finiteVel(b), 'non-finite velocity');
});

test('no tunnelling: a 1000 in/s ball aimed at a hive wall edge never ends inside the hive solid', async () => {
  const env = await hiveEnvForTunnel();
  const { TriMesh } = await import('../web/js/hivemesh.js');
  const hv = env.hives.red, raw = hv.mesh, mesh = new TriMesh(raw.verts, raw.tris);
  // pick the mesh vertex furthest from the pivot (an outer edge), and fire radially at it from 30 in away
  let best = null, bd = -1;
  for (let i = 0; i < mesh.v.length; i += 3) {
    const d = Math.hypot(mesh.v[i], mesh.v[i + 1], mesh.v[i + 2]);
    if (d > bd) { bd = d; best = [mesh.v[i], mesh.v[i + 1], mesh.v[i + 2]]; }
  }
  const dir = best.map(v => v / bd);
  const aim = [hv.pivot[0] + best[0], hv.pivot[1] + best[1], hv.pivot[2] + best[2]];
  const from = [aim[0] + dir[0] * 30, aim[1] + dir[1] * 30, aim[2] + dir[2] * 30];
  const w = new BallWorld(env);
  const b = w.addBall('pollen', from, [-dir[0] * 1000, -dir[1] * 1000, -dir[2] * 1000]);
  for (let t = 0; t < 0.5; t += TEN_F) w.step(TEN_F);
  assert.ok(finiteVel(b), 'non-finite velocity');
  assert.ok(Math.hypot(b.x, b.y, b.z) < 1e6, 'ball flew away');
  // centre must not be inside the closed hive solid: ray parity against every triangle (odd crossings = inside)
  const local = [b.x - hv.pivot[0], b.y - hv.pivot[1], b.z - hv.pivot[2]];
  assert.ok(!insideSolid(mesh, local), 'ball centre ended inside the hive solid');
});

test('no tunnelling: two balls head-on at 800 in/s each keep their x order after 0.5 s', () => {
  const w = new BallWorld({ bounds });
  const a = w.addBall('pollen', [50, 72, BALL.r], [800, 0, 0]);
  const c = w.addBall('pollen', [80, 72, BALL.r], [-800, 0, 0]);
  for (let t = 0; t < 0.5; t += TEN_F) w.step(TEN_F);
  assert.ok(finiteVel(a) && finiteVel(c), 'non-finite velocity');
  assert.ok(a.x < c.x, `pass-through: a.x=${a.x} c.x=${c.x}`);
});

// Odd number of crossings of a generic ray through the triangles = inside a closed solid.
function insideSolid(mesh, p) {
  const d = [0.5773, 0.5774, 0.5773], v = mesh.v;
  let hits = 0;
  for (let i = 0; i < mesh.nTris; i++) {
    const a = mesh.t[3 * i] * 3, bb = mesh.t[3 * i + 1] * 3, c = mesh.t[3 * i + 2] * 3;
    const e1 = [v[bb] - v[a], v[bb + 1] - v[a + 1], v[bb + 2] - v[a + 2]], e2 = [v[c] - v[a], v[c + 1] - v[a + 1], v[c + 2] - v[a + 2]];
    const pv = [d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0]];
    const det = e1[0] * pv[0] + e1[1] * pv[1] + e1[2] * pv[2];
    if (Math.abs(det) < 1e-12) continue;
    const tv = [p[0] - v[a], p[1] - v[a + 1], p[2] - v[a + 2]];
    const u = (tv[0] * pv[0] + tv[1] * pv[1] + tv[2] * pv[2]) / det;
    if (u < 0 || u > 1) continue;
    const qv = [tv[1] * e1[2] - tv[2] * e1[1], tv[2] * e1[0] - tv[0] * e1[2], tv[0] * e1[1] - tv[1] * e1[0]];
    const w = (d[0] * qv[0] + d[1] * qv[1] + d[2] * qv[2]) / det;
    if (w < 0 || u + w > 1) continue;
    const t = (e2[0] * qv[0] + e2[1] * qv[1] + e2[2] * qv[2]) / det;
    if (t > 0) hits++;
  }
  return hits % 2 === 1;
}

async function hiveEnvForTunnel() {
  const { readFileSync } = await import('node:fs');
  const { parseHiveCollider } = await import('../web/js/hivemesh.js');
  const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
  const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
  const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells, mesh: meshes[a] };
  }
  return { bounds, hives };
}

// ---------------------------------------------------------------- ROBOT PUSHES BALLS (no squeeze-through)
// Robot footprint is the default ROBOT box (collision.js: halfX 9, halfY 9) and height 18 in. The intake is disabled so
// only the push/collision path is exercised. A ball must never end (or sit) inside the robot box.
import { ROBOT } from '../web/js/collision.js';
const ROBOT_DIMS = { halfX: ROBOT.halfX, halfY: ROBOT.halfY, height: 18 };
// Distance from a point (world x,y) to the robot's box (robot frame, heading h). 0 when inside the box footprint.
function boxDist(pose, x, y) {
  const c = Math.cos(pose.h), s = Math.sin(pose.h), dx = x - pose.x, dy = y - pose.y;
  const lx = c * dx + s * dy, ly = -s * dx + c * dy;
  const ex = Math.max(Math.abs(lx) - ROBOT_DIMS.halfX, 0), ey = Math.max(Math.abs(ly) - ROBOT_DIMS.halfY, 0);
  return { d: Math.hypot(ex, ey), lx, ly, inside: Math.abs(lx) < ROBOT_DIMS.halfX && Math.abs(ly) < ROBOT_DIMS.halfY };
}

test('robot pushes a resting ball ahead at 60 in/s without the ball entering the robot box', () => {
  const w = new BallWorld({ bounds });
  w.setIntake({ enabled: false });
  const ball = w.addBall('pollen', [80, 72, BALL.r], [0, 0, 0]);
  ball.fired = false;   // a scenario ball, not a launched one (keeps it out of the fired-ball recycler)
  const pose = { x: 50, y: 72, h: 0, vx: 60, vy: 0, om: 0 };
  const DT3 = 1 / 60;
  let worst = Infinity;
  for (let f = 0; f < 60; f++) {
    w.setRobot(pose, ROBOT_DIMS);
    w.step(DT3);
    const bd = boxDist(pose, ball.x, ball.y);
    worst = Math.min(worst, bd.d - ball.r);
    assert.ok(!bd.inside, `frame ${f}: ball centre inside robot box at (${bd.lx.toFixed(2)}, ${bd.ly.toFixed(2)})`);
    assert.ok(bd.d >= ball.r - 0.05, `frame ${f}: ball penetrates robot by ${(ball.r - bd.d).toFixed(3)} in`);
    pose.x += pose.vx * DT3;
  }
  const front = ball.x - (pose.x - pose.vx * DT3);   // robot-frame x (h = 0) at the last stepped pose
  assert.ok(front > ROBOT_DIMS.halfX, `ball robot-frame x ${front.toFixed(2)} not ahead of front face ${ROBOT_DIMS.halfX}`);
  assert.ok(Number.isFinite(worst));
});

test('robot pins a ball against a wall for 1 s: ball never ends inside the robot box and stays under 400 in/s', () => {
  const w = new BallWorld({ bounds });
  w.setIntake({ enabled: false });
  const wallX = bounds.maxX;
  const ball = w.addBall('pollen', [wallX - BALL.r, 72, BALL.r], [0, 0, 0]);
  ball.fired = false;
  const pose = { x: 110, y: 72, h: 0, vx: 60, vy: 0, om: 0 };
  const DT3 = 1 / 60;
  let maxSpeed = 0;
  for (let f = 0; f < 60; f++) {
    w.setRobot(pose, ROBOT_DIMS);
    w.step(DT3);
    maxSpeed = Math.max(maxSpeed, Math.hypot(ball.vx, ball.vy, ball.vz));
    pose.x += pose.vx * DT3;
  }
  const bd = boxDist(pose, ball.x, ball.y);
  assert.ok(!bd.inside, `ball ended inside robot box at (${bd.lx.toFixed(2)}, ${bd.ly.toFixed(2)})`);
  assert.ok(maxSpeed < 400, `max ball speed ${maxSpeed.toFixed(1)} in/s`);
  assert.ok(finiteVel(ball), 'non-finite velocity');
});

// Quadratic air drag (BALL.dragK, ASSUMED value). A ball launched horizontally in the air at 500 in/s
// must lose between 0.5% and 6% of its horizontal speed over 0.25 s. Gravity does not touch vx, so vx isolates drag.
test('airborne ball launched horizontally at 500 in/s loses 0.5-6% speed over 0.25 s', () => {
  const w = new BallWorld({ bounds }, seededRng(7));
  const b = w.addBall('pollen', [8, 72, 40], [500, 0, 0]);
  assert.ok(b.z > BALL.r + 0.05, 'ball must start airborne');
  const DT = 1 / 60;
  for (let i = 0; i < 15; i++) w.step(DT);   // 15 * 1/60 s = 0.25 s
  assert.ok(b.state === 'free' && !b.sleeping, 'ball should still be free and moving');
  assert.ok(b.z > BALL.r + 0.05, 'ball should still be airborne (no floor contact in the window)');
  const loss = (500 - b.vx) / 500;
  assert.ok(loss > 0.005 && loss < 0.06, `horizontal speed loss ${(loss * 100).toFixed(2)}% outside 0.5-6%`);
});

// Single bounce height. A pollen ball whose centre is dropped from 40 in must rebound (centre rise above its
// resting height BALL.r) to e^2 * 40 * [0.8, 1.2], e = BALL.restitutionFloor. The e^2 law is the ideal energy result;
// the band covers drag, the 38.6 in true fall (40 - r) and the discrete step. Measured here: rise ~7.7 in vs band 6.5-9.7.
test('pollen dropped from 40 in rebounds to e^2 * 40 * [0.8, 1.2]', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [70, 70, 40], [0, 0, 0]);
  const DT = 1 / 60;
  let impacted = false, peak = 0;
  for (let i = 0; i < 60 * 3; i++) {
    const prevVz = b.vz;
    w.step(DT);
    if (!impacted && prevVz < 0 && b.vz > 0) impacted = true;
    if (impacted) peak = Math.max(peak, b.z);
  }
  assert.ok(impacted, 'ball never rebounded off the floor');
  const e = BALL.restitutionFloor, H = 40;
  const rise = peak - BALL.r;
  assert.ok(rise >= e * e * H * 0.8 && rise <= e * e * H * 1.2,
    `rebound rise ${rise.toFixed(3)} in outside [${(e * e * H * 0.8).toFixed(3)}, ${(e * e * H * 1.2).toFixed(3)}]`);
});

// ---------------------------------------------------------------- JITTER / SLEEP FIXES (research r4)
// Shared setup: the real red hive at its CAD pose, with the mesh from the field data.
async function realHiveWorld() {
  const { readFileSync } = await import('node:fs');
  const { parseHiveCollider } = await import('../web/js/hivemesh.js');
  const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
  const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
  const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells, mesh: meshes[a] };
  }
  return new BallWorld({ bounds, hives });
}

// Research r4 finding list did not measure this path; found by reading _wakeAbove call sites. The only removal path that
// did not wake supported balls was the scored-cell recycle in _capture (a full cell drops its oldest stored ball). A free
// sleeping ball resting on that scored ball must be woken, or it floats. Fix: _wakeAbove(old) before the splice.
test('recycling a full cell wakes free balls that rest on the recycled ball', async () => {
  const w = await realHiveWorld();
  const hv = w.hv.find(h => h.name === 'red');
  const cell = hv.cells.find(k => k.name === 'audience');
  const cap = w._cellCapacity(cell);
  assert.ok(cap >= 1 && cap < 1000, 'capacity ' + cap);
  w._pending = [];
  const stored = [];
  for (let i = 0; i < cap; i++) {
    const b = w._mk('pollen', [0, 0, 0], [0, 0, 0]);
    w.balls.push(b);
    w._capture(b, hv, cell);
    stored.push(b);
  }
  const victim = stored[0];   // oldest stored ball: the next capture recycles it
  const rester = w.addBall('pollen', [victim.x, victim.y, victim.z + 2 * victim.r - 0.2], [0, 0, 0]);
  rester.sleeping = true;
  rester.state = 'free';
  let wokeVictim = null;
  const orig = w._wakeAbove.bind(w);
  w._wakeAbove = (b) => { if (b === victim) wokeVictim = { inWorld: w.balls.includes(b) }; orig(b); };
  const next = w._mk('pollen', [0, 0, 0], [0, 0, 0]);
  w.balls.push(next);
  w._capture(next, hv, cell);
  assert.ok(wokeVictim, 'recycled ball must be passed to _wakeAbove');
  assert.equal(wokeVictim.inWorld, true, '_wakeAbove runs before the victim is removed');
  assert.equal(w.balls.includes(victim), false, 'victim recycled');
  assert.equal(rester.sleeping, false, 'ball resting on the recycled ball is woken');
});

// ---------------------------------------------------------------- FIXED STEP (stepFixed)
// stepFixed(frameDt, hz) must give the same physics whatever the frame rate: every call runs whole 1/hz steps, so the
// step sequence is the same. Shots are all launched at t=0 (spaced apart), so no frame-timed injection is involved.
const FIX_HZ = 240, FIX_SECONDS = 3;
const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm3 = (a) => { const l = Math.hypot(a[0], a[1], a[2]); return [a[0] / l, a[1] / l, a[2] / l]; };

// 20 pollen balls launched at the red audience opening along its normal (5 in grid, 5 in apart, 80 in/s).
async function fixedShotWorld() {
  const w = await realHiveWorld();
  const cell = w.hv.find(h => h.name === 'red').cells.find(k => k.name === 'audience');
  const n = norm3(cell.pc.n), ap = cell.pc.aim;
  const t1 = norm3(cross3(n, Math.abs(n[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0])), t2 = cross3(n, t1);
  for (let i = 0; i < 20; i++) {
    const a = (i % 5) - 2, c = Math.floor(i / 5) - 1.5, dist = 12 + 3 * (i % 3);
    const p = [0, 1, 2].map(k => ap[k] + n[k] * dist + t1[k] * 5 * a + t2[k] * 5 * c);
    w.addBall('pollen', p, [-n[0] * 80, -n[1] * 80, -n[2] * 80]);
  }
  return w;
}

// Frame-dt schedules covering FIX_SECONDS of simulated time.
function frameSchedule(kind) {
  if (kind === 'mixed') {   // seeded random frame dts in [0.008, 0.025], last frame clamped to the remaining time
    const rng = seededRng(4242), out = [];
    let sum = 0;
    while (sum < FIX_SECONDS - 1e-9) {
      const dt = Math.min(0.008 + rng() * 0.017, FIX_SECONDS - sum);
      out.push(dt); sum += dt;
    }
    return out;
  }
  const dt = kind === '1/60' ? 1 / 60 : 1 / 30;
  return Array(Math.round(FIX_SECONDS / dt)).fill(dt);
}

async function runFixedShots(kind) {
  const w = await fixedShotWorld();
  for (const dt of frameSchedule(kind)) w.stepFixed(dt, FIX_HZ);
  return w;
}

test('stepFixed: 1/60 frames run exactly 4 fixed steps and keep the remainder as alpha', () => {
  const w = new BallWorld({ bounds });
  const calls = [];
  const orig = w.step.bind(w);
  w.step = (dt) => { calls.push(dt); orig(dt); };
  const alpha = w.stepFixed(1 / 60, FIX_HZ);
  assert.equal(calls.length, 4);
  for (const dt of calls) assert.ok(Math.abs(dt - 1 / FIX_HZ) < 1e-12);
  assert.ok(alpha >= 0 && alpha <= 1, 'alpha ' + alpha);
  assert.ok(Math.abs(alpha) < 1e-6, 'exact multiple leaves no remainder, alpha ' + alpha);
  const a2 = w.stepFixed(0.005, FIX_HZ);   // 1.2 steps accumulated: one step, 0.2 left
  assert.equal(calls.length, 5);
  assert.ok(Math.abs(a2 - 0.2) < 0.05, 'alpha ' + a2);
});

test('stepFixed: a huge frame runs at most 8 fixed steps and drops the backlog', () => {
  const w = new BallWorld({ bounds });
  let n = 0; const orig = w.step.bind(w);
  w.step = (dt) => { n++; orig(dt); };
  w.stepFixed(1.0, FIX_HZ);
  assert.equal(n, 8);
  w.stepFixed(0, FIX_HZ);   // nothing accumulated, nothing new to run
  assert.equal(n, 8);
});

test('stepFixed: 3 s of 20 shots at 1/60, 1/30 and mixed frame dts give equal scores and resting positions', async () => {
  const A = await runFixedShots('1/60');
  const B = await runFixedShots('1/30');
  const C = await runFixedShots('mixed');
  const scoredOf = (w) => w.balls.filter(b => b.state === 'scored').length;
  const sA = scoredOf(A), sB = scoredOf(B), sC = scoredOf(C);
  assert.ok(sA + A.balls.filter(b => b.state === 'free' && b.sleeping).length > 0, 'scenario must do something');
  assert.ok(Math.abs(sA - sB) <= 1, `scored 1/60=${sA} 1/30=${sB}`);
  assert.ok(Math.abs(sA - sC) <= 1, `scored 1/60=${sA} mixed=${sC}`);
  let compared = 0;
  for (const [X, name] of [[B, '1/30'], [C, 'mixed']]) {
    assert.equal(X.balls.length, A.balls.length, name + ' ball count');
    A.balls.forEach((a, i) => {
      const b = X.balls[i];
      const restA = a.state === 'scored' || (a.state === 'free' && a.sleeping);
      const restB = b.state === 'scored' || (b.state === 'free' && b.sleeping);
      if (!restA || !restB) return;
      compared++;
      for (const k of ['x', 'y', 'z']) assert.ok(Math.abs(a[k] - b[k]) <= 0.5, `${name} ball ${i} ${k}: ${a[k]} vs ${b[k]}`);
    });
  }
  assert.ok(compared > 0, 'no resting balls to compare');
});

// ---------------------------------------------------------------- INTAKE REGRESSION GUARD (docs/overnight/intake-physics.md)
// Locks the documented roller behaviour. Constants come from INTAKE / MAX_HELD in web/js/balls.js (not changed here).
const INTAKE_ROBOT = { halfX: ROBOT_DIMS.halfX, halfY: ROBOT_DIMS.halfY, height: 18 };
const MOUTH_HALF = INTAKE_ROBOT.halfY - INTAKE.mouthInset;   // ball-centre half width of the mouth (in)
const FRONT_FACE = INTAKE_ROBOT.halfX;                        // robot-frame x of the front roller face (in)

function intakeGuard(space = 10) {
  const w = new BallWorld({ bounds });
  w.setIntake({ enabled: true, space });
  const got = [];
  w.onIntake = (b, side) => got.push([b, side]);
  return { w, got };
}
// Robot-frame (lx, ly) to world, for a robot at heading 0 (x/y are the robot centre).
const atRobot = (robot, lx, ly) => [robot.x + lx, robot.y + ly, BALL.r];
function spinGuard(w, robot, frames) {
  for (let i = 0; i < frames; i++) { w.setRobot(robot, INTAKE_ROBOT); w.step(1 / 60); }
}

test('intake guard (1): a ball inside captureGap and inside the mouth width is loaded', () => {
  const { w, got } = intakeGuard(10);
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  // Surface 0.3 in from the front face (inside captureGap = 0.6), centre laterally on the robot centre line.
  const lx = FRONT_FACE + BALL.r + 0.3;
  assert.ok(0.3 < INTAKE.captureGap, 'test gap must sit inside captureGap');
  const ball = w.addBall('pollen', atRobot(robot, lx, 0), [0, 0, 0]);
  spinGuard(w, robot, 30);
  assert.equal(got.length, 1, 'ball not loaded');
  assert.equal(got[0][0], ball);
  assert.equal(got[0][1], 'front');
  assert.equal(w.held.length, 1);
});

test('intake guard (2): a ball beside the robot (outside the mouth width) is not loaded and is not dragged in', () => {
  const { w, got } = intakeGuard(10);
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  // Centre 0.5 in outside the frame side, lateral offset beyond MOUTH_HALF, level with the robot middle.
  const ly = INTAKE_ROBOT.halfY + BALL.r + 0.5;
  assert.ok(ly > MOUTH_HALF, 'test offset must be outside the mouth width');
  const ball = w.addBall('pollen', atRobot(robot, 0, ly), [0, 0, 0]);
  const y0 = ball.y;
  spinGuard(w, robot, 120);
  assert.equal(got.length, 0, 'beside ball was loaded');
  assert.equal(w.held.length, 0);
  assert.equal(ball.state, 'free');
  assert.ok(Math.abs(ball.y - y0) < 0.5, `beside ball was pulled sideways by ${(ball.y - y0).toFixed(2)} in`);
});

test('intake guard (3): with MAX_HELD (4) held, a 5th ball is refused, stays free and is pushed, not pulled in', () => {
  assert.equal(MAX_HELD, 4, 'hold limit is 4 (official G407)');
  const { w, got } = intakeGuard(10);
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  const lx = FRONT_FACE + BALL.r + 0.3;
  const balls = [];
  // Five balls side by side on the front face, spaced wider than one diameter and inside the mouth width.
  const lys = [0, 3, -3, 5.5, -5.5];
  for (const ly of lys) balls.push(w.addBall('pollen', atRobot(robot, lx, ly), [0, 0, 0]));
  spinGuard(w, robot, 60);
  assert.equal(got.length, MAX_HELD, `loaded ${got.length}, expected ${MAX_HELD}`);
  assert.equal(w.held.length, MAX_HELD);
  const refused = balls.filter(b => b.state === 'free');
  assert.equal(refused.length, 1, 'exactly one ball should be refused');
  const r = refused[0];
  const rlx = (r.x - robot.x);   // robot h = 0, so robot-frame x equals world offset
  assert.ok(rlx > FRONT_FACE, `refused ball was pulled inside the footprint (lx ${rlx.toFixed(2)})`);
  assert.ok(rlx >= lx - 0.1, `refused ball moved toward the roller: lx ${rlx.toFixed(2)} vs start ${lx.toFixed(2)}`);
});

test('intake guard (4): a ball faster than INTAKE.maxSpeed in the capture zone is not intaken (control: slower one is)', () => {
  // The ball sits inside the capture zone (surface 0.3 in from the face) and slides ALONG the face, so it stays in the
  // zone for the first frames. (Radial motion would carry it out of the zone within one step, which tests nothing.)
  const lx = FRONT_FACE + BALL.r + 0.3;
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  // Control: below maxSpeed, sliding along the face: loaded on the first step.
  {
    const { w, got } = intakeGuard(10);
    w.addBall('pollen', atRobot(robot, lx, -3), [0, INTAKE.maxSpeed * 0.5, 0]);
    spinGuard(w, robot, 2);
    assert.equal(got.length, 1, 'control ball (below maxSpeed) should be loaded');
  }
  // Fast ball, same spot and path, above maxSpeed: not loaded.
  {
    const { w, got } = intakeGuard(10);
    const fast = w.addBall('pollen', atRobot(robot, lx, -3), [0, INTAKE.maxSpeed * 1.4, 0]);
    spinGuard(w, robot, 2);
    assert.equal(got.length, 0, 'fast ball was intaken');
    assert.equal(fast.state, 'free');
  }
});

test('intake guard (5): held balls leave in FIFO order via dequeueHeld()', () => {
  const { w, got } = intakeGuard(10);
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  const lx = FRONT_FACE + BALL.r + 0.3;
  // Load three balls one after another, so capture order is A, B, C.
  const ids = [];
  for (const ly of [-2.5, 0, 2.5]) {
    const b = w.addBall('pollen', atRobot(robot, lx, ly), [0, 0, 0]);
    ids.push(b.id);
    spinGuard(w, robot, 20);
  }
  assert.equal(got.length, 3, 'expected three captures');
  assert.deepEqual(got.map(g => g[0].id), ids, 'capture order differs from placement order');
  const out = [];
  for (let i = 0; i < 3; i++) out.push(w.dequeueHeld().id);
  assert.deepEqual(out, ids, 'dequeueHeld did not return held balls oldest first');
  assert.equal(w.dequeueHeld(), null, 'queue should be empty after three dequeues');
});
