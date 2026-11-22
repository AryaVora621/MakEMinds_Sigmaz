import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL, INTAKE } from '../web/js/balls.js';
import { parseHiveCollider } from '../web/js/hivemesh.js';
import * as SR from '../web/js/shotrules.js';

const G = BALL.g;
// Hive tip facts (user-authoritative): a hive tips at 8 pollen. Read from the model when it exists, else fall back to 8.
const HIVE_TIP = await import('../web/js/hivetip_model.js').then(m => m.HIVE_TIP ?? { threshold: 8, pollenWeight: 1 }, () => ({ threshold: 8, pollenWeight: 1 }));
test('hive tip: threshold is 8 pollen-equivalent (not 4)', () => {
  assert.equal(HIVE_TIP.threshold / (HIVE_TIP.pollenWeight ?? 1), 8);
});
const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const bounds = { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 };
const DIMS = { halfX: 9, halfY: 9, height: 18 };
const run = (w, secs, dt = 1 / 60) => { for (let t = 0; t < secs; t += dt) w.step(dt); };

function hiveEnv(withMesh = true) {
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells };
    if (withMesh) hives[a].mesh = meshes[a];
  }
  return { bounds, hives };
}
// the RAISED cell at the CAD pose (its opening faces up/outward)
const raised = (env, a) => { const c = env.hives[a].cells; return c.audience.opening_normal[2] > 0 ? ['audience', c.audience] : ['far', c.far]; };
const fire = (w, from, at, speed) => {   // straight line at `speed` (gravity still acts)
  const d = [at[0] - from[0], at[1] - from[1], at[2] - from[2]], L = Math.hypot(...d);
  return w.addBall('pollen', from, [d[0] / L * speed, d[1] / L * speed, d[2] / L * speed]);
};

// ---------------------------------------------------------------- INTAKE
function intakeWorld(space = 3) {
  const w = new BallWorld({ bounds });
  w.setIntake({ enabled: true, space });
  const got = []; w.onIntake = (b, side) => got.push([b, side]);
  return { w, got };
}

const spin = (w, robot, n = 120) => { for (let i = 0; i < n; i++) { w.setRobot(robot, DIMS); w.step(1 / 60); } };
const place = (robot, lx, ly) => { const c = Math.cos(robot.h), s = Math.sin(robot.h); return [robot.x + c * lx - s * ly, robot.y + s * lx + c * ly, BALL.r]; };

test('intake: balls touching / just ahead of the roller inside the mouth width are taken, front and rear', () => {
  const { w, got } = intakeWorld(10);
  const robot = { x: 72, y: 72, h: 0.6, vx: 0, vy: 0, om: 0 };
  const g = DIMS.halfX + BALL.r + 1;   // centre distance from the robot centre: ball surface 1 in from the face
  for (const [lx, ly] of [[g, 0], [g, 5], [g, -6], [-g, 0], [-g, -4]]) w.addBall('pollen', place(robot, lx, ly), [0, 0, 0]);
  spin(w, robot);
  // Hold limit is 4 total (user fact, balls.js MAX_HELD): the 5th ball is refused and stays on the field.
  assert.equal(got.length, 4, 'taken ' + got.length);
  assert.ok(got.some(x => x[1] === 'front') && got.some(x => x[1] === 'rear'));
  assert.equal(w.balls.length, 1, 'left on field ' + w.balls.length);
});

test('intake: no vacuuming from a distance, from beside the mouth, or past the pull radius', () => {
  const { w, got } = intakeWorld(10);
  const robot = { x: 72, y: 72, h: 0.6, vx: 0, vy: 0, om: 0 };
  const face = DIMS.halfX + BALL.r;
  const far = face + INTAKE.pullRadius + 1.5;
  const spots = [[far, 0], [far + 6, 3], [-far, 0], [face + 1, DIMS.halfY + 2.5], [face + 1, -(DIMS.halfY + 4)], [0, DIMS.halfY + 2.5], [0, -(DIMS.halfY + 2.5)]];
  for (const [lx, ly] of spots) w.addBall('pollen', place(robot, lx, ly), [0, 0, 0]);
  const p0 = w.balls.map(b => [b.x, b.y]);
  spin(w, robot, 180);
  assert.equal(got.length, 0, 'unexpectedly taken ' + got.length);
  w.balls.forEach((b, i) => assert.ok(Math.hypot(b.x - p0[i][0], b.y - p0[i][1]) < 1.0, 'ball ' + i + ' was dragged'));
});

test('intake: a ball hugging the side of the robot, level with the rollers, is not scooped', () => {
  const { w, got } = intakeWorld(10);
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  w.addBall('pollen', place(robot, DIMS.halfX - 1, DIMS.halfY + BALL.r + 0.2), [0, 0, 0]);
  w.addBall('pollen', place(robot, -(DIMS.halfX - 1), -(DIMS.halfY + BALL.r + 0.2)), [0, 0, 0]);
  spin(w, robot, 120);
  assert.equal(got.length, 0);
});

test('intake: a ball whose centre is inside the body (under the robot) is never loaded', () => {
  const { w, got } = intakeWorld(10);
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  w.addBall('pollen', [72, 72, BALL.r], [0, 0, 0]);
  w.setRobot(robot, DIMS); w.step(1 / 600);
  assert.equal(got.length, 0);
  assert.ok(INTAKE.maxOverlap < DIMS.halfX);
});

test('intake: constants are small and named (pull radius a few ball diameters at most, mouth inside the frame)', () => {
  assert.ok(INTAKE.pullRadius > 0 && INTAKE.pullRadius <= 4 * BALL.r);
  assert.ok(INTAKE.mouthInset >= 0);
  assert.equal(INTAKE.funnelLen, undefined);
  assert.equal(INTAKE.flare, undefined);
});

test('intake: sleeping start balls are woken and taken; driving into a ball loads it', () => {
  const { w, got } = intakeWorld(3);
  w.reset([{ id: 'a', type: 'pollen', position: [90, 72, BALL.r], diameter: 2.782 }]);
  const robot = { x: 60, y: 72, h: 0, vx: 50, vy: 0, om: 0 };
  for (let i = 0; i < 90 && !got.length; i++) { robot.x += 50 / 60; w.setRobot(robot, DIMS); w.step(1 / 60); }
  assert.equal(got.length, 1);
});

test('intake: respects capacity (3), extra balls just bounce off the robot and none tunnel in', () => {
  const { w, got } = intakeWorld(3);
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  for (let k = 0; k < 6; k++) w.addBall('pollen', [95 + k * 1.5, 72 + (k - 3) * 2, BALL.r], [-60 - k * 20, 0, 0]);
  for (let i = 0; i < 240; i++) {
    w.setRobot(robot, DIMS); w.step(1 / 60);
    for (const b of w.balls) if (Math.abs(b.x - 72) < 8 && Math.abs(b.y - 72) < 8 && b.z < 17) assert.fail('ball inside robot at ' + i);
  }
  assert.equal(got.length, 3);
  assert.equal(w.intake.space, 0);
  assert.equal(w.balls.length, 3);
});

test('intake: disabled, or a shot ball, is not taken', () => {
  const { w, got } = intakeWorld(5);
  w.setIntake({ enabled: false });
  const robot = { x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  w.addBall('pollen', [84, 72, BALL.r], [0, 0, 0]);
  for (let i = 0; i < 60; i++) { w.setRobot(robot, DIMS); w.step(1 / 60); }
  assert.equal(got.length, 0);
  w.setIntake({ enabled: true });
  const sh = w.addBall('pollen', [86, 66, BALL.r], [0, 0, 0]); sh.shot = true;
  for (let i = 0; i < 20; i++) { w.setRobot(robot, DIMS); w.step(1 / 60); }
  assert.ok(!got.some(g => g[0] === sh));
  assert.ok(INTAKE.pullRadius > 0 && INTAKE.sides.length === 2);
});

test('intake: a robot jumping a long way in one frame cannot skip a ball (swept robot)', () => {
  const { w, got } = intakeWorld(3);
  w.addBall('pollen', [80, 72, BALL.r], [0, 0, 0]);
  w.setRobot({ x: 60, y: 72, h: 0, vx: 0, vy: 0, om: 0 }, DIMS);
  w.step(1 / 60);
  w.setRobot({ x: 95, y: 72, h: 0, vx: 2100, vy: 0, om: 0 }, DIMS);   // 35 in in one frame, straight over the ball
  w.step(1 / 60);
  assert.equal(got.length, 1);
});

// ---------------------------------------------------------------- SHOOTING (rpm, no gating)
const root = {}; vm.runInNewContext(readFileSync(new URL('../web/js/shot_core.js', import.meta.url), 'utf8'), { window: root });
const Core = root.Core; SR.setCore(Core);
const SH = { exitHeight: 12, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const obstacles = JSON.parse(readFileSync(new URL('../web/data/colliders.json', import.meta.url), 'utf8'));
const cellB = Object.values(scene.constants.hives.blue.states.audience_up.cells).find(c => c.up);
const target = { x: cellB.aim_point[0], y: cellB.aim_point[1], z: cellB.aim_point[2], normal: cellB.opening_normal };
const ctxFor = (rpm, extra = {}) => ({ mode: 'auto', pose: { x: 130, y: 10, h: 2.2, vx: 0, vy: 0, om: 0 }, turretDeg: 0, hoodDeg: 45, rpm, target, shooter: SH,
  ammo: { limited: true, count: 3 }, obstacles, alliance: 'blue', ...extra });

test('shooting: any rpm is allowed (no readiness / range gating), only no-ammo blocks', () => {
  for (const mode of ['auto', 'assist', 'manual', 'free']) for (const rpm of [0, 50, 800, 2500, 9000]) {
    const ev = SR.evaluateShot(ctxFor(rpm, { mode }));
    assert.equal(ev.allowed, true, `${mode} ${rpm}: ${ev.summary}`);
  }
  const low = SR.evaluateShot(ctxFor(100));
  assert.ok(low.reasons.some(r => r.wouldBlock), 'still explains why it will miss');
  assert.equal(SR.evaluateShot(ctxFor(2000, { ammo: { limited: true, count: 0 } })).allowed, false);
  assert.equal(SR.evaluateShot({ ...ctxFor(2000), strict: true }).allowed, false, 'strict keeps the old gating');
});

test('shooting: exit speed follows the actual rpm (Core.speedFromRpm inverts rpmEstimate)', () => {
  for (const sp of [100, 250, 400]) assert.ok(Math.abs(Core.speedFromRpm(Core.rpmEstimate(sp, 1.89, 0.45), 1.89, 0.45) - sp) < 1e-9);
  const a = SR.launchBall(ctxFor(1000)), b = SR.launchBall(ctxFor(2000));
  assert.ok(Math.abs(b.speed / a.speed - 2) < 1e-9);
  assert.equal(SR.launchBall(ctxFor(0)).speed, 0);
  const v = Core.launchVelocity(2000, 0.3, 0.7, 1.89, 0.45, 1, [5, -2]);
  assert.ok(Math.abs(Math.hypot(v.vel[0] - 5, v.vel[1] + 2, v.vel[2]) - v.speed) < 1e-9);
});

test('shooting: low rpm shot really falls short, solver rpm shot lands in the hive', () => {
  const env = hiveEnv();
  const [, cell] = raised(env, 'red');
  const n = cell.opening_normal, ap = cell.aim_point;
  const from = [ap[0], ap[1] + n[1] * 40, 14];
  const T = 0.7, v = [(ap[0] - from[0]) / T, (ap[1] - from[1]) / T, (ap[2] - from[2]) / T + 0.5 * G * T];
  const speedOk = Math.hypot(...v);
  const rpmOk = Core.rpmEstimate(speedOk, SH.wheelR, SH.eff);
  const shoot = (rpm) => {
    const w = new BallWorld(env); let cap = 0; w.onCapture = () => cap++;
    const k = Core.speedFromRpm(rpm, SH.wheelR, SH.eff) / speedOk;
    w.addBall('pollen', from, v.map(q => q * k));
    run(w, 4);
    return cap;
  };
  assert.equal(shoot(rpmOk), 1);
  assert.equal(shoot(rpmOk * 0.6), 0);
  assert.equal(shoot(rpmOk * 0.1), 0);
});

// ---------------------------------------------------------------- HIVE: only through the opening, against the normal
test('hive: balls hitting from behind, above or the sides bounce off and are never captured (fast, real mesh)', () => {
  for (const a of ['red', 'blue']) {
    const env = hiveEnv(), w = new BallWorld(env);
    let cap = 0; w.onCapture = () => cap++;
    const piv = env.hives[a].pivot, c = [piv[0], piv[1], piv[2] + 6];
    const [, cell] = raised(env, a), n = cell.opening_normal;
    const back = n[1] > 0 ? -1 : 1;
    const dirs = [[0, back, 0], [1, 0, 0], [-1, 0, 0]];
    for (const d of dirs) for (const sp of [150, 600, 1500]) {
      const from = [c[0] - d[0] * 55, c[1] - d[1] * 55, 14];
      fire(w, from, c, sp);
    }
    for (const sp of [100, 800]) fire(w, [c[0], c[1], c[2] + 60], c, sp);   // from above
    run(w, 3, 1 / 120);
    assert.equal(cap, 0, a + ' captured ' + cap);
    for (const b of w.balls) assert.ok(Number.isFinite(b.x + b.y + b.z) && b.state === 'free');
  }
});

test('hive: fast shot through the opening scores; the same speed into the back never reaches the cell', () => {
  const env = hiveEnv();
  const [, cell] = raised(env, 'red'), n = cell.opening_normal, ap = cell.aim_point;
  const w = new BallWorld(env); let cap = 0; w.onCapture = () => cap++;
  fire(w, [ap[0], ap[1] + n[1] * 50, ap[2] + n[2] * 50], ap, 600);   // straight along -normal into the opening centre
  run(w, 2, 1 / 60);
  assert.equal(cap, 1);
  // only captures into the RAISED cell count: the back shot may legitimately pass the other (down) cell's opening
  const w2 = new BallWorld(env); let cap2 = 0; const rc = raised(env, 'red')[0]; w2.onCapture = (b, al, c) => { if (al === 'red' && c === rc) cap2++; };
  fire(w2, [ap[0], ap[1] - n[1] * 50, ap[2] - n[2] * 50 + 10], ap, 1500);
  run(w2, 2, 1 / 60);
  assert.equal(cap2, 0);
});

test('hive: balls dropped onto the raised cell lid are not captured', () => {
  const env = hiveEnv(), w = new BallWorld(env);
  let cap = 0; w.onCapture = () => cap++;
  const [, cell] = raised(env, 'blue'), ap = cell.aim_point;
  const bc = cell.back_wall_corners, xm = (Math.min(...bc.map(p => p[0])) + Math.max(...bc.map(p => p[0]))) / 2;
  for (let i = 0; i < 6; i++) w.addBall('pollen', [xm + (i - 3) * 2, ap[1] + 6, ap[2] + 25 + i * 4], [0, 0, 0]);
  run(w, 4);
  assert.equal(cap, 0);
});

// ---------------------------------------------------------------- COLLISION STABILITY
test('stability: a dropped pile of balls settles without jitter or deep overlap', () => {
  const w = new BallWorld({ bounds });
  let seed = 3; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 40; i++) w.addBall('pollen', [60 + rnd() * 10, 60 + rnd() * 10, 3 + i * 3], [0, 0, 0]);
  run(w, 8);
  let maxV = 0;
  for (const b of w.balls) { maxV = Math.max(maxV, Math.hypot(b.vx, b.vy, b.vz)); assert.ok(b.z >= BALL.r - 0.05 && b.x > bounds.minX && b.x < bounds.maxX); }
  assert.ok(maxV < 3, 'jitter speed ' + maxV);
  for (let i = 0; i < w.balls.length; i++) for (let j = i + 1; j < w.balls.length; j++) {
    const a = w.balls[i], b = w.balls[j];
    assert.ok(Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z) > 2 * BALL.r - 0.4, 'overlap');
  }
});

test('stability: very fast balls cannot tunnel through the walls or the robot box', () => {
  const w = new BallWorld({ bounds });
  const bs = [];
  for (const sp of [400, 900, 1700, 5000]) { bs.push(w.addBall('pollen', [20, 75, 8], [sp, 0, 0])); bs.push(w.addBall('pollen', [75, 20, 8], [0, sp, 0])); }
  for (let i = 0; i < 180; i++) {
    w.setRobot({ x: 72, y: 72, h: 0.3, vx: 0, vy: 0, om: 0 }, DIMS); w.step(1 / 60);
    for (const b of bs) {
      assert.ok(b.x > bounds.minX - 0.01 && b.x < bounds.maxX + 0.01 && b.y > bounds.minY - 0.01 && b.y < bounds.maxY + 0.01, 'left the field');
      const dx = b.x - 72, dy = b.y - 72, c = Math.cos(0.3), s = Math.sin(0.3);
      assert.ok(!(Math.abs(c * dx + s * dy) < 8 && Math.abs(-s * dx + c * dy) < 8 && b.z < 17), 'inside robot');
    }
  }
});

// ---------------------------------------------------------------- resetField
test('resetField: restores balls, hive angles, intake and robot; hiveDyn.reset is called', () => {
  const env = hiveEnv(), w = new BallWorld(env);
  w.reset(scene.balls.balls);
  const snap = w.balls.map(b => [b.id, b.x, b.y, b.z]);
  w.setIntake({ enabled: true, space: 3 });
  w.addBall('pollen', [50, 50, 5], [10, 0, 0]);
  w.setHiveAngles({ red: 30, blue: -30 }); run(w, 0.3);
  w.setRobot({ x: 72, y: 72, h: 0, vx: 0, vy: 0, om: 0 }, DIMS);
  w.balls[0].x += 30; w.balls[1].state = 'held';
  let resets = 0;
  w.resetField({ hiveDyn: { red: { reset() { resets++; } }, blue: { reset() { resets++; } } } });
  assert.equal(resets, 2);
  assert.equal(w.balls.length, snap.length);
  w.balls.forEach((b, i) => { assert.deepEqual([b.id, b.x, b.y, b.z], snap[i]); assert.equal(b.state, 'free'); assert.ok(b.sleeping); });
  assert.equal(w.robot, null);
  assert.equal(w._angles.red, env.hives.red.cadAngle);
  assert.equal(w.hv[0].delta, 0);
  run(w, 1);
  w.balls.forEach((b, i) => assert.deepEqual([b.id, b.x, b.y, b.z], snap[i]));
  w.resetField({ balls: [] });
  assert.equal(w.balls.length, 0);
});

// ---------------------------------------------------------------- HIVE ENTRY (balls arriving from above / forward)
// Aimed approach from outside the raised opening along its outward normal n, with gravity compensated so the ball
// reaches the aim point (+ox across the opening width, +oy across its slant) in T seconds from S inches out.
const approach = (cell, S, T, ox = 0, oy = 0) => {
  const ap = cell.aim_point, n = cell.opening_normal, uy = [0, n[2], -n[1]];
  const tgt = [ap[0] + ox, ap[1] + oy * uy[1], ap[2] + oy * uy[2]];
  const from = [tgt[0] + n[0] * S, tgt[1] + n[1] * S, tgt[2] + n[2] * S];
  return { from, v: [(tgt[0] - from[0]) / T, (tgt[1] - from[1]) / T, (tgt[2] - from[2]) / T + 0.5 * G * T] };
};
const hiveDynFor = async (a) => {
  const { HiveDynamics, HIVE_PARAMS } = await import('../web/js/hivecycle.js');
  return new HiveDynamics(scene.groups['hive_' + a], scene.constants.hives[a].states, { ...HIVE_PARAMS });
};

test('hive entry: a ball descending into the raised opening (centre, 5 in and 8 in across it) is captured and counted once', async () => {
  for (const a of ['red', 'blue']) for (const [ox, oy, T] of [[0, 0, 0.3], [0, 0, 0.45], [5, 0, 0.45], [8, 0, 0.3], [0, 3, 0.45], [0, -3, 0.6]]) {
    const env = hiveEnv(), w = new BallWorld(env), dyn = await hiveDynFor(a);
    const [cname, cell] = raised(env, a);
    let hooks = 0, kinds = [];
    w.onCapture = (b, al, c, info) => { hooks++; kinds.push(info.kind); dyn.addScoredBall(c, [b.x, b.y, b.z], info.kind); };
    const { from, v } = approach(cell, 70, T, ox, oy);
    w.addBall('pollen', from, v);
    run(w, 2);
    assert.equal(hooks, 1, `${a} ox ${ox} oy ${oy} T ${T}: hooks ${hooks}`);
    assert.equal(dyn.cycleStats().upCellCount, 1, `${a} count`);
    assert.equal(dyn.upCell, cname);
    assert.deepEqual(kinds, ['pollen']);
  }
});

test('hive entry: a ball that aims wide of the opening (11 in or more across it) does not enter or count', async () => {
  for (const a of ['red', 'blue']) for (const ox of [11, 14, -14]) {
    const env = hiveEnv(), w = new BallWorld(env), dyn = await hiveDynFor(a);
    const [, cell] = raised(env, a);
    let hooks = 0;
    w.onCapture = (b, al, c, info) => { hooks++; dyn.addScoredBall(c, [b.x, b.y, b.z], info.kind); };
    const { from, v } = approach(cell, 70, 0.45, ox, 0);
    w.addBall('pollen', from, v);
    run(w, 2);
    assert.equal(hooks, 0, `${a} ox ${ox} captured`);
    assert.equal(dyn.cycleStats().upCellCount, 0);
  }
});

test('hive entry: a solver shot from the standard shooting pose enters the aimed opening and the count goes up by 1', async () => {
  const SHX = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
  for (const a of ['red', 'blue']) {
    const env = hiveEnv(), w = new BallWorld(env), dyn = await hiveDynFor(a);
    const [cname, cell] = raised(env, a), ap = cell.aim_point, n = cell.opening_normal;
    const pose = { x: ap[0] + 8, y: ap[1] + Math.sign(n[1]) * 60, h: 1, vx: 0, vy: 0, om: 0 };
    const tg = { x: ap[0], y: ap[1], z: ap[2], normal: n };
    const p = { x: pose.x, y: pose.y, psi: pose.h, vx: 0, vy: 0, om: 0, rx: 0, ry: 0, z0: SHX.exitHeight, X: tg.x, Y: tg.y, Z: tg.z };
    let r = Core.solve(p, 'gamma', SHX.entryDeg * Math.PI / 180, 'low');
    const e = r.elev * 180 / Math.PI, c = Math.min(SHX.hoodMax, Math.max(SHX.hoodMin, e));
    if (c !== e) r = Core.solve(p, 'theta', c * Math.PI / 180, 'low');
    const f = SR.forwardShot({ pose, shooter: SHX, target: tg, turretDeg: r.turret * 180 / Math.PI, hoodDeg: c, rpm: Core.rpmEstimate(r.speed, SHX.wheelR, SHX.eff) });
    let hooks = 0;
    w.onCapture = (b, al, cc, info) => { hooks++; dyn.addScoredBall(cc, [b.x, b.y, b.z], info.kind); };
    w.setRobot(pose, DIMS);
    const b = w.addBall('pollen', f.exit, f.vel); b.shot = true;
    run(w, 3, 1 / 120);
    assert.equal(hooks, 1, a + ' hooks ' + hooks);
    assert.equal(dyn.cycleStats().upCellCount, 1, a + ' count');
    assert.equal(dyn.upCell, cname);
  }
});

test('hive entry: the capture hook gets the ball kind and the hive totals by kind (nectar and pollen)', async () => {
  const env = hiveEnv(), w = new BallWorld(env);
  const [, cell] = raised(env, 'blue');
  const seen = [];
  w.onCapture = (b, al, c, info) => seen.push(info);
  const shots = [['nectar_blue', 0.3], ['pollen', 0.45]];
  for (const [type, T] of shots) { const { from, v } = approach(cell, 70, T, 0, 0); w.addBall(type, from, v); run(w, 2); }
  assert.equal(seen.length, 2, 'captures ' + seen.length);
  assert.equal(seen[0].kind, 'nectar');
  assert.deepEqual(seen[0].totals, { pollen: 0, nectar: 1 });
  assert.equal(seen[1].kind, 'pollen');
  assert.deepEqual(seen[1].totals, { pollen: 1, nectar: 1 });
});
