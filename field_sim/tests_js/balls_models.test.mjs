// balls_models.test.mjs - the per-model switches in balls.js (BALL_MODELS / BallWorld.models) and setRestitution.
// Units: in, s, in/s. Run: node --test "tests_js/balls*.test.mjs"
//
// DEFAULTS DECIDED HERE (round 2):
//  aero = 'legacy': the ball-aerodynamics stepFlight(legacy) path is wired and is bit-identical to the old inline code
//    (test 1). 'physical' works (test 2) but its drag is ~2.7x BALL.dragK, which moves every shot off the aim that
//    shotrules/shot_core were tuned to, so it stays opt-in.
//  ballBall = 'legacy': 'solver' (ball-ball-collisions BallBallSystem) passes the scene smoke tests below and the whole
//    balls/sim_physics suites, but it loses most restitution in a free-flight head-on hit (e ~0.09 instead of 0.7, the
//    todo test below): ContactSolver.applyRestitution skips any contact whose gap is > linearSlop, and a speculative
//    contact is always caught at a gap. It is therefore not "at least as stable/correct" yet and stays opt-in.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL, BALL_MODELS, setRestitution, syncBallSurfaces } from '../web/js/balls.js';
import { SURFACES } from '../web/js/physics/ball-bounce-rolling/params.js';
import { AERO, buildAeroModel } from '../web/js/physics/ball-aerodynamics/index.js';

const G = BALL.g;
const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 };
const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
function realEnv(models) {
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells };
  }
  return { bounds, hives, models };
}

test('defaults: aero legacy, ballBall legacy; BallWorld.models overrides per world', () => {
  assert.equal(BALL_MODELS.aero, 'legacy');
  assert.equal(BALL_MODELS.ballBall, 'legacy');
  const w = new BallWorld({ bounds, models: { aero: 'physical' } });
  assert.equal(w.model('aero'), 'physical');
  assert.equal(w.model('ballBall'), 'legacy');
});

test('aero legacy through stepFlight is bit-identical to the original inline drag + gravity', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [10, 70, 40], [300, 20, 150]);
  // reference: HEAD balls.js _substep lines (drag before integration, then exact constant-g position update)
  const s = { x: 10, y: 70, z: 40, vx: 300, vy: 20, vz: 150 };
  for (let i = 0; i < 20; i++) {
    w.step(1 / 60);
    const n = Math.min(120, Math.max(1, Math.ceil((1 / 60) / BALL.maxSubstep),
      Math.ceil((1 / 60) * (Math.hypot(s.vx, s.vy, s.vz) + G / 60) / BALL.maxStepTravel))), h = (1 / 60) / n;
    for (let k = 0; k < n; k++) {
      const sp = Math.sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz), f = 1 / (1 + BALL.dragK * sp * h);
      s.vx *= f; s.vy *= f; s.vz *= f;
      s.x += s.vx * h; s.y += s.vy * h; s.z += s.vz * h - 0.5 * G * h * h; s.vz -= G * h;
    }
    if (b.z < b.r + 1) break;   // stop before the floor (contacts are not part of the reference)
    assert.equal(b.x, s.x); assert.equal(b.y, s.y); assert.equal(b.z, s.z);
    assert.equal(b.vx, s.vx); assert.equal(b.vz, s.vz);
  }
});

test('aero physical: speed loss scales with the physical drag coefficient (k = 0.5*rho*Cd*A/m)', () => {
  const loss = (mode) => {
    const w = new BallWorld({ bounds: { minX: -1000, minY: -1000, maxX: 1000, maxY: 1000 }, models: { aero: mode } });
    const b = w.addBall('pollen', [0, 0, 200], [500, 0, 0]);
    for (let i = 0; i < 15; i++) w.step(1 / 60);   // 0.25 s, still far above the floor
    // horizontal speed only: gravity does not change it, drag does
    return 1 - b.vx / 500;
  };
  const kPhys = buildAeroModel(Object.assign({}, AERO, { model: 'physical' }), AERO.kinds.pollen).kDrag;   // 1/in
  const lLeg = loss('legacy'), lPhys = loss('physical');
  // pure-drag analytic, horizontal: v(t) = v0/(1 + k v0 t) (|v| ~ vx here, vz <= 97 in/s adds <2% to |v|)
  const ana = (k) => 1 - 1 / (1 + k * 500 * 0.25);
  assert.ok(Math.abs(lLeg - ana(BALL.dragK)) < 0.1 * ana(BALL.dragK), `legacy ${lLeg} vs ${ana(BALL.dragK)}`);
  assert.ok(Math.abs(lPhys - ana(kPhys)) < 0.1 * ana(kPhys), `physical ${lPhys} vs ${ana(kPhys)}`);
  assert.ok(lPhys > lLeg, 'physical air drag is stronger than the tuned legacy value');
});

test('setRestitution writes SURFACES and the BALL mirror; bounce rise follows e^2; bad input throws', () => {
  const e0 = SURFACES.floor.eRef, eMax0 = SURFACES.floor.eMax;
  const rise = () => {
    const w = new BallWorld({ bounds });
    const b = w.addBall('pollen', [70, 70, 40], [0, 0, 0]);
    let down = false, top = 0;
    for (let i = 0; i < 240; i++) {
      w.step(1 / 240);
      if (b.vz < -50) down = true;
      if (down && b.vz > 0) top = Math.max(top, b.z);
      if (down && top > 0 && b.vz < 0) break;
    }
    return top - b.r;
  };
  try {
    assert.equal(setRestitution('floor', 0.3), 0.3);
    assert.equal(BALL.restitutionFloor, 0.3);
    const r3 = rise();
    setRestitution('floor', 0.6);
    const r6 = rise();
    // e(v) has ePow 0.1 on the floor, so the ratio is close to (0.6/0.3)^2 = 4, not exact
    assert.ok(r6 / r3 > 3.2 && r6 / r3 < 4.8, `rise ratio ${r6 / r3}`);
    setRestitution('floor', 0.9);   // above the floor's own eMax 0.75: the clamp widens so 0.9 is really used
    assert.ok(SURFACES.floor.eMax >= 0.9);
    assert.throws(() => setRestitution('floor', 1.5), RangeError);
    assert.throws(() => setRestitution('carpet', 0.5), RangeError);
  } finally { setRestitution('floor', e0); }
  assert.equal(SURFACES.floor.eRef, e0);
  assert.equal(SURFACES.floor.eMax, eMax0, 'restoring brings the clamp back');
});

test('a BALL.restitution* / rollFriction write (validation-tuning panel) reaches SURFACES on the next step', () => {
  const e0 = SURFACES.wall.eRef, c0 = SURFACES.floor.crr;
  try {
    BALL.restitutionWall = 0.2; BALL.rollFriction = 0.08;
    new BallWorld({ bounds }).step(1 / 60);
    assert.equal(SURFACES.wall.eRef, 0.2);
    assert.equal(SURFACES.floor.crr, 0.08);
    // a direct SURFACES write is NOT overwritten by the (unchanged) BALL copy
    SURFACES.floor.crr = 0.05; syncBallSurfaces();
    assert.equal(SURFACES.floor.crr, 0.05);
  } finally { BALL.restitutionWall = e0; BALL.rollFriction = c0; syncBallSurfaces(); }
  assert.equal(SURFACES.wall.eRef, e0); assert.equal(SURFACES.floor.crr, c0);
});

// 56-ball start scene: the robot drives at 40 in/s (ASSUMED test speed) into all four floor stacks while both hives tip
// (dumping their preloads), 10 s at 60 Hz frames.
const WAYPOINTS = [[30, 49], [12, 49], [30, 30], [95, 24], [95, 12], [110, 95], [130, 95], [100, 120], [49, 120], [49, 130]];   // in
function sceneRun(mode) {
  const w = new BallWorld(realEnv({ ballBall: mode }));
  w.reset(scene.balls.balls);
  assert.equal(w.balls.length, 56);
  const start = w.balls.map(b => [b.x, b.y, b.z]);
  const dims = { halfX: 9, halfY: 9, height: 18 };
  let worstPen = 0, wp = 1, x = WAYPOINTS[0][0], y = WAYPOINTS[0][1];
  const V = 40, dt = 1 / 60, frames = 600;
  const t0 = performance.now();
  for (let i = 0; i < frames; i++) {
    if (i === 60) w.setHiveAngles({ red: 30, blue: 30 });
    let vx = 0, vy = 0;
    if (wp < WAYPOINTS.length) {
      const dx = WAYPOINTS[wp][0] - x, dy = WAYPOINTS[wp][1] - y, d = Math.hypot(dx, dy);
      if (d < V * dt) wp++; else { vx = V * dx / d; vy = V * dy / d; }
    }
    x += vx * dt; y += vy * dt;
    w.setRobot({ x, y, h: 0, vx, vy, om: 0 }, dims);
    w.step(dt);
  }
  const ms = (performance.now() - t0) / frames;
  const moved = w.balls.filter((b, i) => Math.hypot(b.x - start[i][0], b.y - start[i][1], b.z - start[i][2]) > 1).length;
  assert.ok(moved >= 15, `the robot/hives must actually disturb the pile (moved ${moved})`);
  const fr = w.balls.filter(b => b.state === 'free');
  for (let i = 0; i < fr.length; i++) for (let j = i + 1; j < fr.length; j++) {
    const a = fr[i], b = fr[j];
    worstPen = Math.max(worstPen, a.r + b.r - Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
  }
  for (const b of w.balls) assert.ok(Number.isFinite(b.x + b.y + b.z + b.vx + b.vy + b.vz), 'finite state');
  return { ms, worstPen, w };
}
for (const mode of ['legacy', 'solver']) {
  test(`56-ball scene with a driving robot stays finite, non-overlapping and fast (ballBall=${mode})`, () => {
    const r = sceneRun(mode);
    console.log(`56-ball scene ${mode}: ${r.ms.toFixed(3)} ms/frame, worst overlap ${r.worstPen.toFixed(3)} in`);
    assert.ok(r.ms < 4, 'too slow: ' + r.ms);   // budget: well under a 16.7 ms 60 Hz frame (ASSUMED budget)
    assert.ok(r.worstPen < 0.3, 'overlap ' + r.worstPen);   // in; legacy resting tolerance is 0.15 (balls.test)
  });
}

test('solver mode: on-floor head-on hit conserves momentum and keeps e = SURFACES.ball.eRef', () => {
  const c = { mu: SURFACES.floor.muSlide, crr: SURFACES.floor.crr };
  SURFACES.floor.muSlide = 0; SURFACES.floor.crr = 0;   // floor friction is external to the pair (see balls.test.mjs)
  try {
    const w = new BallWorld({ bounds, models: { ballBall: 'solver' } });
    const a = w.addBall('pollen', [60, 70, BALL.r], [40, 0, 0]);
    const b = w.addBall('pollen', [70, 70, BALL.r], [-20, 0, 0]);
    for (let i = 0; i < 12; i++) w.step(1 / 60);
    assert.ok(Math.abs(a.vx + b.vx - 20) < 0.05, 'p ' + (a.vx + b.vx));
    assert.ok(Math.abs((b.vx - a.vx) / 60 - SURFACES.ball.eRef) < 0.02, 'e ' + (b.vx - a.vx) / 60);
  } finally { SURFACES.floor.muSlide = c.mu; SURFACES.floor.crr = c.crr; }
});

test('solver mode: free-flight head-on hit keeps e = 0.7', { todo: 'ball-ball-collisions ContactSolver.applyRestitution skips contacts with s > linearSlop, so speculative contacts lose restitution (measured e ~0.09)' }, () => {
  const w = new BallWorld({ bounds, models: { ballBall: 'solver' } });
  const a = w.addBall('pollen', [60, 70, 40], [40, 0, 0]);
  const b = w.addBall('pollen', [70, 70, 40], [-20, 0, 0]);
  for (let i = 0; i < 12; i++) w.step(1 / 60);
  assert.ok(Math.abs((b.vx - a.vx) / 60 - SURFACES.ball.eRef) < 0.05, 'e ' + (b.vx - a.vx) / 60);
});

test('switching ballBall legacy -> solver -> legacy mid-run keeps a sleeping pile asleep and consistent', () => {
  const w = new BallWorld(realEnv());
  w.reset(scene.balls.balls);
  const snap = w.balls.map(b => [b.x, b.y, b.z]);
  for (let i = 0; i < 30; i++) w.step(1 / 60);
  w.models.ballBall = 'solver';
  for (let i = 0; i < 60; i++) w.step(1 / 60);
  w.models.ballBall = 'legacy';
  for (let i = 0; i < 30; i++) w.step(1 / 60);
  w.balls.forEach((b, i) => assert.deepEqual([b.x, b.y, b.z], snap[i]));
  assert.ok(w.balls.every(b => b._bbIsl === undefined || b._bbIsl === -1), 'islands dropped on switch back');
});
