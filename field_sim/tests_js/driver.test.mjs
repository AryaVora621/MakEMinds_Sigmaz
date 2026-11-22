import test from 'node:test';
import assert from 'node:assert/strict';
import {
  readGamepad, mapKeyboard, MecanumDrive, integrate, MatchClock, driverCamera, ScoreKeeper, PracticeStats,
  driverForwardOffset, DRIVER_STATION,
} from '../web/js/driver.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) < e, `${a} !~ ${b}`);
const pad = (axes, pressed = {}) => ({
  axes, buttons: Array.from({ length: 16 }, (_, i) => ({ pressed: !!pressed[i], value: pressed[i] ? 1 : 0 })),
});

test('gamepad deadzone zeroes small input', () => {
  const c = readGamepad([pad([0.05, -0.05, 0.06, 0])]);
  assert.equal(c.fwd, 0); assert.equal(c.strafe, 0); assert.equal(c.turn, 0); assert.ok(c.connected);
});
test('gamepad deadzone rescales from the edge', () => {
  const c = readGamepad([pad([0, -0.54, 0, 0])]);
  near(c.fwd, (0.54 - 0.08) / 0.92);
  const full = readGamepad([pad([0, -1, 1, 0])]);
  near(full.fwd, 1); near(full.turn, -1);   // right stick right = clockwise
});
test('gamepad tolerates null entries and missing pad', () => {
  assert.equal(readGamepad([null, null]).connected, false);
  assert.equal(readGamepad(null).connected, false);
  const c = readGamepad([null, pad([-1, 0, 0, 0], { 7: true, 8: true })], 1);
  assert.ok(c.connected); near(c.strafe, 1); assert.ok(c.shoot); assert.ok(c.fieldCentricToggle);
});
test('keyboard mapping', () => {
  const c = mapKeyboard(new Set(['w', 'a', 'q', ' ']));
  near(c.fwd, Math.SQRT1_2); near(c.strafe, Math.SQRT1_2); assert.equal(c.turn, 1); assert.ok(c.shoot);
  const d = mapKeyboard(new Set(['s', 'd', 'e']));
  near(d.fwd, -Math.SQRT1_2); near(d.strafe, -Math.SQRT1_2); assert.equal(d.turn, -1);
  assert.equal(mapKeyboard(new Set(['w', 's'])).fwd, 0);
});

test('full stick reaches max speed and strafe is slower', () => {
  const dr = new MecanumDrive(); const S = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  for (let i = 0; i < 200; i++) dr.step({ fwd: 1, strafe: 0, turn: 0 }, 0.02, S);
  near(S.bodyV[0], 60);
  const S2 = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  for (let i = 0; i < 200; i++) dr.step({ fwd: 0, strafe: 1, turn: 1 }, 0.02, S2);
  near(S2.bodyV[1], 51); near(S2.bodyV[2], 3.2);
});
test('acceleration limit respected', () => {
  const dr = new MecanumDrive(); const S = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  dr.step({ fwd: 1, strafe: 0, turn: 0 }, 0.05, S);
  near(S.bodyV[0], 140 * 0.05);
});
test('braking is harder than accelerating and reverse counts as braking', () => {
  const dr = new MecanumDrive(); const S = { x: 0, y: 0, h: 0, bodyV: [60, 0, 0] };
  dr.step({ fwd: 0, strafe: 0, turn: 0 }, 0.05, S);
  near(S.bodyV[0], 60 - 140 * 1.6 * 0.05);
  assert.ok(60 - S.bodyV[0] > 140 * 0.05);
  const S2 = { x: 0, y: 0, h: 0, bodyV: [30, 0, 0] };
  dr.step({ fwd: -1, strafe: 0, turn: 0 }, 0.05, S2);
  near(S2.bodyV[0], 30 - 140 * 1.6 * 0.05);
});
test('robot-centric world direction follows heading', () => {
  const dr = new MecanumDrive();
  for (const h of [0, Math.PI / 2, Math.PI, -Math.PI / 2, 0.7]) {
    const S = { x: 0, y: 0, h, bodyV: [0, 0, 0] };
    let v; for (let i = 0; i < 200; i++) v = dr.step({ fwd: 1, strafe: 0, turn: 0 }, 0.02, S);
    near(v.vx, 60 * Math.cos(h), 1e-4); near(v.vy, 60 * Math.sin(h), 1e-4);
  }
});
test('field-centric: stick forward goes +y and stick left goes -x at any heading', () => {
  for (const h of [0, Math.PI / 2, Math.PI, -Math.PI / 2, 2.1]) {
    const dr = new MecanumDrive({ strafeFactor: 1 });
    const S = { x: 0, y: 0, h, bodyV: [0, 0, 0] };
    let v; for (let i = 0; i < 300; i++) v = dr.step({ fwd: 1, strafe: 0, turn: 0 }, 0.02, S, { fieldCentric: true });
    near(v.vx, 0, 1e-4); near(v.vy, 60, 1e-4);
    const S2 = { x: 0, y: 0, h, bodyV: [0, 0, 0] };
    for (let i = 0; i < 300; i++) v = dr.step({ fwd: 0, strafe: 1, turn: 0 }, 0.02, S2, { fieldCentric: true });
    near(v.vx, -60, 1e-4); near(v.vy, 0, 1e-4);
  }
});
test('field-centric headingOffset rotates the driver frame; integrate moves pose', () => {
  const dr = new MecanumDrive({ strafeFactor: 1 });
  const S = { x: 0, y: 0, h: 0.3, bodyV: [0, 0, 0] };
  let v; for (let i = 0; i < 300; i++) v = dr.step({ fwd: 1, strafe: 0, turn: 0 }, 0.02, S, { fieldCentric: true, headingOffset: Math.PI / 2 });
  near(v.vx, -60, 1e-4); near(v.vy, 0, 1e-4);
  assert.equal(S.x, 0);   // step does not move the pose
  integrate(S, v.vx, v.vy, 1, 0.1);
  near(S.x, -6, 1e-4); near(S.h, 0.4);
});

test('match clock phases and boundaries', () => {
  const c = new MatchClock(); const seen = [];
  c.onPhase((p, prev) => seen.push(prev + '>' + p));
  assert.equal(c.phase, 'idle'); c.update(5); assert.equal(c.elapsed, 0);
  c.start(); assert.equal(c.phase, 'auto'); assert.equal(c.label, '0:30');
  c.update(12); assert.equal(c.label, '0:18');
  c.update(18); assert.equal(c.phase, 'transition'); near(c.remaining, 8);
  c.update(8); assert.equal(c.phase, 'teleop'); assert.equal(c.label, '1:30');
  c.update(89.9); assert.equal(c.phase, 'teleop');
  c.update(0.2); assert.equal(c.phase, 'endgame');
  c.update(29.9); assert.equal(c.phase, 'over'); assert.equal(c.label, '0:00'); assert.equal(c.elapsed, 158);
  assert.deepEqual(seen, ['idle>auto', 'auto>transition', 'transition>teleop', 'teleop>endgame', 'endgame>over']);
});
test('flowerWindowOpen only in the last 60 s', () => {
  const c = new MatchClock(); assert.equal(c.flowerWindowOpen, false);
  c.start(); c.update(97); assert.equal(c.flowerWindowOpen, false);   // 61 s left
  c.update(1); assert.equal(c.flowerWindowOpen, true);                // 60 s left
  assert.equal(c.phase, 'teleop'); c.update(40); assert.equal(c.phase, 'endgame'); assert.equal(c.flowerWindowOpen, true);
});
test('match clock pause, reset, and big dt fires every phase', () => {
  const c = new MatchClock(); c.start(); c.update(10); c.pause(); c.update(50); near(c.elapsed, 10);
  c.start(); c.update(1); near(c.elapsed, 11);
  const seen = []; c.onPhase(p => seen.push(p)); c.update(1000);
  assert.deepEqual(seen, ['transition', 'teleop', 'endgame', 'over']);
  c.reset(); assert.equal(c.phase, 'idle'); assert.equal(c.elapsed, 0);
});

test('driver cameras stand in their own alliance area beside the field', () => {
  for (const mode of ['driver', 'driver_wide']) {
    const r = driverCamera('red', mode, { x: 72, y: 24, h: Math.PI / 2 }, {});
    const b = driverCamera('blue', mode, { x: 72, y: 24, h: Math.PI / 2 }, {});
    // alliance_area polygons: red x -53.6..0.3, blue x 143.7..197.7, both y 23.6..120.4
    assert.ok(r.pos[0] > -53.6 && r.pos[0] < 0.3);
    assert.ok(b.pos[0] > 143.7 && b.pos[0] < 197.7);
    assert.ok(r.pos[1] > 23.6 && r.pos[1] < 120.4 && b.pos[1] > 23.6 && b.pos[1] < 120.4);
    near(r.pos[2], 64); near(b.pos[2], 64);
  }
  near(DRIVER_STATION.red[1], 40);
  assert.equal(driverCamera('red', 'driver', {}, {}).fov, 60);
  assert.equal(driverCamera('red', 'driver_wide', {}, {}).fov, 85);
  const coach = driverCamera('blue', 'coach_box', {}, {});
  assert.ok(coach.pos[0] > 144 && coach.pos[2] > 64);
  assert.ok(driverCamera('red', 'coach_box', {}, {}).pos[0] < 0);
});
test('driver looks across the field at its centre at a shallow downward angle', () => {
  for (const [al, dir] of [['red', 1], ['blue', -1]]) {
    const c = driverCamera(al, 'driver', {}, {});
    assert.ok(Math.sign(c.target[0] - c.pos[0]) === dir);
    const t = (72 - c.pos[0]) / (c.target[0] - c.pos[0]);   // ray parameter reaching x=72
    near(c.pos[1] + t * (c.target[1] - c.pos[1]), 72, 1e-6);
    const pitch = Math.atan2(c.target[2] - c.pos[2], Math.hypot(c.target[0] - c.pos[0], c.target[1] - c.pos[1]));
    assert.ok(pitch < 0 && pitch > -Math.PI / 4);
  }
});
test('driverForwardOffset: red faces +x, blue faces -x', () => {
  near(driverForwardOffset('red'), -Math.PI / 2); near(driverForwardOffset('blue'), Math.PI / 2);
  for (const [al, fx, ly] of [['red', 1, 1], ['blue', -1, -1]]) {
    const dr = new MecanumDrive({ strafeFactor: 1, alliance: al });
    const S = { x: 0, y: 0, h: 0.9, bodyV: [0, 0, 0] };
    let v; for (let i = 0; i < 300; i++) v = dr.step({ fwd: 1, strafe: 0, turn: 0 }, 0.02, S, { fieldCentric: true });
    near(v.vx, 60 * fx, 1e-4); near(v.vy, 0, 1e-4);          // stick up = across the field
    const S2 = { x: 0, y: 0, h: 0.9, bodyV: [0, 0, 0] };
    for (let i = 0; i < 300; i++) v = dr.step({ fwd: 0, strafe: 1, turn: 0 }, 0.02, S2, { fieldCentric: true });
    near(v.vx, 0, 1e-4); near(v.vy, 60 * ly, 1e-4);          // stick left = the driver's left
  }
  const dr = new MecanumDrive({ strafeFactor: 1 });          // alliance per step; headingOffset adds on top
  const S = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  let v; for (let i = 0; i < 300; i++) v = dr.step({ fwd: 1, strafe: 0, turn: 0 }, 0.02, S, { fieldCentric: true, alliance: 'red', headingOffset: Math.PI / 2 });
  near(v.vx, 0, 1e-4); near(v.vy, 60, 1e-4);
});
test('camera look limits clamp', () => {
  const base = driverCamera('red', 'driver', {}, {});
  const yawOf = c => Math.atan2(c.target[0] - c.pos[0], c.target[1] - c.pos[1]) * 180 / Math.PI;
  const a = driverCamera('red', 'driver', {}, { yawDeg: 500, pitchDeg: 0 });
  const b = driverCamera('red', 'driver', {}, { yawDeg: 90, pitchDeg: 0 });
  near(yawOf(a), yawOf(b), 1e-6);
  assert.ok(yawOf(a) - yawOf(base) <= 70 + 1e-6);
  const up = driverCamera('red', 'driver', {}, { yawDeg: 0, pitchDeg: 500 });
  const pitchDeg = Math.atan2(up.target[2] - up.pos[2], Math.hypot(up.target[0] - up.pos[0], up.target[1] - up.pos[1])) * 180 / Math.PI;
  near(pitchDeg, 25, 1e-6);
});

test('ScoreKeeper totals by alliance and history', () => {
  const s = new ScoreKeeper();
  s.addPoints('hive_far', 'red'); s.addPoints('ball', 'red'); s.addPoints('hive_audience', 'blue');
  assert.equal(s.totals.red, 3); assert.equal(s.totals.blue, 2); assert.equal(s.history.length, 3);
  assert.throws(() => s.addPoints('nope', 'red')); assert.throws(() => s.addPoints('ball', 'green'));
  s.reset(); assert.equal(s.totals.red, 0); assert.equal(s.history.length, 0);
});
test('PracticeStats summary', () => {
  const p = new PracticeStats();
  for (let i = 0; i < 4; i++) p.record('shot');
  p.record('scored'); p.record('scored'); p.record('scored'); p.record('miss');
  p.record('cycle', 10); p.record('cycle', 20);
  const s = p.summary();
  assert.equal(s.shots, 4); assert.equal(s.scored, 3); assert.equal(s.misses, 1);
  near(s.accuracy, 0.75); near(s.avgCycle, 15); assert.equal(s.bestCycle, 10); near(s.cyclesPerMin, 4);
  assert.equal(new PracticeStats().summary().accuracy, 0);
});
