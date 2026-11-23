import test from 'node:test';
import assert from 'node:assert/strict';
import { BallBallSystem } from '../../web/js/physics/ball-ball-collisions/system.js';
import { mkBall, runWorld } from './harness.mjs';

const R = 1.391; // in, pollen radius

test('single ball on the floor sleeps after ~0.5 s, not before 0.45 s', () => {
  const sys = new BallBallSystem();
  const b = mkBall('pollen', 0, 0, R);
  let sleptAt = null;
  runWorld([b], { system: sys, seconds: 1, onStep(i, balls) { if (sleptAt === null && balls[0].sleeping) sleptAt = (i + 1) / 60; } });
  assert.ok(sleptAt !== null, 'never slept');
  assert.ok(sleptAt >= 0.45 && sleptAt <= 0.6, `slept at ${sleptAt}`);
});

test('two touching balls form one island and sleep together; wake wakes both', () => {
  const sys = new BallBallSystem();
  const a = mkBall('pollen', 0, 0, R), b = mkBall('pollen', 2 * R, 0, R);
  const balls = [a, b];
  runWorld(balls, { system: sys, seconds: 1 });
  assert.ok(a.sleeping && b.sleeping);
  assert.equal(sys.stats().sleepingIslands, 1);
  assert.equal(sys.wake(a), true);
  assert.ok(!a.sleeping && !b.sleeping);
});

test('onRemoved wakes the remaining island members', () => {
  const sys = new BallBallSystem();
  const balls = [0, 1, 2].map((i) => mkBall('pollen', 2 * R * i, 0, R));
  runWorld(balls, { system: sys, seconds: 1 });
  assert.ok(balls.every((b) => b.sleeping));
  const gone = balls.shift();
  sys.onRemoved(gone, balls);
  assert.ok(balls.every((b) => !b.sleeping));
});

test('an awake ball dropped onto a sleeping group wakes it', () => {
  const sys = new BallBallSystem();
  const s3 = Math.sqrt(3);
  const group = [mkBall('pollen', 0, 0, R), mkBall('pollen', 2 * R, 0, R), mkBall('pollen', R, s3 * R, R)];
  runWorld(group, { system: sys, seconds: 1 });
  assert.ok(group.every((b) => b.sleeping));
  const dropper = mkBall('pollen', R, s3 * R / 3, 12);
  const all = [...group, dropper];
  let woke = false;
  runWorld(all, { system: sys, seconds: 0.5, onStep() { if (group.some((b) => !b.sleeping)) woke = true; } });
  assert.ok(woke, 'group never woke');
});

test('unsupported cluster never sleeps (at rest in zero gravity, and free falling)', () => {
  let sys = new BallBallSystem();
  let balls = [0, 1, 2].map((i) => mkBall('pollen', 2 * R * i, 0, 100));
  runWorld(balls, { system: sys, seconds: 1, gravity: 0, floor: false });
  assert.ok(balls.every((b) => !b.sleeping), 'slept without support (no gravity)');
  sys = new BallBallSystem();
  balls = [0, 1, 2].map((i) => mkBall('pollen', 2 * R * i, 0, 1000));
  runWorld(balls, { system: sys, seconds: 0.8, floor: false });
  assert.ok(balls.every((b) => !b.sleeping), 'slept while falling');
});

test('adoptSleeping groups touching start balls', () => {
  const sys = new BallBallSystem();
  const a = mkBall('pollen', 0, 0, R, 0, 0, 0, { sleeping: true });
  const b = mkBall('pollen', 2 * R, 0, R, 0, 0, 0, { sleeping: true });
  const c = mkBall('pollen', 50, 50, R, 0, 0, 0, { sleeping: true });
  sys.adoptSleeping([a, b, c]);
  assert.equal(sys.stats().sleepingIslands, 2);
  sys.wake(a);
  assert.ok(!a.sleeping && !b.sleeping && c.sleeping);
});
