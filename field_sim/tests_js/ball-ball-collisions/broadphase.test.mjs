import test from 'node:test';
import assert from 'node:assert/strict';
import { BB_PARAMS, G_IN } from '../../web/js/physics/ball-ball-collisions/params.js';
import { ensureBody } from '../../web/js/physics/ball-ball-collisions/bodies.js';
import { computeMargins, BallGrid3D } from '../../web/js/physics/ball-ball-collisions/grid3d.js';
import { mkBall } from './harness.mjs';

const DT = 1 / 60;
function rng(seed) { let s = seed >>> 0; return () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 4294967296); }
const isStatic = (b) => b.sleeping === true || b.pinned === true;

function randomBalls(n, seed) {
  const R = rng(seed), out = [];
  for (let i = 0; i < n; i++) {
    const b = mkBall(R() < 0.5 ? 'pollen' : 'nectar', R() * 140, R() * 140, R() * 40,
      (R() - 0.5) * 300, (R() - 0.5) * 300, (R() - 0.5) * 300);
    if (R() < 0.15) { b.sleeping = true; b.vx = b.vy = b.vz = 0; }
    if (R() < 0.05) b.pinned = true;
    out.push(ensureBody(b));
  }
  return out;
}
function bruteSet(balls, m) {
  const set = new Set();
  for (let i = 0; i < balls.length; i++) for (let j = i + 1; j < balls.length; j++) {
    const a = balls[i], b = balls[j];
    if (a.state !== 'free' || b.state !== 'free' || (isStatic(a) && isStatic(b))) continue;
    const d = Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
    if (d < a.r + b.r + m[i] + m[j]) set.add(`${i},${j}`);
  }
  return set;
}
function gridPairs(grid, balls) {
  const idx = new Map(balls.map((b, i) => [b, i]));
  const out = [];
  for (let k = 0; k < grid.pairCount; k++) out.push([idx.get(grid.pairs[2 * k]), idx.get(grid.pairs[2 * k + 1])]);
  return out;
}

test('grid pairs equal brute force, no duplicates, sorted, deterministic', () => {
  for (const seed of [1, 2, 3]) {
    const balls = randomBalls(300, seed);
    const m = computeMargins(balls, DT, G_IN, BB_PARAMS.speculativeDistance);
    const grid = new BallGrid3D();
    grid.build(balls, m);
    const got = gridPairs(grid, balls);
    const keys = got.map(([i, j]) => `${i},${j}`);
    assert.equal(new Set(keys).size, keys.length, 'duplicates');
    assert.ok(got.every(([i, j]) => i < j), 'first must be lower index');
    for (let k = 1; k < got.length; k++) {
      assert.ok(got[k - 1][0] < got[k][0] || (got[k - 1][0] === got[k][0] && got[k - 1][1] < got[k][1]), 'unsorted');
    }
    const want = bruteSet(balls, m);
    assert.ok(want.size > 0, 'test needs some pairs');
    assert.deepEqual(new Set(keys), want);
    grid.build(balls, m);
    assert.deepEqual(gridPairs(grid, balls), got, 'second build differs');
  }
});

test('fast ball is paired with a ball inside its margin', () => {
  const a = ensureBody(mkBall('pollen', 0, 0, 0, 1500, 0, 0));
  const b = ensureBody(mkBall('nectar', 15, 0, 0));
  const far = ensureBody(mkBall('pollen', 90, 90, 0));
  const balls = [a, b, far];
  const m = computeMargins(balls, DT, G_IN, BB_PARAMS.speculativeDistance);
  assert.ok(m[0] > BB_PARAMS.fastMargin);
  const grid = new BallGrid3D().build(balls, m);
  const got = gridPairs(grid, balls).map(([i, j]) => `${i},${j}`);
  assert.deepEqual(got, ['0,1']);
});

test('sleeping-sleeping pairs are excluded, sleeping-awake kept', () => {
  const a = ensureBody(mkBall('pollen', 0, 0, 1.391, 0, 0, 0, { sleeping: true }));
  const b = ensureBody(mkBall('pollen', 2.5, 0, 1.391, 0, 0, 0, { sleeping: true }));
  let balls = [a, b];
  let grid = new BallGrid3D().build(balls, computeMargins(balls, DT, G_IN, BB_PARAMS.speculativeDistance));
  assert.equal(grid.pairCount, 0);
  const c = ensureBody(mkBall("pollen", 1.25, 2.0, 1.391));
  balls = [a, b, c];
  grid = new BallGrid3D().build(balls, computeMargins(balls, DT, G_IN, BB_PARAMS.speculativeDistance));
  const got = gridPairs(grid, balls).map(([i, j]) => `${i},${j}`).sort();
  assert.deepEqual(got, ['0,2', '1,2']);
});

test('query matches brute force', () => {
  const balls = randomBalls(300, 7);
  const m = computeMargins(balls, DT, G_IN, BB_PARAMS.speculativeDistance);
  const grid = new BallGrid3D().build(balls, m);
  const R = rng(99);
  for (let t = 0; t < 20; t++) {
    const x = R() * 140, y = R() * 140, z = R() * 40, rad = 2 + R() * 15;
    const got = new Set(grid.query(x, y, z, rad, []).map((b) => balls.indexOf(b)));
    const want = new Set();
    balls.forEach((b, i) => {
      if (b.state !== 'free') return;
      if (Math.hypot(b.x - x, b.y - y, b.z - z) < rad + b.r + m[i]) want.add(i);
    });
    assert.deepEqual(got, want);
  }
});
