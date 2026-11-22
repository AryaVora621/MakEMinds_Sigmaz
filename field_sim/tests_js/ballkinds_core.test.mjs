import test from 'node:test';
import assert from 'node:assert/strict';
import { BallWorld, BALL, BALL_KINDS, ballKindCounts } from '../web/js/balls.js';

const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 };

test('BALL_KINDS: pollen 2.8 in and nectar 3.6 in diameters, metres consistent with inches', () => {
  assert.equal(BALL_KINDS.pollen.radiusIn, 1.391);   // 2.782 in dia, rounds to the manual's 2.8 in
  assert.equal(BALL_KINDS.nectar.radiusIn, 1.7995);  // 3.599 in dia, rounds to the manual's 3.6 in
  assert.ok(Math.abs(BALL_KINDS.pollen.radiusM - 0.0353) < 0.0002);
  assert.ok(Math.abs(BALL_KINDS.nectar.radiusM - BALL_KINDS.nectar.radiusIn * 0.0254) < 1e-9);
  assert.ok(BALL_KINDS.nectar.massKg > 0 && BALL_KINDS.pollen.massKg > 0);
  assert.equal(BALL_KINDS.nectar.assumed, true);
  assert.equal(BALL.r, BALL_KINDS.pollen.radiusIn);
  assert.equal(BALL.nectarR, BALL_KINDS.nectar.radiusIn);
});

test('default kind is pollen and gets pollen radius', () => {
  const w = new BallWorld({ bounds });
  const b = w.addBall('pollen', [70, 70, 30], [0, 0, 0]);
  assert.equal(b.kind, 'pollen');
  assert.equal(b.r, BALL_KINDS.pollen.radiusIn);
  const c = w.addBall('pollen', [30, 30, 30], [0, 0, 0]);
  assert.equal(c.kind, 'pollen');
});

test('explicit nectar kind gets nectar radius', () => {
  const w = new BallWorld({ bounds });
  const n = w.addBall('nectar_red', [70, 70, 30], [0, 0, 0], 'nectar');
  assert.equal(n.kind, 'nectar');
  assert.equal(n.r, BALL_KINDS.nectar.radiusIn);
});

test('nectar type string without explicit kind is classified as nectar', () => {
  const w = new BallWorld({ bounds });
  const n = w.addBall('nectar_blue', [70, 70, 30], [0, 0, 0]);
  assert.equal(n.kind, 'nectar');
  assert.equal(n.r, BALL_KINDS.nectar.radiusIn);
});

test('reset start balls honour kind', () => {
  const w = new BallWorld({ bounds });
  w.reset([
    { id: 'p', type: 'pollen', position: [50, 50, 1.4], diameter: undefined },
    { id: 'n', type: 'nectar_red', kind: 'nectar', position: [60, 50, 1.8] },
  ]);
  const p = w.balls.find(b => b.id === 'p'), n = w.balls.find(b => b.id === 'n');
  assert.equal(p.kind, 'pollen');
  assert.equal(n.kind, 'nectar');
  assert.equal(n.r, BALL_KINDS.nectar.radiusIn);
});

test('ballKindCounts counts pollen and nectar', () => {
  assert.deepEqual(ballKindCounts([]), { pollen: 0, nectar: 0 });
  assert.deepEqual(ballKindCounts(null), { pollen: 0, nectar: 0 });
  const balls = [
    { type: 'pollen', kind: 'pollen' },
    { type: 'nectar_red', kind: 'nectar' },
    { type: 'nectar_blue' },
    { type: 'pollen' },
    { type: 'pollen', kind: 'pollen' },
  ];
  assert.deepEqual(ballKindCounts(balls), { pollen: 3, nectar: 2 });
});

test('ballKindCounts on a world after mixed launches', () => {
  const w = new BallWorld({ bounds });
  w.addBall('pollen', [20, 20, 30], [0, 0, 0]);
  w.addBall('nectar_red', [30, 20, 30], [0, 0, 0], 'nectar');
  w.addBall('pollen', [40, 20, 30], [0, 0, 0]);
  assert.deepEqual(ballKindCounts(w.balls), { pollen: 2, nectar: 1 });
});
