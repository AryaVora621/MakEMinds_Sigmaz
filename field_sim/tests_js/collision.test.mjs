import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { buildObstacles, robotCorners, resolveRobot, slideVelocity } from '../web/js/collision.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const colliders = JSON.parse(fs.readFileSync(path.join(here, '..', 'web', 'data', 'colliders.json'), 'utf8'));
const obs = buildObstacles(colliders);
const B = obs.bounds;
const T = 1e-3;

function insideBounds(p) {
  return robotCorners(p.x, p.y, p.h).every(([x, y]) => x >= B.minX - T && x <= B.maxX + T && y >= B.minY - T && y <= B.maxY + T);
}

test('bounds come from the walls, not 144', () => {
  assert.ok(B.minX > 0 && B.maxX < 144 && B.maxX - B.minX > 140);
});

test('open space returns pose unchanged', () => {
  const r = resolveRobot({ x: 30, y: 30, h: 0.4 }, { x: 29, y: 29.5, h: 0.4 }, obs);
  assert.deepEqual([r.x, r.y, r.h, r.hit, r.pushed], [30, 30, 0.4, false, 0]);
  assert.equal(r.normals.length, 0);
});

test('each of the 4 walls blocks', () => {
  const cases = [[{ x: -5, y: 30 }, 'minX'], [{ x: 200, y: 30 }, 'maxX'], [{ x: 30, y: -5 }, 'minY'], [{ x: 30, y: 200 }, 'maxY']];
  for (const [p, side] of cases) {
    const r = resolveRobot({ ...p, h: 0 }, { x: 30, y: 30, h: 0 }, obs);
    assert.ok(r.hit, side);
    assert.ok(insideBounds(r), side);
    if (side === 'minX') assert.ok(Math.abs(r.x - (B.minX + 9)) < T);
    if (side === 'maxX') assert.ok(Math.abs(r.x - (B.maxX - 9)) < T);
    if (side === 'minY') assert.ok(Math.abs(r.y - (B.minY + 9)) < T);
    if (side === 'maxY') assert.ok(Math.abs(r.y - (B.maxY - 9)) < T);
  }
});

test('corner clamps both axes', () => {
  const r = resolveRobot({ x: -10, y: -10, h: 0 }, { x: 20, y: 20, h: 0 }, obs);
  assert.ok(r.hit && insideBounds(r));
  assert.equal(r.normals.length, 2);
  assert.ok(Math.abs(r.x - (B.minX + 9)) < T && Math.abs(r.y - (B.minY + 9)) < T);
});

test('rotated robot (45 deg) stops at its corner extent', () => {
  const h = Math.PI / 4;
  const r = resolveRobot({ x: 0, y: 70, h }, { x: 30, y: 70, h }, obs);
  assert.ok(Math.abs(r.x - (B.minX + 9 * Math.SQRT2)) < T);
  assert.ok(insideBounds({ ...r, h }));
});

test('rotating in place next to a wall translates away from it', () => {
  const p = { x: B.minX + 9, y: 70, h: 0 };
  const h = Math.PI / 4;
  const r = resolveRobot({ ...p, h }, p, obs);
  assert.ok(r.hit && insideBounds({ ...r, h }));
  assert.ok(r.x > p.x + 3);
});

test('fast moves do not tunnel through a wall or a hive leg', () => {
  const r = resolveRobot({ x: 500, y: 30, h: 0 }, { x: 20, y: 30, h: 0 }, obs);
  assert.ok(r.hit && r.x <= B.maxX - 9 + T);
  const r2 = resolveRobot({ x: 100, y: 57, h: 0 }, { x: 20, y: 57, h: 0 }, obs);
  assert.ok(r2.hit && r2.x < 47.3, 'stopped before the hive: x=' + r2.x);
});

test('spinning in a corner stays inside bounds', () => {
  let p = { x: B.minX + 9, y: B.minY + 9, h: 0 };
  for (let i = 0; i < 200; i++) {
    const h = p.h + 0.2;
    const r = resolveRobot({ x: p.x, y: p.y, h }, p, obs);
    p = { x: r.x, y: r.y, h };
    assert.ok(insideBounds(p), 'step ' + i);
  }
});

test('hive frame box blocks and normal points away', () => {
  const leg = colliders.boxes.find((b) => b.kind === 'hive_frame' && b.zMax >= 10);
  assert.ok(leg);
  const cx = (leg.minX + leg.maxX) / 2;
  const r = resolveRobot({ x: cx, y: (leg.minY + leg.maxY) / 2, h: 0 }, { x: cx, y: leg.minY - 30, h: 0 }, obs);
  assert.ok(r.hit);
  assert.ok(Math.abs(r.y - (leg.minY - 9)) < 0.01, 'y=' + r.y);
  assert.ok(r.normals.some((n) => n[1] < -0.99));
});

test('circle obstacle pushes robot out (synthetic circle)', () => {
  const o2 = buildObstacles({ bounds: colliders.bounds, boxes: [], circles: [{ x: 70, y: 70, r: 4, zMin: 0, zMax: 10, kind: 'post' }] });
  const r = resolveRobot({ x: 60, y: 70, h: 0 }, { x: 30, y: 70, h: 0 }, o2);
  assert.ok(r.hit);
  assert.ok(Math.abs(r.x - (70 - 4 - 9)) < 0.01, 'x=' + r.x);
  // rotated robot corner against the circle: corner ends outside it
  const h = Math.PI / 4;
  const r2 = resolveRobot({ x: 62, y: 66, h }, { x: 40, y: 50, h }, o2);
  assert.ok(r2.hit);
  const probe = resolveRobot({ x: r2.x, y: r2.y, h }, null, o2);
  assert.ok(!probe.hit || probe.pushed < 1e-2);
});

test('slideVelocity removes into-wall component only', () => {
  assert.deepEqual(slideVelocity(5, 3, [[-1, 0]]), { vx: 0, vy: 3 });
  assert.deepEqual(slideVelocity(-5, 3, [[-1, 0]]), { vx: -5, vy: 3 });
  const c = slideVelocity(4, 4, [[-1, 0], [0, -1]]);
  assert.deepEqual([c.vx, c.vy], [0, 0]);
});

test('robot never ends inside any box over random moves', () => {
  let seed = 7;
  const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  let p = { x: 20, y: 20, h: 0 };
  const singles = obs.boxes.map((b) => buildObstacles({ bounds: colliders.bounds, boxes: [{ ...b, zMin: 0, zMax: 5 }], circles: [] }));
  for (let i = 0; i < 3000; i++) {
    const t = { x: p.x + (rnd() - 0.5) * 12, y: p.y + (rnd() - 0.5) * 12, h: p.h + (rnd() - 0.5) * 0.8 };
    const r = resolveRobot(t, p, obs);
    p = { x: r.x, y: r.y, h: t.h };
    assert.ok(insideBounds(p));
    for (const s of singles) {
      const probe = resolveRobot(p, null, s);
      assert.ok(!probe.hit || probe.pushed < 1e-2, 'inside box at step ' + i);
    }
  }
});
