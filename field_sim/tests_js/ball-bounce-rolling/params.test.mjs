// Acceptance criteria 1-2 (ball-bounce-rolling). Units: in, s, rad/s.
import test from 'node:test';
import assert from 'node:assert/strict';
import { BBR, SURFACES, inertiaFactor, restitutionAt, bounceArmed, surfaceParams } from '../../web/js/physics/ball-bounce-rolling/params.js';
import { BALL_KINDS } from '../../web/js/balls.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg || ''} ${a} vs ${b} (tol ${tol})`);

test('1. inertia factor: pollen and nectar in [0.60, 0.667]', () => {
  for (const kind of ['pollen', 'nectar']) {
    const k = inertiaFactor(BALL_KINDS[kind].radiusIn, BALL_KINDS[kind].massKg);
    assert.ok(k >= 0.60 && k <= 2 / 3 + 1e-12, `${kind} k=${k}`);
  }
});

test('1. inertia factor: thin shell is 2/3, solid is 0.4', () => {
  near(inertiaFactor(1.391, 1e-12), 2 / 3, 1e-6, 'thin shell');
  assert.equal(inertiaFactor(1.391, 100), 0.4);   // shell thicker than the radius -> x = 0 -> solid
});

test('1. kOverride replaces the computed value', () => {
  BBR.kOverride = 0.5;
  try { assert.equal(inertiaFactor(1.391, 0.025), 0.5); } finally { BBR.kOverride = null; }
  assert.notEqual(inertiaFactor(1.391, 0.025), 0.5);
});

test('2. restitutionAt(floor, 172) = 0.45 exactly', () => {
  near(restitutionAt(SURFACES.floor, SURFACES.floor.vRef), 0.45, 1e-9);
});

test('2. e is monotone non-increasing in vin and stays in its clamps', () => {
  for (const [name, s] of Object.entries(SURFACES)) {
    let prev = Infinity;
    for (let vin = 0.5; vin <= 2000; vin *= 1.05) {
      const e = restitutionAt(s, vin);
      assert.ok(e <= prev + 1e-12, `${name} e rose at vin=${vin}`);
      assert.ok(e >= s.eMin - 1e-12 && e <= s.eMax + 1e-12, `${name} e=${e} out of clamp`);
      prev = e;
    }
  }
});

test('2. floor e is about 0.51 at 50 in/s; non-floor surfaces are constant', () => {
  near(restitutionAt(SURFACES.floor, 50), 0.51, 0.01);
  assert.equal(restitutionAt(SURFACES.wall, 10), SURFACES.wall.eRef);
  assert.equal(restitutionAt(SURFACES.wall, 900), SURFACES.wall.eRef);
});

test('bounceArmed: slow impacts and micro-bounces rest; walls bounce above restSpeed', () => {
  const f = SURFACES.floor;
  assert.equal(bounceArmed(f, 3, restitutionAt(f, 3), 1), false);      // below restSpeed
  assert.equal(bounceArmed(f, 20, restitutionAt(f, 20), 1), false);    // rebound under minBounceH
  assert.equal(bounceArmed(f, 40, restitutionAt(f, 40), 1), true);
  assert.equal(bounceArmed(SURFACES.wall, 10, SURFACES.wall.eRef, 0), true);  // vertical wall: nz = 0
});

test('surfaceParams falls back to wall', () => {
  assert.equal(surfaceParams('floor'), SURFACES.floor);
  assert.equal(surfaceParams('nope'), SURFACES.wall);
});
