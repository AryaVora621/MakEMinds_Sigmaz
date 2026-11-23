import test from 'node:test';
import assert from 'node:assert/strict';
import { BB_PARAMS, makeParams, G_IN } from '../../web/js/physics/ball-ball-collisions/params.js';
import { ensureBody, massOf, inertiaOf, restitutionFor } from '../../web/js/physics/ball-ball-collisions/bodies.js';
import { buildContacts, ContactSolver, ContactCache } from '../../web/js/physics/ball-ball-collisions/contact_solver.js';
import { BallBallSystem } from '../../web/js/physics/ball-ball-collisions/system.js';
import { mkBall, runWorld } from './harness.mjs';

const H = 1 / 240; // s, one substep at dt 1/60 with 4 substeps

function solveOnce(params, a, b, h = H) {
  ensureBody(a, params); ensureBody(b, params);
  const pool = [];
  const n = buildContacts([a, b], 1, h, params, pool);
  const solver = new ContactSolver(params);
  solver.solve(pool, n, h, new ContactCache(), 1);
  return { pool, n, solver };
}

test('1D head-on elastic collision matches analytic formulas and conserves momentum', () => {
  const p = makeParams({ e0: 1, eSpeedExp: 0, eMax: 1, mu: 0 });
  const A = mkBall('pollen', 0, 0, 0, 100, 0, 0);
  const B = mkBall('nectar', A.r + 1.7995, 0, 0, 0, 0, 0); // exactly touching
  const mA = massOf('pollen', p), mB = massOf('nectar', p);
  const pBefore = mA * A.vx + mB * B.vx;
  const vA = ((mA - mB) * 100) / (mA + mB), vB = (2 * mA * 100) / (mA + mB);
  solveOnce(p, A, B);
  assert.ok(Math.abs(A.vx - vA) <= 1e-6 * Math.abs(vA), `vA ${A.vx} vs ${vA}`);
  assert.ok(Math.abs(B.vx - vB) <= 1e-6 * Math.abs(vB), `vB ${B.vx} vs ${vB}`);
  assert.ok(Math.abs(mA * A.vx + mB * B.vx - pBefore) <= 1e-9 * Math.abs(pBefore));
});

test('constant e = 0.5 gives separation/approach ratio 0.5', () => {
  const p = makeParams({ e0: 0.5, eSpeedExp: 0, mu: 0 });
  const A = mkBall('pollen', 0, 0, 0, 100);
  const B = mkBall('pollen', 2 * 1.391, 0, 0);
  solveOnce(p, A, B);
  const ratio = (B.vx - A.vx) / 100;
  assert.ok(Math.abs(ratio - 0.5) <= 0.02, `ratio ${ratio}`);
});

test('approach below the restitution threshold does not bounce', () => {
  const A = mkBall('pollen', 0, 0, 0, 5); // 5 in/s < 7.874 in/s
  const B = mkBall('pollen', 2 * 1.391, 0, 0);
  solveOnce(BB_PARAMS, A, B);
  const sep = B.vx - A.vx;
  assert.ok(sep >= -1e-6 && sep <= 1e-3, `separation speed ${sep}`);
});

test('oblique impact with friction conserves linear and angular momentum and makes spin', () => {
  const A = mkBall('pollen', 0, 0, 0, 100, 0, 0);
  const th = Math.PI / 6, rr = 1.391 + 1.7995;
  const B = mkBall('nectar', rr * Math.cos(th), rr * Math.sin(th), 0, 0, 0, 0);
  ensureBody(A); ensureBody(B);
  const bodies = [A, B];
  const moment = () => {
    const L = [0, 0, 0], P = [0, 0, 0];
    for (const b of bodies) {
      const m = massOf(b.kind), I = inertiaOf(b.kind, b.r);
      P[0] += m * b.vx; P[1] += m * b.vy; P[2] += m * b.vz;
      L[0] += m * (b.y * b.vz - b.z * b.vy) + I * b.wx;
      L[1] += m * (b.z * b.vx - b.x * b.vz) + I * b.wy;
      L[2] += m * (b.x * b.vy - b.y * b.vx) + I * b.wz;
    }
    return { L, P };
  };
  const before = moment();
  // Positions stay at their pre-solve values (s = 0, so the position pass does nothing), which makes L about the origin exact.
  solveOnce(BB_PARAMS, A, B);
  const after = moment();
  const mag = (v) => Math.hypot(...v);
  const dP = mag(after.P.map((x, i) => x - before.P[i])), dL = mag(after.L.map((x, i) => x - before.L[i]));
  assert.ok(dP <= 1e-6 * mag(before.P), `dP ${dP}`);
  assert.ok(dL <= 1e-6 * Math.max(mag(before.L), mag(before.P)), `dL ${dL}`);
  assert.ok(Math.hypot(A.wx, A.wy, A.wz) > 1e-6 || Math.hypot(B.wx, B.wy, B.wz) > 1e-6, 'expected spin');
});

test('restitutionFor is monotone non-increasing above threshold and clamped', () => {
  const p = BB_PARAMS;
  assert.equal(restitutionFor(p.restitutionThreshold * 0.5, p), 0);
  let prev = Infinity;
  for (let v = p.restitutionThreshold; v <= 5000; v *= 1.05) {
    const e = restitutionFor(v, p);
    assert.ok(e >= p.eMin - 1e-12 && e <= p.eMax + 1e-12, `e(${v})=${e}`);
    assert.ok(e <= prev + 1e-12, `not monotone at ${v}`);
    prev = e;
  }
});

test('warm start: resting ball contact impulse matches m*g*h within 5%', () => {
  const sys = new BallBallSystem();
  const base = mkBall('nectar', 0, 0, 0, 0, 0, 0, { pinned: true });
  const top = mkBall('pollen', 0, 0, base.r + 1.391);
  runWorld([base, top], { system: sys, seconds: 0.3, floor: false });
  let ln = 0;
  for (let i = 0; i < sys.contactCount; i++) {
    const c = sys.contacts[i];
    if (c.a === top || c.b === top) ln = c.ln;
  }
  const expect = massOf('pollen') * G_IN * H;
  assert.ok(Math.abs(ln - expect) <= 0.05 * expect, `ln ${ln} vs ${expect}`);
});

test('position pass never changes velocities', () => {
  const A = mkBall('pollen', 0, 0, 0, 30, 4, -2);
  const B = mkBall('nectar', 2.5, 0.3, 0.2, -10, 0, 1); // overlapping by ~0.7 in
  ensureBody(A); ensureBody(B);
  const pool = [];
  const n = buildContacts([A, B], 1, H, BB_PARAMS, pool);
  const solver = new ContactSolver(BB_PARAMS);
  solver.prepare(pool, n, H, new ContactCache(), 1);
  solver.warmStart();
  solver.solveVelocity(BB_PARAMS.velocityIterations);
  const snap = [A, B].map((b) => [b.vx, b.vy, b.vz, b.wx, b.wy, b.wz]);
  const x0 = A.x;
  solver.solvePositions(BB_PARAMS.positionIterations);
  [A, B].forEach((b, i) => assert.deepEqual([b.vx, b.vy, b.vz, b.wx, b.wy, b.wz], snap[i]));
  assert.notEqual(A.x, x0, 'positions should have been corrected');
});
