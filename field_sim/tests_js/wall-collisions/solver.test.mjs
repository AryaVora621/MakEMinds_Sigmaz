import test from 'node:test';
import assert from 'node:assert/strict';
import { makeParams } from '../../web/js/physics/wall-collisions/params.js';
import { bodyMass } from '../../web/js/physics/wall-collisions/geometry.js';
import {
  softCoefficients, prepareContacts, warmStart, solveContacts, applyRestitution, storeImpulses, integratePositions, wrapAngle
} from '../../web/js/physics/wall-collisions/solver.js';

const H = 1 / 240; // s, substep
const near = (a, b, t) => Math.abs(a - b) < t;

function setup({ e = 0, mu = 0.62, v = [60, 0], om = 0, rLocal = [9, 0], s = 0 } = {}) {
  const P = makeParams({ materials: { default: { e, mu } } });
  const m = bodyMass(P);
  const body = { x: 0, y: 0, h: 0, vx: v[0], vy: v[1], om, invM: m.invM, invI: m.invI };
  // wall on +x side: normal into robot is (-1, 0)
  const man = [{ normal: [-1, 0], key: 'w', kind: 'test_kind', points: [{ p: [rLocal[0], rLocal[1]], s, id: 0, rLocal }] }];
  return { P, body, man, m };
}

test('softCoefficients(30, 10, 1/240) matches spec check values', () => {
  const c = softCoefficients(30, 10, 1 / 240);
  assert.ok(near(c.biasRate, 9.06866, 1e-5), `biasRate ${c.biasRate}`);
  assert.ok(near(c.massScale, 0.942279, 1e-5), `massScale ${c.massScale}`);
  assert.ok(near(c.impulseScale, 0.057721, 1e-5), `impulseScale ${c.impulseScale}`);
});

test('wrapAngle into (-pi, pi]', () => {
  assert.ok(near(wrapAngle(3 * Math.PI / 2), -Math.PI / 2, 1e-12));
  assert.ok(near(wrapAngle(-3 * Math.PI / 2), Math.PI / 2, 1e-12));
  assert.ok(near(wrapAngle(0.5), 0.5, 1e-12));
});

test('static point contact with e = 0 drives vn to 0', () => {
  const { P, body, man } = setup({ e: 0 });
  const cons = prepareContacts(body, man, P, H, new Map());
  for (let i = 0; i < 4; i++) solveContacts(body, cons, H, false, P);
  applyRestitution(body, cons, P);
  assert.ok(Math.abs(body.vx) < 1e-6, `vx=${body.vx}`);
});

test('e = 0.5 above threshold bounces: vn\' = -e * relVel', () => {
  const { P, body, man } = setup({ e: 0.5, v: [60, 0] });
  const cons = prepareContacts(body, man, P, H, new Map());
  const relVel = cons[0].points[0].relVel; // n.v = -60 in/s
  assert.ok(near(relVel, -60, 1e-9));
  for (let i = 0; i < 4; i++) solveContacts(body, cons, H, false, P);
  applyRestitution(body, cons, P);
  const vn = -body.vx; // n = (-1,0)
  assert.ok(near(vn, -0.5 * relVel, 0.5), `vn'=${vn}, expected ~30`);
});

test('below the restitution threshold there is no bounce', () => {
  const { P, body, man } = setup({ e: 0.5, v: [P_THRESH_BELOW(), 0] });
  const cons = prepareContacts(body, man, P, H, new Map());
  for (let i = 0; i < 4; i++) solveContacts(body, cons, H, false, P);
  applyRestitution(body, cons, P);
  assert.ok(Math.abs(body.vx) < 1e-6, `vx=${body.vx}`);
});
function P_THRESH_BELOW() { return 5; } // in/s, below restitution.threshold = 10 in/s

test('friction clamp |jt| <= mu*jn', () => {
  const { P, body, man } = setup({ e: 0, mu: 0.62, v: [10, 80] });
  const cons = prepareContacts(body, man, P, H, new Map());
  for (let i = 0; i < 6; i++) {
    solveContacts(body, cons, H, false, P);
    for (const pt of cons[0].points) assert.ok(Math.abs(pt.jt) <= cons[0].mu * pt.jn + 1e-9, `jt=${pt.jt} jn=${pt.jn}`);
  }
  // sliding is slowed but not stopped: friction impulse is bounded by mu * normal impulse
  assert.ok(body.vy < 80 && body.vy > 0);
});

test('mu = 0 leaves tangential velocity untouched', () => {
  const { P, body, man } = setup({ e: 0, mu: 0, v: [10, 80] });
  const cons = prepareContacts(body, man, P, H, new Map());
  for (let i = 0; i < 4; i++) solveContacts(body, cons, H, false, P);
  assert.ok(near(body.vy, 80, 1e-9));
});

test('off-centre point: delta om = invI * (r x Pv)', () => {
  const { P, body, man, m } = setup({ e: 0, mu: 0, v: [20, 0], rLocal: [9, 5] });
  const cons = prepareContacts(body, man, P, H, new Map());
  solveContacts(body, cons, H, false, P);
  const pt = cons[0].points[0];
  assert.ok(pt.jn > 0);
  const Pvx = -pt.jn, Pvy = 0; // dJn * n, n = (-1, 0)
  const expected = m.invI * (9 * Pvy - 5 * Pvx);
  assert.ok(near(body.om, expected, 1e-9), `om=${body.om} expected=${expected}`);
  assert.ok(near(body.vx, 20 + m.invM * Pvx, 1e-9));
  assert.ok(body.om > 0);
});

test('speculative contact (s > 0) does not stop motion that cannot reach the wall', () => {
  const { P, body, man } = setup({ e: 0, v: [10, 0], s: 5 }); // 5 in gap, 10 in/s * (1/240 s) << 5 in
  const cons = prepareContacts(body, man, P, H, new Map());
  solveContacts(body, cons, H, true, P);
  assert.ok(near(body.vx, 10, 1e-9), `vx=${body.vx}`);
});

test('warm start + storeImpulses round-trip, scaled by hSub ratio', () => {
  const { P, body, man } = setup({ e: 0, v: [60, 0] });
  const cache = new Map();
  let cons = prepareContacts(body, man, P, H, cache);
  solveContacts(body, cons, H, false, P);
  const jn = cons[0].points[0].jn;
  storeImpulses(cons, cache, H);
  assert.equal(cache.size, 1);
  assert.ok(near(cache.get('w#0').jn, jn, 1e-12));
  const b2 = { ...body, vx: 0 };
  cons = prepareContacts(b2, man, P, H / 2, cache);
  assert.ok(near(cons[0].points[0].jn, jn / 2, 1e-12));
  warmStart(b2, cons);
  assert.ok(b2.vx < 0, 'warm start pushed the body away (n = -x)');
});

test('integratePositions advances and wraps heading', () => {
  const b = { x: 0, y: 0, h: Math.PI - 0.001, vx: 120, vy: -60, om: 1, invM: 1, invI: 1 };
  integratePositions(b, 0.01);
  assert.ok(near(b.x, 1.2, 1e-12) && near(b.y, -0.6, 1e-12));
  assert.ok(b.h <= Math.PI && b.h > -Math.PI);
  assert.ok(near(b.h, -Math.PI + 0.009, 1e-9));
});
