import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { makeParams, wheelGeometry, forceFactor, G } from '../../web/js/physics/weight-transfer/params.js';
import { computeCG } from '../../web/js/physics/weight-transfer/massmodel.js';
import { mecanumWheelForces, tractionScale, limitBodyVelocity, loadAwareLimits } from '../../web/js/physics/weight-transfer/traction.js';

const spec = JSON.parse(fs.readFileSync(new URL('../../web/data/robot_spec.json', import.meta.url), 'utf8'));
const P = makeParams({}, spec);
const CG = computeCG(P);
const GEO = wheelGeometry(P);
const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) < e, `${a} !~ ${b} (tol ${e})`);

test('pure forward demand: equal force on all four wheels', () => {
  const f = mecanumWheelForces(P, 100, 0, 0);
  for (const v of f) near(v, 25, 1e-9);
});

test('pure strafe demand: sign pattern [-,+,+,-]', () => {
  const f = mecanumWheelForces(P, 0, 100, 0);
  assert.deepEqual(f.map(Math.sign), [-1, 1, 1, -1]);
  for (const v of f) near(Math.abs(v), 25, 1e-9);
});

test('pure rotation demand: sign pattern [-,+,-,+], force sums to zero', () => {
  const f = mecanumWheelForces(P, 0, 0, 50);
  assert.deepEqual(f.map(Math.sign), [-1, 1, -1, 1]);
  near(f.reduce((s, v) => s + v, 0), 0, 1e-9);
});

test('braking scale matches the closed form within 1e-3', () => {
  const ff = forceFactor(P), mu = P.traction.mu * P.traction.muScale;
  const u0 = CG.x - GEO.xc;
  const w0 = CG.y - GEO.yc;
  // worst rear wheel static share n0; load falls as h*A/(4*a*G): ff*A/4 <= mu*G*(n0 - h*A/(4*a*G))
  const n0 = 0.25 - u0 / (4 * GEO.a) - Math.abs(w0) / (4 * GEO.b);
  const A = (4 * mu * G * n0) / (ff + (mu * CG.z) / GEO.a);
  const dt = 1 / 60, want = 224;
  const r = tractionScale(P, CG, { ax: -want, ay: 0, alpha: 0 }, { ax: 0, ay: 0, alpha: 0 });
  near(r.scale, A / want, 1e-3);
  assert.ok(r.slip[2] && r.slip[3] && !r.slip[0]);
  const l = limitBodyVelocity(P, CG, [0, 0, 0], [-want * dt, 0, 0], dt);
  near(l.scale, A / want, 1e-3);
  near(l.bodyV[0], -A * dt, 1e-3 * want * dt);
});

test('commands within grip pass unchanged; dt<=0 returns prev', () => {
  const r = limitBodyVelocity(P, CG, [0, 0, 0], [1, 0, 0], 1 / 60);
  assert.equal(r.scale, 1); assert.equal(r.sliding, false);
  near(r.bodyV[0], 1, 1e-12);
  const z = limitBodyVelocity(P, CG, [3, 1, 0.5], [9, 9, 9], 0);
  assert.deepEqual(z.bodyV, [3, 1, 0.5]); assert.equal(z.scale, 1);
});

test('loadAwareLimits: brake ~140, forward ~163, lateral ~155, brake < forward', () => {
  const L = loadAwareLimits(P, CG);
  near(L.maxBrake, 140, 5); near(L.maxAccelFwd, 163, 5);
  // lateral: the spec quoted ~167 ignoring the static CG offset; with u0 = 0.385 in the BL wheel limits at ~155
  const ff = forceFactor(P), mu = P.traction.mu;
  const n0 = 0.25 - (CG.x - GEO.xc) / (4 * GEO.a) + (CG.y - GEO.yc) / (4 * GEO.b); // BL: left and rear
  near(L.maxAccelLat, (4 * mu * G * n0) / (ff + (mu * CG.z) / GEO.b), 1.0);
  near(L.maxAccelLat, 155, 5);
  assert.ok(L.maxBrake < L.maxAccelFwd);
  assert.ok(L.tipBrake > L.maxBrake);
});

test('muScale=0.5 roughly halves the limits', () => {
  const L1 = loadAwareLimits(P, CG);
  const L2 = loadAwareLimits(makeParams({ traction: { muScale: 0.5 } }, spec), CG);
  for (const k of ['maxBrake', 'maxAccelFwd', 'maxAccelLat']) {
    const r = L2[k] / L1[k];
    assert.ok(r > 0.4 && r < 0.7, `${k} ratio ${r}`);
  }
});

test('a lifted wheel gives scale 0 for a drive command', () => {
  const r = tractionScale(P, CG, { ax: 100, ay: 0, alpha: 0 }, { ax: 0, ay: 0, alpha: 0 }, { lifted: [false, false, true, false] });
  assert.equal(r.scale, 0);
});
