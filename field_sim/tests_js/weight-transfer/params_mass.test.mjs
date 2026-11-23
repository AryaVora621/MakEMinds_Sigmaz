import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { makeParams, wheelGeometry, forceFactor, G, DEFAULT_PARAMS } from '../../web/js/physics/weight-transfer/params.js';
import { computeCG, itemPositions } from '../../web/js/physics/weight-transfer/massmodel.js';

const spec = JSON.parse(fs.readFileSync(new URL('../../web/data/robot_spec.json', import.meta.url), 'utf8'));
const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) < e, `${a} !~ ${b} (tol ${e})`);

test('G is standard gravity in in/s^2', () => near(G, 9.80665 / 0.0254, 1e-3));

test('item masses sum to totalLb and cg.z in 5.0-6.5 in', () => {
  const p = makeParams({}, spec);
  const sum = p.mass.items.reduce((s, i) => s + i.massLb, 0);
  near(sum, p.mass.totalLb, 1e-9);
  const cg = computeCG(p);
  near(cg.massLb, p.mass.totalLb, 1e-9);
  near(p.mass.totalLb, 31.4, 1e-9);
  assert.ok(cg.z >= 5.0 && cg.z <= 6.5, `cg.z=${cg.z}`);
  assert.ok(cg.Ixx > 0 && cg.Iyy > 0 && cg.Izz > 0);
});

test('makeParams does not mutate DEFAULT_PARAMS and overrides deep-merge', () => {
  const before = JSON.stringify(DEFAULT_PARAMS);
  const p = makeParams({ traction: { mu: 0.6 } }, spec);
  near(p.traction.mu, 0.6);
  assert.ok(p.traction.rollerAngleDeg === 45);
  assert.equal(JSON.stringify(DEFAULT_PARAMS), before);
});

test('wheelGeometry from spec: a=5.04 b=6.35, forceFactor sqrt2', () => {
  const p = makeParams({}, spec);
  const g = wheelGeometry(p);
  near(g.a, 5.04, 1e-6); near(g.b, 6.35, 1e-6); near(g.k, 11.39, 1e-6);
  near(g.xc, 0, 1e-9); near(g.yc, 0, 1e-9);
  assert.deepEqual(g.wheels.map((w) => w.name), ['FL', 'FR', 'BL', 'BR']);
  near(forceFactor(p), Math.SQRT2, 1e-9);
});

test('turret rotation moves cg x/y but not z', () => {
  const p = makeParams({}, spec);
  const a = computeCG(p, { turretRad: 0 });
  const b = computeCG(p, { turretRad: Math.PI / 2 });
  assert.ok(Math.hypot(a.x - b.x, a.y - b.y) > 0.1);
  near(a.z, b.z, 1e-9);
  near(a.massLb, b.massLb, 1e-9);
});

test('held balls add mass and shift cg', () => {
  const p = makeParams({}, spec);
  const a = computeCG(p), b = computeCG(p, { ballsHeld: 3 });
  near(b.massLb - a.massLb, 3 * p.mass.ballMassLb, 1e-9);
  assert.equal(itemPositions(p, { ballsHeld: 9 }).length, itemPositions(p, { ballsHeld: 4 }).length);
});

test('intake deploy moves cg.x forward by about intakeMass*travel/M', () => {
  const p = makeParams({}, spec);
  const a = computeCG(p), b = computeCG(p, { intakeDeployed: 1 });
  near(b.x - a.x, (3.0 * 2.7) / 31.4, 0.01);
  near(b.z, a.z, 1e-9);
});
