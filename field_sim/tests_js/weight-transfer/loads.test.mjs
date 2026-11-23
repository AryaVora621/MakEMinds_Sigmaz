import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { makeParams, wheelGeometry, G } from '../../web/js/physics/weight-transfer/params.js';
import { computeCG } from '../../web/js/physics/weight-transfer/massmodel.js';
import { wheelLoads, tipAccels, bodyAccel, zmp } from '../../web/js/physics/weight-transfer/loads.js';

const spec = JSON.parse(fs.readFileSync(new URL('../../web/data/robot_spec.json', import.meta.url), 'utf8'));
const P = makeParams({}, spec);
const CG = computeCG(P);
const GEO = wheelGeometry(P);
const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) < e, `${a} !~ ${b} (tol ${e})`);
const sum = (a) => a.reduce((s, v) => s + v, 0);

test('bodyAccel: zero for dt<=0, midpoint centripetal terms', () => {
  assert.deepEqual(bodyAccel([1, 2, 3], [4, 5, 6], 0), { ax: 0, ay: 0, alpha: 0 });
  const r = bodyAccel([0, 0, 0], [10, 0, 1], 0.1);
  near(r.ax, 100 - 0 * 0.5, 1e-9);
  near(r.ay, 0 + 5 * 0.5, 1e-9);
  near(r.alpha, 10, 1e-9);
});

test('n sums to 1 across modes', () => {
  const tb = tipAccels(P, CG).tipBrake;
  for (const [ax, ay] of [[0, 0], [-224, 0], [100, 80], [-300, 200], [-1.2 * tb, 0], [0, 900], [-224, -150]]) {
    const l = wheelLoads(P, CG, ax, ay);
    near(sum(l.n), 1, 1e-9);
    assert.equal(l.n.length, 4);
    assert.ok(l.n.every((v) => v >= -1e-9), JSON.stringify(l));
  }
});

test('static: front > rear when cg.x > 0, lbf sums to weight', () => {
  assert.ok(CG.x > 0);
  const l = wheelLoads(P, CG, 0, 0);
  assert.equal(l.mode, 'four');
  assert.ok(l.n[0] > l.n[2] && l.n[1] > l.n[3]);
  near(l.n[0], 0.269, 0.01); near(l.n[2], 0.231, 0.01);
  near(sum(l.lbf), CG.massLb, 1e-6);
});

test('braking at -224 in/s^2: front ~0.433, rear ~0.067, four mode', () => {
  const l = wheelLoads(P, CG, -224, 0);
  assert.equal(l.mode, 'four');
  near(l.n[0], 0.433, 0.01); near(l.n[1], 0.433, 0.01);
  near(l.n[2], 0.067, 0.01); near(l.n[3], 0.067, 0.01);
});

test('mirrored cg (y -> -y) gives mirrored loads', () => {
  const cgA = { ...CG, y: 0.8 }, cgB = { ...CG, y: -0.8 };
  const A = wheelLoads(P, cgA, -100, 60), B = wheelLoads(P, cgB, -100, -60);
  near(A.n[0], B.n[1], 1e-9); near(A.n[1], B.n[0], 1e-9);
  near(A.n[2], B.n[3], 1e-9); near(A.n[3], B.n[2], 1e-9);
});

test('three mode: combined ax/ay corner lifts the diagonally opposite wheel', () => {
  const t = tipAccels(P, CG);
  // push ZMP toward the front-left corner: decel (ax<0) and accel toward +y... ZMP = cg - h*a/g
  // ax = -0.9*tipBrake gives u near +0.9a; ay = -0.9*tipRight puts w near +0.9b? use sign check below.
  const l = wheelLoads(P, CG, -0.8 * t.tipBrake, -0.8 * t.tipRight);
  assert.equal(l.mode, 'three');
  assert.ok(l.zmp.x - GEO.xc > 0 && l.zmp.y - GEO.yc > 0);
  near(l.n[3], 0, 1e-9); // BR is diagonal from FL-side ZMP
  assert.ok(l.contacts[0] && l.contacts[1] && l.contacts[2] && !l.contacts[3]);
  near(sum(l.n), 1, 1e-9);
});

test('tipping about the front axis for ax = -1.2*tipBrake', () => {
  const t = tipAccels(P, CG);
  const l = wheelLoads(P, CG, -1.2 * t.tipBrake, 0);
  assert.equal(l.mode, 'tipping');
  assert.equal(l.axis, 'front');
  near(l.n[2], 0); near(l.n[3], 0);
  near(l.n[0] + l.n[1], 1, 1e-9);
  assert.ok(l.margin < 0);
});

test('tipBrake and tipFwd match closed forms and defaults', () => {
  const t = tipAccels(P, CG);
  near(t.tipBrake, (G * (GEO.a - (CG.x - GEO.xc))) / CG.z, 1e-6);
  near(t.tipFwd, (G * (GEO.a + (CG.x - GEO.xc))) / CG.z, 1e-6);
  near(t.tipBrake, 314.7, 20); near(t.tipFwd, 383, 25);
  near(t.tipLeft + t.tipRight, (2 * G * GEO.b) / CG.z, 1e-6);
});

test('at exactly the tip accel the ZMP sits on the support edge', () => {
  const t = tipAccels(P, CG);
  const z = zmp(CG, -t.tipBrake, 0);
  near(z.x, GEO.xc + GEO.a, 1e-6);
});
