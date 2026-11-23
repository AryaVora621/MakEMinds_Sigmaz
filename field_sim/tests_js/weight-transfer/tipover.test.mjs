import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { makeParams } from '../../web/js/physics/weight-transfer/params.js';
import { computeCG } from '../../web/js/physics/weight-transfer/massmodel.js';
import { wheelLoads, tipAccels } from '../../web/js/physics/weight-transfer/loads.js';
import { TipState, impactTipSpeed, edgeGeometry } from '../../web/js/physics/weight-transfer/tipover.js';

const spec = JSON.parse(fs.readFileSync(new URL('../../web/data/robot_spec.json', import.meta.url), 'utf8'));
const P = makeParams({}, spec);
const CG = computeCG(P);
const T = tipAccels(P, CG);
const DT = 1 / 120;

function advance(ts, seconds, ax, ay = 0) {
  for (let t = 0; t < seconds; t += DT) {
    ts.step(DT, { cg: CG, loads: wheelLoads(P, CG, ax, ay), ax, ay });
  }
  return ts.snapshot;
}

test('edgeGeometry front: R, alpha, pivot', () => {
  const e = edgeGeometry(P, CG, 'front');
  assert.ok(Math.abs(e.R - Math.hypot(e.d, e.h)) < 1e-9);
  assert.ok(Math.abs(e.alpha - Math.atan2(e.d, e.h)) < 1e-9);
  assert.ok(e.Ie > 0);
  assert.ok(Math.abs(e.pivot.x - 5.04) < 1e-6 && e.pivot.z === 0);
});

test('impactTipSpeed front finite and > 60 in/s', () => {
  const v = impactTipSpeed(P, CG, 'front');
  assert.ok(Number.isFinite(v) && v > 60, `vCrit=${v}`);
});

test('fresh state is grounded with no lifted wheels', () => {
  const s = new TipState(P).snapshot;
  assert.equal(s.mode, 'grounded'); assert.equal(s.disabled, false);
  assert.deepEqual(s.lifted, [false, false, false, false]);
});

test('impact at 0.9*vCrit returns to grounded within 3 s', () => {
  const ts = new TipState(P);
  const v = impactTipSpeed(P, CG, 'front');
  const r = ts.applyImpact({ x: 0.9 * v, y: 0 }, CG);
  assert.equal(r.edge, 'front'); assert.ok(r.started);
  assert.equal(ts.snapshot.mode, 'rocking');
  assert.deepEqual(ts.snapshot.lifted, [false, false, true, true]);
  const s = advance(ts, 3, 0);
  assert.equal(s.mode, 'grounded'); assert.equal(s.disabled, false);
});

test('impact at 1.1*vCrit falls over and disables', () => {
  const ts = new TipState(P);
  const v = impactTipSpeed(P, CG, 'front');
  ts.applyImpact({ x: 1.1 * v, y: 0 }, CG);
  const s = advance(ts, 3, 0);
  assert.equal(s.mode, 'fallen'); assert.equal(s.disabled, true);
});

test('quasi-static: ax = -1.05*tipBrake falls, -0.95*tipBrake stays grounded', () => {
  const a = new TipState(P);
  assert.equal(advance(a, 2, -1.05 * T.tipBrake).mode, 'fallen');
  const b = new TipState(P);
  assert.equal(advance(b, 2, -0.95 * T.tipBrake).mode, 'grounded');
});

test('reset clears a fallen state', () => {
  const ts = new TipState(P);
  advance(ts, 2, -1.2 * T.tipBrake);
  assert.equal(ts.snapshot.mode, 'fallen');
  ts.reset();
  const s = ts.snapshot;
  assert.equal(s.mode, 'grounded'); assert.equal(s.disabled, false); assert.equal(s.angle, 0);
});

test('tiny impact does not start rocking', () => {
  const ts = new TipState(P);
  const r = ts.applyImpact({ x: 0.5, y: 0 }, CG);
  assert.equal(ts.snapshot.mode, 'grounded');
  assert.equal(r.started, false);
});
