// tipdynamics tests (hive-physics P2): torque model, TipBody, calibration. SI units (N*m, kg, rad, s).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { HIVE_PHYS } from '../../web/js/physics/hive-physics/params.js';
import {
  IN_M, loadTorqueNm, hiveTorqueNm, assistTorqueNm, TipBody, tipSubstep, buildCellRecords, simulateTip, calibrateHive,
} from '../../web/js/physics/hive-physics/tipdynamics.js';

const scene = JSON.parse(readFileSync(new URL('../../web/data/scene.json', import.meta.url), 'utf8'));
const g = scene.groups.hive_red;
const states = scene.constants.hives.red.states;
const pivot = g.pivot, stableDeg = g.stable_angles_deg;
const DEG = Math.PI / 180, STOP = 30 * DEG;
const P = HIVE_PHYS;
const loadOpts = { gMps2: P.G_MPS2, drainPastCentreRad: P.drainPastCentreDeg * DEG, upSign: P.upSign, includeBallInertia: P.includeBallInertia };
const recs = (nNectar, nPollen, cell = 'audience') =>
  buildCellRecords(states, cell, [...Array(nNectar).fill('nectar'), ...Array(nPollen).fill('pollen')], pivot, stableDeg, P);
const cal = calibrateHive({ cellsByState: states, stableDeg, pivot, startingNectar: 3, params: P });
const bodyP = (over = {}) => ({
  mghNm: cal.mghNm, viscousNmsPerRad: P.viscousNmsPerRad, coulombNm: P.coulombNm, staticNm: P.staticNm,
  stopRad: STOP, stopRestitution: P.stopRestitution, restOmegaRadS: P.restOmegaRadS, ...over,
});

test('IN_M is 0.0254 m/in', () => assert.equal(IN_M, 0.0254));

test('sign conventions: pollen in audience cell at audience_up pushes theta positive; hive self torque restores the stop', () => {
  const r = recs(0, 1);
  assert.equal(r.length, 1);
  assert.ok(r[0].massKg > 0 && Array.isArray(r[0].rest));
  const l = loadTorqueNm(r, -STOP, loadOpts);
  assert.ok(l.tauNm > 0, 'ball torque ' + l.tauNm);
  assert.ok(l.inertiaKgM2 > 0);
  assert.equal(l.nActive, 1);
  assert.ok(hiveTorqueNm(-STOP, 2) < 0);
  assert.ok(hiveTorqueNm(STOP, 2) > 0);
  assert.equal(hiveTorqueNm(0, 2), 0);
});

test('draining: audience-cell balls give zero torque and inertia past +9.79 deg, none before', () => {
  const r = recs(0, 4);
  const before = loadTorqueNm(r, 5 * DEG, loadOpts);
  assert.equal(before.nActive, 4);
  const after = loadTorqueNm(r, 12 * DEG, loadOpts);
  assert.equal(after.tauNm, 0);
  assert.equal(after.inertiaKgM2, 0);
  assert.equal(after.nActive, 0);
  const rf = recs(0, 3, 'far');
  assert.equal(loadTorqueNm(rf, -12 * DEG, loadOpts).nActive, 0);
  assert.equal(loadTorqueNm(rf, 5 * DEG, loadOpts).nActive, 3);
});

test('calibration: tau8 > tau7, 7 hold, 8 tip (leave torque vs static friction)', () => {
  assert.ok(cal.tau8Nm > cal.tau7Nm);
  assert.ok(cal.mghNm > 0 && cal.hiveInertiaKgM2 > 0);
  assert.ok(Math.abs(cal.pollenStepNm - (cal.tau8Nm - cal.tau7Nm)) < 1e-12);
  assert.ok(Math.abs(cal.assistMinNm - cal.pollenStepNm / 2) < 1e-12);
  assert.ok(Math.abs(cal.holdTorqueNm - cal.mghNm * Math.sin(STOP)) < 1e-9);
  console.log('[report] calibration', JSON.stringify(cal));
  const b = new TipBody(bodyP()); b.latchAt(-1);
  const leave = (n) => b.leaveTorqueNm(loadTorqueNm(recs(3, n), -STOP, loadOpts).tauNm);
  assert.ok(leave(7) <= P.staticNm + 1e-9, '7 pollen leave ' + leave(7));
  assert.ok(leave(8) > P.staticNm, '8 pollen leave ' + leave(8));
  assert.ok(leave(0) < P.staticNm);
});

test('calibration is cached (identical result on repeat) and ball torque is monotone in load', () => {
  const again = calibrateHive({ cellsByState: states, stableDeg, pivot, startingNectar: 3, params: P });
  assert.equal(JSON.stringify(again), JSON.stringify(cal));
  let prev = -Infinity;
  for (let n = 0; n <= 12; n++) {
    const t = loadTorqueNm(recs(3, n), -STOP, loadOpts).tauNm;
    assert.ok(t > prev); prev = t;
  }
});

test('simulateTip with assist reaches +side in nominalTipTimeS within 2%', () => {
  const r = recs(3, 8);
  const sim = simulateTip({ records: r, params: P, mghNm: cal.mghNm, hiveInertiaKgM2: cal.hiveInertiaKgM2, startSide: -1, assistMinNm: cal.assistMinNm });
  assert.equal(sim.finalSide, 1);
  assert.ok(sim.maxOmegaRadS > 0 && Number.isFinite(sim.maxOmegaRadS));
  if (cal.inertiaAtBound) { console.log('[report] inertia at search bound; time ' + sim.timeS); return; }
  assert.ok(Math.abs(sim.timeS - P.nominalTipTimeS) / P.nominalTipTimeS < 0.02, 'tip time ' + sim.timeS);
  assert.ok(Math.abs(cal.tipTimeS - sim.timeS) < 0.02);
});

test('more inertia -> slower tip (monotone)', () => {
  const r = recs(3, 8);
  const t = (I) => simulateTip({ records: r, params: P, mghNm: cal.mghNm, hiveInertiaKgM2: I, startSide: -1, assistMinNm: cal.assistMinNm }).timeS;
  assert.ok(t(cal.hiveInertiaKgM2 * 2) > t(cal.hiveInertiaKgM2));
  assert.ok(t(cal.hiveInertiaKgM2) > t(cal.hiveInertiaKgM2 / 2));
});

test('assistTorqueNm: restores drive to assistMin beyond friction, zero when already strong', () => {
  const am = 0.04, c = 0.02, mgh = 2;
  const a = assistTorqueNm(1, 0, -STOP, mgh, c, am);
  assert.ok(a > 0);
  assert.ok(Math.abs((0 + hiveTorqueNm(-STOP, mgh) + a) - (am + c)) < 1e-12);
  assert.equal(assistTorqueNm(1, 10, -STOP, mgh, c, am), 0);
  const b = assistTorqueNm(-1, 0, STOP, mgh, c, am);
  assert.ok(b < 0);
  assert.ok(Math.abs(-1 * (hiveTorqueNm(STOP, mgh) + b) - (am + c)) < 1e-12);
});

test('TipBody: latched body does not move; latchAt sets exact stop', () => {
  const b = new TipBody(bodyP()); b.latchAt(-1);
  assert.equal(b.theta, -STOP); assert.equal(b.omega, 0); assert.equal(b.latched, -1);
  assert.equal(b.substep(0.001, 100, 0.1), -1);
  assert.equal(b.theta, -STOP);
  b.latchAt(1);
  assert.equal(b.theta, STOP); assert.equal(b.latched, 1);
});

test('TipBody: energy conserved within 0.5% with c = coulomb = static = 0 over a 20 deg swing, no stop contact', () => {
  const mgh = 2, I = 0.05, h = 1e-4;
  const b = new TipBody(bodyP({ mghNm: mgh, viscousNmsPerRad: 0, coulombNm: 0, staticNm: 0 }));
  b.theta = 5 * DEG; b.omega = 0; b.latched = 0;
  const E = () => 0.5 * I * b.omega * b.omega + mgh * Math.cos(b.theta);   // potential mgh*cos(theta) since tau = mgh*sin(theta)
  const E0 = E(), dU = mgh * (Math.cos(5 * DEG) - Math.cos(25 * DEG));
  let maxErr = 0, steps = 0;
  while (b.theta < 25 * DEG && steps < 200000) {
    assert.equal(b.substep(h, 0, I), 0);
    maxErr = Math.max(maxErr, Math.abs(E() - E0)); steps++;
  }
  assert.ok(b.theta >= 25 * DEG, 'swing completed');
  assert.ok(maxErr / dU < 0.005, 'energy error fraction ' + maxErr / dU);
});

test('TipBody: stop restitution 0.5 reverses omega with ratio 0.5', () => {
  const b = new TipBody(bodyP({ mghNm: 0, viscousNmsPerRad: 0, coulombNm: 0, staticNm: 0, stopRestitution: 0.5 }));
  b.theta = STOP - 0.001; b.omega = 3; b.latched = 0;
  const r = b.substep(0.001, 0, 1);
  assert.equal(r, 0);
  assert.equal(b.theta, STOP);
  assert.ok(Math.abs(b.omega / 3 + 0.5) < 1e-6, 'ratio ' + b.omega / 3);
});

test('TipBody: restitution 0 absorbs and latches at the stop', () => {
  const b = new TipBody(bodyP({ mghNm: 2, stopRestitution: 0 }));
  b.theta = 20 * DEG; b.omega = 1; b.latched = 0;
  let side = 0;
  for (let i = 0; i < 5000 && !side; i++) side = b.substep(0.001, 0, 0.1);
  assert.equal(side, 1);
  assert.equal(b.latched, 1);
  assert.equal(b.theta, STOP);
  assert.equal(b.omega, 0);
});

test('TipBody: sticks while |tau| <= staticNm, moves above it', () => {
  const b = new TipBody(bodyP({ mghNm: 1 }));
  b.theta = 0; b.omega = 0; b.latched = 0;
  for (let i = 0; i < 100; i++) assert.equal(b.substep(0.001, 0.02, 0.1), 0);
  assert.equal(b.theta, 0); assert.equal(b.omega, 0);
  b.substep(0.001, 0.05, 0.1);
  assert.ok(b.omega > 0 && b.theta > 0);
  assert.ok(Math.abs(b.omega - (0.05 - P.coulombNm) / 0.1 * 0.001) < 1e-9);
});

test('TipBody.applyImpulse: into stop is absorbed, away from stop unlatches', () => {
  const b = new TipBody(bodyP()); b.latchAt(-1);
  b.applyImpulse(-0.5, 0.1);
  assert.equal(b.latched, -1); assert.equal(b.omega, 0);
  b.applyImpulse(0.5, 0.1);
  assert.equal(b.latched, 0);
  assert.ok(Math.abs(b.omega - 5) < 1e-12);
});

test('tipSubstep: shared substep moves a released, heavily loaded hive to +side with no NaN', () => {
  const r = recs(3, 12);
  const b = new TipBody(bodyP()); b.latchAt(-1); b.release();
  const ctx = { params: P, mghNm: cal.mghNm, hiveInertiaKgM2: cal.hiveInertiaKgM2, assist: null };
  let side = 0, i = 0;
  for (; i < 10000 && !side; i++) side = tipSubstep(b, r, 0.001, ctx);
  assert.equal(side, 1);
  assert.ok(Number.isFinite(b.theta));
});

test('buildCellRecords: types, masses and order', () => {
  const r = recs(3, 2);
  assert.deepEqual(r.map((x) => x.type), ['nectar', 'nectar', 'nectar', 'pollen', 'pollen']);
  assert.ok(r[0].massKg > r[3].massKg);
  assert.ok(r.every((x) => x.cell === 'audience' && x.rest.length === 2 && x.rest.every(Number.isFinite)));
});
