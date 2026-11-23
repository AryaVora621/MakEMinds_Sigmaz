// solver.test.mjs - drag/spin-aware aim solver. Lengths in inches, speeds in in/s, angles in rad.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AERO } from '../../web/js/physics/ball-aerodynamics/params.js';
import { buildAeroModel } from '../../web/js/physics/ball-aerodynamics/air.js';
import { solveSpeed, solveElevation, dragFreeSpeed, dragFreeMiss, launchVector } from '../../web/js/physics/ball-aerodynamics/solver.js';
import { crossHeight } from '../../web/js/physics/ball-aerodynamics/trajectory.js';

const phys = buildAeroModel(AERO, AERO.kinds.pollen, { model: 'physical', integrator: 'rk4' });
const vac = { ...phys, kDrag: 0, base: 0, lift: { ...phys.lift, enabled: false } };
const p0 = [0, 0, 12], target = [120, 0, 40]; // in
const TOL = AERO.solver.tol; // in
const deg = (d) => d * Math.PI / 180;

function missAt(speed, elevRad, model) {
  const c = crossHeight({ pos: p0, vel: launchVector(speed, 0, elevRad) }, model, target[2], { descending: true });
  assert.ok(c, 'no crossing');
  return Math.hypot(c.pos[0] - p0[0], c.pos[1] - p0[1]) - Math.hypot(target[0] - p0[0], target[1] - p0[1]);
}

test('no drag: solveSpeed matches dragFreeSpeed within 0.1 in/s', () => {
  const e = deg(50);
  const r = solveSpeed({ p0, target, elevRad: e, model: vac });
  assert.ok(r.ok, JSON.stringify(r));
  const v = dragFreeSpeed(p0, target, e, vac.g);
  console.log(`no-drag solve: ${r.speed} in/s vs closed form ${v} in/s`);
  assert.ok(Math.abs(r.speed - v) < 0.1);
});

test('physical drag: solved speed > drag-free and lands within tol', () => {
  const e = deg(50);
  const r = solveSpeed({ p0, target, elevRad: e, model: phys });
  assert.ok(r.ok, JSON.stringify(r));
  const v0 = dragFreeSpeed(p0, target, e, phys.g);
  console.log(`physical solve: ${r.speed} in/s vs drag-free ${v0} in/s`);
  assert.ok(r.speed > v0);
  const m = missAt(r.speed, e, phys);
  assert.ok(Math.abs(m) <= TOL, `miss ${m}`);
});

test('dragFreeMiss > 1 in for a 120 in shot under physical drag', () => {
  const r = dragFreeMiss({ p0, target, elevRad: deg(50), model: phys });
  console.log(`drag-free miss: ${r.missIn} in at speed ${r.speed} in/s`);
  assert.ok(r.missIn > 1);
});

test('solveElevation returns low < high, both hit within tol', () => {
  const sp = solveSpeed({ p0, target, elevRad: deg(50), model: phys });
  const speed = sp.speed * 1.15; // in/s, above the minimum-speed solution so two branches exist
  const r = solveElevation({ p0, target, speed, model: phys });
  assert.ok(r.ok, JSON.stringify(r));
  assert.ok(r.low && r.high);
  assert.ok(r.low.elevRad < r.high.elevRad);
  for (const br of [r.low, r.high]) {
    const m = missAt(speed, br.elevRad, phys);
    assert.ok(Math.abs(m) <= TOL, `miss ${m}`);
  }
});

test('unreachable target gives ok:false', () => {
  const r = solveSpeed({ p0, target, elevRad: deg(50), model: phys, vMin: 50, vMax: 100 });
  assert.equal(r.ok, false);
  assert.equal(r.reason, 'unreachable');
});
