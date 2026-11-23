// integrator.test.mjs - RK4 accuracy vs closed-form vertical flight, order of convergence, legacy-vs-RK4 landing range.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AERO } from '../../web/js/physics/ball-aerodynamics/params.js';
import { buildAeroModel } from '../../web/js/physics/ball-aerodynamics/air.js';
import { stepRK4, stepLegacy, clampSpeed, substepCount } from '../../web/js/physics/ball-aerodynamics/integrator.js';
import { apexHeight, apexTime, risingState, fallingState } from '../../web/js/physics/ball-aerodynamics/analytic.js';

const phys = buildAeroModel(AERO, AERO.kinds.pollen, { model: 'physical', integrator: 'rk4' });
const k = phys.kDrag, g = phys.g; // 1/in, in/s^2

function run(step, s, T, N, model) {
  const h = T / N;
  for (let i = 0; i < N; i++) step(s, h, model, true);
  return s;
}

test('RK4 vertical apex height at h~0.004 s within 1e-6 relative', () => {
  const v0 = 500; // in/s
  const T = apexTime(v0, g, k);
  const N = Math.ceil(T / 0.004);
  const s = run(stepRK4, { x: 0, y: 0, z: 0, vx: 0, vy: 0, vz: v0 }, T, N, phys);
  const want = apexHeight(v0, g, k);
  const rel = Math.abs(s.z - want) / want;
  console.log(`apex height: RK4 ${s.z} in, analytic ${want} in, rel err ${rel.toExponential(2)}, h = ${(T / N).toFixed(5)} s`);
  assert.ok(rel < 1e-6, `rel ${rel}`);
});

test('RK4 fall from rest for 1 s matches fallingState within 1e-6 relative', () => {
  const s = run(stepRK4, { x: 0, y: 0, z: 0, vx: 0, vy: 0, vz: 0 }, 1.0, 250, phys); // h = 0.004 s
  const ref = fallingState(1.0, g, k);
  assert.ok(Math.abs(-s.z - ref.drop) / ref.drop < 1e-6, `drop ${-s.z} vs ${ref.drop}`);
  assert.ok(Math.abs(-s.vz - ref.speed) / ref.speed < 1e-6, `speed ${-s.vz} vs ${ref.speed}`);
});

test('global error falls about 16x when h is halved (4th order)', () => {
  const v0 = 500, T = 0.8; // in/s, s (before apex)
  const err = (N) => Math.abs(run(stepRK4, { x: 0, y: 0, z: 0, vx: 0, vy: 0, vz: v0 }, T, N, phys).z - risingState(v0, T, g, k).z);
  const e1 = err(10), e2 = err(20);
  const ratio = e1 / e2;
  console.log(`RK4 convergence: err(h=0.08)=${e1.toExponential(3)} in, err(h=0.04)=${e2.toExponential(3)} in, ratio ${ratio.toFixed(2)}`);
  assert.ok(ratio > 10 && ratio < 22, `ratio ${ratio}`);
});

// Flight from z=12 in at 500 in/s, 40 deg; returns landing x (in) at z=0 by linear interpolation in the last step.
function landing(step, h, model) {
  const e = 40 * Math.PI / 180;
  const s = { x: 0, y: 0, z: 12, vx: 500 * Math.cos(e), vy: 0, vz: 500 * Math.sin(e) };
  for (let i = 0; i < 1e6; i++) {
    const px = s.x, pz = s.z;
    step(s, h, model, true);
    if (s.z < 0 && s.vz < 0) { const f = pz / (pz - s.z); return px + f * (s.x - px); }
  }
  throw new Error('no landing');
}

// Threshold proof (measured 2026-11-22, pollen, physical model, kDrag 1.1226e-3 1/in, no drag crisis, no lift):
//   RK4 reference convergence, landing x (in):
//     h=4e-3 437.322427 | 2e-3 437.322974 | 1e-3 437.323006 | 5e-4 437.323021 | 1e-4 437.323031 | 5e-5 437.323031
//     -> RK4 is converged to 437.32303 in; 1e-3..1e-4 spread 2.5e-5 in (dominated by the O(h^2) landing interpolation).
//   Legacy (balls.js semi-implicit drag + Euler position, kept bit-identical by legacy_compat.test.mjs) error vs that ref:
//     h=8e-3 1.3248 | 4e-3 0.6613 | 2e-3 0.3307 | 1e-3 0.1653 | 5e-4 0.0826 | 1e-4 0.0165 in
//     -> error halves exactly with h: first-order, err ~= 165.3 in/s * h. At the sim's 0.004 s substep that is 0.661 in,
//        a deterministic truncation error of the legacy scheme, not a bug. The old "< 0.5 in" bound was below the
//        scheme's own O(h) error at h=0.004 and could never pass. stepLegacy cannot be changed (bit-identity spec).
//   New bounds: RK4 self-convergence < 1e-4 in; legacy observed order in [1.9, 2.1] error ratio per halving;
//   legacy(0.004) error < 0.70 in (measured 0.661 in + ~6% margin), and > 0.6 in so a silent scheme change is caught.
test('RK4 landing range converged: h=1e-3, 5e-4, 1e-4 agree within 1e-4 in', () => {
  const a = landing(stepRK4, 1e-3, phys), b = landing(stepRK4, 5e-4, phys), c = landing(stepRK4, 1e-4, phys);
  console.log(`RK4 landing: 1e-3 ${a.toFixed(6)}, 5e-4 ${b.toFixed(6)}, 1e-4 ${c.toFixed(6)} in`);
  assert.ok(Math.abs(a - c) < 1e-4 && Math.abs(b - c) < 1e-4, `spread ${Math.abs(a - c)} in`);
});

test('legacy landing error is first order and 0.6..0.7 in at h=0.004 vs converged RK4', () => {
  const ref = landing(stepRK4, 1e-4, phys);
  const d4 = Math.abs(landing(stepLegacy, 0.004, phys) - ref);
  const d2 = Math.abs(landing(stepLegacy, 0.002, phys) - ref);
  const d1 = Math.abs(landing(stepLegacy, 0.001, phys) - ref);
  console.log(`landing range: RK4(1e-4) ${ref.toFixed(3)} in, legacy err h=4e-3 ${d4.toFixed(4)}, 2e-3 ${d2.toFixed(4)}, 1e-3 ${d1.toFixed(4)} in`);
  const r1 = d4 / d2, r2 = d2 / d1;
  assert.ok(r1 > 1.9 && r1 < 2.1 && r2 > 1.9 && r2 < 2.1, `order ratios ${r1}, ${r2}`);
  assert.ok(d4 < 0.70 && d4 > 0.6, `diff ${d4} in`);
});

test('substepCount', () => {
  const m = { maxSubstep: 0.004, maxStepTravel: 0.8 };
  assert.equal(substepCount(0, 1 / 60, m), 5);      // ceil(0.01667/0.004)
  assert.equal(substepCount(100, 0.004, m), 1);
  assert.equal(substepCount(1000, 0.004, m), 5);    // 1000*0.004/0.8
  assert.equal(substepCount(1800, 0.004, m), 9);
  assert.ok(substepCount(1, 1e-9, m) >= 1);
});

test('clampSpeed caps at 1800 in/s and keeps direction', () => {
  const s = { vx: 3000, vy: 0, vz: 4000 };
  clampSpeed(s, 1800);
  assert.ok(Math.abs(Math.hypot(s.vx, s.vy, s.vz) - 1800) < 1e-9);
  assert.ok(Math.abs(s.vz / s.vx - 4 / 3) < 1e-12);
  const t = { vx: 10, vy: 20, vz: 30 };
  clampSpeed(t, 1800);
  assert.deepEqual([t.vx, t.vy, t.vz], [10, 20, 30]);
});
