// Tests for shot_transfer.js (flywheel-dynamics). DERIVED model: P_w = m*v*(1+k*s)/2, J*(w0-w1) = R*P_w.
import test from 'node:test';
import assert from 'node:assert/strict';
import { FLYWHEEL_PARAMS, RPM_TO_RADS } from '../../web/js/physics/flywheel-dynamics/params.js';
import {
  makeBall, impulseCoefficientKgM2, contactTimeS, predictShot, omegaForExitSpeed,
} from '../../web/js/physics/flywheel-dynamics/shot_transfer.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg || ''} got ${a} want ${b} (rel ${rel})`);
const shot = FLYWHEEL_PARAMS.shot;
const R = FLYWHEEL_PARAMS.wheel.radiusM;
const J = 2.5150e-4;
const w0 = 4000 * RPM_TO_RADS;
const pollen = makeBall('pollen', shot);
const nectar = makeBall('nectar', shot);

test('makeBall resolution rules', () => {
  assert.equal(pollen.massKg, 0.025);
  near(pollen.radiusM, 1.391 * 0.0254, 1e-9);
  assert.equal(pollen.etaScale, 1);
  near(nectar.etaScale, Math.sqrt(0.025 / 0.041), 1e-9);
  assert.deepEqual(makeBall('does-not-exist', shot), pollen);
  assert.deepEqual(makeBall(null, shot), pollen);
  assert.deepEqual(makeBall(undefined, shot), pollen);
  const o = makeBall({ massKg: 0.03 }, shot);
  assert.equal(o.massKg, 0.03);
  assert.equal(o.radiusM, pollen.radiusM);
  assert.equal(o.shellK, 2 / 3);
  assert.equal(o.etaScale, 1);
  const copy = makeBall('pollen', shot);
  copy.massKg = 9;
  assert.equal(shot.balls.pollen.massKg, 0.025); // not aliased
});

test('default pollen: c = 2.1606e-5, beta = 0.08591 (+-0.5%), mid dip 8.24% (+-0.1 pt)', () => {
  near(impulseCoefficientKgM2(pollen, shot, R), 2.1606e-5, 5e-3);
  const r = predictShot(w0, J, pollen, shot, R);
  near(r.beta, 0.08591, 5e-3);
  assert.ok(Math.abs(r.dipFrac - 0.0824) <= 0.001, `dip ${r.dipFrac}`);
  near(r.exitSpeedMps, 8.68, 0.02);
  near(r.contactTimeS, 0.0184, 0.03);
});

test('impulse identity J*(w0-w1) = R*P_w and angular impulse field', () => {
  for (const ref of ['mid', 'end', 'start']) {
    const s = { ...shot, exitSpeedRef: ref };
    const r = predictShot(w0, J, pollen, s, R);
    near(J * (w0 - r.omega1RadS), R * r.wheelImpulseNs, 1e-9, ref);
    near(r.wheelAngularImpulseNms, J * (w0 - r.omega1RadS), 1e-9, ref);
    near(r.wheelImpulseNs, pollen.massKg * r.exitSpeedMps * (1 + pollen.shellK * s.spinRatio) / 2, 1e-9, ref);
  }
});

test('energy bookkeeping: slip loss >= 0 for all refs, balls and speeds', () => {
  for (const ref of ['mid', 'end', 'start']) {
    for (const ball of [pollen, nectar]) {
      for (const rpm of [500, 2000, 4000, 6000]) {
        const r = predictShot(rpm * RPM_TO_RADS, J, ball, { ...shot, exitSpeedRef: ref }, R);
        assert.ok(r.slipLossJ >= -1e-9, `${ref} ${rpm} slip ${r.slipLossJ}`);
        near(r.slipLossJ, r.energyFromWheelJ - r.ballKineticJ, 1e-6);
        assert.ok(r.omega1RadS <= rpm * RPM_TO_RADS && r.omega1RadS >= 0);
      }
    }
  }
  assert.ok(predictShot(w0, J, pollen, shot, R).energyRatio > 1);
});

test('inverse round-trip for mid, end and start', () => {
  for (const ref of ['mid', 'end', 'start']) {
    const s = { ...shot, exitSpeedRef: ref };
    for (const ball of [pollen, nectar]) {
      const r = predictShot(w0, J, ball, s, R);
      near(omegaForExitSpeed(r.exitSpeedMps, J, ball, s, R), w0, 1e-9, ref);
    }
  }
});

test('nectar dips more than pollen (about 10.5% vs 8.2%)', () => {
  const p = predictShot(w0, J, pollen, shot, R);
  const n = predictShot(w0, J, nectar, shot, R);
  assert.ok(n.dipFrac > p.dipFrac);
  assert.ok(Math.abs(n.dipFrac - 0.105) < 0.005, `nectar dip ${n.dipFrac}`);
});

test('w0 = 0 (or non-finite) gives zeros and Infinity energy ratio', () => {
  for (const w of [0, -5, NaN]) {
    const r = predictShot(w, J, pollen, shot, R);
    assert.equal(r.dipFrac, 0);
    assert.equal(r.exitSpeedMps, 0);
    assert.equal(r.contactTimeS, shot.maxContactS);
    assert.equal(r.energyRatio, Infinity);
  }
});

test('doubling J roughly halves the dip', () => {
  const a = predictShot(w0, J, pollen, shot, R).dipFrac;
  const b = predictShot(w0, 2 * J, pollen, shot, R).dipFrac;
  const ratio = b / a;
  assert.ok(ratio > 0.45 && ratio < 0.55, `ratio ${ratio}`);
});

test('contact time is 2*path/v, clamped', () => {
  near(contactTimeS(8.68, shot), 2 * 0.08 / 8.68, 1e-9);
  assert.equal(contactTimeS(1000, shot), shot.minContactS);
  assert.equal(contactTimeS(0, shot), shot.maxContactS);
});

test('start and end exit-speed references', () => {
  const r = predictShot(w0, J, pollen, { ...shot, exitSpeedRef: 'start' }, R);
  near(r.exitSpeedMps, 0.45 * w0 * R, 1e-9);
  const e = predictShot(w0, J, pollen, { ...shot, exitSpeedRef: 'end' }, R);
  near(e.omega1RadS, w0 / (1 + e.beta), 1e-9);
});
