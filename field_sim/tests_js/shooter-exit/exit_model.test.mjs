import { test } from 'node:test';
import assert from 'node:assert/strict';
import { PARAMS, PARAM_SOURCES, mergeParams } from '../../web/js/physics/shooter-exit/params.js';
import * as M from '../../web/js/physics/shooter-exit/exit_model.js';
import { BALL_KINDS, launchScaleFor } from '../../web/js/balls.js';

const rel = (a, b) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-300);
const phys = { model: 'physical' };

function leaves(o, pre = '', out = []) {
  for (const [k, v] of Object.entries(o)) {
    if (v && typeof v === 'object') leaves(v, pre + k + '.', out); else out.push(pre + k);
  }
  return out;
}

test('params frozen, every leaf sourced', () => {
  assert.ok(Object.isFrozen(PARAMS) && Object.isFrozen(PARAMS.compression.GAP_IN));
  for (const p of leaves(PARAMS)) {
    const e = PARAM_SOURCES[p];
    assert.ok(e && typeof e.source === 'string' && typeof e.assumed === 'boolean' && e.measure, 'missing source ' + p);
  }
  assert.equal(Object.keys(PARAM_SOURCES).length, leaves(PARAMS).length);
});

test('mergeParams returns frozen merged copy, original untouched', () => {
  const m = mergeParams({ wheel: { R_IN: 2 }, compression: { GAP_IN: { nectar: 3 } } });
  assert.equal(m.wheel.R_IN, 2); assert.equal(m.wheel.R2_IN, 1.89);
  assert.equal(m.compression.GAP_IN.nectar, 3); assert.equal(m.compression.GAP_IN.pollen, 2.2256);
  assert.equal(PARAMS.wheel.R_IN, 1.89); assert.ok(Object.isFrozen(m.wheel));
});

test('legacy equals shotrules speedFromRpm bit for bit', () => {
  for (const kind of ['pollen', 'nectar']) for (const rpm of [-5, 0, 500, 3000, 4321.5, 6000]) {
    const wheelR = 2, eff = 0.45, rpmScale = 1.3;
    const ref = Math.max(0, rpm) / rpmScale * 2 * Math.PI * wheelR * eff / 60 * launchScaleFor(kind);
    assert.equal(M.exitSpeed(rpm, kind, { model: 'legacy', wheelR, eff, rpmScale }), ref);
  }
});

test('physical pollen equals legacy at several rpm', () => {
  for (const rpm of [500, 3000, 6000]) {
    const l = M.exitSpeed(rpm, 'pollen', { model: 'legacy' }), p = M.exitSpeed(rpm, 'pollen', phys);
    assert.ok(rel(p, l) < 1e-9, `${rpm}: ${p} vs ${l}`);
  }
});

test('nectar s computed and differs from launchScaleFor', () => {
  const s = M.transferFactor('nectar', 3000), ref = launchScaleFor('nectar');
  assert.ok(s > 0 && s <= 1);
  assert.ok(Math.abs(s - ref) > 1e-3);
  const r = M.exitSpeed(3000, 'nectar', phys) / M.exitSpeed(3000, 'pollen', phys);
  assert.ok(Math.abs(r - ref) > 1e-3);
  assert.ok(M.transferFactor('pollen', 3000) > 0.9 && M.transferFactor('pollen', 3000) < 1);
});

test('hooded s equals recalc sqrt(4 iW rB^2 / D)', () => {
  for (const kind of ['pollen', 'nectar']) {
    const B = BALL_KINDS[kind], mB = B.massKg, rB = B.radiusM, iB = 0.4 * mB * rB * rB;
    const rW = PARAMS.wheel.R_IN * 0.0254, iW = PARAMS.wheel.MOI_KGM2;
    const D = 4 * iW * rB * rB + iB * rW * rW + mB * rW * rW * rB * rB;
    const ref = Math.sqrt(4 * iW * rB * rB / D);
    assert.ok(Math.abs(M.transferFactor(kind, 3000) - ref) < 1e-12);
    assert.ok(Math.abs(M.transferFactor(kind, 1000) - ref) < 1e-12);
  }
});

test('dual config gives s in (0,1] and spin from wheel difference', () => {
  const dual = mergeParams({ wheel: { CONFIG: 'dual', SPEED_RATIO_2: 0.8 } });
  const st = M.exitState(3000, 'pollen', { ...phys, params: dual });
  assert.ok(st.s > 0 && st.s <= 1 && st.spinRadS > 0 && st.speed > 0);
  const eq = mergeParams({ wheel: { CONFIG: 'dual' } });
  assert.ok(Math.abs(M.exitState(3000, 'pollen', { ...phys, params: eq }).spinRadS) < 1e-9);
});

test('inverse round trip', () => {
  for (const kind of ['pollen', 'nectar']) for (const rpm of [300, 2500, 4800]) {
    assert.ok(rel(M.rpmForExitSpeed(M.exitSpeed(rpm, kind, phys), kind, phys), rpm) < 1e-6);
    assert.ok(rel(M.rpmForExitSpeed(M.exitSpeed(rpm, kind, { model: 'legacy' }), kind, { model: 'legacy' }), rpm) < 1e-6);
  }
  const slip = mergeParams({ slip: { V_SLIP_INS: 900 } });
  const o = { ...phys, params: slip };
  for (const rpm of [300, 2500, 4800]) assert.ok(rel(M.rpmForExitSpeed(M.exitSpeed(rpm, 'nectar', o), 'nectar', o), rpm) < 1e-6);
  assert.ok(M.exitSpeed(5000, 'pollen', o) < M.exitSpeed(5000, 'pollen', phys));
  assert.ok(Number.isNaN(M.rpmForExitSpeed(1e9, 'pollen', o)));
  assert.ok(Number.isNaN(M.rpmForExitSpeed(NaN, 'pollen', phys)));
});

test('eta_c monotone up to C_OPT and bounded', () => {
  let prev = 0;
  for (let c = 0; c <= PARAMS.compression.C_OPT + 1e-12; c += 0.01) {
    const e = M.etaCompression(c); assert.ok(e >= prev && e >= 0.05 && e <= 1); prev = e;
  }
  assert.ok(M.etaCompression(0.6) < M.etaCompression(PARAMS.compression.C_OPT));
  assert.ok(Math.abs(M.compressionOf('pollen') - 0.2) < 1e-12);
  assert.ok(Math.abs(M.compressionOf('nectar') - 0.2) < 1e-12);
  assert.equal(M.etaSlip(1234), 1);
});

test('rpmAfterShot and contact time', () => {
  assert.ok(M.rpmAfterShot(3000, 'pollen', phys) < 3000);
  assert.ok(Math.abs(M.rpmAfterShot(3000, 'pollen', phys) - 3000 * M.transferFactor('pollen', 3000)) < 1e-9);
  assert.ok(Math.abs(M.rpmAfterShot(3000, 'pollen') - 3000 * 0.94) < 1e-9);
  const st = M.exitState(3000, 'pollen', phys);
  assert.ok(st.contactTimeS > 0 && st.contactTimeS < 0.1);
  assert.ok(M.contactTimeS(0, 'pollen') > 0 && isFinite(M.contactTimeS(0, 'pollen')));
  assert.ok(st.spinRadS > 0 && st.rpmAfter < 3000);
});

test('speed monotone in rpm and default model is legacy', () => {
  let prev = -1;
  for (let r = 0; r <= 6000; r += 250) { const v = M.exitSpeed(r, 'nectar', phys); assert.ok(v >= prev); prev = v; }
  assert.equal(M.exitState(3000, 'pollen').model, 'legacy');
});
