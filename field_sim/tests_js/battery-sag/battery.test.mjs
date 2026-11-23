import test from 'node:test';
import assert from 'node:assert/strict';
import { Battery, ocvPerCell, wiringOhm } from '../../web/js/physics/battery-sag/battery.js';
import { BATTERY, interp } from '../../web/js/physics/battery-sag/params.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b} (tol ${e})`);

test('fresh OCV is 14.0 V (10 cells x 1.40 V)', () => {
  const b = new Battery();
  near(b.ocv(), 14.0, 1e-9);
  near(b.vth(), 14.0, 1e-9);
  near(ocvPerCell(1.0), 1.40, 1e-12);
});

test('interp clamps at both ends and is linear between points', () => {
  const t = [[0, 0], [1, 10]];
  assert.equal(interp(t, -5), 0);
  assert.equal(interp(t, 7), 10);
  near(interp(t, 0.5), 5, 1e-12);
  assert.equal(interp(BATTERY.ocvTable, -1), BATTERY.ocvTable[0][1]);
  assert.equal(interp(BATTERY.ocvTable, 2), 1.40);
});

test('OCV is monotonic increasing in SoC', () => {
  let prev = -Infinity;
  for (let s = 0; s <= 1.0001; s += 0.05) {
    const b = new Battery({ soc: Math.min(1, s) });
    assert.ok(b.ocv() >= prev);
    prev = b.ocv();
  }
});

// Spec quotes 0.01504 ohm; the listed components sum to 0.01500 ohm, so tolerance is 1e-4.
test('wiring resistance is about 0.01504 ohm', () => {
  near(wiringOhm(), 0.01504, 1e-4);
});

test('fresh series resistance is about 0.105 ohm', () => {
  near(new Battery().rs(), 0.105, 0.003);
  assert.equal(new Battery().rs(), new Battery().r0());
});

test('V1 after 10 A for 15 s equals R1*I*(1-e^-1)', () => {
  const b = new Battery();
  b.step(10, 15);
  near(b.state.v1, 0.025 * 10 * (1 - Math.exp(-1)), 1e-3);
});

test('V1 update is step-size independent (exact exponential)', () => {
  const a = new Battery(), c = new Battery();
  a.step(10, 15);
  for (let i = 0; i < 150; i++) c.step(10, 0.1);
  near(a.state.v1, c.state.v1, 1e-9);
});

test('3 A for 3600 s drops SoC by 1.0 Ah / 3.0 Ah capacity', () => {
  const b = new Battery({ soc: 1 });
  b.step(3, 1800);
  near(b.state.soc, 0.5, 1e-9);
  near(b.state.ahUsed, 1.5, 1e-9);
  b.step(3, 1800);
  near(b.state.soc, 0, 1e-9);
  near(b.state.ahUsed, 3.0, 1e-9);
});

test('SoC clamps at 0 and never goes negative', () => {
  const b = new Battery({ soc: 0.01 });
  b.step(30, 3600);
  assert.equal(b.state.soc, 0);
});

test('vTermLast is Vth_before - I*Rs', () => {
  const b = new Battery();
  const vth = b.vth(), rs = b.rs();
  b.step(20, 0.01);
  near(b.state.vTermLast, vth - 20 * rs, 1e-9);
});

test('tired preset has lower vth and higher rs than fresh', () => {
  const f = new Battery(); f.applyPreset('fresh');
  const t = new Battery(); t.applyPreset('tired');
  assert.ok(t.vth() < f.vth());
  assert.ok(t.rs() > f.rs());
  assert.equal(t.state.v1, 0);
  assert.equal(t.state.ahUsed, 0);
});

test('unknown preset throws; snapshot has expected keys', () => {
  const b = new Battery();
  assert.throws(() => b.applyPreset('nope'));
  const s = b.snapshot();
  for (const k of ['soc', 'v1', 'ahUsed', 'ocv', 'vth', 'rs', 'vTermLast']) assert.ok(k in s, k);
});
