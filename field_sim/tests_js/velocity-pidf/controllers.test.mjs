import test from 'node:test';
import assert from 'node:assert/strict';
import { defaultParams } from '../../web/js/physics/velocity-pidf/params.js';
import { SdkFlywheelPIDF, clampTargetTicks, HubVelocityPIDF, hubKfFromMaxTicks, ReadyGate } from '../../web/js/physics/velocity-pidf/controllers.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b}`);

test('SdkFlywheelPIDF hand-computed values', () => {
  const c = new SdkFlywheelPIDF();
  near(c.calculate(1500, 1400, 12.5, 0.02), 0.564, 1e-9);
  near(c.lastVolts, 7.05, 1e-9);
  near(c.calculate(1500, 1450, 12.5, 0.02), 0.544, 1e-9);
});

test('SdkFlywheelPIDF battery fallback and target 0 reset', () => {
  const c = new SdkFlywheelPIDF();
  near(c.calculate(1500, 1400, 0, 0.02), 7.05 / 12, 1e-9);
  const d = new SdkFlywheelPIDF();
  near(d.calculate(1500, 1400, NaN, 0.02), 7.05 / 12, 1e-9);
  near(d.calculate(1500, 1400, 0.5, 0.02), 7.05 / 12, 1e-9);   // <= 1 V also falls back
  assert.equal(c.calculate(0, 100, 12, 0.02), 0);
  // after the reset the next call is a first call again
  near(c.calculate(1500, 1400, 12.5, 0.02), 0.564, 1e-9);
});

test('SdkFlywheelPIDF feedforward kA uses the target rate; output clamps to +/-1; NaN gives 0', () => {
  const c = new SdkFlywheelPIDF();
  c.calculate(1500, 1500, 12, 0.02);
  c.calculate(1600, 1600, 12, 0.02);
  near(c.lastVolts, 0.1 + 0.0043 * 1600 + 0.0005 * 5000, 1e-9);
  assert.equal(new SdkFlywheelPIDF().calculate(1e6, 0, 12, 0.02), 1);
  assert.equal(new SdkFlywheelPIDF().calculate(NaN, 0, 12, 0.02), 0);
  assert.equal(new SdkFlywheelPIDF().calculate(1000, NaN, 12, 0.02), 0);
});

test('SdkFlywheelPIDF integral is clamped to iMaxVolts/|kI|', () => {
  const g = { ...defaultParams().sdkPidf, kS: 0, kV: 0, kA: 0, kP: 0, kI: 1, kD: 0, iMaxVolts: 2 };
  const c = new SdkFlywheelPIDF(g);
  for (let i = 0; i < 20; i++) c.calculate(1000, 0, 12, 1);
  near(c.lastVolts, 2, 1e-9);
});

test('clampTargetTicks', () => {
  near(clampTargetTicks(466.67, 2500, 5000, 28), 1166.67, 0.01);
  near(clampTargetTicks(3000, 2500, 5000, 28), 2333.33, 0.01);
  near(clampTargetTicks(1400, 2500, 5000, 28), 1400, 1e-9);
  assert.equal(clampTargetTicks(NaN, 2500, 5000, 28), 0);
  assert.equal(clampTargetTicks(0, 2500, 5000, 28), 0);
  assert.equal(clampTargetTicks(-5, 2500, 5000, 28), 0);
});

test('hubKfFromMaxTicks', () => {
  near(hubKfFromMaxTicks(2800), 11.70, 0.01);
  near(hubKfFromMaxTicks(1000, 1000), 1, 1e-12);
});

test('HubVelocityPIDF output and clamp', () => {
  const h = new HubVelocityPIDF();
  h.setTarget(2800);
  assert.equal(h.target, 2800);
  const p = h.tick(0, 0.05);
  assert.ok(p > 0.86 && p < 0.875, `p=${p}`);          // (10*2800 + 3*140)/32767
  const big = new HubVelocityPIDF();
  big.setTarget(1e5);
  assert.equal(big.tick(0, 0.05), 1);
  big.setTarget(-1e5);
  assert.equal(big.tick(0, 0.05), -1);
  const z = new HubVelocityPIDF();
  z.setTarget(0);
  assert.equal(z.tick(500, 0.05), 0);
  for (let i = 0; i < 200; i++) { big.setTarget(3000); const o = big.tick(0, 0.05); assert.ok(o >= -1 && o <= 1); }
});

test('ReadyGate ticks mode and settleLoops', () => {
  const g = new ReadyGate({ mode: 'ticks', tolTicks: 50, fracTol: 0.04, settleLoops: 3 });
  assert.equal(g.update(1400, 1380), false);
  assert.equal(g.update(1400, 1390), false);
  assert.equal(g.update(1400, 1395), true);
  assert.equal(g.ready, true);
  assert.equal(g.update(1400, 1000), false);
  assert.equal(g.ready, false);
  assert.equal(g.update(1400, 1400), false);   // counter restarted
  g.reset();
  assert.equal(g.ready, false);
});

test('ReadyGate default is a single-loop check, target 0 is never ready, frac mode', () => {
  const g = new ReadyGate();
  assert.equal(g.update(1400, 1350), true);
  assert.equal(g.update(1400, 1349), false);
  assert.equal(g.update(0, 0), false);
  const f = new ReadyGate({ mode: 'frac', tolTicks: 50, fracTol: 0.04, settleLoops: 1 });
  assert.equal(f.update(1000, 1030), true);
  assert.equal(f.update(1000, 1041), false);
});
