import test from 'node:test';
import assert from 'node:assert/strict';
import { THERMAL, MOTORS } from '../../web/js/physics/motor-model/params.js';
import { ThermalTwoNode } from '../../web/js/physics/motor-model/thermal.js';
import { deriveMotor } from '../../web/js/physics/motor-model/derive.js';

test('initial stall heating rate is 11.04 K/s', () => {
  const m = deriveMotor(MOTORS.gobilda5203);
  const P = 9.2 * 9.2 * m.R;
  const th = new ThermalTwoNode(THERMAL);
  const dt = 1e-4;
  th.step(dt, P);
  const rate = (th.windingC - THERMAL.ambientC) / dt;
  assert.ok(Math.abs(rate - 11.04) / 11.04 < 0.01, `rate ${rate}`);
});

test('constant power steady state is P*(Rwh+Rha)', () => {
  const th = new ThermalTwoNode(THERMAL);
  const P = 5;
  for (let i = 0; i < 200000; i++) th.step(0.1, P);
  const exp = P * (THERMAL.rthWindingHousing + THERMAL.rthHousingAmbient);
  assert.ok(Math.abs(th.windingC - THERMAL.ambientC - exp) / exp < 0.005, `${th.windingC - THERMAL.ambientC} vs ${exp}`);
});

test('scales and overTemp flag', () => {
  const th = new ThermalTwoNode(THERMAL);
  assert.equal(th.rScale(), 1);
  assert.equal(th.kScale(), 1);
  assert.equal(th.overTemp, false);
  for (let i = 0; i < 20000; i++) th.step(0.1, 30);
  assert.ok(th.overTemp);
  assert.ok(th.rScale() > 1 && th.kScale() < 1);
  th.reset();
  assert.equal(th.windingC, THERMAL.ambientC);
});
