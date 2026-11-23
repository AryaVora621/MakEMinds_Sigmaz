// air.test.mjs - air density, viscosity, Reynolds, drag factor. Expected values are from the spec's computed check values.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AERO } from '../../web/js/physics/ball-aerodynamics/params.js';
import { airDensity, pressureAtAltitude, dynamicViscosity, reynolds, dragFactorPerIn, buildAeroModel } from '../../web/js/physics/ball-aerodynamics/air.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg}: got ${a}, want ${b} (+-${rel * 100}%)`);

test('air density at 20 C, 101325 Pa (kg/m^3)', () => {
  near(airDensity({ tempC: 20, pressurePa: 101325, relHumidity: 0 }), 1.2041, 1e-3, 'dry');
  near(airDensity({ tempC: 20, pressurePa: 101325, relHumidity: 0.5 }), 1.1988, 1e-3, 'RH 50%');
});

test('ISA pressure at 300 m is about 97.78 kPa', () => {
  assert.ok(Math.abs(pressureAtAltitude(300) - 97780) <= 50, `got ${pressureAtAltitude(300)} Pa`);
});

test('Sutherland viscosity at 20 C (Pa*s)', () => {
  near(dynamicViscosity(20), 1.813e-5, 5e-3, 'mu');
});

test('Reynolds number of a pollen ball at 500 in/s', () => {
  near(reynolds(500, 1.391, 1.506e-5), 5.96e4, 0.01, 'Re');
});

test('drag factor k = base*Cd (1/in)', () => {
  near(dragFactorPerIn(0.47, 1.2041, 1.391, 0.025), 0.0011275, 5e-3, 'pollen');
  near(dragFactorPerIn(0.47, 1.2041, 1.7995, 0.041), 0.0011506, 5e-3, 'nectar');
});

test('legacy model: kDrag 0.0004, lift disabled, legacy integrator', () => {
  const m = buildAeroModel(AERO, AERO.kinds.pollen, { model: 'legacy' });
  assert.equal(m.mode, 'legacy');
  assert.equal(m.kDrag, 0.0004);
  assert.equal(m.lift.enabled, false);
  assert.equal(m.integrator, 'legacy');
});

test('legacy mode forces lift off even if enabled in params', () => {
  const m = buildAeroModel(AERO, AERO.kinds.pollen, { model: 'legacy', lift: { ...AERO.lift, enabled: true } });
  assert.equal(m.lift.enabled, false);
});
