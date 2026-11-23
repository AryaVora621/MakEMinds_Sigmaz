import test from 'node:test';
import assert from 'node:assert/strict';
import { MOTORS } from '../../web/js/physics/motor-model/params.js';
import { deriveMotor, makeGearmotor, rpmToRadS } from '../../web/js/physics/motor-model/derive.js';

const near = (a, b, e) => assert.ok(Math.abs(a - b) <= e, `${a} !~ ${b} (+-${e})`);

test('5203 derived constants match the spec', () => {
  const m = deriveMotor(MOTORS.gobilda5203);
  near(m.R, 1.304348, 1e-6);
  near(m.Kv, 53.8224, 1e-3);
  near(m.Ke, 0.0185796, 1e-6);
  near(m.Kt, 0.0161071, 1e-6);
  near(m.Tf, 0.00402677, 1e-7);
});

test('both datasheet endpoints hold exactly with friction', () => {
  const s = MOTORS.gobilda5203, m = deriveMotor(s);
  const wf = rpmToRadS(s.freeSpeedRpm);
  const Ifree = (s.nominalV - m.Ke * wf) / m.R;
  near(Ifree, s.freeCurrentA, 1e-9);
  near(m.Kt * Ifree - m.Tf, 0, 1e-12);
  const Istall = s.nominalV / m.R;
  near(Istall, s.stallCurrentA, 1e-9);
  near(m.Kt * Istall - m.Tf, s.stallTorqueNm, 1e-9);
});

test('encoder counts per output rev match goBILDA published PPR', () => {
  near(makeGearmotor('gobilda5203', 'gb5_2').ticksPerMechRev, 145.1, 0.1);
  near(makeGearmotor('gobilda5203', 'gb13_7').ticksPerMechRev, 384.5, 0.1);
  near(makeGearmotor('gobilda5203', 'gb19_2').ticksPerMechRev, 537.7, 0.1);
});

test('gearmotor aggregates', () => {
  const gm = makeGearmotor('gobilda5203', 'gb13_7', { numMotors: 1, extraRatio: 2 });
  near(gm.G, 27.467128, 1e-5);
  near(gm.eta, 0.93 ** 2, 1e-12);
  near(gm.rotorJReflected, 5e-6 * gm.G * gm.G, 1e-12);
  near(gm.freeSpeedOutRadS, rpmToRadS(6000) / gm.G, 1e-9);
});
