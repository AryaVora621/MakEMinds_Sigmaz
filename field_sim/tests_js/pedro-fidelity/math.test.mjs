// Pedro 2.1.0 math/controllers port: hand-computed numbers. Units: rad, s, in, in/s.
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  normalizeAngle, normalizeAngleSigned, smallestAngleDiff, turnDirection, vPolar, vTheta, vSetMag, vMag,
  findNormalizingScaling, Kinematics, SimClock, PIDF, FilteredPIDF, KalmanFilter, PredictiveBraking,
} from '../../web/js/physics/pedro-fidelity/pedromath.js';

const near = (a, b, tol = 1e-9, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} ${a} vs ${b}`);

test('angle helpers', () => {
  near(normalizeAngle(-Math.PI / 2), 1.5 * Math.PI);
  near(normalizeAngle(2 * Math.PI), 0);
  near(normalizeAngleSigned(1.5 * Math.PI), -Math.PI / 2);
  near(smallestAngleDiff(0.1, 2 * Math.PI - 0.1), 0.2);
  near(smallestAngleDiff(0, Math.PI), Math.PI);
  assert.equal(turnDirection(0, Math.PI / 2), 1);
  assert.equal(turnDirection(0, -Math.PI / 2), -1);
  assert.equal(turnDirection(0, Math.PI), 1, 'exactly PI counts as CCW (norm(e-s) in [0,PI])');
});

test('vector quirks: zero-vector theta is 0, negative polar magnitude flips, setMag on zero points +x', () => {
  assert.equal(vTheta({ x: 0, y: 0 }), 0);
  const f = vPolar(-1, 0); near(f.x, -1); near(f.y, 0);
  const z = vSetMag({ x: 0, y: 0 }, 2); near(z.x, 2); near(z.y, 0);
  near(vTheta({ x: 0, y: -1 }), 1.5 * Math.PI);
  near(vMag({ x: 3, y: 4 }), 5);
});

test('findNormalizingScaling', () => {
  // |s + k v| = m with s=0, v=(2,0), m=1 -> k = 0.5
  near(findNormalizingScaling({ x: 0, y: 0 }, { x: 2, y: 0 }, 1), 0.5);
  // s=(0.5,0) v=(1,0) m=1 -> k = 0.5
  near(findNormalizingScaling({ x: 0.5, y: 0 }, { x: 1, y: 0 }, 1), 0.5);
});

test('Kinematics', () => {
  near(Kinematics.velToStopWithDecel(8, -4), 8);       // sqrt(|-2*-4*8|)
  near(Kinematics.velToStopWithDecel(-8, -4), -8);
  near(Kinematics.finalVelAtDistance(10, -4, 8), 6);   // sqrt(100 - 64)
  near(Kinematics.predictNextLoopVelocity(3, 2), 4);
  near(Kinematics.distanceToVelocity(10, -4, 0), 12.5);
  near(Kinematics.stoppingDistance(10, -4), 12.5);
});

test('PIDF exact numbers (P=2 I=0.5 D=0.1 F=3)', () => {
  const clock = new SimClock(0);
  const c = new PIDF({ P: 2, I: 0.5, D: 0.1, F: 3 }, clock); c.reset();
  clock.advance(0.1); c.updateError(1); c.updateFeedForwardInput(1);
  // integral 0.1, deriv (1-0)/0.1 = 10 -> 2 + 0.1*10 + 0.5*0.1 + 3
  near(c.run(), 6.05, 1e-9);
  clock.advance(0.1); c.updateError(0.5);
  // integral 0.15, deriv -5 -> 1 - 0.5 + 0.075 + 3
  near(c.run(), 3.575, 1e-9);
});

test('PIDF dt<=0 guard gives finite output (deviation from Java Inf/NaN)', () => {
  const clock = new SimClock(0);
  const c = new PIDF({ P: 1, I: 1, D: 1, F: 0 }, clock); c.reset();
  c.updateError(1); c.updateError(2);
  assert.ok(Number.isFinite(c.run()));
});

test('FilteredPIDF keeps the RAW previous derivative (Pedro quirk)', () => {
  const clock = new SimClock(0);
  const c = new FilteredPIDF({ P: 0, I: 0, D: 1, T: 0.6, F: 0 }, clock); c.reset();
  clock.advance(0.1); c.updateError(1);        // raw 10, prev 0 -> 0.6*0 + 0.4*10 = 4
  near(c.run(), 4, 1e-9);
  clock.advance(0.1); c.updateError(1);        // raw 0, prev RAW 10 -> 0.6*10 + 0 = 6 (a stored-filtered impl would give 2.4)
  near(c.run(), 6, 1e-9);
});

test('KalmanFilter exact numbers (Q=6, R=1)', () => {
  const k = new KalmanFilter({ modelCovariance: 6, dataCovariance: 1 });
  k.reset(0, 1, 1);
  k.update(1, 3);            // state 1, var 7, K 7/8 -> 1 + 0.875*2 = 2.75, var 0.875
  near(k.state, 2.75, 1e-12);
  k.update(0, 2.75);         // zero innovation: state unchanged
  near(k.state, 2.75, 1e-12);
});

test('PredictiveBraking.computeOutput', () => {
  const pb = new PredictiveBraking({ P: 0.15, kLinearBraking: 0.1, kQuadraticFriction: 0.001, maximumBrakingPower: 0.2 });
  near(pb.computeOutput(10, 5), 0.15 * (10 - (25 * 0.001 + 5 * 0.1)), 1e-12);
});

test('SimClock is exact in ns', () => {
  const c = new SimClock(0);
  for (let i = 0; i < 1000; i++) c.advance(0.01);
  assert.equal(c.nowNs(), 1e10); near(c.nowMs(), 1e4);
});
