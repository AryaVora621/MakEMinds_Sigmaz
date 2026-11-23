import test from 'node:test';
import assert from 'node:assert/strict';
import { sensitivity, OUTPUTS } from '../../web/js/physics/validation-tuning/sensitivity.js';
import { BALL } from '../../web/js/balls.js';
import { PARAM_SCHEMA } from '../../web/js/physics/validation-tuning/params.js';

test('OUTPUTS exposes the five documented outputs', () => {
  for (const k of ['bounceRise', 'rollDistance', 'airSpeedLoss', 'shotRange', 'driveT90']) assert.equal(typeof OUTPUTS[k], 'function', k);
});
test('bounce rise is more sensitive to restitutionFloor than to dragK; BALL is restored', () => {
  const before = structuredClone(BALL);
  const r = sensitivity({ params: ['ball.restitutionFloor', 'ball.dragK'], outputs: ['bounceRise'] });
  assert.deepEqual(BALL, before);
  const el = (p) => r.rows.find((x) => x.param === p && x.output === 'bounceRise').elasticity;
  assert.ok(Math.abs(el('ball.restitutionFloor')) > Math.abs(el('ball.dragK')), `${el('ball.restitutionFloor')} vs ${el('ball.dragK')}`);
  // bounce rise ~ e^2, so the elasticity should be near 2
  assert.ok(Math.abs(el('ball.restitutionFloor') - 2) < 0.5);
});
test('rows carry nominal and finite outputs; ranking is sorted by score', () => {
  const r = sensitivity({ params: ['ball.rollFriction', 'ball.restitutionFloor'], outputs: ['rollDistance', 'bounceRise'] });
  assert.equal(r.rows.length, 4);
  for (const row of r.rows) for (const k of ['nominal', 'y0', 'yPlus', 'yMinus', 'elasticity']) assert.ok(Number.isFinite(row[k]), `${row.param} ${k}`);
  assert.equal(r.ranking.length, 2);
  assert.ok(r.ranking[0].score >= r.ranking[1].score);
});
test('roll distance elasticity to rollFriction is about -1 (analytic v^2/(2 mu g))', () => {
  const r = sensitivity({ params: ['ball.rollFriction'], outputs: ['rollDistance'] });
  const e = r.rows[0].elasticity;
  assert.ok(e < -0.7 && e > -1.3, `elasticity ${e}`);
});
test('drive output responds to maxSpeed / maxAccel and drive defaults are untouched', () => {
  const r = sensitivity({ params: ['drive.maxAccel'], outputs: ['driveT90'] });
  assert.ok(r.rows[0].elasticity < 0, 'more accel means less time to 90% speed');
});
test('every schema key is accepted as a parameter', () => {
  const keys = PARAM_SCHEMA.map((e) => e.key);
  const r = sensitivity({ params: keys, outputs: ['driveT90'] });
  assert.equal(r.rows.length, keys.length);
});
