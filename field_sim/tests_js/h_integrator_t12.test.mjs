// t12: mapKeyboard, deadzone, applyCurve, integrate, MatchClock, ScoreKeeper edge cases.
import test from 'node:test';
import assert from 'node:assert/strict';
import { radialDeadzone, scalarDeadzone, applyCurve, mapKeyboard, integrate, MatchClock, ScoreKeeper, POINTS, DEADZONE } from '../web/js/driver.js';
const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) < e, `${a} !~ ${b}`);
const kb = (...keys) => mapKeyboard(new Set(keys));

test('opposite keys cancel: W+S, Q+E, A+D each give 0', () => {
  assert.equal(kb('w', 's').fwd, 0); assert.equal(kb('q', 'e').strafe, 0); assert.equal(kb('a', 'd').turn, 0);
  const all = kb('w', 's', 'q', 'e', 'a', 'd'); assert.deepEqual([all.fwd, all.strafe, all.turn], [0, 0, 0]);
});
test('diagonal W+Q is normalised to magnitude 1, not faster than a straight key', () => {
  const c = kb('w', 'q'); near(Math.hypot(c.fwd, c.strafe), 1); near(c.fwd, c.strafe); assert.equal(kb('w').fwd, 1);
});
test('radialDeadzone is continuous at the boundary and ramps 0..1', () => {
  assert.deepEqual(radialDeadzone(DEADZONE - 1e-9, 0), [0, 0]);
  assert.ok(Math.abs(radialDeadzone(DEADZONE + 1e-9, 0)[0]) < 1e-6);
  let prev = 0;   // magnitude sweep along +x; no step above ~0.0011
  for (let i = 0; i <= 1000; i++) { const [x] = radialDeadzone(i / 1000, 0); assert.ok(x >= prev - 1e-12 && x - prev < 0.0015); prev = x; }
  near(radialDeadzone(1, 0)[0], 1);
});
test('applyCurve(1)=1 for every cubic blend; curve and scalarDeadzone are odd', () => {
  for (const c of [0, 0.25, 0.5, 1]) { near(applyCurve(1, c), 1); for (const v of [0.1, 0.37, 0.8]) near(applyCurve(-v, c), -applyCurve(v, c)); }
  for (const v of [0.05, 0.3, 0.9]) near(scalarDeadzone(-v), -scalarDeadzone(v));
});
test('integrate with dt=0 leaves the pose unchanged at full velocity', () => {
  const S = { x: 10, y: -20, h: 0.5 }; integrate(S, 60, -60, 3.2, 0); assert.deepEqual(S, { x: 10, y: -20, h: 0.5 });
});
test('MatchClock: time left never negative or rising; phases in order', () => {
  const c = new MatchClock(); const order = []; let prev = Infinity;
  c.onPhase(p => order.push(p)); c.start();
  for (let i = 0; i < 400; i++) {   // 0.5 s steps, past the 158 s match
    c.update(0.5); assert.ok(c.secondsLeft >= 0 && c.remaining >= 0 && c.secondsLeft <= prev + 1e-12, 'bad at ' + i); prev = c.secondsLeft;
  }
  // Source name is 'transition' (not 'transfer'); remaining is per-phase and resets at boundaries by design.
  assert.deepEqual(order, ['auto', 'transition', 'teleop', 'endgame', 'over']);
});
test('ScoreKeeper: totals equal the sum of the POINTS table', () => {
  const s = new ScoreKeeper(); const kinds = Object.keys(POINTS); for (const k of kinds) s.addPoints(k, 'red');
  const expected = kinds.reduce((a, k) => a + POINTS[k], 0);
  assert.equal(s.total('red'), expected); assert.equal(s.total('blue'), 0); assert.equal(s.history.reduce((a, h) => a + h.points, 0), expected);
});
// FAILING-BY-DESIGN FINDING: none; all cases pass on current driver.js. Ran: node --test tests_js/h_integrator_t12.test.mjs. Result: 7 tests, 7 pass, 0 fail (run once; 43 lines).
