// MecanumDrive dynamic-model integration tests (node:test).
import test from 'node:test';
import assert from 'node:assert/strict';
import { MecanumDrive, MecanumDynamics, resolveParams } from '../../web/js/driver.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} expected ${b} +-${tol}, got ${a}`);
const DT = 1 / 60;
const run = (d, cmd, state, seconds, opts) => {
  let r; for (let i = 0; i < Math.round(seconds / DT); i++) r = d.step(cmd, DT, state, opts); return r;
};

test('legacy default model is still kinematic and has no dynamics object', () => {
  const d = new MecanumDrive();
  assert.equal(d.model, 'kinematic'); assert.equal(d._dyn, null); assert.equal(d.lastTelemetry, null);
  const st = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  run(d, { fwd: 1 }, st, 2);
  near(st.bodyV[0], 60, 1e-9);                              // legacy maxSpeed, exact
  assert.equal(d.lastTelemetry, null);
});
test('re-exports MecanumDynamics and resolveParams', () => {
  assert.equal(typeof MecanumDynamics, 'function'); assert.equal(typeof resolveParams, 'function');
});
test('unknown model throws', () => assert.throws(() => new MecanumDrive({ model: 'magic' })));

test('dynamic model reaches speed and drives world +x at h=0 (robot-centric)', () => {
  const d = new MecanumDrive({ model: 'dynamic' });
  assert.equal(d.model, 'dynamic');
  const st = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  const r = run(d, { fwd: 1 }, st, 4);
  assert.ok(r.vx > 55 && r.vx < 66, 'vx ' + r.vx);
  near(r.vy, 0, 1e-3); near(r.om, 0, 1e-6);
});
test('dynamics overrides option is passed through (lower battery -> slower)', () => {
  const slow = new MecanumDrive({ model: 'dynamic', dynamics: { batteryV: 6 } });
  const st = { h: 0, bodyV: [0, 0, 0] };
  assert.ok(run(slow, { fwd: 1 }, st, 4).vx < 40);
});
test('field-centric red: stick up -> world +x; blue -> world -x', () => {
  for (const [alliance, sign] of [['red', 1], ['blue', -1]]) {
    const d = new MecanumDrive({ model: 'dynamic' });
    // At h=0 the world direction is the body forward axis (full speed); at h=0.6 the body command is a
    // diagonal, so top speed is lower (mecanum anisotropy) and direction is only approximately +-x.
    const r0 = run(d, { fwd: 1 }, { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] }, 4, { fieldCentric: true, alliance });
    assert.ok(sign * r0.vx > 50, `${alliance} vx ${r0.vx}`);
    const r = run(new MecanumDrive({ model: 'dynamic' }), { fwd: 1 }, { x: 0, y: 0, h: 0.6, bodyV: [0, 0, 0] }, 4,
      { fieldCentric: true, alliance });
    assert.ok(sign * r.vx > 30, `${alliance} vx ${r.vx}`);
    assert.ok(Math.abs(r.vy) < 0.2 * Math.abs(r.vx), `${alliance} vy ${r.vy}`);
  }
});
test('field-centric stick-left on red -> world +y, same sign as legacy', () => {
  const dyn = new MecanumDrive({ model: 'dynamic' }), leg = new MecanumDrive();
  const a = { h: 1.0, bodyV: [0, 0, 0] }, b = { h: 1.0, bodyV: [0, 0, 0] };
  const rd = run(dyn, { strafe: 1 }, a, 4, { fieldCentric: true, alliance: 'red' });
  const rl = run(leg, { strafe: 1 }, b, 4, { fieldCentric: true, alliance: 'red' });
  assert.ok(rd.vy > 40 && rl.vy > 40, `dyn vy ${rd.vy}, legacy vy ${rl.vy}`);
  assert.ok(Math.abs(rd.vx) < 0.2 * rd.vy, `dyn vx ${rd.vx}`);   // legacy also skews slightly (strafeFactor), so only the dynamic direction is checked
});
test('world velocity equals bodyV rotated by h', () => {
  const d = new MecanumDrive({ model: 'dynamic' });
  const st = { x: 0, y: 0, h: 0.7, bodyV: [0, 0, 0] };
  const r = run(d, { fwd: 0.6, strafe: 0.3, turn: 0 }, st, 0.5);
  const c = Math.cos(0.7), s = Math.sin(0.7);
  near(r.vx, c * st.bodyV[0] - s * st.bodyV[1], 1e-9); near(r.vy, s * st.bodyV[0] + c * st.bodyV[1], 1e-9);
  near(r.om, st.bodyV[2], 1e-12);
});
test('stick magnitude > 1 is normalised to the unit disc', () => {
  const a = new MecanumDrive({ model: 'dynamic' }), b = new MecanumDrive({ model: 'dynamic' });
  const s1 = { h: 0, bodyV: [0, 0, 0] }, s2 = { h: 0, bodyV: [0, 0, 0] };
  run(a, { fwd: 1, strafe: 1 }, s1, 0.3);
  run(b, { fwd: Math.SQRT1_2, strafe: Math.SQRT1_2 }, s2, 0.3);
  near(s1.bodyV[0], s2.bodyV[0], 1e-9); near(s1.bodyV[1], s2.bodyV[1], 1e-9);
});
test('slip = 1 lowers the initial acceleration', () => {
  const free = new MecanumDrive({ model: 'dynamic', slip: 0 }), slick = new MecanumDrive({ model: 'dynamic', slip: 1 });
  const a = { h: 0, bodyV: [0, 0, 0] }, b = { h: 0, bodyV: [0, 0, 0] };
  run(free, { fwd: 1 }, a, 0.1); run(slick, { fwd: 1 }, b, 0.1);
  assert.ok(b.bodyV[0] < a.bodyV[0] * 0.6, `${b.bodyV[0]} vs ${a.bodyV[0]}`);
});
test('step() does not move the pose; lastTelemetry has 4 wheels', () => {
  const d = new MecanumDrive({ model: 'dynamic' });
  const st = { x: 12, y: 34, h: 0.4, bodyV: [0, 0, 0] };
  run(d, { fwd: 1, turn: 0.5 }, st, 0.5);
  assert.equal(st.x, 12); assert.equal(st.y, 34); assert.equal(st.h, 0.4);
  assert.equal(d.lastTelemetry.length, 4);
  assert.ok('torqueNm' in d.lastTelemetry[0]);
});
test('turn command spins up counter-clockwise', () => {
  const d = new MecanumDrive({ model: 'dynamic' });
  const st = { h: 0, bodyV: [0, 0, 0] };
  assert.ok(run(d, { turn: 1 }, st, 3).om > 3.5);
});
