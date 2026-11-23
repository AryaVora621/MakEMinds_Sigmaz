import test from 'node:test';
import assert from 'node:assert/strict';
import { IN_PER_M } from '../../web/js/physics/motor-model/params.js';
import { MotorMecanumDrive, mecanumJacobian, wheelPowers } from '../../web/js/physics/motor-model/mecanum.js';
import { MecanumDrive } from '../../web/js/driver.js';

const mkState = (h = 0) => ({ x: 0, y: 0, h, bodyV: [0, 0, 0] });
const run = (d, cmd, secs, st, opts = {}, dt = 0.02) => {
  let r; for (let i = 0; i < Math.round(secs / dt); i++) r = d.step(cmd, dt, st, opts); return r;
};

test('jacobian and wheel power normalisation', () => {
  const J = mecanumJacobian(0.15, 0.16);
  assert.deepEqual(J[0].slice(0, 2), [1, -1]);
  assert.ok(Math.abs(J[0][2] + 0.31) < 1e-12);
  for (const x of wheelPowers(1, 1, 1)) assert.ok(Math.abs(x) <= 1 + 1e-12);
});

test('return shape is {vx, vy, om}', () => {
  const r = new MotorMecanumDrive().step({ fwd: 1, strafe: 0, turn: 0 }, 0.02, mkState());
  assert.deepEqual(Object.keys(r).sort(), ['om', 'vx', 'vy']);
  for (const k of ['vx', 'vy', 'om']) assert.ok(Number.isFinite(r[k]));
});

test('full forward approaches free speed within 3% after 3 s', () => {
  const d = new MotorMecanumDrive(), st = mkState();
  run(d, { fwd: 1, strafe: 0, turn: 0 }, 3, st);
  const free = d.freeSpeedInS();
  assert.ok(Math.abs(st.bodyV[0] - free) / free < 0.03, `${st.bodyV[0]} vs ${free}`);
  assert.ok(Math.abs(st.bodyV[1]) < 0.5);
});

test('initial acceleration is traction-limited at about 4.85 m/s^2', () => {
  const d = new MotorMecanumDrive(), st = mkState();
  d.step({ fwd: 1, strafe: 0, turn: 0 }, 0.01, st);
  const a = st.bodyV[0] / IN_PER_M / 0.01;
  assert.ok(Math.abs(a - 4.85) / 4.85 < 0.1, `a ${a}`);
});

test('acceleration tapers as speed rises (measured at 85% of free speed; the cap still binds at 50%)', () => {
  const d = new MotorMecanumDrive(), st = mkState();
  d.step({ fwd: 1, strafe: 0, turn: 0 }, 0.01, st);
  const a0 = st.bodyV[0] / IN_PER_M / 0.01;
  const free = d.freeSpeedInS();
  while (st.bodyV[0] < 0.85 * free) d.step({ fwd: 1, strafe: 0, turn: 0 }, 0.01, st);
  const v0 = st.bodyV[0];
  d.step({ fwd: 1, strafe: 0, turn: 0 }, 0.01, st);
  const a1 = (st.bodyV[0] - v0) / IN_PER_M / 0.01;
  assert.ok(a1 < a0, `${a1} !< ${a0}`);
});

test('strafe top speed is between lateralEff*forward and forward', () => {
  // lateralEff scales the force, but with no drag the terminal speed is set by the wheel free speed.
  const f = new MotorMecanumDrive(), sf = mkState();
  run(f, { fwd: 1, strafe: 0, turn: 0 }, 3, sf);
  const s = new MotorMecanumDrive(), ss = mkState();
  run(s, { fwd: 0, strafe: 1, turn: 0 }, 3, ss);
  const ratio = Math.abs(ss.bodyV[1]) / sf.bodyV[0];
  assert.ok(ratio >= 0.85 * 0.97 && ratio <= 1.01, `ratio ${ratio}`);
  assert.ok(ss.bodyV[1] > 0);
});

test('field-centric direction matches driver.js MecanumDrive', () => {
  const cmds = [{ fwd: 1, strafe: 0, turn: 0 }, { fwd: 0, strafe: 1, turn: 0 }, { fwd: 0, strafe: -1, turn: 0 }];
  for (const alliance of ['red', 'blue']) for (const cmd of cmds) {
    const a = new MotorMecanumDrive({ alliance }), sa = mkState(0.4);
    const b = new MecanumDrive({ alliance }), sb = mkState(0.4);
    const ra = run(a, cmd, 1, sa, { fieldCentric: true });
    const rb = run(b, cmd, 1, sb, { fieldCentric: true });
    const dot = (ra.vx * rb.vx + ra.vy * rb.vy) / (Math.hypot(ra.vx, ra.vy) * Math.hypot(rb.vx, rb.vy));
    assert.ok(dot > 0.99, `${alliance} ${JSON.stringify(cmd)} dot ${dot}`);
  }
});

test('BRAKE decelerates faster than FLOAT', () => {
  const speed = (zeroPower) => {
    const d = new MotorMecanumDrive({ zeroPower }), st = mkState();
    run(d, { fwd: 1, strafe: 0, turn: 0 }, 1.5, st);
    run(d, { fwd: 0, strafe: 0, turn: 0 }, 0.3, st);
    return st.bodyV[0];
  };
  assert.ok(speed('BRAKE') < speed('FLOAT'));
});

test('syncBodyVelocity writes world velocity into the body frame', () => {
  const d = new MotorMecanumDrive(), st = mkState(Math.PI / 2);
  d.syncBodyVelocity(st, 0, 10, 1);   // world +y at heading +y = body forward
  assert.ok(Math.abs(st.bodyV[0] - 10) < 1e-9);
  assert.ok(Math.abs(st.bodyV[1]) < 1e-9);
  assert.equal(st.bodyV[2], 1);
});
