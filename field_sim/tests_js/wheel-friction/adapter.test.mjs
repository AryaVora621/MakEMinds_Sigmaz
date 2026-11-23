import test from 'node:test';
import assert from 'node:assert/strict';
import { FrictionMecanumDrive } from '../../web/js/physics/wheel-friction/index.js';
import { MecanumDrive } from '../../web/js/driver.js';

const run = (drive, cmd, h, seconds, opts = {}) => {
  const state = { x: 0, y: 0, h, bodyV: [0, 0, 0] };
  let r;
  for (let i = 0; i < Math.round(seconds * 60); i++) r = drive.step(cmd, 1 / 60, state, opts);
  return { r, state };
};

test('step returns finite values', () => {
  const { r } = run(new FrictionMecanumDrive(), { fwd: 1, strafe: 0.3, turn: -0.2 }, 0.4, 0.5);
  for (const k of ['vx', 'vy', 'om']) assert.ok(Number.isFinite(r[k]));
});

test('forward and strafe directions in the world frame', () => {
  let { r } = run(new FrictionMecanumDrive(), { fwd: 1, strafe: 0, turn: 0 }, 0, 1);
  assert.ok(r.vx > 20 && Math.abs(r.vy) < 1, `h=0 fwd ${r.vx},${r.vy}`);
  ({ r } = run(new FrictionMecanumDrive(), { fwd: 1, strafe: 0, turn: 0 }, Math.PI / 2, 1));
  assert.ok(r.vy > 20 && Math.abs(r.vx) < 1, `h=pi/2 fwd ${r.vx},${r.vy}`);
  ({ r } = run(new FrictionMecanumDrive(), { fwd: 0, strafe: 1, turn: 0 }, 0, 1));
  assert.ok(r.vy > 20 && Math.abs(r.vx) < 1, `h=0 strafe ${r.vx},${r.vy}`);
});

test('field-centric direction signs match MecanumDrive', () => {
  for (const alliance of ['red', 'blue']) {
    for (const cmd of [{ fwd: 1, strafe: 0, turn: 0 }, { fwd: 0, strafe: 1, turn: 0 }]) {
      const a = run(new FrictionMecanumDrive(), cmd, 0, 1, { fieldCentric: true, alliance }).r;
      const b = run(new MecanumDrive(), cmd, 0, 1, { fieldCentric: true, alliance }).r;
      // compare only components clearly non-zero (>2 in/s) in either model; the rest is numerical noise
      for (const k of ['vx', 'vy']) {
        if (Math.abs(a[k]) > 2 || Math.abs(b[k]) > 2) assert.equal(Math.sign(a[k]), Math.sign(b[k]), `${alliance} ${JSON.stringify(cmd)} ${k}: ${a[k]} vs ${b[k]}`);
      }
    }
  }
});

test('state.bodyV is in in/s and external zeroing restarts from about zero', () => {
  const d = new FrictionMecanumDrive();
  const state = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  for (let i = 0; i < 60; i++) d.step({ fwd: 1, strafe: 0, turn: 0 }, 1 / 60, state);
  assert.ok(state.bodyV[0] > 20 && state.bodyV[0] < 80, `bodyV ${state.bodyV[0]}`);
  state.bodyV = [0, 0, 0];
  const r = d.step({ fwd: 0, strafe: 0, turn: 0 }, 1 / 60, state);
  assert.ok(Math.hypot(r.vx, r.vy) < 5, `after zero ${r.vx}`);
});

test('slip = 1 lowers launch accel', () => {
  const v = (s) => {
    const d = new FrictionMecanumDrive({ slip: s });
    const state = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
    let r;
    for (let i = 0; i < 15; i++) r = d.step({ fwd: 1, strafe: 0, turn: 0 }, 1 / 60, state);
    return r.vx;
  };
  assert.ok(v(1) < v(0) * 0.8);
});
