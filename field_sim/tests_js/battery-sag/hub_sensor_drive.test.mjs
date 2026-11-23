import test from 'node:test';
import assert from 'node:assert/strict';
import { HubBrownout, portOvercurrent, hubLoads } from '../../web/js/physics/battery-sag/hub.js';
import { VoltageSensor, compensatedPower, mulberry32 } from '../../web/js/physics/battery-sag/sensor.js';
import { mecanumDuties, wheelLinearSpeeds, wheelMotorOmegas, driveLimits } from '../../web/js/physics/battery-sag/drive_adapter.js';
import { DRIVE_ELEC, SENSOR, HUB } from '../../web/js/physics/battery-sag/params.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b} (tol ${e})`);

// ---------------- hub ----------------
test('hub: healthy voltage keeps mode ok and outputs enabled', () => {
  const h = new HubBrownout();
  h.step(12, 0.01);
  assert.equal(h.state.mode, 'ok');
  assert.equal(h.state.outputsEnabled, true);
  assert.equal(h.state.servoRailOk, true);
  assert.equal(h.state.ledFault, 'none');
});

test('hub: 5.0 V for 0.01 s resets with the expected events', () => {
  const h = new HubBrownout();
  h.step(5.0, 0.01);
  assert.equal(h.state.mode, 'reset');
  assert.equal(h.state.outputsEnabled, false);
  assert.equal(h.state.resetCount, 1);
  const ev = h.consumeEvents();
  for (const e of ['hubReset', 'encodersCleared', 'pinpointLost']) assert.ok(ev.includes(e), e);
  assert.deepEqual(h.consumeEvents(), []);
});

test('hub: a dip shorter than the dwell does not reset', () => {
  const h = new HubBrownout();
  h.step(5.0, 0.0005);
  h.step(12, 0.01);
  assert.equal(h.state.mode, 'ok');
  assert.equal(h.state.resetCount, 0);
});

test('hub: recovers to ok after V >= 7 plus the reboot time', () => {
  const h = new HubBrownout();
  h.step(5.0, 0.01);
  h.consumeEvents();
  h.step(8.0, 0.01);
  assert.equal(h.state.mode, 'rebooting');
  assert.equal(h.state.outputsEnabled, false);
  for (let i = 0; i < 10; i++) h.step(12, 0.1);
  assert.equal(h.state.mode, 'ok');
  assert.equal(h.state.outputsEnabled, true);
  assert.ok(h.consumeEvents().includes('recovered'));
});

test('hub: stays in reset while V remains below the recover threshold', () => {
  const h = new HubBrownout();
  h.step(5.0, 0.01);
  for (let i = 0; i < 20; i++) h.step(6.0, 0.1);
  assert.equal(h.state.mode, 'reset');
});

test('hub: low-battery LED fault below 7 V latches, clears above 7 V', () => {
  const h = new HubBrownout();
  h.step(6.5, 0.001);
  assert.equal(h.state.lowBattery, true);
  assert.equal(h.state.ledFault, 'blinkOrange');
  assert.equal(h.state.mode, 'ok');
  h.step(9, 0.01);
  assert.equal(h.state.lowBattery, false);
  assert.equal(h.state.ledFault, 'none');
});

test('hub: below8V informational flag and servo rail', () => {
  const h = new HubBrownout();
  h.step(7.5, 0.01);
  assert.equal(h.state.below8V, true);
  assert.equal(h.state.servoRailOk, true);
  h.step(5.9, 0.01);
  assert.equal(h.state.servoRailOk, false);
});

test('hub: dead mode below 1 V (blown fuse) and recovery via rebooting', () => {
  const h = new HubBrownout();
  h.step(0, 0.01);
  assert.equal(h.state.mode, 'dead');
  assert.equal(h.state.outputsEnabled, false);
  h.step(12, 0.01);
  assert.equal(h.state.mode, 'rebooting');
});

test('portOvercurrent flags |I| above 10 A', () => {
  assert.deepEqual(portOvercurrent([5, 10.5, -11, 10]), [false, true, true, false]);
  assert.equal(portOvercurrent([HUB.motorPortContA])[0], false);
});

test('hubLoads returns two constant-power loads', () => {
  const l = hubLoads({ servosMoving: 2, servosIdle: 3 });
  assert.equal(l.length, 2);
  assert.ok(l.every(x => x.kind === 'power'));
  near(l[0].watts, HUB.electronicsW + HUB.pinpointW, 1e-9);
  near(l[1].watts, (2 * HUB.servoMovingA + 3 * HUB.servoIdleA) * HUB.railV / HUB.railEff, 1e-9);
});

// ---------------- sensor ----------------
test('sensor: quantises to 0.01 V', () => {
  const s = new VoltageSensor({ ...SENSOR, noiseSigmaV: 0, lpTauS: 0 });
  near(s.sample(12.3456, 0.01), 12.35, 1e-9);
  near(s.sample(11.994, 0.01), 11.99, 1e-9);
});

test('sensor: seeded noise is deterministic and actually noisy', () => {
  const p = { quantV: 0.001, noiseSigmaV: 0.05, lpTauS: 0, seed: 7 };
  const a = new VoltageSensor(p), b = new VoltageSensor(p);
  const xs = [], ys = [];
  for (let i = 0; i < 50; i++) { xs.push(a.sample(12, 0.01)); ys.push(b.sample(12, 0.01)); }
  assert.deepEqual(xs, ys);
  assert.ok(new Set(xs).size > 10);
  const q = new VoltageSensor({ ...p, seed: 8 });
  const zs = []; for (let i = 0; i < 50; i++) zs.push(q.sample(12, 0.01));
  assert.notDeepEqual(xs, zs);
});

test('sensor: low-pass lags a step change', () => {
  const s = new VoltageSensor({ quantV: 0.0001, noiseSigmaV: 0, lpTauS: 0.1, seed: 1 });
  s.sample(12, 0.01);
  const v = s.sample(8, 0.01);
  assert.ok(v > 8 && v < 12);
});

test('mulberry32 is deterministic and in [0,1)', () => {
  const a = mulberry32(5), b = mulberry32(5);
  for (let i = 0; i < 20; i++) { const x = a(); assert.equal(x, b()); assert.ok(x >= 0 && x < 1); }
});

test('compensatedPower(0.5, 10) = 0.6 and clamps to +-1', () => {
  near(compensatedPower(0.5, 10), 0.6, 1e-12);
  assert.equal(compensatedPower(0.9, 8), 1);
  assert.equal(compensatedPower(-0.9, 8), -1);
});

// ---------------- drive adapter ----------------
test('mecanumDuties: signs for forward, strafe, turn', () => {
  assert.deepEqual(mecanumDuties({ fwd: 1 }), [1, 1, 1, 1]);
  assert.deepEqual(mecanumDuties({ strafe: 1 }), [-1, 1, 1, -1]);
  assert.deepEqual(mecanumDuties({ turn: 1 }), [-1, 1, -1, 1]);
  assert.deepEqual(mecanumDuties({}), [0, 0, 0, 0]);
});

test('mecanumDuties: normalised so max |duty| <= 1, ratios kept', () => {
  const d = mecanumDuties({ fwd: 1, strafe: 1, turn: 1 });
  assert.ok(Math.max(...d.map(Math.abs)) <= 1 + 1e-12);
  near(d[1], 1, 1e-12);
  near(d[0], -1 / 3, 1e-12);
  const half = mecanumDuties({ fwd: 0.4 });
  near(half[0], 0.4, 1e-12);
});

test('wheelLinearSpeeds and wheelMotorOmegas', () => {
  const v = wheelLinearSpeeds([10, 0, 0]);
  v.forEach(x => near(x, 10, 1e-9));
  const k = DRIVE_ELEC.halfTrackIn + DRIVE_ELEC.halfBaseIn;
  const t = wheelLinearSpeeds([0, 0, 1]);
  near(t[0], -k, 1e-9); near(t[1], k, 1e-9); near(t[2], -k, 1e-9); near(t[3], k, 1e-9);
  const w = wheelMotorOmegas([10, 0, 0]);
  w.forEach(x => near(x, 10 / DRIVE_ELEC.wheelRadiusIn * DRIVE_ELEC.gearRatio, 1e-9));
});

const NOM = { maxSpeed: 60, maxAccel: 120, maxTurn: 5 };

test('driveLimits at 12 V with no compensation gives maxSpeed 60', () => {
  const l = driveLimits(12, [0, 0, 0], NOM, true, { ...DRIVE_ELEC, compensation: 'none' });
  near(l.maxSpeed, 60, 1e-9);
  near(l.maxTurn, 5, 1e-9);
  near(l.speedScale, 1, 1e-12);
});

test('driveLimits at 10.8 V with no compensation gives maxSpeed 54', () => {
  const l = driveLimits(10.8, [0, 0, 0], NOM, true, { ...DRIVE_ELEC, compensation: 'none' });
  near(l.maxSpeed, 54, 1e-9);
  near(l.maxTurn, 4.5, 1e-9);
});

test('driveLimits with nominal12 compensation is capped at 60 at 13.5 V', () => {
  const l = driveLimits(13.5, [0, 0, 0], NOM, true, { ...DRIVE_ELEC, compensation: 'nominal12' });
  near(l.maxSpeed, 60, 1e-9);
  const lo = driveLimits(10.8, [0, 0, 0], NOM, true, { ...DRIVE_ELEC, compensation: 'nominal12' });
  near(lo.maxSpeed, 54, 1e-9);
});

test('driveLimits: disabled outputs give zero speed and turn', () => {
  const l = driveLimits(12, [0, 0, 0], NOM, false);
  assert.equal(l.maxSpeed, 0);
  assert.equal(l.maxTurn, 0);
});

test('driveLimits: accel bound only reduces, decel defaults to 1.6x nominal accel', () => {
  const l = driveLimits(12, [0, 0, 0], NOM, true);
  assert.ok(l.maxAccel <= NOM.maxAccel + 1e-9 && l.maxAccel > 0);
  near(l.maxDecel, NOM.maxAccel * 1.6, 1e-9);
  const lowV = driveLimits(8, [40, 0, 0], NOM, true);
  assert.ok(lowV.maxAccel <= l.maxAccel + 1e-9);
});
