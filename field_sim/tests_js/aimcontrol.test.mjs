import test from 'node:test';
import assert from 'node:assert/strict';
import {
  AIM_MODES, ShooterPlant, ManualAim, pickEquivalentTurretAngle, aimInputFromKeys,
  aimInputFromGamepad, modeCommand, describeMode,
} from '../web/js/aimcontrol.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b}`);

test('AIM_MODES list', () => assert.deepEqual(AIM_MODES, ['auto', 'assist', 'manual', 'free']));

test('turret rate limit', () => {
  const p = new ShooterPlant();
  p.step(0.01, { turretDeg: 170 });
  near(p.state.turretDeg, 5.4);
});

test('turret converges without overshoot', () => {
  const p = new ShooterPlant();
  for (let i = 0; i < 300; i++) { p.step(0.01, { turretDeg: 90 }); assert.ok(p.state.turretDeg <= 90 + 1e-9); }
  near(p.state.turretDeg, 90, 0.5);
});

test('turret hard limit with huge dt', () => {
  const p = new ShooterPlant({ turretMinDeg: -90, turretMaxDeg: 90 });
  const r = p.step(100, { turretDeg: 500 });
  assert.equal(p.state.turretDeg, 90);
  assert.equal(r.atTurretLimit, 'max');
  p.step(100, { turretDeg: -500 });
  assert.equal(p.state.turretDeg, -90);
  assert.equal(p.atLimit, 'min');
});

test('turret null holds', () => {
  const p = new ShooterPlant();
  p.state.turretDeg = 33;
  p.step(0.1, { turretDeg: null });
  assert.equal(p.state.turretDeg, 33);
});

test('equivalent angle chooses closest', () => {
  assert.equal(pickEquivalentTurretAngle(170, -170, -270, 270), -190);
  assert.equal(pickEquivalentTurretAngle(10, 0, -180, 180), 10);
});

test('equivalent angle impossible', () => {
  assert.equal(pickEquivalentTurretAngle(200, 0, -90, 90), null);
  assert.equal(pickEquivalentTurretAngle(NaN, 0, -180, 180), null);
});

test('hood rate limit and clamp', () => {
  const p = new ShooterPlant();
  p.step(0.1, { hoodDeg: 60 });
  near(p.state.hoodDeg, 25 + 12);
  const r = p.step(100, { hoodDeg: 200 });
  assert.equal(p.state.hoodDeg, 65);
  assert.equal(r.atHoodLimit, 'max');
  p.step(100, { hoodDeg: -5 });
  assert.equal(p.state.hoodDeg, 25);
});

test('flywheel lag converges', () => {
  const p = new ShooterPlant();
  p.step(0.35, { rpm: 3000 });
  near(p.state.rpm, 3000 * (1 - Math.exp(-1)), 1e-6);
  for (let i = 0; i < 500; i++) p.step(0.02, { rpm: 3000 });
  near(p.state.rpm, 3000, 1);
});

test('rpm clamp', () => {
  const p = new ShooterPlant();
  for (let i = 0; i < 1000; i++) p.step(0.05, { rpm: 99999 });
  assert.ok(p.state.rpm <= 6000);
  for (let i = 0; i < 1000; i++) p.step(0.05, { rpm: -50 });
  assert.ok(p.state.rpm >= 0);
});

test('kick dip', () => {
  const p = new ShooterPlant();
  p.state.rpm = 3000;
  p.kick();
  near(p.state.rpm, 3000 * 0.94);
});

test('atSetpoint', () => {
  const p = new ShooterPlant();
  p.state = { turretDeg: 10, hoodDeg: 40, rpm: 3000 };
  const r = p.atSetpoint({ turretDeg: 10.5, hoodDeg: 45, rpm: 3020 }, { turretDeg: 1, hoodDeg: 1, rpm: 50 });
  assert.deepEqual(r, { turret: true, hood: false, rpm: true });
});

test('manual integration with fine mode', () => {
  const m = new ManualAim();
  m.update(1, { turretAxis: 1 });
  near(m.state.turretDeg, 120);
  m.update(1, { turretAxis: -1, fine: true });
  near(m.state.turretDeg, 90);
  m.update(1, { hoodAxis: 1 });
  near(m.state.hoodDeg, 55);
  m.update(1, { rpmAxis: 1 });
  near(m.state.rpm, 1500);
  m.update(1, { rpmAxis: 1, fine: true });
  near(m.state.rpm, 1875);
  for (let i = 0; i < 10; i++) m.update(1, { turretAxis: 1, hoodAxis: 1, rpmAxis: 1 });
  assert.equal(m.state.turretDeg, 180);
  assert.equal(m.state.hoodDeg, 65);
  assert.equal(m.state.rpm, 6000);
});

test('manual presets and cycle', () => {
  const m = new ManualAim();
  m.update(0.01, { rpmPreset: 'high' });
  assert.equal(m.state.rpm, 4200);
  m.update(0.01, { rpmPreset: 'stop' });
  assert.equal(m.state.rpm, 0);
  m.update(0.01, { rpmPreset: 'low' });
  assert.equal(m.state.rpm, 2200);
  const c = new ManualAim({ presets: { low: 1000 } });
  c.update(0.01, { rpmPreset: 'cycle' });
  assert.equal(c.state.rpm, 1000);
  c.update(0.01, { rpmPreset: 'cycle' }); // held, no advance
  assert.equal(c.state.rpm, 1000);
  c.update(0.01, { rpmPreset: null });
  c.update(0.01, { rpmPreset: 'cycle' });
  assert.equal(c.state.rpm, 3200);
});

test('snapToAuto only on rising edge', () => {
  const m = new ManualAim();
  const sol = { ok: true, turretDeg: 45, elevDeg: 50, rpm: 3500 };
  m.update(0.01, { snapToAuto: true, sol });
  assert.equal(m.state.turretDeg, 45);
  assert.equal(m.state.hoodDeg, 50);
  assert.equal(m.state.rpm, 3500);
  m.update(0.01, { snapToAuto: true, sol: { ...sol, turretDeg: -45 } });
  assert.equal(m.state.turretDeg, 45);
  m.update(0.01, { snapToAuto: false, sol });
  m.update(0.01, { snapToAuto: true, sol: { ...sol, turretDeg: -45, elevDeg: 90 } });
  assert.equal(m.state.turretDeg, -45);
  assert.equal(m.state.hoodDeg, 65);
});

test('key mapping', () => {
  let i = aimInputFromKeys(new Set(['arrowleft', 'arrowup', ']', 'shift', 'x']));
  assert.deepEqual(i, { turretAxis: 1, hoodAxis: 1, rpmAxis: 1, fine: true, rpmPreset: null, snapToAuto: true });
  i = aimInputFromKeys(new Set(['ArrowRight', 'ArrowDown', '[', 'z']));
  assert.equal(i.turretAxis, -1);
  assert.equal(i.hoodAxis, -1);
  assert.equal(i.rpmAxis, -1);
  assert.equal(i.rpmPreset, 'stop');
  assert.equal(aimInputFromKeys(new Set(['arrowleft', 'arrowright'])).turretAxis, 0);
  assert.equal(aimInputFromKeys(new Set(['c'])).rpmPreset, 'cycle');
  assert.equal(aimInputFromKeys(new Set()).fine, false);
});

test('gamepad mapping, null pad, deadzone', () => {
  const zero = { turretAxis: 0, hoodAxis: 0, rpmAxis: 0, fine: false, rpmPreset: null, snapToAuto: false };
  assert.deepEqual(aimInputFromGamepad(null), zero);
  const btn = (...on) => Array.from({ length: 17 }, (_, k) => ({ pressed: on.includes(k), value: on.includes(k) ? 1 : 0 }));
  assert.deepEqual(aimInputFromGamepad({ axes: [0.9, 0.9, 0.05, -0.09], buttons: btn() }), zero);
  const i = aimInputFromGamepad({ axes: [0, 0, 0.5, -1], buttons: btn(12, 2, 4, 1) });
  assert.equal(i.turretAxis, -0.5);
  assert.equal(i.hoodAxis, 1);
  assert.equal(i.rpmAxis, 1);
  assert.equal(i.snapToAuto, true);
  assert.equal(i.fine, true);
  assert.equal(i.rpmPreset, 'stop');
  assert.equal(aimInputFromGamepad({ axes: [], buttons: btn(13) }).rpmAxis, -1);
});

test('modeCommand auto, no-solution hold', () => {
  const man = new ManualAim();
  const sol = { ok: true, turretDeg: 190, elevDeg: 80, rpm: 9000 };
  const c = modeCommand('auto', sol, man, undefined, { turretDeg: 0, hoodDeg: 40, rpm: 3000 });
  assert.equal(c.turretDeg, -170);
  assert.equal(c.hoodDeg, 65);
  assert.equal(c.rpm, 6000);
  const h = modeCommand('auto', { ok: false }, man, undefined, { turretDeg: 12, hoodDeg: 40, rpm: 3000 });
  assert.deepEqual(h, { turretDeg: 12, hoodDeg: 40, rpm: 3000, label: 'no solution' });
});

test('modeCommand assist/manual/free', () => {
  const man = new ManualAim();
  man.state = { turretDeg: 20, hoodDeg: 45, rpm: 8000 };
  const a = modeCommand('assist', { ok: true, turretDeg: 0, elevDeg: 30, rpm: 1 }, man);
  assert.deepEqual([a.turretDeg, a.hoodDeg, a.rpm], [20, 45, 6000]);
  assert.equal(modeCommand('manual', null, man).label, 'manual');
  assert.equal(modeCommand('free', null, man).rpm, 8000);
  man.state.rpm = 20000;
  assert.equal(modeCommand('free', null, man).rpm, 9000);
});

test('describeMode', () => {
  for (const m of AIM_MODES) {
    const d = describeMode(m);
    assert.ok(d.label && d.summary && Array.isArray(d.keys));
  }
});
