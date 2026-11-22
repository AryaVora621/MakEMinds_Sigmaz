// Regression tests for shoot-physics integration fixes (aimcontrol guards).
import test from 'node:test';
import assert from 'node:assert/strict';
import { pickEquivalentTurretAngle, ShooterPlant, modeCommand } from '../web/js/aimcontrol.js';

test('pickEquivalentTurretAngle rejects non-finite limits', () => {
  assert.equal(pickEquivalentTurretAngle(10, 0, NaN, 180), null);
  assert.equal(pickEquivalentTurretAngle(10, 0, -180, Infinity), null);
});
test('ShooterPlant.step with negative or NaN dt does not move away from target', () => {
  for (const dt of [-0.1, NaN]) {
    const p = new ShooterPlant();
    p.step(0, { turretDeg: 0, rpm: 3000 });
    const before = { t: p.state.turretDeg, r: p.state.rpm };
    p.step(dt, { turretDeg: 90, rpm: 6000 });
    assert.equal(p.state.turretDeg, before.t);
    assert.equal(p.state.rpm, before.r);
  }
});
test('modeCommand auto holds previous rpm when sol.rpm is missing', () => {
  const prev = { turretDeg: 10, hoodDeg: 40, rpm: 2500 };
  const c = modeCommand('auto', { ok: true, turretDeg: 20, elevDeg: 45 }, null, undefined, prev);
  assert.ok(Number.isFinite(c.rpm), 'rpm must be finite');
  assert.equal(c.rpm, 2500);
});
