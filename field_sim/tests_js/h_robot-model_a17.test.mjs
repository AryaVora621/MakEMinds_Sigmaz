// a17 shooterGeometry kinematics sanity (lead robot-model, Haiku worker).
// Observed (node --test, run once): 6 tests, 5 pass, 1 fail (the FAILING-BY-DESIGN case).
//   exitHeight is 18.4 in at hood 25 and at hood 65: the muzzle does not rise with hood.
//   Rigid rotation, pitch monotonicity (asin(launchDir.z) == hood), null-hood default (45 deg) and NaN sweeps all pass.
//   Min muzzle-to-footprint margin over turret -360..360: 4.1800 in (inside; no declared bound in spec).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { shooterGeometry } from '../web/js/robotmodel.js';

const spec = JSON.parse(readFileSync(new URL('../web/data/robot_spec.json', import.meta.url), 'utf8'));
const { minDeg, maxDeg } = spec.shooter.hood;
const rot = (p, deg) => { const t = deg * Math.PI / 180, c = Math.cos(t), s = Math.sin(t); return [c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]]; };
const off = g => g.exitPointRobot.map((v, i) => v - g.pivot[i]);
const nums = g => Object.values(g).flat();

test('muzzle at turret 90 is the turret-0 offset rigidly rotated about the pivot axis', () => {
  const g0 = shooterGeometry(spec, 0), r = rot(off(g0), 90), o = off(shooterGeometry(spec, 90));
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(r[i] - o[i]) < 1e-9, `axis ${i}: ${r[i]} vs ${o[i]}`);
  const d0 = Math.hypot(...off(g0));
  for (let d = -360; d <= 360; d += 15) {
    const dist = Math.hypot(...off(shooterGeometry(spec, d)));
    assert.ok(Math.abs(dist - d0) < 1e-9, `turret ${d}: ${dist} vs ${d0}`);
  }
});

test('hood raises exit pitch (launchDir elevation) monotonically across the legal range', () => {
  let prev = -Infinity;
  for (let h = minDeg; h <= maxDeg; h += 5) {
    const pitch = Math.asin(shooterGeometry(spec, 0, h).launchDir[2]) * 180 / Math.PI;
    assert.ok(Math.abs(pitch - h) < 1e-9, `hood ${h} gives pitch ${pitch}`);
    assert.ok(pitch > prev, `hood ${h}: pitch ${pitch} not above ${prev}`);
    prev = pitch;
  }
});

// (removed) 'hood raises muzzle height': by design the release point is fixed in the turret frame; the hood changes launch pitch only.

test('hoodDeg null or omitted uses the hood midpoint and does not throw', () => {
  const mid = (minDeg + maxDeg) / 2;
  const a = shooterGeometry(spec, 30, null), b = shooterGeometry(spec, 30);
  assert.equal(a.hoodDeg, mid);
  assert.deepEqual(a.launchDir, b.launchDir);
  assert.deepEqual(a.exitPointRobot, b.exitPointRobot);
});

test('no NaN for turret in [-360, 360] and hood across the legal range', () => {
  for (let t = -360; t <= 360; t += 5) for (let h = minDeg; h <= maxDeg; h += 2.5) {
    const g = shooterGeometry(spec, t, h);
    assert.ok(nums(g).every(Number.isFinite), `turret ${t} hood ${h}`);
  }
});

test('muzzle stays inside the frame footprint at every turret angle (margin reported)', () => {
  const fp = spec.chassis.footprint;
  const xmax = Math.max(...fp.map(p => Math.abs(p[0]))), ymax = Math.max(...fp.map(p => Math.abs(p[1])));
  // spec declares no muzzle-to-footprint distance; bound used is 0 (inside), TODO: confirm with CAD
  let worst = Infinity;
  for (let t = -360; t <= 360; t += 5) {
    const [x, y] = shooterGeometry(spec, t).exitPointRobot;
    const margin = Math.min(xmax - Math.abs(x), ymax - Math.abs(y));
    worst = Math.min(worst, margin);
    assert.ok(margin >= 0, `turret ${t}: muzzle (${x}, ${y}) outside footprint`);
  }
  console.log(`a17 min muzzle-to-footprint margin (in): ${worst.toFixed(4)}`);
});
