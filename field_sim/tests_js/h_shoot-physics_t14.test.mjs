// t14 (lead shoot-physics): aimcontrol.js property tests, seeded mulberry32, 10000 cases each. Expected: all pass (ran green).
// validAngles is a brute-force reference: all angles t+360k inside [min,max].
import test from 'node:test';
import assert from 'node:assert/strict';
import { ShooterPlant, pickEquivalentTurretAngle, DEFAULT_PLANT as L } from '../web/js/aimcontrol.js';
const rng = (seed) => { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = Math.imul(a ^ (a >>> 15), a | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; };
const uni = (r, lo, hi) => lo + (hi - lo) * r(), N = 10000;
const rndState = (r) => ({ turretDeg: uni(r, L.turretMinDeg, L.turretMaxDeg), hoodDeg: uni(r, L.hoodMinDeg, L.hoodMaxDeg), rpm: uni(r, 0, L.maxRpm) });
const validAngles = (t, min, max) => { const out = []; for (let k = Math.floor((min - t) / 360) - 2; k <= Math.ceil((max - t) / 360) + 2; k++) { const a = t + 360 * k; if (a >= min - 1e-9 && a <= max + 1e-9) out.push(a); } return out; };

test('pickEquivalentTurretAngle: congruent, in range, closest; null iff none exists', () => {
  const r = rng(0xA11CE);
  for (let i = 0; i < N; i++) {
    const t = uni(r, -900, 900), cur = uni(r, -1200, 1200), min = uni(r, -540, 0), max = min + uni(r, 0, 900);
    const got = pickEquivalentTurretAngle(t, cur, min, max), ctx = `t=${t} cur=${cur} min=${min} max=${max} got=${got}`;
    const valid = validAngles(t, min, max);
    if (valid.length === 0) { assert.equal(got, null, `expected null: ${ctx}`); continue; }
    assert.notEqual(got, null, `expected value: ${ctx}`);
    const q = (got - t) / 360;
    assert.ok(Math.abs(q - Math.round(q)) < 1e-9 && got >= min - 1e-9 && got <= max + 1e-9, `not congruent/in range: ${ctx}`);
    assert.ok(Math.abs(got - cur) <= Math.min(...valid.map((a) => Math.abs(a - cur))) + 1e-9, `not closest: ${ctx}`);
  }
});

test('ShooterPlant: no NaN and limits respected under random commands', () => {
  const r = rng(0xB0B), junk = [undefined, null, NaN, Infinity, -Infinity];
  const pick = (span) => (r() < 0.1 ? junk[Math.floor(r() * junk.length)] : uni(r, -span, span));
  for (let i = 0; i < N; i++) {
    const p = new ShooterPlant(); p.state = rndState(r);
    for (let s = 0; s < 20; s++) {
      const cmd = { turretDeg: pick(2000), hoodDeg: pick(2000), rpm: pick(20000) };
      if (r() < 0.05) p.kick();
      p.step(uni(r, 1e-4, 2), cmd);
      const st = p.state, ctx = `state=${JSON.stringify(st)} cmd=${JSON.stringify(cmd)}`;
      assert.ok(Number.isFinite(st.turretDeg) && Number.isFinite(st.hoodDeg) && Number.isFinite(st.rpm), `NaN: ${ctx}`);
      assert.ok(st.turretDeg >= L.turretMinDeg && st.turretDeg <= L.turretMaxDeg, `turret limit: ${ctx}`);
      assert.ok(st.hoodDeg >= L.hoodMinDeg && st.hoodDeg <= L.hoodMaxDeg, `hood limit: ${ctx}`);
      assert.ok(st.rpm >= 0 && st.rpm <= L.maxRpm, `rpm limit: ${ctx}`);
    }
  }
});

test('ShooterPlant: converges to random setpoints within a measured time for each dt', () => {
  const r = rng(0xC0FFEE), LIMIT_S = 10, TOL = { turretDeg: 0.5, hoodDeg: 0.5, rpm: 5 };
  for (const dt of [1 / 240, 1 / 60, 1 / 30, 0.1]) { let worst = 0;
    for (let c = 0; c < 200; c++) {
      const sp = rndState(r), p = new ShooterPlant(); p.state = rndState(r);
      const within = () => Object.keys(TOL).every((k) => Math.abs(p.state[k] - sp[k]) <= TOL[k]); let tConv = null;
      for (let k = 1; k <= Math.round(LIMIT_S / dt); k++) {
        p.step(dt, sp);
        if (tConv === null && within()) tConv = k * dt;
        else if (tConv !== null) assert.ok(within(), `left tolerance: dt=${dt} sp=${JSON.stringify(sp)}`);
      }
      assert.notEqual(tConv, null, `no convergence: dt=${dt} sp=${JSON.stringify(sp)} state=${JSON.stringify(p.state)}`); worst = Math.max(worst, tConv);
    }
    console.log(`t14 dt=${dt.toFixed(5)} worst convergence ${worst.toFixed(3)} s over 200 setpoints`);
    assert.ok(worst < LIMIT_S);
  }
});
