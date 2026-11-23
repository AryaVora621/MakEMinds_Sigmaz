// HiveDynamics physics integration tests (hive-physics P3). Angles in degrees at the API, seconds, dt = 1/60 s unless noted.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { HiveDynamics, HIVE_PARAMS } from '../../web/js/hivecycle.js';
import { HIVE_PHYS } from '../../web/js/physics/hive-physics/params.js';
import { simulateTip, buildCellRecords } from '../../web/js/physics/hive-physics/tipdynamics.js';

const scene = JSON.parse(readFileSync(new URL('../../web/data/scene.json', import.meta.url), 'utf8'));
const g = scene.groups.hive_red, states = scene.constants.hives.red.states;
const DT = 1 / 60;
const make = (p) => new HiveDynamics(g, states, p ? { ...HIVE_PARAMS, ...p } : HIVE_PARAMS);
const run = (h, secs, dt = DT) => { for (let t = 0; t < secs - 1e-12; t += dt) h.step(dt); };
const feed = (h, n, type = 'pollen') => { for (let i = 0; i < n; i++) h.addScoredBall(h.upCell, null, type); };
/** Step until the state changes; returns seconds (Infinity if it never does within maxS). */
function timeToTip(h, maxS = 6, dt = 1 / 240) {
  const s0 = h.state;
  for (let t = 0; t < maxS; t += dt) { h.step(dt); if (h.state !== s0) return t + dt; }
  return Infinity;
}

test('rule+dynamic default: 8 pollen -> moving synchronously, monotone swing, exactly +30 within 2 s', () => {
  const h = make();
  feed(h, 7);
  assert.equal(h.moving, false);
  assert.equal(h.angleDeg, -30);
  h.addScoredBall('audience', null, 'pollen');
  assert.equal(h.moving, true, 'moving true synchronously after the 8th ball');
  let prev = h.angleDeg, t = 0;
  while (h.moving && t < 2.0) {
    h.step(DT); t += DT;
    assert.ok(h.angleDeg >= prev - 1e-9, `monotone ${prev} -> ${h.angleDeg}`);
    assert.ok(Number.isFinite(h.angleDeg) && h.angleDeg <= 30 + 1e-9);
    prev = h.angleDeg;
  }
  assert.equal(h.moving, false, 'finished in time: ' + t);
  assert.equal(h.angleDeg, 30);
  assert.equal(h.state, 'far_up');
  assert.equal(h.cycleStats().tips, 1);
  assert.ok(Number.isFinite(h.omegaDegS));
  assert.equal(h.cycleStats().tipModel, 'rule');
  assert.equal(h.cycleStats().motionModel, 'dynamic');
});

test('rule mode: 7 pollen never tip', () => {
  const h = make();
  feed(h, 7); run(h, 5);
  assert.equal(h.state, 'audience_up');
  assert.equal(h.moving, false);
  assert.equal(h.angleDeg, -30);
});

test('ease mode reproduces the legacy 0.6 s smoothstep trace', () => {
  const h = make({ motionModel: 'ease' });
  feed(h, 8);
  assert.equal(h.moving, true);
  let k = 0;
  while (h.moving && k < 200) {
    h.step(DT); k++;
    const t = Math.min(0.6, k * DT), u = t / 0.6;
    if (h.moving) assert.ok(Math.abs(h.angleDeg - (-30 + 60 * u * u * (3 - 2 * u))) < 1e-6, `step ${k}: ${h.angleDeg}`);
  }
  assert.equal(h.angleDeg, 30);
  assert.ok(Math.abs(k * DT - 0.6) < 2 * DT);
  assert.equal(h.cycleStats().motionModel, 'ease');
});

test('torque mode: 7 pollen hold for 3 s, 8 pollen tip', () => {
  const h7 = make({ tipModel: 'torque' });
  feed(h7, 7); run(h7, 3);
  assert.equal(h7.state, 'audience_up');
  assert.equal(h7.cycleStats().tips, 0);
  const h8 = make({ tipModel: 'torque' });
  feed(h8, 8);
  assert.equal(h8.moving, true);
  run(h8, 4);
  assert.equal(h8.state, 'far_up');
  assert.equal(h8.cycleStats().tips, 1);
  assert.equal(h8.angleDeg, 30);
  assert.ok(h8.cycleStats().holdTorqueNm > 0);
});

test('torque mode: a larger excess torque tips faster (12 vs 8 pollen at module level, 7p+nectar vs 8p at hive level)', () => {
  const cal = make().calibration;
  const stable = g.stable_angles_deg;
  const rec = (n, nec) => buildCellRecords(states, 'audience', [...Array(3).fill('nectar'), ...Array(n).fill('pollen'), ...Array(nec).fill('nectar')], g.pivot, stable, HIVE_PHYS);
  const sim = (r) => simulateTip({ records: r, params: HIVE_PHYS, mghNm: cal.mghNm, hiveInertiaKgM2: cal.hiveInertiaKgM2, startSide: -1, assistMinNm: null });
  const s8 = sim(rec(8, 0)), s12 = sim(rec(12, 0));
  assert.equal(s8.finalSide, 1); assert.equal(s12.finalSide, 1);
  assert.ok(s12.timeS < s8.timeS, `12 pollen ${s12.timeS} vs 8 pollen ${s8.timeS}`);
  // hive level: crossing the threshold with a nectar (heavier) tips faster than with a pollen
  const hp = make({ tipModel: 'torque' }); feed(hp, 7); hp.addScoredBall('audience', null, 'pollen');
  const hn = make({ tipModel: 'torque' }); feed(hn, 7); hn.addScoredBall('audience', null, 'nectar');
  const tp = timeToTip(hp), tn = timeToTip(hn);
  assert.ok(Number.isFinite(tp) && Number.isFinite(tn));
  assert.ok(tn < tp, `nectar-trigger ${tn} vs pollen-trigger ${tp}`);
});

test('failed tip (torque mode, sub-threshold capture impulse): returns to audience_up, counts kept, tips = 0, no onTip', () => {
  const h = make({ tipModel: 'torque' });
  let tipped = 0; h.onTip = () => { tipped++; };
  feed(h, 6);
  h.addScoredBall('audience', [59.25, 60, 52], 'pollen', { velocity: [0, 0, -5] });   // 5 in/s downward: tiny angular impulse
  run(h, 4);
  assert.equal(h.moving, false);
  assert.equal(h.state, 'audience_up');
  assert.equal(h.angleDeg, -30);
  const s = h.cycleStats();
  assert.equal(s.tips, 0);
  assert.equal(s.upCellCount, 7, 'counts kept');
  assert.equal(tipped, 0);
});

test('huge dt (1e9) finishes the tip without NaN, in both tip models', () => {
  for (const tipModel of ['rule', 'torque']) {
    const h = make({ tipModel });
    feed(h, 8);
    h.step(1e9);
    assert.ok(Number.isFinite(h.angleDeg) && Number.isFinite(h.theta), tipModel);
    assert.equal(h.moving, false, tipModel);
    assert.equal(h.state, 'far_up', tipModel);
    assert.equal(h.angleDeg, 30);
  }
});

test('step ignores non-finite and non-positive dt', () => {
  const h = make();
  feed(h, 8);
  const a = h.angleDeg;
  h.step(0); h.step(-1); h.step(NaN); h.step(Infinity);
  assert.equal(h.angleDeg, a);
  assert.equal(h.moving, true);
});

test('setState animated: dynamic motion, no tip counted, onTip not called', () => {
  const h = make();
  let tipped = 0; h.onTip = () => { tipped++; };
  h.setState('far_up', true);
  assert.equal(h.moving, true);
  run(h, 4);
  assert.equal(h.moving, false);
  assert.equal(h.state, 'far_up');
  assert.equal(h.angleDeg, 30);
  assert.equal(h.cycleStats().tips, 0);
  assert.equal(tipped, 0);
  h.setState('audience_up', false);
  assert.equal(h.angleDeg, -30);
  assert.equal(h.moving, false);
});

test('records carry massKg and rest coordinates; local is unchanged in shape', () => {
  const h = make();
  feed(h, 2);
  for (const b of h.scored) {
    assert.ok(b.massKg > 0);
    assert.equal(b.rest.length, 2); assert.ok(b.rest.every(Number.isFinite));
    assert.equal(b.local.length, 2);
  }
  assert.ok(h.scored.find((b) => b.type === 'nectar').massKg > h.scored.find((b) => b.type === 'pollen').massKg);
});

test('netTorqueNm: positive net push with 8+ pollen, restoring (negative) when empty at the audience stop; omegaDegS 0 at rest', () => {
  const h = make({ tipModel: 'torque' });
  assert.ok(h.netTorqueNm() < 0);
  assert.equal(h.omegaDegS, 0);
  assert.equal(h.torque(), 0);
  feed(h, 7);
  const n7 = h.netTorqueNm();
  h.forgetBall('audience', 3);
  assert.ok(h.netTorqueNm() < n7, 'removing balls lowers the push');
});

test('setRule validates tipModel and motionModel', () => {
  const h = make();
  assert.throws(() => h.setRule({ tipModel: 'banana' }));
  assert.throws(() => h.setRule({ motionModel: 'banana' }));
  h.setRule({ tipModel: 'torque', motionModel: 'dynamic' });
  assert.equal(h.cycleStats().tipModel, 'torque');
});

test('REPORT ONLY: team 14343 tip table under torque mode + emergent nectar/pollen torque ratio', () => {
  const rows = [[0, 8], [1, 7], [2, 6], [3, 3], [4, 1], [5, 0], [4, 0]];
  const res = [];
  for (const [nn, np] of rows) {
    const h = make({ tipModel: 'torque', startingNectar: 0 });
    for (let i = 0; i < nn; i++) h.addScoredBall('audience', null, 'nectar');
    for (let i = 0; i < np; i++) h.addScoredBall('audience', null, 'pollen');
    run(h, 4);
    res.push(`${nn}n${np}p=${h.cycleStats().tips ? 'TIP' : 'hold'}`);
  }
  console.log('[report] 14343 table (torque mode, fresh hive, 0 starting nectar):', res.join('  '));
  const net = (type) => { const h = make({ tipModel: 'torque', startingNectar: 0 }); if (type) h.addScoredBall('audience', null, type); return h.netTorqueNm(); };
  const base = net(null), tp = net('pollen') - base, tn = net('nectar') - base;
  console.log(`[report] emergent single-ball torque at the audience stop: pollen ${tp.toFixed(5)} N*m, nectar ${tn.toFixed(5)} N*m, ratio nectar/pollen ${(tn / tp).toFixed(3)} (rule table uses 1.75, mass ratio ${(HIVE_PHYS.ballMassKg.nectar / HIVE_PHYS.ballMassKg.pollen).toFixed(3)})`);
  assert.ok(tp > 0 && tn > 0);
});
