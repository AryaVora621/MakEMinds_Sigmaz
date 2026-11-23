import test from 'node:test';
import assert from 'node:assert/strict';
import { energyOf, checkWorld, snapshotPositions, createMonitor } from '../../web/js/physics/validation-tuning/invariants.js';
import { makeScenario } from '../../web/js/physics/validation-tuning/scenarios.js';
import { forAll, randomDropArb, randomDropProperty, randomPushArb, randomPushProperty } from '../../web/js/physics/validation-tuning/proptest.js';
import { PROP } from '../../web/js/physics/validation-tuning/params.js';
import { seedFromEnv } from '../../web/js/physics/validation-tuning/rng.js';

const runs = Number(globalThis.process?.env?.[PROP.runsEnv]) || PROP.numRunsCI;
const seed = seedFromEnv(PROP.seedEnv, 0x5eed);

test('energyOf returns kin, pot, total with total = kin + pot', () => {
  const sc = makeScenario('drop40');
  const e = energyOf(sc.world);
  assert.ok(e.kin >= 0 && e.pot > 0);
  assert.ok(Math.abs(e.total - (e.kin + e.pot)) < 1e-9 * Math.max(1, e.total));
});
test('monitor passes on drop40 for every step', () => {
  const sc = makeScenario('drop40');
  const mon = createMonitor(sc.world, { closed: sc.closed });
  for (let i = 0; i < sc.steps; i++) { sc.step(i); mon.step(); }
  const r = mon.report();
  assert.equal(r.ok, true, JSON.stringify(r.violations.slice(0, 3)));
  assert.equal(r.steps, sc.steps);
});
test('monitor passes on the other closed single-ball scenes', () => {
  for (const name of ['wallBounce', 'roll', 'airDrag', 'shotArc']) {
    const sc = makeScenario(name);
    const mon = createMonitor(sc.world, { closed: sc.closed });
    for (let i = 0; i < sc.steps; i++) { sc.step(i); mon.step(); }
    const r = mon.report();
    assert.equal(r.ok, true, `${name}: ${JSON.stringify(r.violations.slice(0, 3))}`);
  }
});
test('checkWorld is quiet on a healthy world', () => {
  const sc = makeScenario('drop40');
  sc.step(0);
  const r = checkWorld(sc.world, snapshotPositions(sc.world));
  assert.equal(r.ok, true);
});
test('injected floor penetration is flagged', () => {
  const sc = makeScenario('drop40');
  const b = sc.world.balls[0];
  b.z = b.r - 1;
  const r = checkWorld(sc.world, null);
  assert.equal(r.ok, false);
  assert.ok(r.violations.some((v) => /floor/i.test(v.kind)), JSON.stringify(r.violations));
  assert.ok(r.maxPen >= 1 - 1e-9);
});
test('injected wall penetration, non-finite state and overspeed are flagged', () => {
  let sc = makeScenario('drop40');
  sc.world.balls[0].x = -5;
  assert.ok(checkWorld(sc.world, null).violations.some((v) => /wall/i.test(v.kind)));
  sc = makeScenario('drop40');
  sc.world.balls[0].vx = NaN;
  assert.ok(checkWorld(sc.world, null).violations.some((v) => /finite/i.test(v.kind)));
  sc = makeScenario('drop40');
  sc.world.balls[0].vx = 1e5;
  assert.ok(checkWorld(sc.world, null).violations.some((v) => /speed/i.test(v.kind)));
});
test('injected ball-ball overlap is flagged', () => {
  const sc = makeScenario('pile30');
  const [a, b] = sc.world.balls;
  b.x = a.x; b.y = a.y; b.z = a.z;
  const r = checkWorld(sc.world, null);
  assert.ok(r.violations.some((v) => /overlap/i.test(v.kind)));
});
test('energy gain in a closed scene is flagged', () => {
  const sc = makeScenario('drop40');
  const mon = createMonitor(sc.world, { closed: true });
  sc.step(0); mon.step();
  sc.world.balls[0].vz += 500; // inject energy
  sc.step(1); mon.step();
  const r = mon.report();
  assert.equal(r.ok, false);
  assert.ok(r.violations.some((v) => /energy/i.test(v.kind)));
});
test(`property randomDrop: ${runs} runs, seed ${seed}`, () => {
  const res = forAll(randomDropArb, randomDropProperty, { numRuns: runs, seed });
  assert.equal(res.ok, true, res.ok ? '' : `seed ${res.seed} run ${res.failure.run} shrunk ${JSON.stringify(res.failure.shrunk)} ${res.failure.error}`);
});
test(`property randomPush: ${runs} runs, seed ${seed}`, () => {
  const res = forAll(randomPushArb, randomPushProperty, { numRuns: Math.max(5, Math.floor(runs / 2)), seed });
  assert.equal(res.ok, true, res.ok ? '' : `seed ${res.seed} run ${res.failure.run} shrunk ${JSON.stringify(res.failure.shrunk)} ${res.failure.error}`);
});
