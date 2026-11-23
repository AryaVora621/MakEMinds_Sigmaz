// Golden-file regression. UPDATE_GOLDEN=1 rewrites the files from the current sim instead of comparing.
import test from 'node:test';
import assert from 'node:assert/strict';
import { GOLDEN_SCENES, SCENARIO_NAMES, makeScenario, runScenario } from '../../web/js/physics/validation-tuning/scenarios.js';
import { recordGolden, compareGolden, loadGolden, saveGolden } from '../../web/js/physics/validation-tuning/golden.js';
import { MecanumDrive } from '../../web/js/driver.js';

const update = globalThis.process?.env?.UPDATE_GOLDEN === '1';

test('scene lists', () => {
  assert.equal(GOLDEN_SCENES.length, 7);
  assert.equal(SCENARIO_NAMES.length, 9);
  for (const n of GOLDEN_SCENES) assert.ok(SCENARIO_NAMES.includes(n));
});

for (const name of GOLDEN_SCENES) {
  test(`golden ${name}${update ? ' (UPDATE_GOLDEN)' : ''}`, async () => {
    if (update) { await saveGolden(name, recordGolden(name)); return; }
    const g = await loadGolden(name);
    assert.equal(g.name, name); assert.equal(g.version, 1);
    const run = runScenario(makeScenario(name));
    const cmp = compareGolden(g, run);
    assert.equal(cmp.pass, true, `worst: ${JSON.stringify(cmp.worst)}`);
    assert.deepEqual(Object.keys(cmp.rmseByField).sort(), [...g.fields].sort());
  });
}

test('a perturbed golden fails the comparison and names the worst field', async () => {
  const g = structuredClone(await loadGolden('drop40'));
  const zi = g.fields.indexOf('z');
  assert.ok(zi >= 0);
  g.data[Math.floor(g.data.length / 2)][zi] += 0.5; // 0.5 in >> atolPos 1e-3
  const cmp = compareGolden(g, runScenario(makeScenario('drop40')));
  assert.equal(cmp.pass, false);
  assert.equal(cmp.worst.field, 'z');
});

test('scenarios are deterministic run to run', () => {
  for (const name of GOLDEN_SCENES) {
    assert.deepEqual(runScenario(makeScenario(name)).data, runScenario(makeScenario(name)).data, name);
  }
});

test('chaotic pile30 is deterministic for a fixed seed (statistical scene, no golden)', () => {
  const a = runScenario(makeScenario('pile30', { seed: 1 })), b = runScenario(makeScenario('pile30', { seed: 1 }));
  assert.deepEqual(a.data, b.data);
});

test('driveStep matches the analytic min(maxSpeed, maxAccel*t) to 1e-9 while fwd = 1', () => {
  const run = runScenario(makeScenario('driveStep'));
  const d = new MecanumDrive();
  const ti = run.fields.indexOf('t'), vi = run.fields.indexOf('bvx');
  assert.ok(ti >= 0 && vi >= 0, run.fields.join(','));
  let checked = 0;
  for (const row of run.data) {
    if (row[ti] > 1 + 1e-9) break; // command is fwd = 1 for the first 60 steps (1 s)
    assert.ok(Math.abs(row[vi] - Math.min(d.maxSpeed, d.maxAccel * row[ti])) < 1e-9, `t=${row[ti]} v=${row[vi]}`);
    checked++;
  }
  assert.ok(checked >= 30);
});
