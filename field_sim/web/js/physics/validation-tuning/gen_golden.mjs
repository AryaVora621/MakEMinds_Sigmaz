// Regenerates golden/<scene>.json for the 7 non-chaotic scenes against the CURRENT sim.
// Run from field_sim/: node web/js/physics/validation-tuning/gen_golden.mjs
// Only do this when a physics change is intended; the diff of the golden files is the review artefact.
import { GOLDEN_SCENES } from './scenarios.js';
import { recordGolden, saveGolden } from './golden.js';

for (const name of GOLDEN_SCENES) {
  const g = recordGolden(name);
  await saveGolden(name, g);
  console.log('wrote golden/' + name + '.json', g.steps, 'steps');
}
