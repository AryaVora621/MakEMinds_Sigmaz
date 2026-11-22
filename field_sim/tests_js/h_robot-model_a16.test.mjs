// h_robot-model_a16: robot_spec.json invariants. Observed (node --test): 7 tests, 6 pass, 1 fail (FAILING-BY-DESIGN frame test).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { RULES } from '../web/js/matchrules.js';
import { validateSpec, deriveConstants, provenanceReport, footprintCorners } from '../web/js/robotspec.js';

const load = () => JSON.parse(readFileSync(new URL('../web/data/robot_spec.json', import.meta.url), 'utf8'));
const deepFreeze = o => { Object.values(o).forEach(v => v && typeof v === 'object' && deepFreeze(v)); return Object.freeze(o); };
const area = p => Math.abs(p.reduce((s, a, i) => { const b = p[(i + 1) % p.length]; return s + a[0] * b[1] - b[0] * a[1]; }, 0)) / 2;
const SKIP = new Set(['schema_version', 'prov', 'provenance_legend', 'cad_todo']);
const leaves = (o, p, out) => {   // numeric leaf paths, provenanceReport format (polygon = one leaf)
  if (typeof o === 'number') out.push(p);
  else if (Array.isArray(o)) o.length && o.every(Array.isArray) ? out.push(p) : o.forEach((v, i) => leaves(v, `${p}[${i}]`, out));
  else if (o && typeof o === 'object') for (const k of Object.keys(o)) if (!SKIP.has(k)) leaves(o[k], p ? `${p}.${k}` : k, out);
  return out;
};

test('hold limit is 4 (RULES.holdLimit, matchrules.js:9)', () => assert.equal(RULES.holdLimit, 4));
test('footprint is 381 x 375 mm (0.5 mm tolerance)', () => {
  const d = deriveConstants(load());
  assert.ok(Math.abs(2 * d.halfX * 25.4 - 381) <= 0.5 && Math.abs(2 * d.halfY * 25.4 - 375) <= 0.5);
});

test('every numeric leaf is listed in provenanceReport with a valid tag', () => {
  const spec = load(), rep = provenanceReport(spec);
  const tags = new Map(rep.items.map(i => [i.path, i.provenance]));
  const paths = leaves(spec, '', []);
  assert.equal(paths.length, rep.items.length);
  for (const p of paths) assert.ok(['cad', 'repo', 'vault', 'placeholder'].includes(tags.get(p)), p);
});

test('validateSpec rejects negative sizes, NaN and a missing chassis', () => {
  const s = load(); s.wheels[0].width = -1; s.chassis.height = -2; s.shooter.flywheelRadius = -1;
  const r = validateSpec(s);
  assert.equal(r.ok, false);
  for (const re of [/wheels\[0\]\.width/, /chassis\.height/, /shooter\.flywheelRadius/]) assert.ok(r.errors.some(e => re.test(e)));
  const n = load(); n.drive.maxAccel = NaN;
  assert.ok(validateSpec(n).errors.some(e => /drive\.maxAccel/.test(e)));
  const c = load(); delete c.chassis;
  assert.equal(validateSpec(c).ok, false);
});

test('validateSpec warns (not errors) on a missing top-level frame string', () => {
  const s = load(); delete s.frame;
  const r = validateSpec(s);
  assert.equal(r.ok, true);
  assert.ok(r.warnings.some(w => /frame/.test(w)));
});

test('deriveConstants is deterministic and does not mutate a deep-frozen spec', () => {
  const s = deepFreeze(load()), before = JSON.stringify(s);
  assert.deepEqual(deriveConstants(s), deriveConstants(s));
  assert.equal(JSON.stringify(s), before);
});

test('footprintCorners rotated by pi/2 keeps area (rectangle and L-shape)', () => {
  const L = [[0, 0], [4, 0], [4, 1], [1, 1], [1, 3], [0, 3]];
  for (const poly of [load().chassis.footprint, L]) assert.ok(Math.abs(area(footprintCorners(poly, 5, -3, Math.PI / 2)) - area(poly)) < 1e-9);
});
