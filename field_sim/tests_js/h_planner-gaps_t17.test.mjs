// Static touch-target / a11y checks for the planner (reads source text; no browser).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const read = p => fs.readFileSync(path.join(root, p), 'utf8');
const js = read('web/js/pathui.js');
const css = read('web/css/planner.css');

// ICONS/TIPS are not exported, so parse their object literals from source text.
const entries = (src, name) => {
  const m = src.match(new RegExp(`const ${name} = \\{([\\s\\S]*?)\\n\\};`));
  assert.ok(m, `${name} literal not found in pathui.js`);
  return [...m[1].matchAll(/(\w+)\s*:\s*'((?:[^'\\]|\\.)*)'/g)].map(x => [x[1], x[2]]);
};
const ICON_ENTRIES = entries(js, 'ICONS');
const ICON_KEYS = ICON_ENTRIES.map(([k]) => k);
const TIP_KEYS = new Set(entries(js, 'TIPS').map(([k]) => k));

// Icons drawn without their own TIPS entry: decorative, or titled by the parent action (e.g. pause via toggle, plus via addTip).
// Exception list computed from pathui.js on 2026-11-07. Any new ICONS key without TIPS must be added here deliberately.
const DECORATIVE = ['pan', 'waypoints', 'pause', 'plus', 'reset', 'pin', 'marker', 'rot', 'toward', 'start', 'final',
  'speedo', 'zonei', 'optim', 'cog', 'chev', 'route'];

test('(1) every ICONS key has a TIPS entry or is a listed decorative exception', () => {
  assert.ok(ICON_KEYS.length > 40, `parsed only ${ICON_KEYS.length} icons`);
  const missing = ICON_KEYS.filter(k => !TIP_KEYS.has(k) && !DECORATIVE.includes(k));
  assert.deepEqual(missing, [], 'ICONS keys with no TIPS entry and not listed as decorative');
  const stale = DECORATIVE.filter(k => !ICON_KEYS.includes(k) || TIP_KEYS.has(k));
  assert.deepEqual(stale, [], 'DECORATIVE entries that are no longer icon-only');
});

test('KNOWN-BUG: planner.css sets touch-action: none for the planner canvas', { todo: true }, () => {
  assert.match(css, /touch-action\s*:\s*none/, 'no touch-action: none declaration in planner.css');
});

test('KNOWN-BUG: no two ICONS share an identical SVG string', { todo: true }, () => {
  const groups = new Map();
  for (const [k, v] of ICON_ENTRIES) groups.set(v, [...(groups.get(v) || []), k]);
  const dup = [...groups.values()].filter(g => g.length > 1);
  assert.deepEqual(dup, [], 'identical icon groups: ' + dup.map(g => g.join('=')).join(' | '));
});

test('KNOWN-BUG: coarse-pointer query enlarges .pp-mini and .pp-icon to >=32px', { todo: true }, () => {
  const i = css.search(/@media[^{]*pointer\s*:\s*coarse/);
  assert.ok(i >= 0, 'no (pointer: coarse) media query in planner.css (.pp-mini is 24px)');
  const seg = css.slice(i, i + 1500);
  for (const sel of ['.pp-mini', '.pp-icon']) {
    const rule = seg.match(new RegExp('\\' + sel + '[^{]*\\{([^}]*)\\}'));
    assert.ok(rule, `${sel} not enlarged under coarse pointer`);
    const px = [...rule[1].matchAll(/(?:width|height)\s*:\s*(\d+)px/g)].map(m => +m[1]);
    assert.ok(px.length && Math.min(...px) >= 32, `${sel} sizes under coarse pointer: ${px}`);
  }
});
