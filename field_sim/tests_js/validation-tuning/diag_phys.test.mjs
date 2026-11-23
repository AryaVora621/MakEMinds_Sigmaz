import test from 'node:test';
import assert from 'node:assert/strict';
import { initDiag, formatPhysicsStats } from '../../web/js/diag.js';
import { fakeDoc } from './_fixtures.mjs';

function makeDiag() {
  const doc = fakeDoc();
  const wl = {};
  const win = { addEventListener: (t, f) => (wl[t] ||= []).push(f), location: { href: 'http://x/' } };
  const copied = [];
  const diag = initDiag({ doc, win, root: doc.body, now: () => 1000, setTimeout: () => 0, clearTimeout: () => {},
    console: { error() {} }, navigator: { userAgent: 'UA', clipboard: { writeText: (s) => copied.push(s) } } });
  return { doc, diag, copied };
}
const findPanel = (doc, cls) => doc.body.findAll((c) => c.className.split(' ').includes(cls))[0] || null;
const isHidden = (el) => el.style.display === 'none' || el.style.visibility === 'hidden' || el.hidden === true || /\b(dg-hidden|hidden)\b/.test(el.className);

test('formatPhysicsStats produces the documented lines', () => {
  const lines = formatPhysicsStats({ stepMs: 0.42, stepMsP95: 0.8, balls: 34, awake: 5, contacts: 12, energyJ: 0.153, maxPenIn: 0.03, violations: 0 });
  assert.deepEqual(lines, ['step 0.42 ms (p95 0.80)', 'balls 34 (awake 5)', 'contacts 12', 'energy 0.153 J', 'max pen 0.03 in', 'violations 0']);
});
test('formatPhysicsStats skips fields that are missing and never throws', () => {
  assert.deepEqual(formatPhysicsStats({ balls: 3 }).filter((l) => l.startsWith('balls')), ['balls 3']);
  assert.ok(Array.isArray(formatPhysicsStats({})));
  assert.ok(formatPhysicsStats({ stepMs: 1.5 })[0].startsWith('step 1.50 ms'));
});
test('existing diag exports are intact', async () => {
  const m = await import('../../web/js/diag.js');
  for (const k of ['severityTtl', 'dedupeToasts', 'summarizeReasons', 'formatReport', 'initDiag', 'formatPhysicsStats']) assert.equal(typeof m[k], 'function', k);
});
test('setPhysicsStats shows the dg-phys panel with the stats and null hides it', () => {
  const { doc, diag } = makeDiag();
  assert.equal(typeof diag.setPhysicsStats, 'function');
  diag.setPhysicsStats({ stepMs: 0.42, stepMsP95: 0.8, balls: 34, awake: 5, contacts: 12, energyJ: 0.153, maxPenIn: 0.03, violations: 0 });
  const panel = findPanel(doc, 'dg-phys');
  assert.ok(panel, 'dg-phys panel exists');
  assert.equal(isHidden(panel), false);
  assert.match(panel.textContent, /balls 34 \(awake 5\)/);
  assert.match(panel.textContent, /violations 0/);
  diag.setPhysicsStats(null);
  assert.equal(isHidden(findPanel(doc, 'dg-phys')), true);
  diag.setPhysicsStats({ balls: 1 });
  assert.equal(isHidden(findPanel(doc, 'dg-phys')), false);
});
test('showReport shows a panel with the title and lines', () => {
  const { doc, diag } = makeDiag();
  assert.equal(typeof diag.showReport, 'function');
  diag.showReport('Golden report', ['line one', 'line two']);
  const all = doc.body.textContent;
  assert.match(all, /Golden report/); assert.match(all, /line one/); assert.match(all, /line two/);
});
