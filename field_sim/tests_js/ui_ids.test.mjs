import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', 'web');
const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
const jsDir = path.join(root, 'js');
const jsFiles = fs.readdirSync(jsDir).filter(f => f.endsWith('.js'));
const src = jsFiles.map(f => fs.readFileSync(path.join(jsDir, f), 'utf8')).join('\n');

// Ids created dynamically by JS or never present on purpose.
const DYNAMIC = [/^layer-/, /^autoplanner-/, /^pathui-/];
const WHITELIST = new Set(['view']); // 'view' exists; kept for clarity

const referenced = new Set();
for (const m of src.matchAll(/(?:\$|getElementById)\(\s*['"]([A-Za-z][\w-]*)['"]\s*\)/g)) referenced.add(m[1]);

const tagOf = {};
for (const m of html.matchAll(/<(\w+)\b([^>]*)>/g)) {
  const id = /\bid="([^"]+)"/.exec(m[2]);
  if (id) tagOf[id[1]] = { tag: m[1].toLowerCase(), attrs: m[2] };
}

test('every id referenced by js exists in index.html', () => {
  const missing = [...referenced].filter(id => !DYNAMIC.some(r => r.test(id)) && !WHITELIST.has(id) && !tagOf[id]);
  assert.deepEqual(missing, []);
});

test('required new ids exist', () => {
  for (const id of ['tab-auto', 'autopanel', 'autopanel-left', 'autopanel-right', 'timeline', 'help', 'pathpanel', 'opts', 'cam-box', 'match', 'hud-link', 'plan-flip', 'kind-pollen', 'kind-nectar', 'ball-kind'])
    assert.ok(tagOf[id], id);
});

test('hive tip HUD ids exist for both alliances', () => {
  for (const a of ['red', 'blue']) {
    assert.ok(tagOf['hud-hive-' + a], 'hud-hive-' + a);
    assert.equal(tagOf['hud-hive-' + a + '-counts'].tag, 'div');
    assert.equal(tagOf['hud-hive-' + a + '-fill'].tag, 'i');
    assert.equal(tagOf['hud-hive-' + a + '-state'].tag, 'span');
    assert.match(html, new RegExp('id="hud-hive-' + a + '"[^]*?id="hud-hive-' + a + '-fill"'));
  }
  assert.match(src, /function updateHiveHud\(/);
  assert.match(src, /import\('\.\/hivetip_model\.js'\)/);
});

test('element types match', () => {
  const sel = ['aim-mode', 'alliance', 'zone-preset'];
  const chk = ['opt-zone', 'opt-region', 'opt-intake', 'opt-ammo', 'opt-fc', 'opt-shadow', 'opt-noise'];
  const rng = ['opt-speed', 'opt-accel'];
  const btn = ['tab-3d', 'tab-auto', 'tip-red', 'tip-blue', 'm-start', 'm-pause', 'm-reset', 'btn-reset', 'btn-clear', 'btn-refill'];
  for (const id of sel) assert.equal(tagOf[id].tag, 'select', id);
  for (const id of btn) assert.equal(tagOf[id].tag, 'button', id);
  for (const id of chk) { assert.equal(tagOf[id].tag, 'input', id); assert.match(tagOf[id].attrs, /type="checkbox"/, id); }
  for (const id of rng) { assert.equal(tagOf[id].tag, 'input', id); assert.match(tagOf[id].attrs, /type="range"/, id); }
  assert.equal(tagOf.view.tag, 'canvas'); assert.equal(tagOf.map2d.tag, 'canvas');
});

test('all data-cam values present', () => {
  for (const c of ['iso', 'top', 'audience', 'side', 'follow', 'pov', 'driver', 'driver_wide', 'chase', 'coach'])
    assert.ok(html.includes(`data-cam="${c}"`), c);
});

test('importmap and script tags intact, ui.css linked', () => {
  assert.match(html, /<script type="importmap">\s*\{ "imports": \{ "three": "\.\/vendor\/three\.module\.js", "three\/addons\/": "\.\/vendor\/addons\/" \} \}\s*<\/script>/);
  assert.ok(html.includes('<script src="js/shot_core.js"></script>'));
  assert.ok(html.includes('<script type="module" src="js/app.js"></script>'));
  assert.ok(html.includes("import { initDiag } from './js/diag.js'"));
  assert.ok(html.includes('href="css/ui.css"'));
  assert.ok(fs.existsSync(path.join(root, 'css', 'ui.css')));
  assert.match(fs.readFileSync(path.join(root, 'css', 'ui.css'), 'utf8'), /\/\* LAYOUT: left=\d+ right=\d+ top=\d+ bottom=\d+ \*\//);
});

test('ids are unique', () => {
  const ids = [...html.matchAll(/\sid="([^"]+)"/g)].map(m => m[1]);
  const dup = ids.filter((v, i) => ids.indexOf(v) !== i);
  assert.deepEqual(dup, []);
});

test('html tags are balanced', () => {
  const VOID = new Set(['meta', 'link', 'input', 'br', 'hr', 'img']);
  // strip script/style bodies and comments
  const clean = html.replace(/<!--[\s\S]*?-->/g, '').replace(/(<script\b[^>]*>)[\s\S]*?(<\/script>)/g, '$1$2').replace(/(<style\b[^>]*>)[\s\S]*?(<\/style>)/g, '$1$2');
  const stack = [];
  for (const m of clean.matchAll(/<(\/?)([a-zA-Z][\w-]*)\b[^>]*?(\/?)>/g)) {
    const [, close, name, self] = m; const n = name.toLowerCase();
    if (n === '!doctype' || VOID.has(n) || self) continue;
    if (!close) stack.push(n);
    else { const top = stack.pop(); assert.equal(top, n, `mismatched </${n}> (open <${top}>) near ${m.index}`); }
  }
  assert.deepEqual(stack, []);
});

test('3D auto-run panel: ids, buttons, stale 2D label removed, host hooks wired', () => {
  for (const id of ['autorun', 'ar-name', 'ar-state', 'ar-fill', 'ar-time', 'ar-stats', 'ar-src']) assert.ok(tagOf[id], id);
  for (const id of ['ar-pause', 'ar-stop', 'ar-restart', 'ar-edit', 'ar-close']) assert.equal(tagOf[id].tag, 'button', id);
  assert.ok(!/2D map\)/.test(html), 'stale "(2D map)" label is gone');
  assert.ok(html.includes('Show shootable area'));
  const app = fs.readFileSync(path.join(jsDir, 'app.js'), 'utf8');
  assert.match(app, /snapshot: \(\) => \{ prepareAutoRun\(\)/);          // Run in sim resets the field via the runner's snapshot hook
  assert.match(app, /restore: \(\) => \{ prepareAutoRun\(\)/);           // Stop / Restart put the robot and field back
  assert.match(app, /if \(S\.view !== '3d'\) setView\('3d'\)/);          // a started run switches to the 3D view
  assert.match(app, /\$\('ar-stop'\)\.onclick = \(\) => \{ autoPlanner\.stop\(\); resetAllState\(\); \}/);   // Stop also resets hives, kind, robot
  assert.match(app, /\$\('ar-restart'\)\.onclick/);
  assert.ok(!/\bM\b<\/b> switch 2D/.test(app), 'stale M hotkey help removed');
});

test('planner view flip: mx/my/screenToField share one transform and the path editor uses it', () => {
  const app = fs.readFileSync(path.join(jsDir, 'app.js'), 'utf8');
  assert.match(app, /function planFlip\(\) \{ return S\.view === 'auto' && allianceViewTransform\(PLAN_VIEW\.alliance\)\.rotate180 && PLAN_FLIP; \}/);
  assert.match(app, /toScreen, fromScreen, allianceViewTransform, viewScale \} from '\.\/pathing\.js'/);   // flip math comes from pathing.js
  assert.match(app, /function mx\(x\) \{ const s = viewBoxPx\(\); return s\.left \+ toScreen\(viewXf\(\), x, 0, s\.w, s\.h\)\.x; \}/);
  assert.match(app, /p = fromScreen\(viewXf\(\), sx - s\.left, sy - s\.top, s\.w, s\.h\)/);
  assert.match(app, /let PLAN_FLIP = true;/);
  assert.match(app, /\$\('plan-flip'\)\.onchange = e => \{ PLAN_FLIP = !!e\.target\.checked; MAP\.fitKey = null; drawMap\(\); \};/);
  assert.equal(tagOf['plan-flip'].tag, 'input'); assert.match(tagOf['plan-flip'].attrs, /type="checkbox"/);
  assert.match(app, /planFlip\(\) \? 'flip' : 'plain'/);                 // fitKey recomputes when the flip changes
  assert.match(app, /const pathEditor = createPathEditor\(\{ mx, my,/);  // editor draws through the same mappers
  assert.match(app, /const fieldXY = e => screenToField\(e\.clientX, e\.clientY\)/);
  assert.match(app, /const \[x, y\] = fieldXY\(e\);\n  if \(\(S\.view === 'path'/);  // pointerdown uses the inverse, not a bare formula
});

test('3D run entry: #ar-open button exists, is wired, and Resume does not reopen a hidden panel', () => {
  assert.equal(tagOf['ar-open'].tag, 'button');
  const app = fs.readFileSync(path.join(jsDir, 'app.js'), 'utf8');
  assert.ok(/\$\('ar-open'\)\.onclick/.test(app) && /AR\.prev !== 'paused'/.test(app));
  assert.ok(/world\.onRelease = world\.onRecycle/.test(app), 'hive release hook wired');
  assert.ok(/once = !e\.repeat/.test(app), 'hotkeys ignore key repeat');
});
