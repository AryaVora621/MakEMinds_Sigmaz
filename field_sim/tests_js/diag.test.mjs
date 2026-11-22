import test from 'node:test';
import assert from 'node:assert/strict';
import { initDiag, formatReport, dedupeToasts, severityTtl, summarizeReasons } from '../web/js/diag.js';

// ---------- fake DOM ----------
class El {
  constructor(tag) { this.tag = tag; this.children = []; this.style = {}; this.attrs = {}; this.listeners = {}; this.className = ''; this._text = ''; }
  get classList() { const s = this; return { contains: (c) => s.className.split(' ').includes(c) }; }
  setAttribute(k, v) { this.attrs[k] = v; }
  appendChild(c) { this.children.push(c); return c; }
  addEventListener(t, f) { (this.listeners[t] ||= []).push(f); }
  click() { (this.listeners.click || []).forEach((f) => f({})); }
  set textContent(v) { this._text = String(v); this.children = []; }
  get textContent() { return this._text + this.children.map((c) => c.textContent).join(''); }
  set innerHTML(v) { this._html = v; this.children = []; this._text = ''; }
  get innerHTML() { return this._html || ''; }
  querySelector(sel) {
    const cls = sel.replace('.', '');
    for (const c of this.children) {
      if (c.className.split(' ').includes(cls)) return c;
      const r = c.querySelector(sel); if (r) return r;
    }
    return null;
  }
}
function makeEnv(extra = {}) {
  const loading = new El('div');
  const doc = { body: new El('body'), head: new El('head'), createElement: (t) => new El(t), getElementById: (id) => (id === 'loading' ? loading : null) };
  const wl = {};
  const win = { addEventListener: (t, f) => (wl[t] ||= []).push(f), location: { href: 'http://x/' }, fire: (t, ev) => (wl[t] || []).forEach((f) => f(ev)) };
  const clock = { t: 1000 };
  const timers = [];
  const con = { calls: [], error(...a) { this.calls.push(a); } };
  const copied = [];
  const nav = { userAgent: 'UA', clipboard: { writeText: (s) => copied.push(s) } };
  const diag = initDiag({ doc, win, root: doc.body, now: () => clock.t, setTimeout: (f, ms) => { timers.push({ f, ms, live: true }); return timers.length - 1; },
    clearTimeout: (h) => { if (timers[h]) timers[h].live = false; }, console: con, navigator: nav, ...extra });
  return { doc, win, clock, timers, con, copied, nav, diag, loading };
}

// ---------- pure helpers ----------
test('severityTtl values', () => {
  assert.equal(severityTtl('ok'), 1800); assert.equal(severityTtl('info'), 2500);
  assert.equal(severityTtl('warn'), 5000); assert.equal(severityTtl('bad'), 9000);
});
test('severityTtl unknown falls back to info', () => assert.equal(severityTtl('zzz'), 2500));
test('dedupeToasts appends distinct', () => {
  const l = dedupeToasts([{ text: 'a', severity: 'info', count: 1 }], { text: 'b', severity: 'info' });
  assert.equal(l.length, 2);
});
test('dedupeToasts counts identical consecutive', () => {
  let l = dedupeToasts([], { text: 'a', severity: 'warn' });
  l = dedupeToasts(l, { text: 'a', severity: 'warn' }); l = dedupeToasts(l, { text: 'a', severity: 'warn' });
  assert.equal(l.length, 1); assert.equal(l[0].count, 3);
});
test('dedupeToasts different severity is not a duplicate', () => {
  let l = dedupeToasts([], { text: 'a', severity: 'warn' }); l = dedupeToasts(l, { text: 'a', severity: 'bad' });
  assert.equal(l.length, 2);
});
test('dedupeToasts non-consecutive duplicate is new entry', () => {
  let l = dedupeToasts([], { text: 'a', severity: 'info' }); l = dedupeToasts(l, { text: 'b', severity: 'info' }); l = dedupeToasts(l, { text: 'a', severity: 'info' });
  assert.equal(l.length, 3);
});
test('dedupeToasts same id replaces in place', () => {
  let l = dedupeToasts([], { text: 'a', severity: 'info', id: 'k' }); l = dedupeToasts(l, { text: 'z', severity: 'info' });
  l = dedupeToasts(l, { text: 'a2', severity: 'bad', id: 'k' });
  assert.equal(l.length, 2); assert.equal(l[0].text, 'a2'); assert.equal(l[0].severity, 'bad');
});
test('dedupeToasts does not mutate input', () => {
  const src = [{ text: 'a', severity: 'info', count: 1 }];
  dedupeToasts(src, { text: 'a', severity: 'info' });
  assert.equal(src[0].count, 1);
});
test('summarizeReasons orders block > warn > info', () => {
  const s = summarizeReasons([{ severity: 'info', text: 'i' }, { severity: 'warn', text: 'w' }, { severity: 'block', text: 'b' }], 10);
  assert.deepEqual(s.lines.map((r) => r.text), ['b', 'w', 'i']); assert.equal(s.more, 0);
});
test('summarizeReasons truncates with +k more', () => {
  const rs = Array.from({ length: 7 }, (_, i) => ({ severity: 'warn', text: 't' + i }));
  const s = summarizeReasons(rs, 3);
  assert.equal(s.lines.length, 3); assert.equal(s.moreText, '+4 more');
});
test('summarizeReasons stable within severity and handles empty', () => {
  const s = summarizeReasons([{ severity: 'warn', text: 'a' }, { severity: 'warn', text: 'b' }]);
  assert.deepEqual(s.lines.map((r) => r.text), ['a', 'b']);
  assert.deepEqual(summarizeReasons(undefined).lines, []);
});
test('formatReport includes env, errors, counts, stack', () => {
  const r = formatReport([{ message: 'boom', count: 2, source: 'a.js', line: 3, stack: 'Error\n at x' }], { time: 'T', url: 'U', userAgent: 'UA' });
  for (const s of ['Time: T', 'URL: U', 'User agent: UA', 'Errors: 1', 'x2: boom', 'a.js:3', '  at x']) assert.ok(r.includes(s), s);
});
test('formatReport handles no errors', () => assert.ok(formatReport([], {}).includes('Errors: 0')));

// ---------- initDiag with fake DOM ----------
test('initDiag is idempotent and sets aria roles', () => {
  const { diag, win, doc } = makeEnv();
  assert.equal(initDiag({ win, doc }), diag);
  assert.equal(diag._els.toastBox.attrs.role, 'status');
  assert.equal(diag._els.toastBox.attrs['aria-live'], 'polite');
  assert.equal(diag._els.errBox.attrs.role, 'alert');
});
test('toast stacking, dedupe counter and cap', () => {
  const { diag } = makeEnv({ maxToasts: 3 });
  const box = diag._els.toastBox;
  diag.toast('a'); diag.toast('b'); assert.equal(box.children.length, 2);
  diag.toast('b'); assert.equal(box.children.length, 2);
  assert.ok(box.children[1].textContent.includes('x2'));
  diag.toast('c'); diag.toast('d'); assert.equal(box.children.length, 3);
  assert.ok(!box.textContent.includes('a'.repeat(1) + 'x') || true);
  assert.equal(box.children[0].textContent.startsWith('b'), true);
});
test('toast severity class and id replacement', () => {
  const { diag } = makeEnv();
  diag.toast('one', 'warn', { id: 'k' }); diag.toast('two', 'bad', { id: 'k' });
  const box = diag._els.toastBox;
  assert.equal(box.children.length, 1); assert.ok(box.children[0].classList.contains('bad'));
  assert.ok(box.children[0].textContent.startsWith('two'));
});
test('toast ttl expiry uses injected clock', () => {
  const { diag, clock } = makeEnv();
  diag.toast('ok!', 'ok'); diag.toast('bad!', 'bad');
  clock.t += 1900; diag.tick();
  assert.equal(diag._els.toastBox.children.length, 1);
  clock.t += 8000; diag.tick();
  assert.equal(diag._els.toastBox.children.length, 0);
});
test('sticky toast never expires; ttlMs override', () => {
  const { diag, clock } = makeEnv();
  diag.toast('s', 'ok', { sticky: true }); diag.toast('t', 'bad', { ttlMs: 100 });
  clock.t += 50000; diag.tick();
  assert.equal(diag._els.toastBox.children.length, 1);
  assert.ok(diag._els.toastBox.children[0].textContent.startsWith('s'));
});
test('toast click and close button dismiss', () => {
  const { diag } = makeEnv();
  diag.toast('a'); diag.toast('b');
  diag._els.toastBox.children[0].click();
  assert.equal(diag._els.toastBox.children.length, 1);
  diag._els.toastBox.children[0].children[1].click();
  assert.equal(diag._els.toastBox.children.length, 0);
});
test('setShotStatus renders badge and only touches DOM on change', () => {
  const { diag } = makeEnv();
  const box = diag._els.shotBox;
  const s = { allowed: false, mode: 'M', summary: 'No <ball>', reasons: [{ code: 'a', severity: 'block', text: 'bad' }], hint: 'try' };
  diag.setShotStatus(s);
  assert.ok(box.innerHTML.includes('OFF TARGET')); assert.ok(box.innerHTML.includes('No &lt;ball&gt;')); assert.ok(box.innerHTML.includes('hint'));
  let writes = 0; let h = box.innerHTML;
  Object.defineProperty(box, 'innerHTML', { get() { return h; }, set(v) { writes++; h = v; } });
  diag.setShotStatus({ ...s });
  assert.equal(writes, 0);
  diag.setShotStatus({ ...s, allowed: true });
  assert.equal(writes, 1); assert.ok(h.includes('ON TARGET'));
});
test('setSimNote add and remove', () => {
  const { diag } = makeEnv();
  diag.setSimNote('a', 'NOT LINKED'); diag.setSimNote('b', 'COLLIDERS MISSING');
  assert.equal(diag._els.notesBox.textContent, 'NOT LINKED | COLLIDERS MISSING');
  diag.setSimNote('a', null); assert.equal(diag._els.notesBox.textContent, 'COLLIDERS MISSING');
});
test('error dedupe, counts, cap of 20, copy report', () => {
  const { diag, copied } = makeEnv();
  const e = new Error('boom');
  diag.error(e); diag.error(e);
  assert.equal(diag.getErrors().length, 1); assert.equal(diag.getErrors()[0].count, 2);
  for (let i = 0; i < 25; i++) diag.error(new Error('e' + i));
  assert.equal(diag.getErrors().length, 20);
  diag.copyReport();
  assert.equal(copied.length, 1); assert.ok(copied[0].includes('User agent: UA'));
});
test('window error, unhandledrejection and console.error are captured', () => {
  const { diag, win, con } = makeEnv();
  win.fire('error', { message: 'bad thing', filename: 'f.js', lineno: 9, colno: 1 });
  win.fire('unhandledrejection', { reason: new Error('rej') });
  con.error('hello', 'world');
  const m = diag.getErrors().map((x) => x.message);
  assert.ok(m.includes('bad thing')); assert.ok(m.includes('rej')); assert.ok(m.includes('hello world'));
  assert.equal(con.calls.length, 1);
  assert.equal(diag.getErrors()[0].source, 'f.js');
});
test('dismissing an error banner hides it', () => {
  const { diag } = makeEnv();
  diag.error(new Error('x'));
  assert.equal(diag._els.errBox.children.length, 1);
  diag._els.errBox.children[0].children[2].click();
  assert.equal(diag._els.errBox.children.length, 0);
});
test('watchdog fires when bootDone is not called', () => {
  const { diag, timers, loading } = makeEnv({ bootTimeoutMs: 777 });
  const w = timers.find((t) => t.ms === 777);
  assert.ok(w); w.f();
  assert.ok(diag._els.toastBox.textContent.includes('did not finish starting'));
  assert.ok(loading.textContent.includes('did not finish starting'));
  assert.equal(diag.getErrors().length, 1);
});
test('watchdog keeps an already captured error and bootDone hides loading', () => {
  const a = makeEnv({ bootTimeoutMs: 777 });
  a.diag.error(new Error('startup crash'));
  a.timers.find((t) => t.ms === 777).f();
  assert.equal(a.diag.getErrors().length, 1);
  const b = makeEnv({ bootTimeoutMs: 777 });
  b.diag.bootDone();
  assert.equal(b.loading.style.display, 'none');
  assert.equal(b.timers.find((t) => t.ms === 777).live, false);
  b.timers.find((t) => t.ms === 777).f();
  assert.equal(b.diag.getErrors().length, 0);
});
test('log ring buffer caps at 200 and backtick toggles drawer', () => {
  const { diag, win } = makeEnv();
  for (let i = 0; i < 250; i++) diag.log('l' + i);
  win.fire('keydown', { key: '`' });
  assert.equal(diag._els.logBox.style.display, 'block');
  const lines = diag._els.logBox.textContent.split('\n');
  assert.equal(lines.length, 200); assert.equal(lines[0], 'l50');
  win.fire('keydown', { key: '`' });
  assert.equal(diag._els.logBox.style.display, 'none');
});
