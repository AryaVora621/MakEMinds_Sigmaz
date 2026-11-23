// tuning_panel.js - DOM panel with one slider + number input per PARAM_SCHEMA entry, plus preset
// select/save/load/delete, .json export/import and reset to DEFAULT_PRESET. All DOM comes through the
// injected doc. Values are read/written through getValue/setValue so the panel never touches the sim itself.
// Units are shown per entry (in, in/s, rad/s, ...); the tag (SOURCED/PRIOR/ASSUMED) comes from the schema.
import { PARAM_SCHEMA, PRESET } from './params.js';
import * as presetsMod from './presets.js';

const CSS = `
.vt-panel{position:fixed;top:48px;right:12px;z-index:10004;width:340px;max-height:85vh;overflow:auto;background:#0e1422f2;border:1px solid #2a3652;border-radius:8px;padding:8px 10px;color:#dfe6f5;font:12px 'Segoe UI',sans-serif}
.vt-panel h4{margin:8px 0 4px;font-size:12px;color:#7fd4ff}
.vt-row{display:grid;grid-template-columns:1fr 70px;gap:2px 6px;margin-bottom:5px}
.vt-row label{grid-column:1 / 3}
.vt-row .vt-meta{color:#7d8aa6;font-size:10px}
.vt-panel input[type=range]{width:100%}
.vt-panel input[type=number]{width:70px;background:#162036;color:#dfe6f5;border:1px solid #2a3652;border-radius:4px}
.vt-panel button,.vt-panel select,.vt-panel input[type=text]{background:#162036;color:#dfe6f5;border:1px solid #2a3652;border-radius:4px;margin:2px 4px 2px 0}
.vt-status{color:#ffcf5c;min-height:14px}
`;

export function createTuningPanel(opts) {
  const doc = opts.doc;
  const schema = opts.schema || PARAM_SCHEMA;
  const getValue = opts.getValue, setValue = opts.setValue;
  const onChange = opts.onChange || (() => {});
  const P = { ...presetsMod, ...(opts.presets || {}) };
  const root = opts.root || doc.body;
  const mk = (tag, cls) => { const e = doc.createElement(tag); if (cls) e.className = cls; return e; };
  const txt = (tag, text, cls) => { const e = mk(tag, cls); e.textContent = text; return e; };

  const style = mk('style'); style.textContent = CSS;
  (doc.head || root).appendChild(style);
  const el = mk('div', 'vt-panel');
  el.style.display = 'none';
  el.appendChild(txt('b', 'Physics tuning (F9)'));
  const status = mk('div', 'vt-status');
  el.appendChild(status);
  const say = (m) => { status.textContent = String(m || ''); };

  const rows = new Map(); // key -> {range, num}
  const groups = new Map();
  function setBoth(key, v) {
    const r = rows.get(key);
    if (r) { r.range.value = String(v); r.num.value = String(v); }
  }
  for (const s of schema) {
    if (!groups.has(s.group)) { el.appendChild(txt('h4', s.group)); groups.set(s.group, true); }
    const row = mk('div', 'vt-row');
    row.appendChild(txt('label', `${s.field}${s.unit ? ' [' + s.unit + ']' : ''}`));
    const range = mk('input'); range.type = 'range';
    const num = mk('input'); num.type = 'number';
    for (const inp of [range, num]) { inp.min = String(s.min); inp.max = String(s.max); inp.step = String(s.step); }
    const apply = (raw) => {
      let v = Number(raw);
      if (!Number.isFinite(v)) return;
      v = Math.min(s.max, Math.max(s.min, v));
      setValue(s.key, v);
      setBoth(s.key, v);
      onChange(s.key, v);
    };
    range.addEventListener('input', () => apply(range.value));
    num.addEventListener('change', () => apply(num.value));
    row.appendChild(range); row.appendChild(num);
    row.appendChild(txt('span', `${s.min}..${s.max} step ${s.step} ${s.unit || ''} [${s.tag}]`, 'vt-meta'));
    rows.set(s.key, { range, num });
    el.appendChild(row);
  }

  // ---- presets
  el.appendChild(txt('h4', 'presets'));
  const select = mk('select');
  const nameIn = mk('input'); nameIn.type = 'text'; nameIn.value = 'my-preset';
  function refreshList() {
    let names = [];
    try { names = P.listPresets() || []; } catch (_) { names = []; }
    select.innerHTML = '';
    for (const n of names) { const o = mk('option'); o.value = n; o.textContent = n; select.appendChild(o); }
    return names;
  }
  function currentPreset(name) {
    const p = { version: PRESET.version, name, created: new Date().toISOString(), ball: {}, drive: {}, source: {} };
    for (const s of schema) { p[s.group][s.field] = getValue(s.key); p.source[s.key] = s.tag; }
    return p;
  }
  function applyPresetValues(p) {
    for (const s of schema) {
      const v = p[s.group] && p[s.group][s.field];
      if (typeof v === 'number' && Number.isFinite(v)) { setValue(s.key, v); onChange(s.key, v); }
    }
    refresh();
  }
  const btn = (label, fn) => { const b = txt('button', label); b.addEventListener('click', fn); el.appendChild(b); return b; };
  el.appendChild(select); el.appendChild(nameIn);
  btn('Save', () => {
    const name = String(nameIn.value || '').trim();
    if (!name) { say('enter a preset name'); return; }
    const p = currentPreset(name);
    const v = P.validatePreset(p);
    if (!v.ok) { say('invalid: ' + v.errors.join('; ')); return; }
    say(P.savePreset(p) ? `saved "${name}"` : 'save failed (storage unavailable)');
    refreshList();
  });
  btn('Load', () => {
    const p = P.loadPreset(select.value);
    if (!p) { say('load failed'); return; }
    applyPresetValues(p); say(`loaded "${p.name}"`);
  });
  btn('Delete', () => { say(P.deletePreset(select.value) ? 'deleted' : 'delete failed'); refreshList(); });
  btn('Reset to default', () => {
    if (P.DEFAULT_PRESET) { applyPresetValues(P.DEFAULT_PRESET); say('reset to defaults'); }
  });
  btn('Export .json', () => {
    try {
      const body = P.serializePreset(currentPreset(String(nameIn.value || 'export')));
      const win = doc.defaultView || globalThis;
      const blob = new win.Blob([body], { type: 'application/json' });
      const a = mk('a');
      a.href = win.URL.createObjectURL(blob);
      a.download = `${nameIn.value || 'preset'}.json`;
      a.click();
      if (win.URL.revokeObjectURL) win.URL.revokeObjectURL(a.href);
      say('exported');
    } catch (e) { say('export failed: ' + (e && e.message)); }
  });
  const file = mk('input'); file.type = 'file'; file.accept = '.json,application/json';
  file.addEventListener('change', async () => {
    try {
      const f = file.files && file.files[0];
      if (!f) return;
      const p = P.parsePreset(await f.text());
      const v = P.validatePreset(p);
      if (!v.ok) { say('invalid: ' + v.errors.join('; ')); return; }
      applyPresetValues(p); say(`imported "${p.name}"`);
    } catch (e) { say('import failed: ' + (e && e.message)); }
  });
  el.appendChild(file);

  root.appendChild(el);
  refreshList();

  function refresh() {
    for (const s of schema) {
      let v; try { v = getValue(s.key); } catch (_) { v = undefined; }
      if (typeof v === 'number' && Number.isFinite(v)) setBoth(s.key, v);
    }
  }
  refresh();

  return {
    el, refresh,
    show() { el.style.display = 'block'; refresh(); },
    hide() { el.style.display = 'none'; },
    toggle() { if (el.style.display === 'none') this.show(); else this.hide(); },
    destroy() {
      el.style.display = 'none';
      if (root.removeChild) { try { root.removeChild(el); } catch (_) { /* ignore */ } }
    },
  };
}
