// diag.js - on-screen diagnostics: toast stack, shot-status panel, error banner,
// log drawer, boot watchdog. Creates its own DOM. Pure helpers are exported.

const TTL = { ok: 1800, info: 2500, warn: 5000, bad: 9000 };
const SEV_RANK = { block: 0, bad: 0, warn: 1, info: 2 };

export function severityTtl(sev) {
  return TTL[sev] != null ? TTL[sev] : TTL.info;
}

// list: array of {text, severity, count, id}; next: {text, severity, id}.
// Returns a NEW list. Same id replaces in place; identical text+severity as the
// last entry bumps its count.
export function dedupeToasts(list, next) {
  const out = list.map((t) => ({ ...t }));
  if (next.id != null) {
    const i = out.findIndex((t) => t.id === next.id);
    if (i >= 0) {
      out[i] = { ...out[i], ...next, count: 1 };
      return out;
    }
  }
  const last = out[out.length - 1];
  if (last && last.text === next.text && last.severity === next.severity) {
    last.count = (last.count || 1) + 1;
    if (next.id != null) last.id = next.id;
    return out;
  }
  out.push({ count: 1, ...next });
  return out;
}

// Orders block > warn > info; at most maxLines lines, then '+k more'.
export function summarizeReasons(reasons, maxLines = 4) {
  const list = (reasons || []).map((r, i) => ({ r, i }));
  list.sort((a, b) => {
    const d = (SEV_RANK[a.r.severity] ?? 3) - (SEV_RANK[b.r.severity] ?? 3);
    return d || a.i - b.i;
  });
  const lines = list.slice(0, maxLines).map((x) => x.r);
  const more = list.length - lines.length;
  return { lines, more, moreText: more > 0 ? `+${more} more` : '' };
}

// errors: [{message, source, line, col, stack, count, context, time}]
export function formatReport(errors, env = {}) {
  const out = [];
  out.push('BIOBUZZ sim diagnostic report');
  out.push(`Time: ${env.time || ''}`);
  out.push(`URL: ${env.url || ''}`);
  out.push(`User agent: ${env.userAgent || ''}`);
  out.push(`Errors: ${(errors || []).length}`);
  (errors || []).forEach((e, i) => {
    out.push('');
    out.push(`#${i + 1} x${e.count || 1}: ${e.message}`);
    if (e.context) out.push(`  context: ${e.context}`);
    if (e.source) out.push(`  at ${e.source}${e.line != null ? ':' + e.line : ''}${e.col != null ? ':' + e.col : ''}`);
    if (e.stack) out.push(String(e.stack).split('\n').map((l) => '  ' + l).join('\n'));
  });
  return out.join('\n');
}

// Pure formatter for the physics HUD (validation-tuning). Fields that are absent are skipped.
// Units: ms per step, in for penetration, J for energy.
export function formatPhysicsStats(s) {
  s = s || {};
  const out = [];
  const num = (v) => typeof v === 'number' && Number.isFinite(v);
  if (num(s.stepMs)) out.push(`step ${s.stepMs.toFixed(2)} ms` + (num(s.stepMsP95) ? ` (p95 ${s.stepMsP95.toFixed(2)})` : ''));
  if (num(s.balls)) out.push(`balls ${s.balls}` + (num(s.awake) ? ` (awake ${s.awake})` : ''));
  if (num(s.contacts)) out.push(`contacts ${s.contacts}`);
  if (num(s.energyJ)) out.push(`energy ${s.energyJ.toFixed(3)} J`);
  if (num(s.energyDriftRel)) out.push(`energy drift ${(s.energyDriftRel * 100).toFixed(1)} %`);
  if (num(s.maxPenIn)) out.push(`max pen ${s.maxPenIn.toFixed(2)} in`);
  if (num(s.violations)) out.push(`violations ${s.violations}`);
  return out;
}

function esc(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

const CSS = `
.dg-toasts{position:fixed;top:48px;right:12px;z-index:10000;display:flex;flex-direction:column;gap:6px;max-width:380px;pointer-events:none;font-family:'Segoe UI',sans-serif}
.dg-toast{background:#0e1422;border:1px solid #2a3652;border-left:4px solid #8aa0c8;color:#dfe6f5;padding:6px 8px;border-radius:6px;font-size:13px;display:flex;gap:8px;align-items:flex-start;pointer-events:auto;cursor:pointer}
.dg-toast.ok{border-left-color:#5fe08a}.dg-toast.warn{border-left-color:#ffcf5c}.dg-toast.bad{border-left-color:#ff6b6b}
.dg-toast .dg-x{background:none;border:0;color:#8aa0c8;cursor:pointer;pointer-events:auto}
.dg-shot{position:fixed;bottom:12px;left:50%;transform:translateX(-50%);z-index:10000;background:#0e1422;border:1px solid #2a3652;border-radius:8px;padding:8px 12px;min-width:260px;max-width:560px;color:#dfe6f5;font:13px 'Segoe UI',sans-serif;pointer-events:none}
.dg-shot .badge{font-size:20px;font-weight:700;margin-right:8px}
.dg-shot.ready .badge{color:#5fe08a}.dg-shot.blocked .badge{color:#ff6b6b}
.dg-shot .r-block{color:#ff6b6b}.dg-shot .r-warn{color:#ffcf5c}.dg-shot .r-info{color:#9aa6bf}.dg-shot .hint{color:#7fd4ff;margin-top:4px}
.dg-notes{position:fixed;bottom:12px;left:12px;z-index:10000;color:#ffcf5c;font:11px 'Segoe UI',sans-serif;pointer-events:none}
.dg-errs{position:fixed;top:0;left:0;right:0;z-index:10001;font-family:'Segoe UI',sans-serif;pointer-events:none}
.dg-err{background:#2a0f14;border-bottom:1px solid #ff6b6b;color:#ffd6d6;padding:6px 10px;font-size:13px;pointer-events:auto}
.dg-err pre{white-space:pre-wrap;font-size:11px;margin:4px 0 0;max-height:160px;overflow:auto}
.dg-err button,.dg-log button{pointer-events:auto;background:#162036;color:#dfe6f5;border:1px solid #2a3652;border-radius:4px;margin-left:6px;cursor:pointer}
.dg-phys{position:fixed;top:48px;left:12px;z-index:10000;background:#0e1422e6;border:1px solid #2a3652;border-radius:6px;padding:6px 8px;color:#b8c4de;font:11px Consolas,monospace;white-space:pre;pointer-events:none;display:none}
.dg-report{position:fixed;top:80px;left:50%;transform:translateX(-50%);z-index:10003;background:#0e1422;border:1px solid #2a3652;border-radius:8px;padding:8px 12px;max-width:640px;max-height:60%;overflow:auto;color:#dfe6f5;font:12px Consolas,monospace;pointer-events:auto}
.dg-report pre{white-space:pre-wrap;margin:6px 0}
.dg-report button{background:#162036;color:#dfe6f5;border:1px solid #2a3652;border-radius:4px;margin-right:6px;cursor:pointer}
.dg-log{position:fixed;left:0;right:0;bottom:0;height:35%;z-index:10002;background:#0e1422ee;border-top:1px solid #2a3652;color:#b8c4de;font:11px Consolas,monospace;overflow:auto;white-space:pre-wrap;display:none;pointer-events:auto}
`;

export function initDiag(opts = {}) {
  const doc = opts.doc || (typeof document !== 'undefined' ? document : null);
  const win = opts.win || (typeof window !== 'undefined' ? window : null);
  if (win && win.__diag) return win.__diag;
  const root = opts.root || (doc && doc.body);
  const maxToasts = opts.maxToasts || 5;
  const now = opts.now || (() => Date.now());
  const setT = opts.setTimeout || ((f, ms) => setTimeout(f, ms));
  const clearT = opts.clearTimeout || ((h) => clearTimeout(h));
  const bootTimeoutMs = opts.bootTimeoutMs != null ? opts.bootTimeoutMs : 15000;
  const con = opts.console || (typeof console !== 'undefined' ? console : null);
  const nav = opts.navigator || (typeof navigator !== 'undefined' ? navigator : {});
  const mk = (tag, cls) => { const e = doc.createElement(tag); if (cls) e.className = cls; return e; };

  const style = mk('style'); style.textContent = CSS; (doc.head || root).appendChild(style);
  const toastBox = mk('div', 'dg-toasts');
  toastBox.setAttribute('role', 'status'); toastBox.setAttribute('aria-live', 'polite');
  const shotBox = mk('div', 'dg-shot'); shotBox.style.display = 'none';
  const notesBox = mk('div', 'dg-notes');
  const errBox = mk('div', 'dg-errs'); errBox.setAttribute('role', 'alert');
  const logBox = mk('div', 'dg-log');
  const physBox = mk('div', 'dg-phys');
  [toastBox, shotBox, notesBox, errBox, logBox, physBox].forEach((e) => root.appendChild(e));

  // ---- toasts
  let toasts = []; // {text,severity,count,id,expires,sticky}
  let toastSig = '';
  function renderToasts() {
    const sig = JSON.stringify(toasts.map((t) => [t.text, t.severity, t.count, t.id, t.uid]));
    if (sig === toastSig) return;
    toastSig = sig;
    toastBox.innerHTML = '';
    for (const t of toasts) {
      const el = mk('div', 'dg-toast ' + t.severity);
      const txt = mk('span'); txt.textContent = t.text + (t.count > 1 ? ' x' + t.count : '');
      const x = mk('button', 'dg-x'); x.textContent = 'x'; x.setAttribute('aria-label', 'Dismiss');
      const dismiss = () => { toasts = toasts.filter((q) => q.uid !== t.uid); renderToasts(); };
      x.addEventListener('click', (e) => { if (e && e.stopPropagation) e.stopPropagation(); dismiss(); });
      el.addEventListener('click', dismiss);
      el.appendChild(txt); el.appendChild(x);
      toastBox.appendChild(el);
    }
  }
  let uidc = 0;
  function tick() {
    const t = now();
    const before = toasts.length;
    toasts = toasts.filter((q) => q.sticky || q.expires > t);
    if (toasts.length !== before) renderToasts();
  }
  function toast(text, severity = 'info', o = {}) {
    const { sticky = false, ttlMs = null, id = null } = o;
    text = String(text);
    tick();
    const prevLen = toasts.length;
    const sameLast = toasts[prevLen - 1];
    toasts = dedupeToasts(toasts, { text, severity, id });
    const ttl = ttlMs != null ? ttlMs : severityTtl(severity);
    let cur = id != null ? toasts.find((q) => q.id === id) : toasts[toasts.length - 1];
    if (!cur) cur = toasts[toasts.length - 1];
    if (cur.uid == null) cur.uid = ++uidc;
    cur.sticky = !!sticky; cur.expires = now() + ttl;
    while (toasts.length > maxToasts) toasts.shift();
    renderToasts();
    if (!sticky) setT(tick, ttl + 5);
    void sameLast;
    return cur.uid;
  }

  // ---- shot status
  let shotSig = null;
  function setShotStatus(s) {
    const sig = JSON.stringify(s || null);
    if (sig === shotSig) return;
    shotSig = sig;
    if (!s) { shotBox.style.display = 'none'; return; }
    shotBox.style.display = 'block';
    shotBox.className = 'dg-shot ' + (s.allowed ? 'ready' : 'blocked');
    const sum = summarizeReasons(s.reasons || [], 4);
    const cls = (v) => (v === 'block' || v === 'bad' ? 'r-block' : v === 'warn' ? 'r-warn' : 'r-info');
    shotBox.innerHTML =
      `<span class="badge">${s.allowed ? 'ON TARGET' : 'OFF TARGET'}</span>` +
      (s.mode ? `<small>${esc(s.mode)}</small> ` : '') + `<span>${esc(s.summary || '')}</span>` +
      sum.lines.map((r) => `<div class="${cls(r.severity)}">${esc(r.text)}</div>`).join('') +
      (sum.more ? `<div class="r-info">${esc(sum.moreText)}</div>` : '') +
      (s.hint ? `<div class="hint">${esc(s.hint)}</div>` : '');
  }

  // ---- sim notes
  const notes = new Map();
  function setSimNote(key, text) {
    if (text == null) notes.delete(key); else notes.set(key, String(text));
    notesBox.textContent = [...notes.values()].join(' | ');
  }

  // ---- log
  const logBuf = [];
  function log(text) {
    logBuf.push(String(text));
    if (logBuf.length > 200) logBuf.shift();
    if (logBox.style.display === 'block') logBox.textContent = logBuf.join('\n');
  }
  function toggleLog() {
    const open = logBox.style.display !== 'block';
    logBox.style.display = open ? 'block' : 'none';
    if (open) logBox.textContent = logBuf.join('\n');
  }

  // ---- errors
  const errors = [];
  let errSig = '';
  function renderErrors() {
    const sig = JSON.stringify(errors.map((e) => [e.message, e.count, e.dismissed]));
    if (sig === errSig) return;
    errSig = sig;
    errBox.innerHTML = '';
    for (const e of errors) {
      if (e.dismissed) continue;
      const el = mk('div', 'dg-err');
      const head = mk('span');
      head.textContent = e.message + (e.count > 1 ? ' (x' + e.count + ')' : '') +
        (e.source ? '  [' + e.source + (e.line != null ? ':' + e.line : '') + ']' : '');
      el.appendChild(head);
      const copy = mk('button'); copy.textContent = 'Copy report';
      copy.addEventListener('click', () => copyReport());
      const close = mk('button'); close.textContent = 'Dismiss';
      close.addEventListener('click', () => { e.dismissed = true; errSig = ''; renderErrors(); });
      el.appendChild(copy); el.appendChild(close);
      if (e.stack) {
        const pre = mk('pre'); pre.textContent = e.stack; pre.style.display = 'none';
        const tg = mk('button'); tg.textContent = 'Stack';
        tg.addEventListener('click', () => { pre.style.display = pre.style.display === 'none' ? 'block' : 'none'; });
        el.appendChild(tg); el.appendChild(pre);
      }
      errBox.appendChild(el);
    }
  }
  function error(err, context) {
    const isErr = err && typeof err === 'object';
    const message = isErr ? String(err.message || err) : String(err);
    const source = isErr ? (err.source || err.filename || '') : '';
    const line = isErr ? (err.line != null ? err.line : err.lineno) : undefined;
    const stack = isErr && err.stack ? String(err.stack) : '';
    const key = message + '|' + source + '|' + line;
    let e = errors.find((q) => q.key === key);
    if (e) { e.count++; e.dismissed = false; }
    else {
      e = { key, message, source, line, col: isErr ? (err.col != null ? err.col : err.colno) : undefined,
        stack, count: 1, context: context || '', time: new Date(now()).toISOString() };
      errors.push(e);
      if (errors.length > 20) errors.shift();
    }
    log('ERROR ' + message);
    renderErrors();
    return e;
  }
  function getErrors() { return errors.map((e) => ({ ...e })); }
  function reportText() {
    return formatReport(errors, {
      time: new Date(now()).toISOString(),
      url: win && win.location ? win.location.href : '',
      userAgent: nav.userAgent || '',
    });
  }
  function copyReport() {
    const text = reportText();
    try {
      if (nav.clipboard && nav.clipboard.writeText) return nav.clipboard.writeText(text);
    } catch (_) { /* ignore */ }
    return null;
  }

  // ---- physics HUD and report panel (validation-tuning)
  function setPhysicsStats(st) {
    if (st == null) { physBox.style.display = 'none'; physBox.textContent = ''; return; }
    physBox.textContent = formatPhysicsStats(st).join('\n');
    physBox.style.display = 'block';
  }
  let reportBox = null;
  function showReport(title, lines) {
    if (reportBox) { reportBox.style.display = 'none'; reportBox = null; }
    const box = mk('div', 'dg-report');
    const head = mk('b'); head.textContent = String(title);
    const pre = mk('pre'); const text = (lines || []).map(String).join('\n'); pre.textContent = text;
    const copy = mk('button'); copy.textContent = 'Copy';
    copy.addEventListener('click', () => {
      try { if (nav.clipboard && nav.clipboard.writeText) nav.clipboard.writeText(text); } catch (_) { /* ignore */ }
    });
    const close = mk('button'); close.textContent = 'Dismiss';
    close.addEventListener('click', () => { box.style.display = 'none'; if (reportBox === box) reportBox = null; });
    box.appendChild(head); box.appendChild(pre); box.appendChild(copy); box.appendChild(close);
    root.appendChild(box);
    reportBox = box;
  }

  // ---- boot watchdog
  let booted = false;
  let watchdog = null;
  function bootDone() {
    booted = true;
    if (watchdog != null) { clearT(watchdog); watchdog = null; }
    const l = doc.getElementById && doc.getElementById('loading');
    if (l) l.style.display = 'none';
  }
  watchdog = setT(() => {
    watchdog = null;
    if (booted) return;
    toast('The sim did not finish starting (see errors above); check the console', 'bad', { sticky: true, id: 'boot' });
    const l = doc.getElementById && doc.getElementById('loading');
    if (l) l.textContent = 'The sim did not finish starting (see errors above); check the console';
    if (!errors.length) error(new Error('Boot timeout: bootDone() was never called'), 'watchdog');
  }, bootTimeoutMs);

  // ---- hooks
  if (win && win.addEventListener) {
    win.addEventListener('error', (ev) => {
      const er = ev && ev.error;
      error({ message: (er && er.message) || ev.message || 'Error', source: ev.filename, line: ev.lineno, col: ev.colno, stack: er && er.stack }, 'window.onerror');
    });
    win.addEventListener('unhandledrejection', (ev) => {
      const r = ev && ev.reason;
      error(r instanceof Error ? r : { message: 'Unhandled rejection: ' + (r && r.message || String(r)), stack: r && r.stack }, 'unhandledrejection');
    });
    win.addEventListener('keydown', (ev) => { if (ev && ev.key === '`') toggleLog(); });
  }
  if (con && typeof con.error === 'function') {
    const orig = con.error;
    con.error = function (...args) {
      try {
        const a = args[0];
        error(a instanceof Error ? a : { message: args.map(String).join(' ') }, 'console.error');
      } catch (_) { /* ignore */ }
      return orig.apply(this, args);
    };
  }

  const diag = { toast, tick, setShotStatus, setSimNote, error, getErrors, log, toggleLog, bootDone, copyReport, reportText,
    setPhysicsStats, showReport,
    format: { formatReport, dedupeToasts, severityTtl, summarizeReasons },
    _els: { toastBox, shotBox, notesBox, errBox, logBox, physBox } };
  if (win) win.__diag = diag;
  return diag;
}
