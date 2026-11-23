// Synthetic data generators and fakes for the validation-tuning tests.
// Units follow the sim: inches, seconds, radians. Every generator is seeded and deterministic.
import { mulberry32, gaussian } from '../../web/js/physics/validation-tuning/rng.js';

const G_IN_S2 = 386.0886; // in/s^2, SOURCED (same value as BALL.g in balls.js)

// Feedforward samples from u = kS*sgn(v) + kV*v + kA*a, integrated with small sub-steps.
// u is smooth and strictly positive so v stays > 0 and no |du| > 0.05 jumps occur at dt = 0.01 s.
export function ffSamples(opts = {}) {
  const { kS = 0.05, kV = 0.0167, kA = 0.004, sigma = 0.02, seed = 7, dt = 0.01, durationS = 8 } = opts; // sigma in in/s (ASSUMED)
  const gauss = gaussian(mulberry32(seed));
  const n = Math.round(durationS / dt);
  const uAt = (t) => 0.55 + 0.3 * Math.sin(2 * Math.PI * t / 2.3) + 0.15 * Math.sin(2 * Math.PI * t / 0.9);
  let v = (uAt(0) - kS) / kV; // start at steady state
  const out = { t: [], u: [], v: [] };
  const sub = 20;
  for (let i = 0; i < n; i++) {
    const t = i * dt;
    out.t.push(t); out.u.push(uAt(t)); out.v.push(v + sigma * gauss());
    for (let s = 0; s < sub; s++) {
      const ts = t + (s * dt) / sub;
      v += (((uAt(ts) - kS * Math.sign(v) - kV * v) / kA) * dt) / sub;
    }
  }
  return out;
}

// Coast-down v(t) = v0 - decel*t until v reaches 0 (in/s, in/s^2).
export function coastSamples(opts = {}) {
  const { v0 = 60, decel = 120, sigma = 0.1, seed = 11, dt = 0.01 } = opts;
  const gauss = gaussian(mulberry32(seed));
  const out = { t: [], v: [] };
  for (let i = 0; v0 - decel * i * dt > 0; i++) { out.t.push(i * dt); out.v.push(v0 - decel * i * dt + sigma * gauss()); }
  return out;
}

// Ballistic flight in the (x,z) plane with quadratic drag dv/dt = -k|v|v - g z_hat, RK4, plus position noise.
export function dragTrack(opts = {}) {
  const { k = 0.0008, v0 = [300, 200], p0 = [0, 20], sigma = 0.05, seed = 5, n = 30, dt = 0.02 } = opts;
  const gauss = gaussian(mulberry32(seed));
  const f = (s) => {
    const sp = Math.hypot(s[2], s[3]);
    return [s[2], s[3], -k * sp * s[2], -k * sp * s[3] - G_IN_S2];
  };
  const add = (s, d, h) => s.map((q, i) => q + d[i] * h);
  let s = [p0[0], p0[1], v0[0], v0[1]];
  const out = { t: [], x: [], z: [] };
  const sub = 20, h = dt / sub;
  for (let i = 0; i < n; i++) {
    out.t.push(i * dt); out.x.push(s[0] + sigma * gauss()); out.z.push(s[1] + sigma * gauss());
    for (let j = 0; j < sub; j++) {
      const k1 = f(s), k2 = f(add(s, k1, h / 2)), k3 = f(add(s, k2, h / 2)), k4 = f(add(s, k3, h));
      s = s.map((q, i2) => q + (h / 6) * (k1[i2] + 2 * k2[i2] + 2 * k3[i2] + k4[i2]));
    }
  }
  return out;
}

// ---------- fake DOM (superset of the pattern in tests_js/diag.test.mjs) ----------
class El {
  constructor(tag) {
    this.tag = tag; this.tagName = String(tag).toUpperCase(); this.children = []; this.style = {}; this.attrs = {};
    this.listeners = {}; this.className = ''; this._text = ''; this.parentNode = null; this.dataset = {};
    this.value = ''; this.checked = false; this.disabled = false; this.files = [];
  }
  get classList() {
    const s = this;
    const list = () => s.className.split(' ').filter(Boolean);
    return {
      contains: (c) => list().includes(c),
      add: (...cs) => { const l = list(); cs.forEach((c) => { if (!l.includes(c)) l.push(c); }); s.className = l.join(' '); },
      remove: (...cs) => { s.className = list().filter((c) => !cs.includes(c)).join(' '); },
      toggle: (c, force) => {
        const has = list().includes(c); const want = force === undefined ? !has : !!force;
        if (want && !has) s.className = list().concat(c).join(' ');
        else if (!want && has) s.className = list().filter((x) => x !== c).join(' ');
        return want;
      },
    };
  }
  setAttribute(k, v) { this.attrs[k] = v; if (k === 'class') this.className = String(v); if (k === 'id') this.id = String(v); }
  getAttribute(k) { return this.attrs[k] ?? null; }
  appendChild(c) { if (c.parentNode) c.parentNode.removeChild(c); c.parentNode = this; this.children.push(c); return c; }
  append(...cs) { cs.forEach((c) => { if (typeof c === 'string') { const t = new El('#text'); t._text = c; this.appendChild(t); } else this.appendChild(c); }); }
  removeChild(c) { const i = this.children.indexOf(c); if (i >= 0) this.children.splice(i, 1); c.parentNode = null; return c; }
  remove() { if (this.parentNode) this.parentNode.removeChild(this); }
  addEventListener(t, f) { (this.listeners[t] ||= []).push(f); }
  removeEventListener(t, f) { this.listeners[t] = (this.listeners[t] || []).filter((x) => x !== f); }
  dispatch(t, ev = {}) { (this.listeners[t] || []).forEach((f) => f({ target: this, currentTarget: this, preventDefault() {}, stopPropagation() {}, ...ev })); }
  click() { this.dispatch('click'); }
  set textContent(v) { this._text = String(v); this.children = []; }
  get textContent() { return this._text + this.children.map((c) => c.textContent).join(''); }
  set innerHTML(v) { this._html = v; this.children = []; this._text = ''; }
  get innerHTML() { return this._html || ''; }
  findAll(pred, acc = []) { for (const c of this.children) { if (pred(c)) acc.push(c); c.findAll(pred, acc); } return acc; }
  querySelector(sel) { return this.querySelectorAll(sel)[0] || null; }
  querySelectorAll(sel) {
    if (sel.startsWith('.')) { const cls = sel.slice(1); return this.findAll((c) => c.className.split(' ').includes(cls)); }
    return this.findAll((c) => c.tag === sel);
  }
  focus() {}
  blur() {}
}

export function fakeDoc() {
  const loading = new El('div');
  const listeners = {};
  const doc = {
    body: new El('body'), head: new El('head'), documentElement: new El('html'),
    createElement: (t) => new El(t),
    createElementNS: (_ns, t) => new El(t),
    createTextNode: (s) => { const e = new El('#text'); e._text = String(s); return e; },
    getElementById: (id) => (id === 'loading' ? loading : doc.body.findAll((c) => c.id === id)[0] || null),
    addEventListener: (t, f) => (listeners[t] ||= []).push(f),
    removeEventListener: (t, f) => { listeners[t] = (listeners[t] || []).filter((x) => x !== f); },
    fire: (t, ev = {}) => (listeners[t] || []).forEach((f) => f({ preventDefault() {}, ...ev })),
    listeners,
  };
  return doc;
}

// ---------- fake THREE: just enough of BufferGeometry / LineSegments for the overlay adapter ----------
export function fakeThree() {
  class BufferAttribute {
    constructor(array, itemSize) { this.array = array; this.itemSize = itemSize; this.count = array.length / itemSize; this.needsUpdate = false; this.usage = null; }
    setUsage(u) { this.usage = u; return this; }
  }
  class BufferGeometry {
    constructor() { this.attributes = {}; this.drawRange = { start: 0, count: Infinity }; this.disposed = false; }
    setAttribute(name, attr) { this.attributes[name] = attr; return this; }
    getAttribute(name) { return this.attributes[name]; }
    setDrawRange(start, count) { this.drawRange = { start, count }; }
    computeBoundingSphere() {}
    computeBoundingBox() {}
    dispose() { this.disposed = true; }
  }
  class LineBasicMaterial { constructor(p = {}) { Object.assign(this, p); this.disposed = false; } dispose() { this.disposed = true; } }
  class Object3D {
    constructor() {
      this.children = []; this.parent = null; this.visible = true; this.renderOrder = 0; this.frustumCulled = true;
      this.add = (c) => { c.parent = this; this.children.push(c); return this; };
      this.remove = (c) => { const i = this.children.indexOf(c); if (i >= 0) this.children.splice(i, 1); c.parent = null; return this; };
    }
  }
  class LineSegments extends Object3D { constructor(geometry, material) { super(); this.geometry = geometry; this.material = material; this.isLineSegments = true; } }
  class Scene extends Object3D {}
  class Group extends Object3D {}
  return { BufferAttribute, Float32BufferAttribute: BufferAttribute, BufferGeometry, LineBasicMaterial, LineSegments, Object3D, Scene, Group, DynamicDrawUsage: 35048 };
}

// ---------- storage fakes ----------
export function memStorage() {
  const m = new Map();
  return {
    getItem: (k) => (m.has(k) ? m.get(k) : null),
    setItem: (k, v) => { m.set(k, String(v)); },
    removeItem: (k) => { m.delete(k); },
    clear: () => m.clear(),
    key: (i) => [...m.keys()][i] ?? null,
    get length() { return m.size; },
  };
}

export function throwingStorage() {
  const boom = () => { throw new Error('storage blocked'); };
  return { getItem: boom, setItem: boom, removeItem: boom, clear: boom, key: boom, get length() { return boom(); } };
}
