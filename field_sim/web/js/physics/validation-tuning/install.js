// install.js - live validation tools: step timing, invariant counting, debug overlay, tuning panel, HUD.
//
// Integration (one line, in app.js after the world and drive exist; app.js is not owned by this area):
//   installValidationTools({ THREE, scene3d: <three scene>, world, getDrive: () => drive, diag, doc: document, win: window });
// then call `tools.tick(frameMs)` once per frame AFTER the physics update and BEFORE render.
// F8 toggles the overlay, F9 toggles the tuning panel. The tools are also exposed as win.__vt.
//
// The tools only READ world state. The one thing they change is world.step, wrapped on the instance to time it;
// dispose() restores it. The panel writes BALL/drive values only when the user moves a slider.
import { BALL } from '../../balls.js';
import { LAYERS, OVERLAY, PARAM_SCHEMA, schemaEntry } from './params.js';
import { checkWorld, snapshotPositions } from './invariants.js';
import { buildDebugGeometry, createOverlayState, energyReadout, probeContacts } from './overlay.js';
import { createThreeOverlay } from './overlay_three.js';
import { createTuningPanel } from './tuning_panel.js';

function pct(sorted, q) { // quantile, type 7
  if (!sorted.length) return 0;
  const h = (sorted.length - 1) * q, lo = Math.floor(h), hi = Math.ceil(h);
  return sorted[lo] + (sorted[hi] - sorted[lo]) * (h - lo);
}

export function installValidationTools(opts) {
  const { THREE, scene3d, world, getDrive, diag } = opts;
  const doc = opts.doc || (typeof document !== 'undefined' ? document : null);
  const win = opts.win || (typeof window !== 'undefined' ? window : null);
  let layers = opts.layers != null ? opts.layers : LAYERS.DEFAULT;
  const now = (win && win.performance && win.performance.now) ? () => win.performance.now()
    : (typeof performance !== 'undefined' ? () => performance.now() : () => Date.now());

  // ---- wrap world.step on the instance (stepFixed reaches it through this.step)
  const hadOwnStep = Object.prototype.hasOwnProperty.call(world, 'step');
  const origStep = world.step;
  let pending = []; // ms of each step() call since the last tick
  world.step = function wrappedStep(...args) {
    const t0 = now();
    const r = origStep.apply(this, args);
    pending.push(now() - t0);
    return r;
  };

  const state = createOverlayState();
  const ring = []; // last OVERLAY.stepWindow step times, ms
  let prevSnap = null;
  let violations = 0, maxPenSinceHud = 0, lastCheck = { ok: true, maxPen: 0 };
  let frame = 0;
  let lastStats = {};

  const overlay = (THREE && scene3d) ? createThreeOverlay(THREE, scene3d) : null;

  let panel = null;
  if (doc) {
    const driveOf = () => (getDrive ? getDrive() : null);
    panel = createTuningPanel({
      doc,
      schema: PARAM_SCHEMA,
      getValue(key) {
        const e = schemaEntry(key);
        const o = e.group === 'ball' ? BALL : driveOf();
        return o ? o[e.field] : NaN;
      },
      setValue(key, v) {
        const e = schemaEntry(key);
        const o = e.group === 'ball' ? BALL : driveOf();
        if (o) o[e.field] = v;
      },
    });
  }

  function computeStats() {
    const sorted = ring.slice().sort((a, b) => a - b);
    const balls = (world.balls || []).filter((b) => b.state === 'free');
    const e = energyReadout(world);
    return {
      stepMs: pct(sorted, 0.5), stepMsP95: pct(sorted, 0.95),
      balls: balls.length, awake: balls.filter((b) => !b.sleeping).length,
      contacts: probeContacts(world).length,
      energyJ: e.totalJ, maxPenIn: maxPenSinceHud, violations,
    };
  }

  function tick(frameMs) {
    frame++;
    // (1) step times into the ring
    for (const ms of pending) { ring.push(ms); if (ring.length > OVERLAY.stepWindow) ring.shift(); }
    pending = [];
    // (2) invariants, no energy (the scene is open)
    lastCheck = checkWorld(world, prevSnap);
    violations += lastCheck.violations ? lastCheck.violations.length : 0;
    if (lastCheck.maxPen > maxPenSinceHud) maxPenSinceHud = lastCheck.maxPen;
    prevSnap = snapshotPositions(world);
    // (3) overlay
    if (overlay && overlay.visible) {
      overlay.update(buildDebugGeometry(world, { layers, state, dt: (frameMs || 0) / 1000 }));
    }
    // (4) HUD
    if (frame % OVERLAY.hudEveryFrames === 0) {
      lastStats = computeStats();
      maxPenSinceHud = 0;
      if (diag && diag.setPhysicsStats) diag.setPhysicsStats(lastStats);
    }
  }

  function toggleOverlay() { if (overlay) overlay.setVisible(!overlay.visible); }
  function togglePanel() { if (panel) panel.toggle(); }

  function onKey(ev) {
    if (!ev) return;
    if (ev.key === OVERLAY.keyOverlay) { if (ev.preventDefault) ev.preventDefault(); toggleOverlay(); }
    else if (ev.key === OVERLAY.keyPanel) { if (ev.preventDefault) ev.preventDefault(); togglePanel(); }
  }
  if (win && win.addEventListener) win.addEventListener('keydown', onKey);

  const api = {
    tick,
    setLayers(mask) { layers = mask | 0; },
    toggleOverlay, togglePanel,
    stats() { return { ...lastStats }; },
    overlay, panel,
    dispose() {
      if (hadOwnStep) world.step = origStep; else delete world.step;
      if (win && win.removeEventListener) win.removeEventListener('keydown', onKey);
      if (overlay) overlay.dispose();
      if (panel) panel.destroy();
      if (diag && diag.setPhysicsStats) diag.setPhysicsStats(null);
      if (win && win.__vt === api) delete win.__vt;
    },
  };
  if (win) win.__vt = api;
  return api;
}
