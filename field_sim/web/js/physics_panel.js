// physics_panel.js - "Physics models" card: one control per physics_glue.js switch, plus a live telemetry block.
// DOM only (no three.js). Built into index.html #phys-panel by app.js. Settings persist per viewer in localStorage
// (a convenience only: every read/write is guarded, and the page works without it).
import { PHYS_OPTIONS } from './physics_glue.js';

const STORE_KEY = 'fieldsim.physics.v1';

/** Label + help text per switch. The help text names the area module it turns on. */
const ROWS = [
  { key: 'fixedStep', label: 'Fixed timestep', help: 'timestep-determinism FixedClock: the whole sim steps at a fixed dt; off = one step of the frame time' },
  { key: 'fixedHz', label: 'Step rate [Hz]', help: 'fixed step rate; 60 Hz matches the auto runner' },
  { key: 'renderInterp', label: 'Smooth drawing', help: 'timestep-determinism: draw the robot and balls between the last two fixed steps (FixedClock alpha); visual only' },
  { key: 'driveModel', label: 'Drive', help: 'motor = motor-model DC motors + gearbox + traction cap; dynamics = mecanum-dynamics rigid body with roller and rolling losses; friction = wheel-friction per-wheel Stribeck friction and load transfer; kinematic = old slew-rate model (the only one the speed/accel sliders change). All physical models use robot_spec.json drive.gearing and the battery bus voltage' },
  { key: 'robotCollision', label: 'Robot vs field', help: 'legacy = pose correction + slide; contact = wall-collisions contact solver (impulses, friction, small bounce); rigid = rigid-body-solver world' },
  { key: 'batterySag', label: 'Battery sag', help: 'battery-sag: bus voltage under load, fuse, hub brownout; limits the drive' },
  { key: 'batteryPreset', label: 'Battery', help: 'battery charge / age preset' },
  { key: 'weightTransfer', label: 'Weight transfer', help: 'weight-transfer: wheel loads, load-aware traction limit, chassis pitch/roll, tip-over' },
  { key: 'flywheelModel', label: 'Flywheel', help: 'dynamics = flywheel-dynamics motor plant; pidf = velocity-pidf repo SDK PIDF loop with encoder; legacy = first-order lag' },
  { key: 'exitModel', label: 'Ball exit', help: 'shooter-exit: physical = slip/compression exit speed, hood map and exit geometry; legacy = rpm x efficiency' },
  { key: 'odometry', label: 'Pinpoint odometry', help: 'pinpoint-odometry: pods, IMU, firmware rate, host read latency; off = perfect pose' },
  { key: 'imuModel', label: 'Pinpoint IMU', help: 'basic = pinpoint-odometry yaw model; gyro = imu-heading MEMS gyro (bias drift, scale error, temperature, shocks)' },
  { key: 'aimFromOdometry', label: 'Aim from odometry', help: 'the shot solver aims from the believed pose, like the robot code' },
  { key: 'autoFollower', label: 'Auto follower', help: 'auto runner path follower: auto = replay a Pedro trajectory if there is one, else trapezoid; pedro = closed-loop Pedro 2.1.0 follower (pedro-fidelity). A per-auto setting still wins' },
  { key: 'pedroPlant', label: 'Pedro plant', help: 'pedro-fidelity drivetrain plant: spec = track/wheelbase/traction from robot_spec.json; library = the area defaults (ESTIMATE). Pedro follower constants are the 2.1.0 library values either way' },
  { key: 'ballAero', label: 'Ball flight', help: 'ball-aerodynamics: legacy = the drag the shot solver was tuned to; physical = air-density drag (about 2.7x) + Magnus, shots will miss the solver aim' },
  { key: 'ballBall', label: 'Ball-ball contacts', help: 'ball-ball-collisions: legacy = Jacobi passes; solver = sequential impulses + island sleep (known low restitution in free flight)' },
  { key: 'hiveTipModel', label: 'Hive tip rule', help: 'hive-physics: rule = 8-pollen rule decides when; torque = ball torque decides' },
  { key: 'hiveMotion', label: 'Hive motion', help: 'hive-physics: dynamic = rigid-body rotation; ease = old animation' },
  { key: 'validationTools', label: 'Validation tools', help: 'validation-tuning: contact/velocity overlay (F8) and tuning panel (F9)' },
];

export function loadPhysicsSettings(storage) {
  try { const s = storage && storage.getItem(STORE_KEY); return s ? JSON.parse(s) : {}; } catch (_) { return {}; }
}
export function savePhysicsSettings(storage, cfg) {
  try { if (storage) storage.setItem(STORE_KEY, JSON.stringify(cfg)); } catch (_) { /* private window / blocked storage */ }
}

/**
 * @param {Document} doc
 * @param {HTMLElement} root     container for the controls
 * @param {HTMLElement|null} telEl  container for the telemetry lines
 * @param {PhysicsRuntime} rt
 * @param {(key:string, value:any) => void} onChange   called after rt.set()
 * @param {Storage|null} storage
 */
export function createPhysicsPanel(doc, root, telEl, rt, onChange, storage = null) {
  const inputs = {};
  root.textContent = '';
  for (const r of ROWS) {
    const cur = rt.cfg[r.key];
    if (typeof cur === 'boolean') {
      const lab = doc.createElement('label'); lab.className = 't'; lab.title = r.help;
      const cb = doc.createElement('input'); cb.type = 'checkbox'; cb.checked = cur;
      cb.onchange = () => apply(r.key, cb.checked);
      lab.append(cb, doc.createTextNode(' ' + r.label));
      root.append(lab); inputs[r.key] = cb;
    } else {
      const row = doc.createElement('div'); row.className = 'row'; row.title = r.help;
      const sp = doc.createElement('span'); sp.textContent = r.label;
      const sel = doc.createElement('select'); sel.setAttribute('aria-label', r.label);
      for (const v of PHYS_OPTIONS[r.key]) { const o = doc.createElement('option'); o.value = String(v); o.textContent = String(v); sel.append(o); }
      sel.value = String(cur);
      sel.onchange = () => apply(r.key, typeof cur === 'number' ? Number(sel.value) : sel.value);
      row.append(sp, sel); root.append(row); inputs[r.key] = sel;
    }
  }
  function apply(key, value) {
    const got = rt.set(key, value);
    const el = inputs[key];
    if (el.type === 'checkbox') el.checked = got; else el.value = String(got);
    const persist = {}; for (const r of ROWS) persist[r.key] = rt.cfg[r.key];
    savePhysicsSettings(storage, persist);
    if (onChange) onChange(key, got);
  }
  return {
    setTelemetry(lines) {
      if (!telEl) return;
      telEl.textContent = '';
      for (const l of lines) { const d = doc.createElement('div'); d.textContent = l; telEl.append(d); }
    },
    refresh() { for (const r of ROWS) { const el = inputs[r.key]; if (el.type === 'checkbox') el.checked = rt.cfg[r.key]; else el.value = String(rt.cfg[r.key]); } },
  };
}
