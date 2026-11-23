// Shooter exit model: flywheel rpm -> ball exit speed, backspin, rpm drop and contact time.
// 'legacy' reproduces shotrules speedFromRpm bit for bit. 'physical' uses the energy-transfer factor from
// tervay/recalc ballShot.ts (https://raw.githubusercontent.com/tervay/recalc/main/app/lib/math/ballShot.ts), with an
// ASSUMED compression/slip efficiency (see params.js) calibrated so pollen equals legacy.
import { BALL_KINDS, launchScaleFor, LAUNCH_REF_KIND } from '../../balls.js';
import { PARAMS } from './params.js';

const IN = 0.0254;   // m per in
const kindOk = (kind) => (BALL_KINDS[kind] ? kind : LAUNCH_REF_KIND);
const P = (opts) => (opts && opts.params) || PARAMS;

/** Wheel angular speed in rad/s. */
export function wheelOmega(rpm, rpmScale = 1) { return Math.max(0, rpm) / (rpmScale || 1) * 2 * Math.PI / 60; }

/** Ball compression c = (2 rB - gap) / (2 rB), unitless. */
export function compressionOf(kind, gapIn, params = PARAMS) {
  const k = kindOk(kind), d = 2 * BALL_KINDS[k].radiusIn;
  const gap = gapIn != null ? gapIn : params.compression.GAP_IN[k];
  return (d - gap) / d;
}

/** Grip efficiency vs compression (ASSUMED shape), clamped to [0.05, 1]. */
export function etaCompression(c, params = PARAMS) {
  const q = params.compression, cc = Math.max(c, 0), over = Math.max(0, cc - q.C_OPT);
  const e = (1 - Math.exp(-cc / q.C0)) * (1 - q.K_OVER * over * over);
  return Math.min(1, Math.max(0.05, e));
}

/** Slip efficiency vs wheel surface speed (in/s). V_SLIP_INS = Infinity gives 1. */
export function etaSlip(vSurfIns, params = PARAMS) {
  const V = params.slip.V_SLIP_INS;
  if (!isFinite(V)) return 1;
  const r = vSurfIns / V;
  return 1 / Math.sqrt(1 + r * r);
}

// Ideal surface kinematics at omega1 = 1 rad/s (everything scales with omega, so s is rpm independent).
// Returns {s, k, a, b}: a [m/s per rad/s] and b [rad/s per rad/s] are the no-slip centre speed and backspin.
function kinematics(kind, opts) {
  const p = P(opts), w = p.wheel, k = kindOk(kind), B = BALL_KINDS[k];
  const mB = B.massKg, rB = B.radiusM, iB = p.ball.MOI_FACTOR * mB * rB * rB;
  const rW1 = (opts.wheelR != null ? opts.wheelR : w.R_IN) * IN;
  let a, b, E0, kk;
  if (w.CONFIG === 'dual') {
    const rW2 = w.R2_IN * IN;
    const ratio = (opts.rpm2 != null && opts.rpm > 0) ? opts.rpm2 / opts.rpm : w.SPEED_RATIO_2;
    a = (rW1 + rW2 * ratio) / 2; b = (rW1 - rW2 * ratio) / (2 * rB);
    E0 = 0.5 * (w.MOI_KGM2 + w.MOI2_KGM2 * ratio * ratio); kk = 1.0;
  } else {
    a = rW1 / 2; b = rW1 / (2 * rB);
    E0 = 0.5 * w.MOI_KGM2; kk = 0.5;
  }
  const K = 0.5 * mB * a * a + 0.5 * iB * b * b;
  return { s: Math.sqrt(E0 / (E0 + K)), k: kk, a, b };
}

/** Energy-transfer factor s in (0, 1]: every wheel slows by s while the ball takes its kinetic energy. */
export function transferFactor(kind, rpm, opts = {}) {
  return kinematics(kind, { ...opts, rpm }).s;
}

/** Calibration factor so that the reference kind (pollen) equals the legacy speed. */
export function etaScale(opts = {}) {
  const p = P(opts), m = p.model;
  if (!m.CALIBRATE_TO_LEGACY) return m.ETA_SCALE_MANUAL;
  const eff = opts.eff != null ? opts.eff : m.LEGACY_EFF;
  const ref = m.CALIBRATION_KIND;
  const kin = kinematics(ref, { ...opts, rpm: undefined, rpm2: undefined });
  return eff / (kin.k * kin.s * etaCompression(compressionOf(ref, undefined, p), p));
}

const modelOf = (opts) => opts.model || P(opts).model.DEFAULT_EXIT_MODEL;

/** Contact time (s) of the ball on the wheel, ASSUMED wrap arc at half the exit speed. */
export function contactTimeS(speed, kind, opts = {}) {
  const p = P(opts), k = kindOk(kind);
  const gap = opts.gapIn != null ? opts.gapIn : p.compression.GAP_IN[k];
  const rW = opts.wheelR != null ? opts.wheelR : p.wheel.R_IN;
  return (p.hood.WRAP_DEG * Math.PI / 180) * (rW + gap / 2) / (0.5 * Math.max(speed, 1));
}

export function exitState(rpm, kind, opts = {}) {
  const p = P(opts), k = kindOk(kind), model = modelOf(opts);
  const rpmScale = opts.rpmScale || 1;
  const rW = opts.wheelR != null ? opts.wheelR : p.wheel.R_IN;
  const c = compressionOf(k, opts.gapIn, p);
  const omega = wheelOmega(rpm, rpmScale);
  const vSurf = rW * omega;
  if (model !== 'physical') {
    const eff = opts.eff != null ? opts.eff : p.model.LEGACY_EFF;
    const ms = launchScaleFor(k);
    // same operation order as shotrules speedFromRpm
    const speed = Math.max(0, rpm) / rpmScale * 2 * Math.PI * rW * eff / 60 * ms;
    return { model: 'legacy', speed, spinRadS: 0, s: ms, eta: eff, compression: c, vSurf,
      rpmAfter: rpm * (1 - p.flywheel.RPM_DIP_FRAC_LEGACY), contactTimeS: contactTimeS(speed, k, opts) };
  }
  const kin = kinematics(k, { ...opts, rpm });
  const eta = etaScale(opts) * etaCompression(c, p) * etaSlip(vSurf, p);
  const speed = eta * kin.s * (kin.a / IN) * omega;
  return { model: 'physical', speed, spinRadS: eta * kin.s * kin.b * omega, s: kin.s, eta, compression: c, vSurf,
    rpmAfter: rpm * kin.s, contactTimeS: contactTimeS(speed, k, opts) };
}

export function exitSpeed(rpm, kind, opts = {}) { return exitState(rpm, kind, opts).speed; }

/** Exact inverse of exitSpeed. Closed form when linear in rpm, bisection otherwise. NaN if unreachable. */
export function rpmForExitSpeed(speed, kind, opts = {}) {
  if (!(speed >= 0)) return NaN;
  const p = P(opts), k = kindOk(kind), rpmScale = opts.rpmScale || 1;
  if (modelOf(opts) !== 'physical') {
    const eff = opts.eff != null ? opts.eff : p.model.LEGACY_EFF;
    const rW = opts.wheelR != null ? opts.wheelR : p.wheel.R_IN;
    return 60 * (speed / launchScaleFor(k)) / (2 * Math.PI * rW * eff) * rpmScale;
  }
  if (speed === 0) return 0;
  if (!isFinite(p.slip.V_SLIP_INS)) {
    const probe = 1000, v = exitSpeed(probe, k, opts);
    return v > 0 ? speed * probe / v : NaN;
  }
  const maxRpm = opts.maxRpm != null ? opts.maxRpm : 6000;
  let lo = 0, hi = 4 * maxRpm;
  if (exitSpeed(hi, k, opts) < speed) return NaN;
  for (let i = 0; i < 60; i++) {
    const mid = 0.5 * (lo + hi);
    if (exitSpeed(mid, k, opts) < speed) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);
}

/** Flywheel rpm right after a shot. Legacy: flat dip. Physical: rpm * s (energy that left the wheel). */
export function rpmAfterShot(rpm, kind, opts = {}) {
  const p = P(opts);
  if (modelOf(opts) !== 'physical') return rpm * (1 - p.flywheel.RPM_DIP_FRAC_LEGACY);
  return rpm * transferFactor(kind, rpm, opts);
}
