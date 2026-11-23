// fit.js - calibration tools for real flywheel logs (e.g. FTC Dashboard velocity traces).
// Pure ES module, no DOM, no dependencies. Inputs are arrays of time (s) and flywheel speed (rpm); outputs are SI.
// Purpose: because no measured FTC flywheel data was found during research (see params.js), these tools let a
// real coast-down, voltage step and shot log calibrate friction (Tc/J, b/J), kV, kA and J.
// Model basis: WPILib SysId simple-motor V = kS*sgn(v) + kV*v + kA*a
//   https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html (SOURCED)
// All functions never throw on bad input; they return NaN fields (or Infinity for recovery) instead.

const RPM_TO_RADS = 2 * Math.PI / 60; // rad/s per rpm (same value as params.js RPM_TO_RADS)
const fin = (x) => typeof x === 'number' && Number.isFinite(x);

function nanFit(extra) { return Object.assign({ n: 0 }, extra); }

/**
 * Fit a coast-down log (motor output zero) to dw/dt = -a - k*w by least squares.
 * dw/dt uses central differences; samples with w < 5% of the maximum speed are ignored.
 * @param {number[]} tS sample times, s (strictly increasing)
 * @param {number[]} rpm flywheel speed, rpm
 * @returns {{aRadS2:number, kPerS:number, rmsRadS2:number, n:number}} a = Tc/J in rad/s^2, k = b/J in 1/s,
 *   rms residual of dw/dt in rad/s^2, number of samples used
 */
export function fitCoastDown(tS, rpm) {
  const bad = nanFit({ aRadS2: NaN, kPerS: NaN, rmsRadS2: NaN });
  if (!Array.isArray(tS) || !Array.isArray(rpm) || tS.length !== rpm.length || tS.length < 3) return bad;
  const N = tS.length;
  const w = rpm.map((r) => r * RPM_TO_RADS);
  let wmax = 0;
  for (const x of w) if (fin(x) && x > wmax) wmax = x;
  if (!(wmax > 0)) return bad;
  const xs = [], ys = [];
  for (let i = 1; i < N - 1; i++) {
    const dt = tS[i + 1] - tS[i - 1];
    if (!(dt > 0) || !fin(w[i]) || !fin(w[i + 1]) || !fin(w[i - 1])) continue;
    if (w[i] < 0.05 * wmax) continue;
    xs.push(w[i]);
    ys.push((w[i + 1] - w[i - 1]) / dt);
  }
  const n = xs.length;
  if (n < 2) return bad;
  // y = c0 + c1*x, with c0 = -a, c1 = -k
  const { c0, c1 } = linreg(xs, ys);
  if (!fin(c0) || !fin(c1)) return bad;
  let ss = 0;
  for (let i = 0; i < n; i++) { const r = ys[i] - (c0 + c1 * xs[i]); ss += r * r; }
  return { aRadS2: -c0, kPerS: -c1, rmsRadS2: Math.sqrt(ss / n), n };
}

function linreg(xs, ys) {
  const n = xs.length;
  let sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (let i = 0; i < n; i++) { sx += xs[i]; sy += ys[i]; sxx += xs[i] * xs[i]; sxy += xs[i] * ys[i]; }
  const den = n * sxx - sx * sx;
  if (!(Math.abs(den) > 1e-300)) return { c0: NaN, c1: NaN };
  const c1 = (n * sxy - sx * sy) / den;
  return { c0: (sy - c1 * sx) / n, c1 };
}

/**
 * Fit a constant-voltage spin-up from rest to w = wSs*(1 - exp(-t/tau)).
 * wSs = mean of the last 10% of samples; tau from linear regression of ln(1 - w/wSs) vs t over
 * samples with 5% < w/wSs < 90%. kV = (V - kS)/wSs (kS = 0 unless given), kA = kV*tau.
 * @param {number[]} tS sample times, s
 * @param {number[]} rpm flywheel speed, rpm
 * @param {number} voltageV applied constant voltage, V
 * @param {{kSVolts?:number}} [opts] kS in V to subtract before computing kV
 * @returns {{tauS:number, omegaSsRadS:number, kVVoltsPerRadS:number, kAVoltsPerRadS2:number, n:number}}
 */
export function fitStepResponse(tS, rpm, voltageV, opts) {
  const bad = nanFit({ tauS: NaN, omegaSsRadS: NaN, kVVoltsPerRadS: NaN, kAVoltsPerRadS2: NaN });
  if (!Array.isArray(tS) || !Array.isArray(rpm) || tS.length !== rpm.length || tS.length < 5 || !fin(voltageV)) return bad;
  const N = tS.length;
  const w = rpm.map((r) => r * RPM_TO_RADS);
  const k0 = Math.max(0, N - Math.max(1, Math.floor(N * 0.1)));
  let s = 0, c = 0;
  for (let i = k0; i < N; i++) if (fin(w[i])) { s += w[i]; c++; }
  if (c === 0) return bad;
  const wss = s / c;
  if (!(wss > 0)) return bad;
  const xs = [], ys = [];
  for (let i = 0; i < N; i++) {
    const f = w[i] / wss;
    if (fin(f) && fin(tS[i]) && f > 0.05 && f < 0.9) { xs.push(tS[i]); ys.push(Math.log(1 - f)); }
  }
  if (xs.length < 2) return bad;
  const { c1 } = linreg(xs, ys);
  if (!fin(c1) || !(c1 < 0)) return bad;
  const tau = -1 / c1;
  const kS = opts && fin(opts.kSVolts) ? opts.kSVolts : 0;
  const kV = (voltageV - kS) / wss;
  return { tauS: tau, omegaSsRadS: wss, kVVoltsPerRadS: kV, kAVoltsPerRadS2: kV * tau, n: xs.length };
}

/**
 * Moment of inertia from the acceleration gain: V = G*Ke*w + (J*R/(count*G*Kt))*alpha,
 * so J = kA*count*G*Kt/R.
 * @param {number} kAVoltsPerRadS2 kA, V per (rad/s^2)
 * @param {{R:number, Kt:number, count:number}} motor R in ohm, Kt in N*m/A
 * @param {number} gearRatio motor rev per wheel rev
 * @returns {number} J, kg*m^2 (NaN on bad input)
 */
export function inertiaFromKa(kAVoltsPerRadS2, motor, gearRatio) {
  if (!motor || !fin(kAVoltsPerRadS2) || !fin(gearRatio) || !(motor.R > 0)) return NaN;
  return kAVoltsPerRadS2 * motor.count * gearRatio * motor.Kt / motor.R;
}

/**
 * J from the coast-down Coulomb term: a = Tc/J so J = Tc/a.
 * @param {number} aRadS2 fitted a, rad/s^2
 * @param {number} knownCoulombNm independently known Coulomb torque, N*m
 * @returns {number} J, kg*m^2 (NaN if a <= 0)
 */
export function jFromCoastDown(aRadS2, knownCoulombNm) {
  if (!fin(aRadS2) || !fin(knownCoulombNm) || !(aRadS2 > 0)) return NaN;
  return knownCoulombNm / aRadS2;
}

/**
 * Extract shot dip and recovery from a speed log.
 * preRpm = mean over the 50 ms before the fire time; minRpm = min within 150 ms after; recoveryS = time from the
 * fire to the first sample after the minimum that is back within 2% of preRpm (Infinity if never).
 * @param {number[]} tS sample times, s
 * @param {number[]} rpm speed, rpm
 * @param {number[]} fireTimesS shot times, s
 * @returns {Array<{fireS:number, preRpm:number, minRpm:number, dipFrac:number, recoveryS:number}>}
 */
export function dipFromLog(tS, rpm, fireTimesS) {
  if (!Array.isArray(tS) || !Array.isArray(rpm) || tS.length !== rpm.length || !Array.isArray(fireTimesS)) return [];
  const N = tS.length;
  const out = [];
  for (const f of fireTimesS) {
    if (!fin(f)) continue;
    let s = 0, c = 0;
    for (let i = 0; i < N; i++) if (tS[i] >= f - 0.05 && tS[i] < f && fin(rpm[i])) { s += rpm[i]; c++; }
    let minR = Infinity, minI = -1;
    for (let i = 0; i < N; i++) {
      if (tS[i] >= f && tS[i] <= f + 0.15 && fin(rpm[i]) && rpm[i] < minR) { minR = rpm[i]; minI = i; }
    }
    if (c === 0 || minI < 0) { out.push({ fireS: f, preRpm: NaN, minRpm: NaN, dipFrac: NaN, recoveryS: Infinity }); continue; }
    const pre = s / c;
    let rec = Infinity;
    for (let i = minI; i < N; i++) {
      if (fin(rpm[i]) && Math.abs(rpm[i] - pre) <= 0.02 * Math.abs(pre)) { rec = tS[i] - f; break; }
    }
    out.push({ fireS: f, preRpm: pre, minRpm: minR, dipFrac: pre > 0 ? 1 - minR / pre : 0, recoveryS: rec });
  }
  return out;
}
