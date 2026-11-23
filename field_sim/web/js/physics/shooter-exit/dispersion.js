// Shot dispersion: sigmas, launch perturbation, analytic and Monte Carlo hit probability.
// Units: inches, seconds, radians. All sigma defaults live in params.js (assumed, with measure hints).
import { PARAMS } from './params.js';
import { compressionOf, etaCompression } from './exit_model.js';
import { BALL_KINDS } from '../../balls.js';
import { mulberry32, makeGaussian, normCdf } from './rng.js';

const D2R = Math.PI / 180;

// 1-sigma shot errors for a ball kind. speedFrac is dimensionless, angles in rad, feedS in s.
export function shotSigmas(kind, opts = {}) {
  const params = opts.params || PARAMS;
  const dp = params.dispersion;
  const gap = opts.gapIn ?? params.compression.GAP_IN[kind];
  const c = compressionOf(kind, gap, params);
  const C0 = params.compression.C0;
  const h = 1e-4;
  const dLnEta = (Math.log(etaCompression(c + h, params)) - Math.log(etaCompression(c - h, params))) / (2 * h);
  const dIn = 2 * BALL_KINDS[kind].radiusM / 0.0254;   // ball diameter, in
  const dcdd = gap / (dIn * dIn);
  const base = dp.SIGMA_SPEED_FRAC + dp.SIGMA_SPEED_FRAC_LOWCOMP * Math.exp(-c / C0);
  const diam = dLnEta * dcdd * params.ball.DIAM_SIGMA_IN;
  const speedFrac = Math.sqrt(base * base + diam * diam);
  const hd = opts.hoodDeadbandDeg || 0, td = opts.turretDeadbandDeg || 0;
  const elevRad = D2R * Math.sqrt(dp.SIGMA_ELEV_DEG ** 2 + (hd / Math.sqrt(12)) ** 2);
  const azRad = D2R * Math.sqrt(dp.SIGMA_AZ_DEG ** 2 + (td / Math.sqrt(12)) ** 2);
  return { speedFrac, elevRad, azRad, feedS: dp.SIGMA_FEED_S };
}

// Opening coordinates, same lat/up frame as shotrules landing(). Returns [a (lateral), v (up)] in inches.
export function openingCoords(p, target) {
  const n = target.normal, hn = Math.hypot(n[0], n[1]) || 1;
  const lat = [-n[1] / hn, n[0] / hn, 0];
  const up = [n[1] * lat[2] - n[2] * lat[1], n[2] * lat[0] - n[0] * lat[2], n[0] * lat[1] - n[1] * lat[0]];
  const d = [p[0] - target.x, p[1] - target.y, p[2] - target.z];
  return [d[0] * lat[0] + d[1] * lat[1] + d[2] * lat[2], d[0] * up[0] + d[1] * up[1] + d[2] * up[2]];
}

// launch = { pos:[x,y,z], speed in/s, elevRad, azRad, vBase:[vx,vy,vz] }
export function launchVel(l) {
  const ce = Math.cos(l.elevRad), vb = l.vBase || [0, 0, 0];
  return [l.speed * ce * Math.cos(l.azRad) + vb[0], l.speed * ce * Math.sin(l.azRad) + vb[1], l.speed * Math.sin(l.elevRad) + vb[2]];
}

export function perturbLaunch(l, sig, gauss) {
  const speed = l.speed * (1 + sig.speedFrac * gauss());
  const elevRad = l.elevRad + sig.elevRad * gauss();
  const azRad = l.azRad + sig.azRad * gauss();
  const dt = sig.feedS * gauss();
  const vb = l.vBase || [0, 0, 0];
  return { ...l, speed, elevRad, azRad, vBase: vb.slice(), pos: [l.pos[0] + vb[0] * dt, l.pos[1] + vb[1] * dt, l.pos[2] + vb[2] * dt] };
}

function crossCoords(l, target, core) {
  const c = core.planeCrossing(l.pos, launchVel(l), target);
  return c ? openingCoords(c.p, target) : null;
}

// Linearised (Jacobian) hit probability; marginal Phi-difference product, correlation reported only.
export function analyticHitProbability({ launch, sig, target, ballR, core }) {
  const hw = (target.halfW || 10) - ballR, hh = (target.halfH || 7) - ballR;
  const mu = crossCoords(launch, target, core);
  if (!mu) return { p: 0, mu: null, sigma: [0, 0], rho: 0, J: [[0, 0, 0, 0], [0, 0, 0, 0]] };
  const vb = launch.vBase || [0, 0, 0];
  const steps = [1e-3 * launch.speed, 1e-5, 1e-5, 1e-4];
  const make = (i, s) => {
    const l = { ...launch, vBase: vb.slice(), pos: launch.pos.slice() };
    if (i === 0) l.speed += s;
    else if (i === 1) l.elevRad += s;
    else if (i === 2) l.azRad += s;
    else l.pos = [launch.pos[0] + vb[0] * s, launch.pos[1] + vb[1] * s, launch.pos[2] + vb[2] * s];
    return l;
  };
  const J = [[0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i++) {
    const hp = crossCoords(make(i, steps[i]), target, core), hm = crossCoords(make(i, -steps[i]), target, core);
    let dp = null;
    if (hp && hm) dp = [(hp[0] - hm[0]) / (2 * steps[i]), (hp[1] - hm[1]) / (2 * steps[i])];
    else if (hp) dp = [(hp[0] - mu[0]) / steps[i], (hp[1] - mu[1]) / steps[i]];
    else if (hm) dp = [(mu[0] - hm[0]) / steps[i], (mu[1] - hm[1]) / steps[i]];
    if (dp) { J[0][i] = dp[0]; J[1][i] = dp[1]; }
  }
  const sg = [launch.speed * sig.speedFrac, sig.elevRad, sig.azRad, sig.feedS];
  const cov = (r, s) => { let t = 0; for (let i = 0; i < 4; i++) t += J[r][i] * J[s][i] * sg[i] * sg[i]; return t; };
  const s00 = cov(0, 0), s11 = cov(1, 1), s01 = cov(0, 1);
  const sa = Math.sqrt(s00), sv = Math.sqrt(s11);
  const rho = sa > 0 && sv > 0 ? s01 / (sa * sv) : 0;
  const marg = (m, s, half) => s > 0 ? normCdf((half - m) / s) - normCdf((-half - m) / s) : (Math.abs(m) <= half ? 1 : 0);
  const p = Math.min(1, Math.max(0, marg(mu[0], sa, hw) * marg(mu[1], sv, hh)));
  return { p, mu, sigma: [sa, sv], rho, J };
}

// Monte Carlo over caller-supplied simulate(launch) -> {lands, miss:[a,v]|null}.
export function monteCarloHit({ launch, sig, n = PARAMS.dispersion.MC_SAMPLES, seed = PARAMS.dispersion.SEED, simulate }) {
  const gauss = makeGaussian(mulberry32(seed));
  let hits = 0, k = 0, sa = 0, sv = 0, qa = 0, qv = 0;
  for (let i = 0; i < n; i++) {
    const r = simulate(perturbLaunch(launch, sig, gauss));
    if (r.lands) hits++;
    if (r.miss) { k++; sa += r.miss[0]; sv += r.miss[1]; qa += r.miss[0] ** 2; qv += r.miss[1] ** 2; }
  }
  const sd = (s, q) => k > 1 ? Math.sqrt(Math.max(0, (q - s * s / k) / (k - 1))) : 0;
  return { p: n > 0 ? hits / n : 0, hits, n, spread: [sd(sa, qa), sd(sv, qv)] };
}
