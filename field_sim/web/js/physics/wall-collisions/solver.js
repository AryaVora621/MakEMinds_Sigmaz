// solver.js - sequential-impulse contact solver, one dynamic body vs static geometry (Box2D v3 soft step style).
// No DOM. Units: in, s, rad, lb. Velocity in/s, impulse lb*in/s, angular impulse lb*in^2/s.
// Body = { x, y, h, vx, vy, om, invM, invI }. Normals point from obstacle INTO the robot.
// All tunables come from params.js (P); nothing is hard-coded here.
import { materialFor } from './params.js';
import { cross2 } from './geometry.js';

// Solver2D soft constraint coefficients (box2d.org/posts/2024/02/solver2d). SOURCED (recollection of article).
export function softCoefficients(hertz, zeta, h) {
  const omega = 2 * Math.PI * hertz;
  const a1 = 2 * zeta + h * omega;
  const a2 = h * omega * a1;
  const a3 = 1 / (1 + a2);
  return { biasRate: omega / a1, massScale: a2 * a3, impulseScale: a3 };
}

// Wrap to (-pi, pi].
export function wrapAngle(a) {
  const TWO_PI = 2 * Math.PI;
  if (a > -Math.PI && a <= Math.PI) return a;
  let r = a - TWO_PI * Math.floor((a + Math.PI) / TWO_PI); // [-pi, pi)
  if (r <= -Math.PI) r = Math.PI;
  return r;
}

// Apply point impulse (px,py) at world arm (rx,ry) from centre.
export function applyImpulse(body, px, py, rx, ry) {
  body.vx += body.invM * px;
  body.vy += body.invM * py;
  body.om += body.invI * cross2(rx, ry, px, py);
}

export function prepareContacts(body, manifolds, P, hSub, cache) {
  const cons = [];
  const c = Math.cos(body.h), s = Math.sin(body.h);
  const cap = P.solver.hertzSubstepCap / hSub;
  for (const m of manifolds) {
    const mat = materialFor(P, m.kind);
    const nx = m.normal[0], ny = m.normal[1];
    const tx = -ny, ty = nx;
    const con = {
      key: m.key, kind: m.kind,
      normal: [nx, ny], tangent: [tx, ty],
      mu: mat.mu, e: mat.e,
      soft: softCoefficients(Math.min(mat.hertz, cap), mat.zeta, hSub),
      x0: body.x, y0: body.y, h0: body.h,
      points: []
    };
    for (const pt of m.points) {
      const rl = pt.rLocal;
      const rx = c * rl[0] - s * rl[1];
      const ry = s * rl[0] + c * rl[1];
      const rn = cross2(rx, ry, nx, ny);
      const rt = cross2(rx, ry, tx, ty);
      const kn = body.invM + body.invI * rn * rn;
      const kt = body.invM + body.invI * rt * rt;
      const vpx = body.vx - body.om * ry;
      const vpy = body.vy + body.om * rx;
      const entry = cache ? cache.get(m.key + '#' + pt.id) : undefined;
      let jn = 0, jt = 0;
      if (entry && entry.hSub > 0) {
        const k = hSub / entry.hSub;
        jn = entry.jn * k;
        jt = entry.jt * k;
      }
      con.points.push({
        id: pt.id, rLocal: [rl[0], rl[1]], r: [rx, ry], s0: pt.s,
        normalMass: kn > 0 ? 1 / kn : 0,
        tangentMass: kt > 0 ? 1 / kt : 0,
        relVel: nx * vpx + ny * vpy,
        jn, jt, maxJn: 0, totalJn: 0, totalJt: 0
      });
    }
    cons.push(con);
  }
  return cons;
}

export function warmStart(body, cons) {
  for (const con of cons) {
    const nx = con.normal[0], ny = con.normal[1], tx = con.tangent[0], ty = con.tangent[1];
    for (const p of con.points) {
      applyImpulse(body, p.jn * nx + p.jt * tx, p.jn * ny + p.jt * ty, p.r[0], p.r[1]);
      p.totalJn += p.jn;
      p.totalJt += p.jt;
    }
  }
}

// Max condition number of the 2x2 normal block before it is treated as singular (dimensionless).
// SOURCED (recollection, unverified line by line): Box2D v2.4 b2_maxConditionNumber = 1000 in b2ContactSolver.
const BLOCK_MAX_CONDITION = 1000;

// Normal velocity of a contact point along n (in/s), positive = separating.
function normalVel(body, p, nx, ny) {
  const rx = p.r[0], ry = p.r[1];
  return nx * (body.vx - body.om * ry) + ny * (body.vy + body.om * rx);
}

// Exact 2-point normal LCP (Catto's block solver, Box2D v2.4 b2ContactSolver::SolveVelocityConstraints).
// Finds accumulated impulses x >= 0 with w = vn(x) + target >= 0 and x_i * w_i = 0 for BOTH points at once.
// Why: sequential Gauss-Seidel on two coupled points (they share the body's rotation) leaves an
// order-dependent residual: point 1 is solved, point 2's impulse then disturbs point 1 through om.
// For a flush face contact that residual shows up as spurious yaw (|om| ~1e-3 rad/s with mu = 0)
// and an over-pushed trailing corner that inflates the normal (and so friction) impulse.
// target[i] (in/s) is the bias added to vn (0 = rigid, sep/h = speculative, e*relVel = restitution).
// Returns false if the block is ill-conditioned, so the caller can fall back to sequential.
function blockSolveNormal(body, con, target) {
  const nx = con.normal[0], ny = con.normal[1];
  const p1 = con.points[0], p2 = con.points[1];
  const rn1 = cross2(p1.r[0], p1.r[1], nx, ny), rn2 = cross2(p2.r[0], p2.r[1], nx, ny);
  const k11 = body.invM + body.invI * rn1 * rn1;
  const k22 = body.invM + body.invI * rn2 * rn2;
  const k12 = body.invM + body.invI * rn1 * rn2;
  const det = k11 * k22 - k12 * k12;
  if (!(k11 > 0 && k22 > 0) || !(k11 * k11 < BLOCK_MAX_CONDITION * det)) return false;
  const a1 = p1.jn, a2 = p2.jn;
  // w = K x + q with q = vn - K a + target
  const q1 = normalVel(body, p1, nx, ny) - (k11 * a1 + k12 * a2) + target[0];
  const q2 = normalVel(body, p2, nx, ny) - (k12 * a1 + k22 * a2) + target[1];
  let x1, x2;
  // case 1: both active
  x1 = -(k22 * q1 - k12 * q2) / det;
  x2 = -(k11 * q2 - k12 * q1) / det;
  if (!(x1 >= 0 && x2 >= 0)) {
    // case 2: point 1 active, point 2 separating
    x1 = -q1 / k11; x2 = 0;
    if (!(x1 >= 0 && k12 * x1 + q2 >= 0)) {
      // case 3: point 2 active, point 1 separating
      x1 = 0; x2 = -q2 / k22;
      if (!(x2 >= 0 && k12 * x2 + q1 >= 0)) {
        // case 4: both separating
        x1 = 0; x2 = 0;
        if (!(q1 >= 0 && q2 >= 0)) return false; // cannot happen for SPD K; let the caller iterate
      }
    }
  }
  const d1 = x1 - a1, d2 = x2 - a2;
  p1.jn = x1; p2.jn = x2;
  applyImpulse(body, d1 * nx, d1 * ny, p1.r[0], p1.r[1]);
  applyImpulse(body, d2 * nx, d2 * ny, p2.r[0], p2.r[1]);
  p1.totalJn += d1; p2.totalJn += d2;
  if (x1 > p1.maxJn) p1.maxJn = x1;
  if (x2 > p2.maxJn) p2.maxJn = x2;
  return true;
}

// Current separation (in) of a prepared point, from the body's motion since prepareContacts.
function currentSep(body, con, p, c, s, c0, s0) {
  const rl = p.rLocal;
  const ax = (c * rl[0] - s * rl[1]) - (c0 * rl[0] - s0 * rl[1]);
  const ay = (s * rl[0] + c * rl[1]) - (s0 * rl[0] + c0 * rl[1]);
  return p.s0 + con.normal[0] * (body.x - con.x0 + ax) + con.normal[1] * (body.y - con.y0 + ay);
}

export function solveContacts(body, cons, hSub, useBias, P) {
  const maxPush = P.solver.maxPushSpeed;
  const c = Math.cos(body.h), s = Math.sin(body.h);
  for (const con of cons) {
    const nx = con.normal[0], ny = con.normal[1], tx = con.tangent[0], ty = con.tangent[1];
    const c0 = Math.cos(con.h0), s0 = Math.sin(con.h0);
    const dx = body.x - con.x0, dy = body.y - con.y0;
    const soft = con.soft;
    // Rigid pass (relax): exact 2-point block solve; target = speculative sep/h for open points, 0 otherwise.
    let blocked = false;
    if (!useBias && con.points.length === 2) {
      const tg = con.points.map((p) => { const sp = currentSep(body, con, p, c, s, c0, s0); return sp > 0 ? sp / hSub : 0; });
      blocked = blockSolveNormal(body, con, tg);
    }
    // normal first
    if (!blocked) for (const p of con.points) {
      const rl = p.rLocal;
      const ax = (c * rl[0] - s * rl[1]) - (c0 * rl[0] - s0 * rl[1]);
      const ay = (s * rl[0] + c * rl[1]) - (s0 * rl[0] + c0 * rl[1]);
      const sep = p.s0 + nx * (dx + ax) + ny * (dy + ay);
      let bias, mScale, iScale;
      if (sep > 0) { bias = sep / hSub; mScale = 1; iScale = 0; }
      else if (useBias) {
        bias = Math.max(soft.biasRate * sep, -maxPush);
        mScale = soft.massScale; iScale = soft.impulseScale;
      } else { bias = 0; mScale = 1; iScale = 0; }
      const rx = p.r[0], ry = p.r[1];
      const vn = nx * (body.vx - body.om * ry) + ny * (body.vy + body.om * rx);
      const dj = -p.normalMass * mScale * (vn + bias) - iScale * p.jn;
      const newJ = Math.max(p.jn + dj, 0);
      const d = newJ - p.jn;
      p.jn = newJ;
      applyImpulse(body, d * nx, d * ny, rx, ry);
      p.totalJn += d;
      if (p.jn > p.maxJn) p.maxJn = p.jn;
    }
    // friction
    for (const p of con.points) {
      const rx = p.r[0], ry = p.r[1];
      const vt = tx * (body.vx - body.om * ry) + ty * (body.vy + body.om * rx);
      const dj = -p.tangentMass * vt;
      const maxF = con.mu * p.jn;
      const newJ = Math.max(-maxF, Math.min(p.jt + dj, maxF));
      const d = newJ - p.jt;
      p.jt = newJ;
      applyImpulse(body, d * tx, d * ty, rx, ry);
      p.totalJt += d;
    }
  }
}

export function applyRestitution(body, cons, P) {
  const thr = P.restitution.threshold;
  const iters = P.solver.restitutionIterations;
  for (let it = 0; it < iters; it++) {
    for (const con of cons) {
      if (con.e === 0) continue;
      const nx = con.normal[0], ny = con.normal[1];
      // Both points eligible: exact block solve to w_i = vn_i + e*relVel_i >= 0 (see blockSolveNormal).
      if (con.points.length === 2 && con.points.every((p) => !(p.relVel > -thr || p.maxJn === 0))
        && blockSolveNormal(body, con, [con.e * con.points[0].relVel, con.e * con.points[1].relVel])) continue;
      for (const p of con.points) {
        if (p.relVel > -thr || p.maxJn === 0) continue;
        const rx = p.r[0], ry = p.r[1];
        const vn = nx * (body.vx - body.om * ry) + ny * (body.vy + body.om * rx);
        const dj = -p.normalMass * (vn + con.e * p.relVel);
        const newJ = Math.max(p.jn + dj, 0);
        const d = newJ - p.jn;
        p.jn = newJ;
        applyImpulse(body, d * nx, d * ny, rx, ry);
        p.totalJn += d;
      }
    }
  }
}

export function storeImpulses(cons, cache, hSub) {
  cache.clear();
  for (const con of cons) {
    for (const p of con.points) {
      cache.set(con.key + '#' + p.id, { jn: p.jn, jt: p.jt, hSub });
    }
  }
}

export function integratePositions(body, hSub) {
  body.x += body.vx * hSub;
  body.y += body.vy * hSub;
  body.h = wrapAngle(body.h + body.om * hSub);
}
