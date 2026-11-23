// Per-frame robot-vs-field contact step (WALL-COLLISIONS SPEC v1, section 5). Pure ES module (browser + node).
// Units: in, s, rad, lb. Impulse lb*in/s, angular impulse lb*in^2/s. Frame: Pedro (x right, y far, z up).
import { transformPolygon } from './geometry.js';
import { collideRobot, maxPenetration } from './manifold.js';
import {
  prepareContacts, warmStart, solveContacts, applyRestitution, storeImpulses,
  integratePositions,
} from './solver.js';
import { UNITS } from './params.js';
import { resolveRobot, slideVelocity } from './legacy.js';

export function createContactCache() { return new Map(); }

export function robotBodyFromState(S, mass) {
  return { x: S.x, y: S.y, h: S.h, vx: S.vx || 0, vy: S.vy || 0, om: S.om || 0, invM: mass.invM, invI: mass.invI };
}

export function writeBodyToState(body, S) {
  S.x = body.x; S.y = body.y; S.h = body.h; S.vx = body.vx; S.vy = body.vy; S.om = body.om;
  const c = Math.cos(body.h), s = Math.sin(body.h);
  S.bodyV = [c * body.vx + s * body.vy, -s * body.vx + c * body.vy, body.om];
}

const finite3 = (a, b, c) => Number.isFinite(a) && Number.isFinite(b) && Number.isFinite(c);

function emptyReport(fallback, pen) {
  return {
    hit: fallback, chunks: 0, fallback, maxPenetration: pen,
    impulse: { x: 0, y: 0, ang: 0 }, forceN: { x: 0, y: 0 }, torqueNm: 0, contacts: [], normals: [],
  };
}

export function contactStep(body, ctx, dt) {
  const { P, poly, prepared, cache, obstacles } = ctx;
  const sv = P.solver;
  // NaN guards: a non-finite velocity is treated as at rest; a non-finite pose or dt cannot be stepped,
  // so the body is left untouched and a finite empty report is returned (this function never creates NaN).
  if (!finite3(body.vx, body.vy, body.om)) { body.vx = 0; body.vy = 0; body.om = 0; }
  if (!finite3(body.x, body.y, body.h) || !(Number.isFinite(dt) && dt > 0)) return emptyReport(false, 0);
  const x0f = body.x, y0f = body.y, h0f = body.h;
  const reach = poly.reach;
  const travel = Math.hypot(body.vx, body.vy) * dt + Math.abs(body.om) * reach * dt;
  const n = Math.min(sv.maxChunks, Math.max(1, Math.ceil(travel / sv.maxChunkTravel)));
  const hc = dt / n, hSub = hc / sv.substeps;

  const imp = { x: 0, y: 0, ang: 0 };
  const contactMap = new Map();
  const normals = [];
  let hit = false;

  for (let k = 0; k < n; k++) {
    const A = transformPolygon(poly, body.x, body.y, body.h);
    const margin = sv.speculativeDistance + Math.hypot(body.vx, body.vy) * hc + Math.abs(body.om) * reach * hc;
    const ms = collideRobot(A, prepared, margin, P);
    if (!ms || ms.length === 0) { integratePositions(body, hc); continue; }
    hit = true;
    const cons = prepareContacts(body, ms, P, hSub, cache);
    for (let i = 0; i < sv.substeps; i++) {
      if (sv.warmStart) warmStart(body, cons);
      solveContacts(body, cons, hSub, true, P);
      integratePositions(body, hSub);
      for (let j = 0; j < sv.relaxIterations; j++) solveContacts(body, cons, hSub, false, P);
    }
    applyRestitution(body, cons, P);
    storeImpulses(cons, cache, hSub);
    for (const c of cons) {
      const nx = c.normal[0], ny = c.normal[1], tx = c.tangent[0], ty = c.tangent[1];
      let dup = false;
      for (const q of normals) if (q[0] * nx + q[1] * ny > 0.9999) { dup = true; break; }
      if (!dup) normals.push([nx, ny]);
      for (const pt of c.points) {
        const px = pt.totalJn * nx + pt.totalJt * tx, py = pt.totalJn * ny + pt.totalJt * ty;
        imp.x += px; imp.y += py; imp.ang += pt.r[0] * py - pt.r[1] * px;
        const ck = c.key + '#' + pt.id;
        const cur = { p: [c.x0 + pt.r[0], c.y0 + pt.r[1]], n: [nx, ny], jn: pt.totalJn, jt: pt.totalJt, s: pt.s0, kind: c.kind, key: c.key };
        const old = contactMap.get(ck);
        if (old) { cur.jn += old.jn; cur.jt += old.jt; }
        contactMap.set(ck, cur);
      }
    }
  }

  const Af = transformPolygon(poly, body.x, body.y, body.h);
  const pen = maxPenetration(collideRobot(Af, prepared, 0, P));
  let fallback = false, finalPen = pen;
  if (pen > sv.hardPenetration) {
    const r = resolveRobot({ x: body.x, y: body.y, h: body.h }, { x: x0f, y: y0f, h: h0f }, obstacles, { halfX: poly.hx, halfY: poly.hy });
    body.x = r.x; body.y = r.y; body.h = r.h;
    if (r.x === x0f && r.y === y0f && r.h === h0f) body.om = 0;
    const sl = slideVelocity(body.vx, body.vy, r.normals);
    body.vx = sl.vx; body.vy = sl.vy;
    fallback = true; hit = true;
    for (const nn of r.normals) {
      let dup = false;
      for (const q of normals) if (q[0] * nn[0] + q[1] * nn[1] > 0.9999) { dup = true; break; }
      if (!dup) normals.push(nn);
    }
    finalPen = maxPenetration(collideRobot(transformPolygon(poly, body.x, body.y, body.h), prepared, 0, P));
  }
  // Last line of defence: anything non-finite (degenerate geometry, overflow) reverts to the frame-start
  // pose at rest instead of propagating NaN into the app state.
  if (!finite3(body.x, body.y, body.h) || !finite3(body.vx, body.vy, body.om)) {
    body.x = x0f; body.y = y0f; body.h = h0f; body.vx = 0; body.vy = 0; body.om = 0;
    cache.clear();
    const p0 = maxPenetration(collideRobot(transformPolygon(poly, x0f, y0f, h0f), prepared, 0, P));
    return emptyReport(true, Number.isFinite(p0) ? p0 : 0);
  }
  if (!finite3(imp.x, imp.y, imp.ang)) { imp.x = 0; imp.y = 0; imp.ang = 0; }
  if (!Number.isFinite(finalPen)) finalPen = 0;

  const fk = UNITS.N_PER_LB_IN_S2, tk = UNITS.NM_PER_LB_IN2_S2;
  return {
    hit, chunks: n, fallback, maxPenetration: finalPen,
    impulse: imp,
    forceN: { x: imp.x / dt * fk, y: imp.y / dt * fk },
    torqueNm: imp.ang / dt * tk,
    contacts: [...contactMap.values()],
    normals,
  };
}
