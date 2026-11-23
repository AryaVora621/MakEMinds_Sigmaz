// manifold.js - narrowphase for robot (oriented polygon) vs fixed field geometry. Pure functions, no DOM.
// Units: inches. Frame: Pedro (x right, y far, z up). Algorithm after Box2D v3 b2CollidePolygons / b2ClipPolygons
// (box2d.org, manifold.c; recollection, not re-verified line by line).
// Manifold = { normal:[nx,ny], points:[{ p:[x,y], s, id, rLocal:[x,y] }], key, kind }
//   normal: unit, from OBSTACLE INTO ROBOT (direction the robot is pushed). s: signed gap along normal (<0 penetrating).
import { dot2, boxPolygon, polygonAabb } from './geometry.js';

const EPS = 1e-9;          // degenerate-length / parallel guard (dimensionless / in); ASSUMPTION (FLT_EPSILON-style)

// ---------- prepareObstacles (ghost-collision filter) ----------
function insideExpanded(x, y, b, slop) {
  return x > b.minX - slop && x < b.maxX + slop && y > b.minY - slop && y < b.maxY + slop;
}
function sameBox(a, b) {
  return a.minX === b.minX && a.minY === b.minY && a.maxX === b.maxX && a.maxY === b.maxY;
}

export function prepareObstacles(obstacles, P) {
  const slop = P.solver.linearSlop;
  const bd = obstacles.bounds;
  const src = obstacles.boxes || [];
  const boxes = src.map((box, k) => {
    const poly = boxPolygon(box);
    const internalCorner = [false, false, false, false];
    const internalFace = [false, false, false, false];
    // Other boxes; an exact duplicate only counts if it comes earlier, so two copies do not hide each other.
    const others = [];
    for (let j = 0; j < src.length; j++) {
      if (j === k) continue;
      if (sameBox(src[j], box) && j > k) continue;
      others.push(src[j]);
    }
    for (let i = 0; i < 4; i++) {
      const v = poly.verts[i], w = poly.verts[(i + 1) % 4];
      const mx = 0.5 * (v[0] + w[0]), my = 0.5 * (v[1] + w[1]);
      let ic = false, iff = false;
      for (const o of others) {
        if (!ic && insideExpanded(v[0], v[1], o, slop)) ic = true;
        if (!iff && insideExpanded(mx, my, o, slop)) iff = true;
      }
      // corner on/outside the play-area edge (within slop) belongs to the bounds walls
      if (bd && (v[0] <= bd.minX + slop || v[0] >= bd.maxX - slop || v[1] <= bd.minY + slop || v[1] >= bd.maxY - slop)) ic = true;
      internalCorner[i] = ic;
      internalFace[i] = iff;
    }
    return { ...box, poly, internalCorner, internalFace };
  });
  return {
    bounds: bd ? { minX: bd.minX, minY: bd.minY, maxX: bd.maxX, maxY: bd.maxY } : bd,
    boxes,
    circles: (obstacles.circles || []).map((c) => ({ ...c }))
  };
}

// ---------- helpers ----------
// Max over faces of A (not skipped) of min over B verts of nA.(vB - vA). Returns { sep, edge }.
function findMaxSeparation(A, B, skip) {
  let best = -Infinity, edge = -1;
  const n = A.verts.length;
  for (let i = 0; i < n; i++) {
    if (skip && skip[i]) continue;
    const nm = A.normals[i];
    if (!(Math.abs(nm[0]) + Math.abs(nm[1]) > 0.5)) continue; // zero-length edge
    const v = A.verts[i];
    let min = Infinity;
    for (let j = 0; j < B.verts.length; j++) {
      const w = B.verts[j];
      const d = nm[0] * (w[0] - v[0]) + nm[1] * (w[1] - v[1]);
      if (d < min) min = d;
    }
    if (min > best) { best = min; edge = i; }
  }
  return { sep: best, edge };
}

// Closest points between segments p1-q1 and p2-q2 (Ericson). Returns { u, w, c1:[x,y], c2:[x,y] }.
function segmentClosest(p1, q1, p2, q2) {
  const d1x = q1[0] - p1[0], d1y = q1[1] - p1[1];
  const d2x = q2[0] - p2[0], d2y = q2[1] - p2[1];
  const rx = p1[0] - p2[0], ry = p1[1] - p2[1];
  const a = d1x * d1x + d1y * d1y, e = d2x * d2x + d2y * d2y;
  const f = d2x * rx + d2y * ry;
  let s, t;
  if (a <= EPS && e <= EPS) { s = 0; t = 0; }
  else if (a <= EPS) { s = 0; t = Math.min(1, Math.max(0, f / e)); }
  else {
    const c = d1x * rx + d1y * ry;
    if (e <= EPS) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
    else {
      const b = d1x * d2x + d1y * d2y;
      const denom = a * e - b * b;
      s = denom > EPS * a * e ? Math.min(1, Math.max(0, (b * f - c * e) / denom)) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
      else if (t > 1) { t = 1; s = Math.min(1, Math.max(0, (b - c) / a)); }
    }
  }
  return {
    u: s, w: t,
    c1: [p1[0] + d1x * s, p1[1] + d1y * s],
    c2: [p2[0] + d2x * t, p2[1] + d2y * t]
  };
}

// ---------- polygon (robot) vs box ----------
export function collidePolygonBox(wpoly, pbox, margin, P) {
  const slop = P.solver.linearSlop, refTol = P.solver.refFaceTol;
  const A = wpoly, B = pbox.poly;
  const rA = findMaxSeparation(A, B, null);
  const rB = findMaxSeparation(B, A, pbox.internalFace);
  if (rA.edge < 0) return null;
  const sepMax = Math.max(rA.sep, rB.sep);
  if (!(sepMax <= margin)) return null; // also rejects NaN

  // incident edge: normal most anti-parallel to the reference normal
  const incidentOf = (inc, nr) => {
    let e = 0, minDot = Infinity;
    for (let j = 0; j < inc.verts.length; j++) {
      const nj = inc.normals[j];
      const d = nj[0] * nr[0] + nj[1] * nr[1];
      if (d < minDot) { minDot = d; e = j; }
    }
    return e;
  };
  let flip = rB.edge >= 0 && rB.sep > rA.sep + refTol;
  let incEdge = incidentOf(flip ? A : B, (flip ? B : A).normals[flip ? rB.edge : rA.edge]);
  // Ghost-normal filter for a ROBOT reference face. The internal-face skip above only removes the box's
  // own internal faces as reference candidates; a robot face can still win SAT against a box whose
  // matching (incident) face is internal, e.g. sliding up the x = 60 face of two stacked boxes, the robot's
  // +y face crosses the seam y = 70 and its separation against box B (~ -0.005 in) beats the true x-face
  // penetration (~ -0.01 in), giving normal (0,-1) at the seam. A box face that is shared with a neighbour
  // can never be touched from outside, so the robot face is not a valid reference there: use the box's
  // best non-internal face instead (its sep is within margin, as sepMax >= rB.sep), else no contact.
  if (!flip && pbox.internalFace[incEdge]) {
    if (!(rB.edge >= 0 && rB.sep <= margin)) return null;
    flip = true;
    incEdge = incidentOf(A, B.normals[rB.edge]);
  }
  const ref = flip ? B : A, inc = flip ? A : B;
  const refEdge = flip ? rB.edge : rA.edge;
  const nr = ref.normals[refEdge];
  const nRef = ref.verts.length, nInc = inc.verts.length;
  const v1 = ref.verts[refEdge], v2 = ref.verts[(refEdge + 1) % nRef];
  const w1 = inc.verts[incEdge], w2 = inc.verts[(incEdge + 1) % nInc];
  const baseId = ((flip ? 1 : 0) << 16) | (refEdge << 8) | (incEdge << 1);
  const sign = flip ? 1 : -1; // output normal (into robot) = sign * nr
  const normalOut = [sign * nr[0], sign * nr[1]];
  const boxCornerInternal = (vi) => pbox.internalCorner[vi];
  // incident / reference vertex index -> box vertex index (box is inc when !flip, ref when flip)
  const boxIncVert = (k) => (incEdge + k) % 4;
  const boxRefVert = (k) => (refEdge + k) % 4;

  // Vertex-vertex branch (Box2D: separated by more than 0.1*slop and the nearest features are two endpoints).
  if (sepMax > 0.1 * slop) {
    const cp = segmentClosest(v1, v2, w1, w2);
    const atEnd = (f) => f <= EPS || f >= 1 - EPS;
    if (atEnd(cp.u) && atEnd(cp.w)) {
      const dx = cp.c2[0] - cp.c1[0], dy = cp.c2[1] - cp.c1[1];
      const dist = Math.hypot(dx, dy);
      if (dist > EPS) {
        if (dist > margin) return null;
        const bi = flip ? boxRefVert(cp.u >= 0.5 ? 1 : 0) : boxIncVert(cp.w >= 0.5 ? 1 : 0);
        if (boxCornerInternal(bi)) return null; // ghost corner; its owner box generates the contact
        const nx = flip ? dx / dist : -dx / dist, ny = flip ? dy / dist : -dy / dist;
        return {
          normal: [nx, ny],
          points: [{ p: [0.5 * (cp.c1[0] + cp.c2[0]), 0.5 * (cp.c1[1] + cp.c2[1])], s: dist, id: baseId | 0, rLocal: [0, 0] }],
          key: '', kind: ''
        };
      }
    }
  }

  // Edge clip (b2ClipPolygons): clip the incident segment to the reference edge slab.
  const ex = v2[0] - v1[0], ey = v2[1] - v1[1];
  const len = Math.hypot(ex, ey);
  if (len < EPS) return null;
  const tx = ex / len, ty = ey / len;
  const t0 = tx * (w1[0] - v1[0]) + ty * (w1[1] - v1[1]);
  const t1 = tx * (w2[0] - v1[0]) + ty * (w2[1] - v1[1]);
  let ulo, uhi;
  const dt = t1 - t0;
  if (Math.abs(dt) < EPS) {
    if (t0 < 0 || t0 > len) return null;
    ulo = 0; uhi = 1;
  } else {
    const ua = (0 - t0) / dt, ub = (len - t0) / dt;
    ulo = Math.max(0, Math.min(ua, ub));
    uhi = Math.min(1, Math.max(ua, ub));
    if (ulo > uhi) return null;
  }
  const cand = [ulo, uhi];
  const points = [];
  for (let k = 0; k < 2; k++) {
    const u = cand[k];
    if (k === 1 && Math.abs(uhi - ulo) * Math.hypot(w2[0] - w1[0], w2[1] - w1[1]) < EPS) break; // coincident
    const wx = w1[0] + (w2[0] - w1[0]) * u, wy = w1[1] + (w2[1] - w1[1]) * u;
    const s = nr[0] * (wx - v1[0]) + nr[1] * (wy - v1[1]);
    if (s > margin) continue;
    // incident vertex still unclipped? then it may be a box corner
    if (!flip) {
      if (k === 0 && ulo === 0 && boxCornerInternal(boxIncVert(0))) continue;
      if (k === 1 && uhi === 1 && boxCornerInternal(boxIncVert(1))) continue;
      if (k === 0 && ulo === 1 && boxCornerInternal(boxIncVert(1))) continue;
    }
    points.push({
      p: [wx - 0.5 * s * nr[0], wy - 0.5 * s * nr[1]],
      s,
      id: baseId | k,
      rLocal: [0, 0]
    });
  }
  if (points.length === 0) return null;
  return { normal: normalOut, points, key: '', kind: '' };
}

// ---------- polygon (robot) vs circle ----------
export function collidePolygonCircle(wpoly, circle, margin) {
  const { x: cx, y: cy, r } = circle;
  const n = wpoly.verts.length;
  let best = -Infinity, fi = -1;
  for (let i = 0; i < n; i++) {
    const nm = wpoly.normals[i];
    if (!(Math.abs(nm[0]) + Math.abs(nm[1]) > 0.5)) continue;
    const v = wpoly.verts[i];
    const d = nm[0] * (cx - v[0]) + nm[1] * (cy - v[1]); // centre-to-face-plane distance
    if (d > best) { best = d; fi = i; }
  }
  if (fi < 0) return null;
  const sep = best - r;
  if (!(sep <= margin)) return null;
  const v1 = wpoly.verts[fi], v2 = wpoly.verts[(fi + 1) % n];
  const nf = wpoly.normals[fi];
  const u1 = (cx - v1[0]) * (v2[0] - v1[0]) + (cy - v1[1]) * (v2[1] - v1[1]);
  const u2 = (cx - v2[0]) * (v1[0] - v2[0]) + (cy - v2[1]) * (v1[1] - v2[1]);
  let vi = -1;
  if (best > EPS) { if (u1 < 0) vi = fi; else if (u2 < 0) vi = (fi + 1) % n; }
  if (vi >= 0) {
    const v = wpoly.verts[vi];
    const dx = cx - v[0], dy = cy - v[1];
    const dist = Math.hypot(dx, dy);
    if (dist > EPS) {
      const s = dist - r;
      if (s > margin) return null;
      const k = 0.5 * s / dist;
      return {
        normal: [-dx / dist, -dy / dist],
        points: [{ p: [v[0] + k * dx, v[1] + k * dy], s, id: 100 + vi, rLocal: [0, 0] }],
        key: '', kind: ''
      };
    }
    // centre exactly on a vertex: fall through to face normal
  }
  const k = r + 0.5 * sep;
  return {
    normal: [-nf[0], -nf[1]],
    points: [{ p: [cx - nf[0] * k, cy - nf[1] * k], s: sep, id: fi, rLocal: [0, 0] }],
    key: '', kind: ''
  };
}

// ---------- polygon (robot) vs play-area bounds ----------
const BOUND_NORMALS = [[1, 0], [-1, 0], [0, 1], [0, -1]]; // minX, maxX, minY, maxY (inward)
export function collidePolygonBounds(wpoly, bounds, margin) {
  const out = [];
  const gaps = [
    (v) => v[0] - bounds.minX,
    (v) => bounds.maxX - v[0],
    (v) => v[1] - bounds.minY,
    (v) => bounds.maxY - v[1]
  ];
  for (let side = 0; side < 4; side++) {
    const nrm = BOUND_NORMALS[side];
    const cands = [];
    for (let i = 0; i < wpoly.verts.length; i++) {
      const s = gaps[side](wpoly.verts[i]);
      if (s <= margin) cands.push({ i, s });
    }
    if (cands.length === 0) continue;
    cands.sort((a, b) => a.s - b.s || a.i - b.i);
    const keep = cands.slice(0, 2).sort((a, b) => a.i - b.i);
    out.push({
      normal: [nrm[0], nrm[1]],
      points: keep.map(({ i, s }) => {
        const v = wpoly.verts[i];
        return { p: [v[0] - 0.5 * s * nrm[0], v[1] - 0.5 * s * nrm[1]], s, id: i, rLocal: [0, 0] };
      }),
      key: 'bounds:' + side,
      kind: 'bounds'
    });
  }
  return out;
}

// ---------- all obstacles ----------
function fillLocal(m, wpoly) {
  const c = Math.cos(wpoly.h), s = Math.sin(wpoly.h);
  for (const pt of m.points) {
    const dx = pt.p[0] - wpoly.cx, dy = pt.p[1] - wpoly.cy;
    pt.rLocal = [c * dx + s * dy, -s * dx + c * dy];
  }
}

export function collideRobot(wpoly, prepared, margin, P) {
  const out = [];
  const bb = polygonAabb(wpoly);
  const b = prepared.bounds;
  if (b) {
    for (const m of collidePolygonBounds(wpoly, b, margin)) { fillLocal(m, wpoly); out.push(m); }
  }
  const boxes = prepared.boxes;
  for (let i = 0; i < boxes.length; i++) {
    const bx = boxes[i];
    if (bb.maxX + margin < bx.minX || bb.minX - margin > bx.maxX || bb.maxY + margin < bx.minY || bb.minY - margin > bx.maxY) continue;
    const m = collidePolygonBox(wpoly, bx, margin, P);
    if (!m) continue;
    m.key = 'box:' + i;
    m.kind = bx.kind || 'bounds';
    fillLocal(m, wpoly);
    out.push(m);
  }
  const circles = prepared.circles;
  for (let i = 0; i < circles.length; i++) {
    const c = circles[i];
    const rr = c.r + margin;
    if (bb.maxX < c.x - rr || bb.minX > c.x + rr || bb.maxY < c.y - rr || bb.minY > c.y + rr) continue;
    const m = collidePolygonCircle(wpoly, c, margin);
    if (!m) continue;
    m.key = 'circle:' + i;
    m.kind = c.kind || 'bounds';
    fillLocal(m, wpoly);
    out.push(m);
  }
  return out;
}

export function maxPenetration(manifolds) {
  let pen = 0;
  for (const m of manifolds) for (const pt of m.points) if (-pt.s > pen) pen = -pt.s;
  return pen;
}
