// collide.js - narrowphase for the rigid-body solver (world-space manifolds, normal from A to B).
// Sources:
//   box2d-lite Collide.cpp (Erin Catto) - SAT face search, incident edge, ClipSegmentToLine, relativeTol/absoluteTol.
//   https://github.com/erincatto/box2d-lite/blob/master/src/Collide.cpp
//   Box2D v2.4 b2CollidePolygons / b2CollidePolygonAndCircle (planck.js port) - reference/incident structure,
//   Voronoi regions, b2ContactFeature id layout.
// Tunables arrive via opts {speculativeDistance (m), faceRelTol, faceAbsTolFrac}; this file does not import params.
import { sub, dot, normalize, rotate, transformPoint, invTransformPoint } from './vec2.js';

export const FEATURE_VERTEX = 0;
export const FEATURE_FACE = 1;

export function makeFeatureId(indexA, typeA, indexB, typeB) {
  return ((indexA & 0xFF) | ((typeA & 0xFF) << 8) | ((indexB & 0xFF) << 16) | ((typeB & 0xFF) << 24)) >>> 0;
}

export function swapFeatureId(id) {
  const iA = id & 0xFF, tA = (id >>> 8) & 0xFF, iB = (id >>> 16) & 0xFF, tB = (id >>> 24) & 0xFF;
  return makeFeatureId(iB, tB, iA, tA);
}

const empty = () => ({ normal: { x: 1, y: 0 }, pointCount: 0, points: [], flipped: false });

// Max over edges of poly1 of the min signed distance of poly2's vertices to that edge's plane.
export function findMaxSeparation(poly1, xf1, poly2, xf2) {
  let best = -Infinity, bestEdge = 0;
  const w2 = poly2.vertices.map((v) => transformPoint(xf2, v));
  for (let i = 0; i < poly1.count; i++) {
    const n = rotate(xf1.q, poly1.normals[i]);
    const v1 = transformPoint(xf1, poly1.vertices[i]);
    let si = Infinity;
    for (let j = 0; j < w2.length; j++) {
      const d = n.x * (w2[j].x - v1.x) + n.y * (w2[j].y - v1.y);
      if (d < si) si = d;
    }
    if (si > best) { best = si; bestEdge = i; }
  }
  return { separation: best, edge: bestEdge };
}

// box2d-lite ClipSegmentToLine. pts = [{p,id},{p,id}]; a point created by clipping gets clipId.
function clipSegment(pts, nrm, offset, clipId) {
  const out = [];
  const d0 = dot(nrm, pts[0].p) - offset;
  const d1 = dot(nrm, pts[1].p) - offset;
  if (d0 <= 0) out.push(pts[0]);
  if (d1 <= 0) out.push(pts[1]);
  if (d0 * d1 < 0) {
    const t = d0 / (d0 - d1);
    out.push({
      p: { x: pts[0].p.x + t * (pts[1].p.x - pts[0].p.x), y: pts[0].p.y + t * (pts[1].p.y - pts[0].p.y) },
      id: clipId,
    });
  }
  return out;
}

export function collidePolygons(polyA, xfA, polyB, xfB, opts) {
  const spec = opts.speculativeDistance;
  const sA = findMaxSeparation(polyA, xfA, polyB, xfB);
  if (sA.separation > spec) return empty();
  const sB = findMaxSeparation(polyB, xfB, polyA, xfA);
  if (sB.separation > spec) return empty();

  const flip = sB.separation > sA.separation + (1 - opts.faceRelTol) * Math.abs(sA.separation)
    + opts.faceAbsTolFrac * polyA.boundRadius;
  const ref = flip ? polyB : polyA, inc = flip ? polyA : polyB;
  const xfR = flip ? xfB : xfA, xfI = flip ? xfA : xfB;
  const refEdge = flip ? sB.edge : sA.edge;

  const nRef = rotate(xfR.q, ref.normals[refEdge]);
  const v1 = transformPoint(xfR, ref.vertices[refEdge]);
  const v2 = transformPoint(xfR, ref.vertices[(refEdge + 1) % ref.count]);

  // incident edge: most anti-parallel to nRef
  let i1 = 0, minDot = Infinity;
  for (let i = 0; i < inc.count; i++) {
    const d = dot(nRef, rotate(xfI.q, inc.normals[i]));
    if (d < minDot) { minDot = d; i1 = i; }
  }
  const i2 = (i1 + 1) % inc.count;
  let pts = [
    { p: transformPoint(xfI, inc.vertices[i1]), id: makeFeatureId(refEdge, FEATURE_FACE, i1, FEATURE_VERTEX) },
    { p: transformPoint(xfI, inc.vertices[i2]), id: makeFeatureId(refEdge, FEATURE_FACE, i2, FEATURE_VERTEX) },
  ];

  const tan = normalize(sub(v2, v1));
  pts = clipSegment(pts, { x: -tan.x, y: -tan.y }, -dot(tan, v1), makeFeatureId(refEdge, FEATURE_FACE, i1, FEATURE_FACE));
  if (pts.length < 2) return empty();
  pts = clipSegment(pts, tan, dot(tan, v2), makeFeatureId(refEdge | 0x80, FEATURE_FACE, i1, FEATURE_FACE));
  if (pts.length < 2) return empty();

  const points = [];
  for (const c of pts) {
    const sep = dot(nRef, sub(c.p, v1));
    if (sep > spec) continue;
    points.push({
      point: { x: c.p.x - 0.5 * sep * nRef.x, y: c.p.y - 0.5 * sep * nRef.y },
      separation: sep,
      id: flip ? swapFeatureId(c.id) : c.id,
    });
  }
  return {
    normal: flip ? { x: -nRef.x, y: -nRef.y } : nRef,
    pointCount: points.length,
    points,
    flipped: flip,
  };
}

export function collidePolygonCircle(polyA, xfA, circleB, xfB, opts) {
  const spec = opts.speculativeDistance;
  const r = circleB.radius;
  const cW = transformPoint(xfB, circleB.center);
  const c = invTransformPoint(xfA, cW);

  let s = -Infinity, edge = 0;
  for (let i = 0; i < polyA.count; i++) {
    const d = dot(polyA.normals[i], sub(c, polyA.vertices[i]));
    if (d > s) { s = d; edge = i; }
  }
  if (s > r + spec) return empty();

  const count = polyA.count;
  const v1 = polyA.vertices[edge], v2 = polyA.vertices[(edge + 1) % count];
  let nLocal, sep, id;
  if (s < 1e-12) {
    nLocal = polyA.normals[edge]; sep = s - r;
    id = makeFeatureId(edge, FEATURE_FACE, 0, FEATURE_VERTEX);
  } else {
    const u1 = dot(sub(c, v1), sub(v2, v1));
    const u2 = dot(sub(c, v2), sub(v1, v2));
    if (u1 <= 0 || u2 <= 0) {
      const vtx = u1 <= 0 ? v1 : v2;
      const d = sub(c, vtx), len = Math.hypot(d.x, d.y);
      if (len - r > spec) return empty();
      nLocal = len > 1e-12 ? { x: d.x / len, y: d.y / len } : polyA.normals[edge];
      sep = len - r;
      id = makeFeatureId(u1 <= 0 ? edge : (edge + 1) % count, FEATURE_VERTEX, 0, FEATURE_VERTEX);
    } else {
      nLocal = polyA.normals[edge]; sep = s - r;
      id = makeFeatureId(edge, FEATURE_FACE, 0, FEATURE_VERTEX);
    }
  }
  const normal = rotate(xfA.q, nLocal);
  const k = r + 0.5 * sep;
  return {
    normal,
    pointCount: 1,
    points: [{ point: { x: cW.x - normal.x * k, y: cW.y - normal.y * k }, separation: sep, id }],
    flipped: false,
  };
}

export function collideCircles(circleA, xfA, circleB, xfB, opts) {
  const cA = transformPoint(xfA, circleA.center), cB = transformPoint(xfB, circleB.center);
  const dx = cB.x - cA.x, dy = cB.y - cA.y;
  const dist = Math.hypot(dx, dy);
  const sep = dist - circleA.radius - circleB.radius;
  if (sep > opts.speculativeDistance) return empty();
  const normal = dist < 1e-12 ? { x: 1, y: 0 } : { x: dx / dist, y: dy / dist };
  const k = circleA.radius + 0.5 * sep;
  return {
    normal,
    pointCount: 1,
    points: [{ point: { x: cA.x + normal.x * k, y: cA.y + normal.y * k }, separation: sep, id: 0 }],
    flipped: false,
  };
}

export function collide(shapeA, xfA, shapeB, xfB, opts) {
  const pa = shapeA.type === 'polygon', pb = shapeB.type === 'polygon';
  if (pa && pb) return collidePolygons(shapeA, xfA, shapeB, xfB, opts);
  if (pa) return collidePolygonCircle(shapeA, xfA, shapeB, xfB, opts);
  if (pb) {
    const m = collidePolygonCircle(shapeB, xfB, shapeA, xfA, opts);
    return {
      normal: { x: -m.normal.x, y: -m.normal.y },
      pointCount: m.pointCount,
      points: m.points.map((p) => ({ point: p.point, separation: p.separation, id: swapFeatureId(p.id) })),
      flipped: true,
    };
  }
  return collideCircles(shapeA, xfA, shapeB, xfB, opts);
}
