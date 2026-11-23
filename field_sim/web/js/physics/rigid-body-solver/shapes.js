// shapes.js - convex polygon / circle shapes, mass data and AABBs for the rigid-body solver.
// SI units (metres, kg). Pure ES module, no DOM. Polygon vertices are CCW; normals[i] is the outward
// unit normal of edge vertices[i] -> vertices[(i+1)%count].
// Sources: Box2D b2PolygonShape::ComputeMass (triangle-fan inertia), box2d-lite Body (box inertia (w^2+h^2)/12).
// Convex hull: Andrew monotone chain (public-domain algorithm). Only constant: DUP_EPS.

const DUP_EPS = 1e-9; // m; near-duplicate point distance (spec section 2, assumed)

function toPt(p) {
  return Array.isArray(p) ? { x: p[0], y: p[1] } : { x: p.x, y: p.y };
}

function polygonAreaCentroid(vs) {
  const o = vs[0];
  let area = 0, cx = 0, cy = 0;
  for (let i = 1; i < vs.length - 1; i++) {
    const e1x = vs[i].x - o.x, e1y = vs[i].y - o.y;
    const e2x = vs[i + 1].x - o.x, e2y = vs[i + 1].y - o.y;
    const a = 0.5 * (e1x * e2y - e1y * e2x);
    area += a;
    cx += a * (e1x + e2x) / 3;
    cy += a * (e1y + e2y) / 3;
  }
  if (!(area > 0)) return { area, centroid: { x: o.x, y: o.y } };
  return { area, centroid: { x: o.x + cx / area, y: o.y + cy / area } };
}

function buildPolygon(hull) {
  const count = hull.length;
  const vertices = hull.map((p) => ({ x: p.x, y: p.y }));
  const normals = [];
  for (let i = 0; i < count; i++) {
    const a = vertices[i], b = vertices[(i + 1) % count];
    const ex = b.x - a.x, ey = b.y - a.y;
    const len = Math.hypot(ex, ey) || 1;
    normals.push({ x: ey / len, y: -ex / len });
  }
  const { area, centroid } = polygonAreaCentroid(vertices);
  if (!(area > 0)) throw new Error('degenerate polygon');
  let boundRadius = 0;
  for (const v of vertices) boundRadius = Math.max(boundRadius, Math.hypot(v.x - centroid.x, v.y - centroid.y));
  return { type: 'polygon', count, vertices, normals, centroid, radius: 0, boundRadius };
}

export function makePolygon(points) {
  const pts = [];
  for (const raw of points) {
    const p = toPt(raw);
    if (!pts.some((q) => Math.hypot(q.x - p.x, q.y - p.y) < DUP_EPS)) pts.push(p);
  }
  if (pts.length < 3) throw new Error('polygon needs at least 3 distinct points');
  pts.sort((a, b) => (a.x - b.x) || (a.y - b.y));
  const cr = (o, a, b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const lower = [];
  for (const p of pts) {
    while (lower.length >= 2 && cr(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop();
    lower.push(p);
  }
  const upper = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && cr(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop();
    upper.push(p);
  }
  lower.pop(); upper.pop();
  const hull = lower.concat(upper); // CCW
  if (hull.length < 3) throw new Error('polygon points are collinear');
  return buildPolygon(hull);
}

export function makeBox(hx, hy, center = { x: 0, y: 0 }, angle = 0) {
  const c = Math.cos(angle), s = Math.sin(angle);
  const corners = [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]]; // CCW
  const hull = corners.map(([x, y]) => ({ x: center.x + c * x - s * y, y: center.y + s * x + c * y }));
  return buildPolygon(hull);
}

export function makeCircle(radius, center = { x: 0, y: 0 }) {
  return { type: 'circle', radius, center: { x: center.x, y: center.y }, boundRadius: radius };
}

// Inertia per unit mass about the shape centroid (uniform density).
export function shapeUnitInertia(shape) {
  if (shape.type === 'circle') return 0.5 * shape.radius * shape.radius;
  const vs = shape.vertices, o = vs[0];
  let area = 0, I0 = 0; // I0: inertia about o per unit density
  for (let i = 1; i < vs.length - 1; i++) {
    const e1x = vs[i].x - o.x, e1y = vs[i].y - o.y;
    const e2x = vs[i + 1].x - o.x, e2y = vs[i + 1].y - o.y;
    const D = e1x * e2y - e1y * e2x;
    area += 0.5 * D;
    const intx2 = e1x * e1x + e2x * e1x + e2x * e2x;
    const inty2 = e1y * e1y + e2y * e1y + e2y * e2y;
    I0 += (0.25 / 3) * D * (intx2 + inty2);
  }
  const c = shape.centroid;
  const d2 = (c.x - o.x) * (c.x - o.x) + (c.y - o.y) * (c.y - o.y);
  return (I0 - area * d2) / area;
}

export function computeMassData(shape, density) {
  if (shape.type === 'circle') {
    const mass = density * Math.PI * shape.radius * shape.radius;
    return { mass, center: { x: shape.center.x, y: shape.center.y }, inertia: mass * shapeUnitInertia(shape) };
  }
  const { area } = polygonAreaCentroid(shape.vertices);
  const mass = density * area;
  return { mass, center: { x: shape.centroid.x, y: shape.centroid.y }, inertia: mass * shapeUnitInertia(shape) };
}

export function computeAABB(shape, xf, margin = 0) {
  const { p, q } = xf;
  if (shape.type === 'circle') {
    const cx = q.c * shape.center.x - q.s * shape.center.y + p.x;
    const cy = q.s * shape.center.x + q.c * shape.center.y + p.y;
    const r = shape.radius + margin;
    return { minX: cx - r, minY: cy - r, maxX: cx + r, maxY: cy + r };
  }
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const v of shape.vertices) {
    const x = q.c * v.x - q.s * v.y + p.x;
    const y = q.s * v.x + q.c * v.y + p.y;
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }
  return { minX: minX - margin, minY: minY - margin, maxX: maxX + margin, maxY: maxY + margin };
}

export function aabbOverlap(a, b) {
  return a.minX <= b.maxX && a.maxX >= b.minX && a.minY <= b.maxY && a.maxY >= b.minY;
}
