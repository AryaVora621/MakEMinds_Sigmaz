// geometry.js - pure polygon helpers for wall-collisions. No DOM. Units: inches, radians, lb.
// Frame: Pedro (x right, y far), heading CCW, robot +x forward. All polygons CCW; normals unit and outward.

export function dot2(ax, ay, bx, by) { return ax * bx + ay * by; }
export function cross2(ax, ay, bx, by) { return ax * by - ay * bx; }

function edgeNormals(verts) {
  const n = verts.length, out = [];
  for (let i = 0; i < n; i++) {
    const a = verts[i], b = verts[(i + 1) % n];
    const ex = b[0] - a[0], ey = b[1] - a[1];
    const len = Math.hypot(ex, ey);
    out.push(len > 0 ? [ey / len + 0, -ex / len + 0] : [0, 0]);
  }
  return out;
}

// Robot local polygon (CCW). bumper is added to both half extents; chamfer c cuts corners (8 verts when c > 0).
export function robotPolygon(robot) {
  const c = robot.chamfer || 0, b = robot.bumper || 0;
  const hx = robot.halfX + b, hy = robot.halfY + b;
  const verts = c > 0
    ? [[hx, -hy + c], [hx, hy - c], [hx - c, hy], [-hx + c, hy],
       [-hx, hy - c], [-hx, -hy + c], [-hx + c, -hy], [hx - c, -hy]]
    : [[hx, -hy], [hx, hy], [-hx, hy], [-hx, -hy]];
  let reach = 0;
  for (const v of verts) reach = Math.max(reach, Math.hypot(v[0], v[1]));
  return { verts, normals: edgeNormals(verts), hx, hy, reach };
}

// World polygon: p = (x,y) + R(h)*v; normals rotated by R(h).
export function transformPolygon(poly, x, y, h) {
  const c = Math.cos(h), s = Math.sin(h);
  return {
    verts: poly.verts.map((v) => [x + c * v[0] - s * v[1], y + s * v[0] + c * v[1]]),
    normals: poly.normals.map((n) => [c * n[0] - s * n[1], s * n[0] + c * n[1]]),
    cx: x, cy: y, h
  };
}

// AABB obstacle polygon, CCW from (minX,minY); normals (0,-1),(1,0),(0,1),(-1,0).
export function boxPolygon(box) {
  return {
    verts: [[box.minX, box.minY], [box.maxX, box.minY], [box.maxX, box.maxY], [box.minX, box.maxY]],
    normals: [[0, -1], [1, 0], [0, 1], [-1, 0]]
  };
}

export function polygonAabb(wpoly) {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const v of wpoly.verts) {
    if (v[0] < minX) minX = v[0];
    if (v[0] > maxX) maxX = v[0];
    if (v[1] < minY) minY = v[1];
    if (v[1] > maxY) maxY = v[1];
  }
  return { minX, minY, maxX, maxY };
}

// Mass (lb) and plate inertia (lb*in^2): I = inertiaScale * m * ((2hx)^2 + (2hy)^2) / 12, hx/hy include bumper.
export function bodyMass(P) {
  const r = P.robot, m = r.massLb;
  const hx = r.halfX + (r.bumper || 0), hy = r.halfY + (r.bumper || 0);
  const I = r.inertiaScale * m * ((2 * hx) ** 2 + (2 * hy) ** 2) / 12;
  return { m, invM: 1 / m, I, invI: 1 / I };
}
