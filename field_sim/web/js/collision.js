// Robot-vs-field collision. Pure ES module (browser + node). Inches, Pedro frame (x right, y far, z up).
// Robot = rotated rectangle (heading h about z, robot +x forward). Obstacles = AABB boxes + circles + play-area bounds.
export const ROBOT = { halfX: 9, halfY: 9 };

const Z_ROBOT = 18, EPS = 1e-7, MARGIN = 1e-4;

export function buildObstacles(colliders) {
  const b = colliders.bounds;
  const zcap = colliders.z_cap || Z_ROBOT;
  const boxes = [], circles = [];
  for (const o of colliders.boxes || []) {
    if (o.zMin >= zcap || o.zMax <= 0) continue;
    boxes.push({ minX: o.minX, minY: o.minY, maxX: o.maxX, maxY: o.maxY, cx: (o.minX + o.maxX) / 2, cy: (o.minY + o.maxY) / 2, hx: (o.maxX - o.minX) / 2, hy: (o.maxY - o.minY) / 2, kind: o.kind });
  }
  for (const o of colliders.circles || []) {
    if (o.zMin >= zcap || o.zMax <= 0) continue;
    circles.push({ x: o.x, y: o.y, r: o.r, kind: o.kind });
  }
  return { bounds: { minX: b.minX, minY: b.minY, maxX: b.maxX, maxY: b.maxY }, boxes, circles };
}

export function robotCorners(x, y, h, halfX = ROBOT.halfX, halfY = ROBOT.halfY) {
  const c = Math.cos(h), s = Math.sin(h);
  const ax = c * halfX, ay = s * halfX, bx = -s * halfY, by = c * halfY;
  return [[x + ax + bx, y + ay + by], [x + ax - bx, y + ay - by], [x - ax - bx, y - ay - by], [x - ax + bx, y - ay + by]];
}

// Working state (mutable, one per call): st = {x, y, c, s, hx, hy, pushed, hit, normals}
function addNormal(st, nx, ny) {
  const n = st.normals;
  for (let i = 0; i < n.length; i++) if (n[i][0] * nx + n[i][1] * ny > 0.9999) return;
  n.push([nx, ny]);
}
function push(st, nx, ny, d) {
  d += MARGIN;
  st.x += nx * d; st.y += ny * d; st.pushed += d; st.hit = true;
  addNormal(st, nx, ny);
}

function resolveBounds(st, b) {
  const ex = Math.abs(st.c) * st.hx + Math.abs(st.s) * st.hy;
  const ey = Math.abs(st.s) * st.hx + Math.abs(st.c) * st.hy;
  let d;
  if ((d = b.minX + ex - st.x) > EPS) push(st, 1, 0, d);
  if ((d = st.x + ex - b.maxX) > EPS) push(st, -1, 0, d);
  if ((d = b.minY + ey - st.y) > EPS) push(st, 0, 1, d);
  if ((d = st.y + ey - b.maxY) > EPS) push(st, 0, -1, d);
}

// SAT: robot OBB vs AABB; pushes robot along the minimum-overlap axis. probe=true: only report overlap.
function resolveBox(st, o, probe) {
  const c = st.c, s = st.s, hx = st.hx, hy = st.hy;
  const ac = Math.abs(c), as = Math.abs(s);
  const ex = ac * hx + as * hy, ey = as * hx + ac * hy;
  const dx = st.x - o.cx, dy = st.y - o.cy;
  const oX = ex + o.hx - Math.abs(dx); if (oX <= EPS) return false;
  const oY = ey + o.hy - Math.abs(dy); if (oY <= EPS) return false;
  const du = dx * c + dy * s, dv = -dx * s + dy * c;
  const oU = hx + o.hx * ac + o.hy * as - Math.abs(du); if (oU <= EPS) return false;
  const oV = hy + o.hx * as + o.hy * ac - Math.abs(dv); if (oV <= EPS) return false;
  if (probe) return true;
  let best = oX, nx = dx >= 0 ? 1 : -1, ny = 0;
  if (oY < best) { best = oY; nx = 0; ny = dy >= 0 ? 1 : -1; }
  if (oU < best) { best = oU; const sg = du >= 0 ? 1 : -1; nx = c * sg; ny = s * sg; }
  if (oV < best) { best = oV; const sg = dv >= 0 ? 1 : -1; nx = -s * sg; ny = c * sg; }
  push(st, nx, ny, best);
  return true;
}

function resolveCircle(st, o, probe) {
  const c = st.c, s = st.s, hx = st.hx, hy = st.hy;
  const dx = o.x - st.x, dy = o.y - st.y;
  const lx = dx * c + dy * s, ly = -dx * s + dy * c;       // circle centre in robot frame
  const qx = lx < -hx ? -hx : lx > hx ? hx : lx, qy = ly < -hy ? -hy : ly > hy ? hy : ly;
  const fx = lx - qx, fy = ly - qy, d2 = fx * fx + fy * fy;
  if (d2 > EPS * EPS) {
    const r = o.r - EPS;
    if (d2 >= r * r) return false;
    if (probe) return true;
    const dist = Math.sqrt(d2);
    const nxl = -fx / dist, nyl = -fy / dist;               // robot moves away from the circle
    push(st, nxl * c - nyl * s, nxl * s + nyl * c, o.r - dist);
    return true;
  }
  if (probe) return true;
  const a = hx - Math.abs(lx), b = hy - Math.abs(ly);        // centre inside robot: exit via nearest face
  let nxl = 0, nyl = 0, d;
  if (a < b) { nxl = lx >= 0 ? -1 : 1; d = a + o.r; } else { nyl = ly >= 0 ? -1 : 1; d = b + o.r; }
  push(st, nxl * c - nyl * s, nxl * s + nyl * c, d);
  return true;
}

function pass(st, obs) {
  const before = st.pushed;
  resolveBounds(st, obs.bounds);
  const B = obs.boxes, C = obs.circles;
  for (let i = 0; i < B.length; i++) resolveBox(st, B[i], false);
  for (let i = 0; i < C.length; i++) resolveCircle(st, C[i], false);
  return st.pushed > before;
}

function isFree(x, y, h, obs, hx, hy) {
  const st = { x, y, c: Math.cos(h), s: Math.sin(h), hx, hy, pushed: 0, hit: false, normals: [] };
  const ex = Math.abs(st.c) * hx + Math.abs(st.s) * hy, ey = Math.abs(st.s) * hx + Math.abs(st.c) * hy, b = obs.bounds;
  if (b.minX + ex - x > EPS || x + ex - b.maxX > EPS || b.minY + ey - y > EPS || y + ey - b.maxY > EPS) return false;
  for (const o of obs.boxes) if (resolveBox(st, o, true)) return false;
  for (const o of obs.circles) if (resolveCircle(st, o, true)) return false;
  return true;
}

const wrap = (a) => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };

export function resolveRobot(pose, prev, obstacles, opts = {}) {
  const hx = opts.halfX ?? ROBOT.halfX, hy = opts.halfY ?? ROBOT.halfY;
  const passes = opts.passes ?? 4, maxStep = opts.maxStep ?? 1.5;
  const st = { x: 0, y: 0, c: 1, s: 0, hx, hy, pushed: 0, hit: false, normals: [] };
  let h = pose.h;
  const setH = (a) => { st.c = Math.cos(a); st.s = Math.sin(a); };
  const settle = (n) => { for (let i = 0; i < n; i++) if (!pass(st, obstacles)) break; };

  if (prev) {
    // substep along the path (translation and rotation) so fast moves cannot tunnel
    const dx = pose.x - prev.x, dy = pose.y - prev.y, dh = wrap(pose.h - prev.h);
    const reach = Math.hypot(hx, hy);
    const dist = Math.max(Math.hypot(dx, dy), Math.abs(dh) * reach);
    const n = Math.min(200, Math.max(1, Math.ceil(dist / maxStep)));
    st.x = prev.x; st.y = prev.y; h = prev.h;
    for (let i = 1; i <= n; i++) {
      st.x += dx / n; st.y += dy / n; h += dh / n;
      setH(h); settle(passes);
    }
    h = prev.h + dh;
  } else {
    st.x = pose.x; st.y = pose.y; setH(h); settle(passes);
  }
  setH(h);
  settle(8);
  if (!isFree(st.x, st.y, h, obstacles, hx, hy)) {
    // wedged: fall back to the previous pose if it was valid, else at least clamp to the play area
    if (prev && isFree(prev.x, prev.y, prev.h, obstacles, hx, hy)) return { x: prev.x, y: prev.y, h: prev.h, hit: true, normals: st.normals, pushed: st.pushed };
    resolveBounds(st, obstacles.bounds);
  }
  if (!st.hit) return { x: pose.x, y: pose.y, h: pose.h, hit: false, normals: st.normals, pushed: 0 };
  return { x: st.x, y: st.y, h: pose.h, hit: true, normals: st.normals, pushed: st.pushed };
}

export function slideVelocity(vx, vy, normals) {
  // two sweeps so resolving one contact cannot leave residual into another (corner)
  for (let k = 0; k < 2; k++) {
    for (let i = 0; i < normals.length; i++) {
      const n = normals[i], d = vx * n[0] + vy * n[1];
      if (d < 0) { vx -= d * n[0]; vy -= d * n[1]; }
    }
  }
  return { vx, vy };
}
