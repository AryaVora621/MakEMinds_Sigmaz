// robot_geometry.js - robot union-of-boxes shape, ball-vs-robot contacts, intake pocket test.
// Pure functions, no DOM. Units: length in, angle rad. Robot-local frame: +x forward, +y left, z up.
// world = (c*lx - s*ly, s*lx + c*ly) with c=cos(h), s=sin(h).
// All dimension defaults come from params.js (shape.*): cheekWidthIn, pocketDepthIn, pocketHeightIn are ASSUMED.
import { RBC_PARAMS } from './params.js';

/**
 * Build the robot shape as axis-aligned local boxes.
 * @param {{halfX:number, halfY:number, height:number}} dims  in (half extents, full height)
 * @returns {{boxes:Array, sides:number[], dims:object}}  sides: +1 front, -1 rear
 */
export function buildRobotShape(dims, params = RBC_PARAMS) {
  const { halfX, halfY, height } = dims;
  const sp = params.shape;
  const sides = [];
  for (const s of sp.sides || []) {
    if (s === 'front') sides.push(1);
    else if (s === 'rear') sides.push(-1);
  }
  const depth = sp.pocketDepthIn, cheek = sp.cheekWidthIn, ph = sp.pocketHeightIn;
  const hasFront = sides.includes(1), hasRear = sides.includes(-1);
  const boxes = [];
  boxes.push({
    minX: -halfX + (hasRear ? depth : 0), maxX: halfX - (hasFront ? depth : 0),
    minY: -halfY, maxY: halfY, minZ: 0, maxZ: height, tag: 'body',
  });
  for (const sd of sides) {
    const x0 = sd > 0 ? halfX - depth : -halfX;
    const x1 = sd > 0 ? halfX : -halfX + depth;
    if (ph < height) {
      boxes.push({ minX: x0, maxX: x1, minY: -halfY, maxY: halfY, minZ: ph, maxZ: height, tag: 'roof' });
    }
    const zTop = Math.min(ph, height);
    boxes.push({ minX: x0, maxX: x1, minY: halfY - cheek, maxY: halfY, minZ: 0, maxZ: zTop, tag: 'cheek' });
    boxes.push({ minX: x0, maxX: x1, minY: -halfY, maxY: -halfY + cheek, minZ: 0, maxZ: zTop, tag: 'cheek' });
  }
  return { boxes, sides, dims: { halfX, halfY, height } };
}

/** World xy (in) -> robot-local [lx, ly] (in). */
export function toLocal(robot, x, y) {
  const dx = x - robot.x, dy = y - robot.y;
  const c = Math.cos(robot.h), s = Math.sin(robot.h);
  return [c * dx + s * dy, -s * dx + c * dy];
}

/** Robot-local direction/vector (lx, ly) -> world [wx, wy] (rotation only). */
export function toWorldDir(robot, lx, ly) {
  const c = Math.cos(robot.h), s = Math.sin(robot.h);
  return [c * lx - s * ly, s * lx + c * ly];
}

/**
 * Contacts between a ball (centre ball.x/y/z, radius ball.r, in) and the robot shape.
 * Normal n points robot -> ball (world, unit). pen in inches. p = ball centre - r*n. (rx, ry) = p_xy - robot_xy.
 * Sorted by pen descending, at most maxContacts.
 */
export function robotContacts(ball, robot, shape, maxContacts = 2) {
  const r = ball.r;
  const [lx, ly] = toLocal(robot, ball.x, ball.y);
  const lz = ball.z;
  const out = [];
  for (const b of shape.boxes) {
    const qx = Math.min(Math.max(lx, b.minX), b.maxX);
    const qy = Math.min(Math.max(ly, b.minY), b.maxY);
    const qz = Math.min(Math.max(lz, b.minZ), b.maxZ);
    let dx = lx - qx, dy = ly - qy, dz = lz - qz;
    const d2 = dx * dx + dy * dy + dz * dz;
    if (d2 > r * r) continue;
    let nlx, nly, nlz, pen;
    if (d2 > 1e-18) {
      const d = Math.sqrt(d2);
      nlx = dx / d; nly = dy / d; nlz = dz / d; pen = r - d;
    } else {
      // centre inside box: push out through the min-depth face
      const cand = [
        [lx - b.minX, -1, 0, 0], [b.maxX - lx, 1, 0, 0],
        [ly - b.minY, 0, -1, 0], [b.maxY - ly, 0, 1, 0],
        [lz - b.minZ, 0, 0, -1], [b.maxZ - lz, 0, 0, 1],
      ];
      let best = cand[0];
      for (const c of cand) if (c[0] < best[0]) best = c;
      nlx = best[1]; nly = best[2]; nlz = best[3];
      pen = best[0] + r;
    }
    const [nx, ny] = toWorldDir(robot, nlx, nly);
    const nz = nlz;
    const px = ball.x - r * nx, py = ball.y - r * ny, pz = ball.z - r * nz;
    out.push({ nx, ny, nz, pen, px, py, pz, rx: px - robot.x, ry: py - robot.y, tag: b.tag });
  }
  out.sort((a, b) => b.pen - a.pen);
  if (out.length > maxContacts) out.length = maxContacts;
  return out;
}

/**
 * Where is the ball relative to the intake pockets?
 * u = sd*lx - halfX (in): distance of the centre beyond the face plane; negative = inside the face.
 * inMouth: within the cheeks laterally and below the pocket roof height.
 * Side chosen by the sign of lx among available sides.
 */
export function pocketTest(ball, robot, shape, params = RBC_PARAMS) {
  const [lx, ly] = toLocal(robot, ball.x, ball.y);
  const { halfX, halfY } = shape.dims;
  const sp = params.shape;
  let sd = 0;
  if (shape.sides.length) {
    const want = lx >= 0 ? 1 : -1;
    if (shape.sides.includes(want)) sd = want;
  }
  if (sd === 0) return { side: null, sd: 0, u: Infinity, inMouth: false, lx, ly };
  const u = sd * lx - halfX;
  const inMouth = Math.abs(ly) <= halfY - sp.cheekWidthIn && ball.z - ball.r < sp.pocketHeightIn;
  return { side: sd > 0 ? 'front' : 'rear', sd, u, inMouth, lx, ly };
}
