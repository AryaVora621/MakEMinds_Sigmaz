// Field background of the Auto planner, drawn like FRC PathPlanner's field view: one flat neutral surface, a white field border,
// tape lines and field elements as solid outlines / solid fills. No translucent boxes, no captions.
// Pure canvas code; the host (app.js drawMap) passes everything it knows in env:
//   { W, H, k, mx, my, bounds:{x0,x1,y0,y1}, scene (scene.json), hive:{red,blue} tip states, mouse:[px,py]|null, fromScreen(px,py)->[x,y] }
// Colours: PathPlanner dark theme, see docs/overnight/planner-redesign.md for sources.
export const FIELD_COLORS = {
  bg: '#1b1b1f',          // field-view background; sampled from a PathPlanner screenshot (Screenshot 2026-11-07 091143.png): #1b1b1f
  floor: '#1f2025',       // the field surface itself, a hair lighter than the background so the border reads without a box
  border: '#e6e6ea', red: '#c62828', blue: '#1f5fd6', white: '#cfd2da',
  hiveFill: '#34353b', hiveEdge: '#55565e', element: '#ff9800', dim: '#90909a',
};
const TAU = Math.PI * 2;

function hull(pts) {
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]); const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = []; for (const q of p) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  const up = []; for (const q of p.reverse()) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}

/** Screen rectangle {L,T,R,B} of the visible bounds (min/max of the corners, so it holds in every rotation). */
export function boundsBox(env) {
  const b = env.bounds, xs = [env.mx(b.x0), env.mx(b.x1)], ys = [env.my(b.y0), env.my(b.y1)];
  return { L: Math.min(...xs), R: Math.max(...xs), T: Math.min(...ys), B: Math.max(...ys) };
}

export function drawPlannerField(ctx, env) {
  const { W, H, k, mx, my, bounds: b, scene, hive = {} } = env, C = FIELD_COLORS, box = boundsBox(env);
  ctx.save();
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, W, H);
  ctx.beginPath(); ctx.rect(box.L, box.T, box.R - box.L, box.B - box.T); ctx.clip();
  ctx.fillStyle = C.floor; ctx.fillRect(box.L - 1, box.T - 1, box.R - box.L + 2, box.B - box.T + 2);
  const poly = (pts, fill, stroke, lw) => {
    ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(mx(p[0]), my(p[1])) : ctx.moveTo(mx(p[0]), my(p[1])))); ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.lineWidth = lw; ctx.strokeStyle = stroke; ctx.lineJoin = 'round'; ctx.stroke(); }
  };
  const tcol = c => (c === 'red' ? C.red : c === 'blue' ? C.blue : C.white);
  // tape lines (solid, to scale)
  ctx.lineCap = 'butt';
  for (const t of (scene && scene.tapes && scene.tapes.tapes) || []) {
    if (!t.p0 || !t.p1) continue;
    ctx.beginPath(); ctx.moveTo(mx(t.p0[0]), my(t.p0[1])); ctx.lineTo(mx(t.p1[0]), my(t.p1[1]));
    ctx.strokeStyle = tcol(t.color); ctx.lineWidth = Math.max(1.5, (t.width || 2) * k); ctx.stroke();
  }
  // hive frame and the current tip-state cells: solid grey with alliance outlines
  if (scene && scene.constants) {
    poly([[72 - 24.73, 72 - 19.475], [72 + 24.73, 72 - 19.475], [72 + 24.73, 72 + 19.475], [72 - 24.73, 72 + 19.475]], C.hiveFill, C.hiveEdge, 2);
    for (const a of ['red', 'blue']) {
      const h = scene.constants.hives && scene.constants.hives[a], st = h && h.states && h.states[hive[a] || Object.keys(h.states)[0]];
      if (!st) continue;
      for (const c of Object.values(st.cells || {})) {
        const pts = [...(c.opening_corners || []), ...(c.floor_corners || []), ...(c.back_wall_corners || [])].map(p => [p[0], p[1]]);
        if (pts.length >= 3) poly(hull(pts), c.up ? C.hiveEdge : null, tcol(a), c.up ? 2.5 : 1.5);
      }
    }
    // flowers: orange rings like PathPlanner's note markers
    for (const f of scene.constants.flowers || []) {
      const c = f.opening_center; if (!c) continue;
      ctx.beginPath(); ctx.arc(mx(c[0]), my(c[1]), Math.max(4, 3.2 * k), 0, TAU); ctx.strokeStyle = C.element; ctx.lineWidth = 2; ctx.stroke();
    }
  }
  // loose balls at their starting spots: small solid dots
  for (const bl of (scene && scene.balls && scene.balls.balls) || []) {
    if (bl.location !== 'field' || !bl.position) continue;
    ctx.beginPath(); ctx.arc(mx(bl.position[0]), my(bl.position[1]), Math.max(1.8, (bl.diameter || 2.8) / 2 * k), 0, TAU);
    ctx.fillStyle = bl.type === 'nectar' ? '#ff9a3d' : '#d9b92e'; ctx.fill();
  }
  ctx.restore();

  // field border: only the real field edges (the cut through the middle of an alliance half gets none)
  ctx.save(); ctx.strokeStyle = C.border; ctx.lineWidth = 3; ctx.lineCap = 'square';
  const edge = (x0, y0, x1, y1) => { ctx.beginPath(); ctx.moveTo(mx(x0), my(y0)); ctx.lineTo(mx(x1), my(y1)); ctx.stroke(); };
  if (b.x0 <= 0) edge(0, b.y0, 0, b.y1);
  if (b.x1 >= 144) edge(144, b.y0, 144, b.y1);
  if (b.y0 <= 0) edge(b.x0, 0, b.x1, 0);
  if (b.y1 >= 144) edge(b.x0, 144, b.x1, 144);
  if (b.x0 > 0 || b.x1 < 144) {                     // alliance half: the cut is a thin dashed line, not a wall
    const cx = b.x0 > 0 ? b.x0 : b.x1; ctx.setLineDash([6, 6]); ctx.lineWidth = 1; ctx.strokeStyle = C.dim; edge(cx, b.y0, cx, b.y1); ctx.setLineDash([]);
  }
  ctx.restore();

}

/** Screen-space overlay (drawn after the view rotation is undone): axes gizmo at the top-left of the field and a plain-text cursor readout at its bottom-right.
 *  env.px(x, y) -> [pixelX, pixelY] gives final screen positions in every rotation, so nothing here can clip or overlap field content. */
export function drawFieldOverlay(ctx, env) {
  const C = FIELD_COLORS, b = env.bounds, px = env.px || ((x, y) => [env.mx(x), env.my(y)]);
  const cs = [px(b.x0, b.y0), px(b.x1, b.y0), px(b.x0, b.y1), px(b.x1, b.y1)], L = Math.min(...cs.map(p => p[0])), T = Math.min(...cs.map(p => p[1])), R = Math.max(...cs.map(p => p[0])), B = Math.max(...cs.map(p => p[1]));
  const av = env.avail || { L, T, R, B };   // the free area the field is fitted into: when the field does not fill it (full field), the gizmo and readout sit in the empty part
  const o = px(0, 0), dir = (vx, vy) => { const p = px(vx, vy), dx = p[0] - o[0], dy = p[1] - o[1], n = Math.hypot(dx, dy) || 1; return [dx / n, dy / n]; };
  const dX = dir(10, 0), dY = dir(0, 10), reach = 36, xs = [0, dX[0] * reach, dY[0] * reach], ys = [0, dX[1] * reach, dY[1] * reach];
  const ox = av.L + 16 - Math.min(...xs), oy = av.B - 24 - Math.max(...ys);   // bottom-left corner, shifted so both arrows and their labels stay inside the field box
  const ax = (vx, vy, label) => {
    const [ux, uy] = dir(vx, vy), len = 22;
    ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + ux * len, oy + uy * len);
    ctx.moveTo(ox + ux * len - uy * 3 - ux * 5, oy + uy * len + ux * 3 - uy * 5); ctx.lineTo(ox + ux * len, oy + uy * len); ctx.lineTo(ox + ux * len + uy * 3 - ux * 5, oy + uy * len - ux * 3 - uy * 5);
    ctx.stroke(); ctx.fillText(label, ox + ux * (len + 9), oy + uy * (len + 9));
  };
  ctx.save(); ctx.strokeStyle = C.dim; ctx.fillStyle = C.dim; ctx.lineWidth = 1.5; ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.font = '600 11px Inter, "Segoe UI", sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ax(10, 0, 'x'); ax(0, 10, 'y');
  if (env.mouse && env.fromScreen) {
    const [x, y] = env.fromScreen(env.mouse[0], env.mouse[1]);
    if (x >= -1 && x <= 145 && y >= -1 && y <= 145) { ctx.textAlign = 'right'; ctx.textBaseline = 'alphabetic'; const txt = 'x ' + x.toFixed(1) + '   y ' + y.toFixed(1) + ' in'; ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(27,27,31,0.95)'; ctx.strokeText(txt, av.R - 10, av.B - 10); ctx.fillText(txt, av.R - 10, av.B - 10); }
  }
  ctx.restore();
}
