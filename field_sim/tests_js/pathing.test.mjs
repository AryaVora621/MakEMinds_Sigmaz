import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b}`);
function sampleRouteObj() {
  const r = P.newRoute({ name: 'Demo', start: { x: 24, y: 24, h: Math.PI / 2 } });
  P.addLine(r, 24, 72);
  P.addCurve(r, 60, 96, 24, 96);
  P.addLine(r, 100, 96);
  r.segments[0].heading = { mode: 'linear', start: Math.PI / 2, end: Math.PI, reversed: false };
  r.segments[2].heading = { mode: 'constant', start: 0.3, reversed: false };
  r.segments[1].callbacks.push({ t: 0.5, name: 'shoot' });
  return r;
}

test('bezier endpoints and midpoint', () => {
  const pts = [{ x: 0, y: 0 }, { x: 10, y: 20 }, { x: 20, y: 0 }];
  assert.deepEqual(P.bezierPoint(pts, 0), { x: 0, y: 0 });
  assert.deepEqual(P.bezierPoint(pts, 1), { x: 20, y: 0 });
  const m = P.bezierPoint(pts, 0.5); near(m.x, 10); near(m.y, 10);
});

test('bezier derivative', () => {
  const line = [{ x: 0, y: 0 }, { x: 10, y: 5 }];
  assert.deepEqual(P.bezierDerivative(line, 0.3), { x: 10, y: 5 });
  const c = [{ x: 0, y: 0 }, { x: 10, y: 20 }, { x: 20, y: 0 }];
  const d = P.bezierDerivative(c, 0); near(d.x, 20); near(d.y, 40);
  const d1 = P.bezierDerivative(c, 1); near(d1.x, 20); near(d1.y, -40);
});

test('arc-length sampling is evenly spaced and route length matches', () => {
  const r = P.newRoute({ start: { x: 10, y: 10, h: 0 } }); P.addCurve(r, 100, 40, 10, 80);
  const pts = P.sampleRoute(r, 2);
  for (let i = 1; i < pts.length - 1; i++) near(Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y), 2, 0.05);
  near(pts.at(-1).s, P.routeLength(r), 1e-6);
  assert.deepEqual([pts[0].x, pts[0].y], [10, 10]); near(pts.at(-1).x, 100); near(pts.at(-1).y, 40);
});

test('linear heading takes the shortest way round the wrap', () => {
  const r = P.newRoute({ start: { x: 10, y: 10, h: 0 } }); P.addLine(r, 50, 10);
  r.segments[0].heading = { mode: 'linear', start: 170 * Math.PI / 180, end: -170 * Math.PI / 180, reversed: false };
  const pts = P.sampleRoute(r, 1), mid = pts[Math.floor(pts.length / 2)];
  assert.ok(Math.abs(Math.abs(mid.h) - Math.PI) < 0.05, `mid heading ${mid.h}`);
  near(pts[0].h, 170 * Math.PI / 180); near(pts.at(-1).h, -170 * Math.PI / 180);
});

test('constant heading holds, defaulting to the previous end heading', () => {
  const r = P.newRoute({ start: { x: 10, y: 10, h: 1.0 } }); P.addLine(r, 50, 50);
  r.segments[0].heading = { mode: 'constant', reversed: false };
  for (const p of P.sampleRoute(r, 3)) near(p.h, 1.0);
});

test('tangent heading follows the path; reversed flips by pi', () => {
  const r = P.newRoute({ start: { x: 10, y: 10, h: 0 } }); P.addLine(r, 10, 60);
  r.segments[0].heading = { mode: 'tangent', reversed: false };
  for (const p of P.sampleRoute(r, 5)) near(p.h, Math.PI / 2);
  r.segments[0].heading.reversed = true;
  for (const p of P.sampleRoute(r, 5)) near(Math.abs(p.h), Math.PI / 2);
  near(P.sampleRoute(r, 5)[0].h, -Math.PI / 2);
});

test('mirror twice is identity and flips x about 72', () => {
  const r = sampleRouteObj(), m = P.mirrorRoute(r);
  assert.equal(m.alliance, 'red'); near(m.start.x, 120); near(m.segments[0].points[1].x, 120);
  near(m.start.h, Math.PI / 2);
  near(m.segments[0].heading.end, 0, 1e-9);
  const mm = P.mirrorRoute(m);
  assert.equal(mm.alliance, r.alliance);
  assert.equal(mm.segments.length, r.segments.length);
  mm.segments.forEach((s, i) => s.points.forEach((p, j) => { near(p.x, r.segments[i].points[j].x, 1e-9); near(p.y, r.segments[i].points[j].y, 1e-9); }));
  near(P.wrapAngle(mm.segments[2].heading.start), 0.3, 1e-9);
});

test('JSON round trip preserves the route', () => {
  const r = sampleRouteObj(), back = P.fromJSON(P.toJSON(r));
  assert.deepEqual(back.segments, r.segments); assert.deepEqual(back.start, r.start);
  assert.equal(back.name, 'Demo');
});

test('fromJSON rejects bad input with clear errors', () => {
  const good = JSON.parse(P.toJSON(sampleRouteObj()));
  const mod = f => { const o = structuredClone(good); f(o); return JSON.stringify(o); };
  assert.throws(() => P.fromJSON('not json'), /Invalid JSON/);
  assert.throws(() => P.fromJSON(mod(o => { o.schema = 99; })), /schema/);
  assert.throws(() => P.fromJSON(mod(o => { o.segments[0].points[1].x = null; })), /segments\[0\]\.points\[1\]\.x.*finite/);
  assert.throws(() => P.fromJSON(mod(o => { o.segments[0].points[1].y = 200; })), /outside the field/);
  assert.throws(() => P.fromJSON(mod(o => { o.segments[0].type = 'line'; o.segments[0].points.push({ x: 1, y: 1 }); })), /exactly 2/);
  assert.throws(() => P.fromJSON(mod(o => { o.segments[1].points.length = 2; })), /at least 3/);
  assert.throws(() => P.fromJSON(mod(o => { o.segments[0].heading.mode = 'spin'; })), /heading\.mode/);
});

test('Java export has the right imports and constructors', () => {
  const j = P.toJava(sampleRouteObj(), { className: 'DemoAuto' });
  for (const imp of ['com.pedropathing.geometry.Pose', 'com.pedropathing.geometry.BezierLine', 'com.pedropathing.geometry.BezierCurve', 'com.pedropathing.paths.PathChain', 'com.pedropathing.follower.Follower'])
    assert.ok(j.includes(`import ${imp};`), imp);
  assert.match(j, /public class DemoAuto extends OpMode/);
  assert.match(j, /new Pose\(24\.0, 24\.0, Math\.toRadians\(90\)\)/);
  assert.match(j, /new BezierLine\(startPose, pose1\)/);
  assert.match(j, /new BezierCurve\(pose1, control2_1, pose2\)/);
  assert.match(j, /setLinearHeadingInterpolation\(Math\.toRadians\(90\), Math\.toRadians\(180\)\)/);
  assert.match(j, /setTangentHeadingInterpolation\(\)/);
  assert.match(j, /setConstantHeadingInterpolation\(Math\.toRadians\(17\.19\)\)/);
  assert.doesNotMatch(j, /addParametricCallback/, 'chain callbacks never fire in Pedro 2.1.0');
  assert.match(j, /follower\.getCurrentTValue\(\) >= 0\.5/); assert.match(j, /private boolean ev\d+_1;/); assert.match(j, /shooter\.shoot\(1\);/);
});

test('Java export is structurally sound', () => {
  const j = P.toJava(sampleRouteObj());
  const count = c => j.split(c).length - 1;
  assert.equal(count('{'), count('}')); assert.equal(count('('), count(')'));
  assert.ok(!/NaN|undefined|Infinity/.test(j));
  const declared = new Set([...j.matchAll(/private PathChain ([^;]+);/g)].flatMap(m => m[1].split(',').map(s => s.trim())));
  const used = new Set([...j.matchAll(/\bpath\d+\b/g)].map(m => m[0]));
  for (const u of used) assert.ok(declared.has(u), `${u} undefined`);
  assert.equal(declared.size, 3);
  assert.ok(!/1\.5707963/.test(j));
  assert.match(P.toJavaSnippet(sampleRouteObj()), /public void buildPaths\(\) \{/);
});

test('Java export of an empty route still compiles structurally', () => {
  const j = P.toJava(P.newRoute());
  assert.equal(j.split('{').length, j.split('}').length); assert.ok(!j.includes('NaN'));
});

test('followSim reaches the end and respects vmax', () => {
  const r = sampleRouteObj(), st = {}; let maxV = 0, last, steps = 0;
  do { last = P.followSim(r, 0.01, st, { vmax: 60, aMax: 90 }); maxV = Math.max(maxV, last.v); steps++; } while (!last.done && steps < 100000);
  assert.ok(last.done); near(last.progress, 1); near(last.x, 100, 1e-6); near(last.y, 96, 1e-6);
  assert.ok(maxV <= 60 + 1e-9, `maxV ${maxV}`); assert.ok(maxV > 50);
  const T = P.estimateTime(r, 60, 90), L = P.routeLength(r);
  assert.ok(T >= L / 60 && T < L / 60 + 2, `T ${T} L ${L}`);
});

test('validateRoute finds out-of-bounds, zero-length, heading jumps and obstacles', () => {
  const r = P.newRoute({ start: { x: 20, y: 20, h: 0 } });
  P.addLine(r, 150, 20); P.addLine(r, 150, 20);
  assert.ok(P.validateRoute(r).some(i => i.severity === 'error' && /outside the field/.test(i.msg)));
  assert.ok(P.validateRoute(r).some(i => /zero length/.test(i.msg)));
  const ok = P.newRoute({ start: { x: 20, y: 20, h: 0 } }); P.addLine(ok, 100, 20);
  ok.segments[0].heading = { mode: 'linear', start: Math.PI, end: Math.PI, reversed: false };
  assert.ok(P.validateRoute(ok).some(i => /Heading jumps/.test(i.msg)));
  assert.equal(P.validateRoute(P.newRoute()).length, 0);
  const iss = P.validateRoute(ok, [{ minX: 50, minY: 10, maxX: 60, maxY: 30, name: 'wall' }]);
  assert.ok(iss.some(i => /obstacle wall/.test(i.msg)));
});

test('editing: move keeps continuity, delete, split, type change', () => {
  const r = sampleRouteObj();
  P.movePoint(r, 0, 1, 30, 80);
  assert.deepEqual(r.segments[1].points[0], { x: 30, y: 80 });
  const len = P.routeLength(r);
  assert.ok(P.splitSegment(r, 0, 0.5)); assert.equal(r.segments.length, 4);
  near(P.routeLength(r), len, 1e-6);
  assert.deepEqual(r.segments[0].points.at(-1), r.segments[1].points[0]);
  assert.ok(P.deletePoint(r, 2, 1)); assert.equal(r.segments[2].type, 'line');
  assert.equal(P.deletePoint(r, 0, 0), false);
  const n = r.segments.length; P.deleteSegment(r, n - 1); assert.equal(r.segments.length, n - 1);
});

test('history undo / redo restores in place', () => {
  const r = P.newRoute(), h = P.createHistory(r);
  P.addLine(r, 50, 50); h.commit(); P.addLine(r, 60, 60); h.commit();
  assert.equal(r.segments.length, 2);
  h.undo(); assert.equal(r.segments.length, 1); h.undo(); assert.equal(r.segments.length, 0);
  assert.equal(h.undo(), false); h.redo(); h.redo(); assert.equal(r.segments.length, 2);
});

test('angle formatting is clean', () => {
  assert.equal(P.fmtAngle(Math.PI / 2), 'Math.toRadians(90)');
  assert.equal(P.fmtAngle(-Math.PI / 2), 'Math.toRadians(-90)');
  assert.equal(P.fmtAngle(0), 'Math.toRadians(0)');
  assert.equal(P.fmtNum(12), '12.0'); assert.equal(P.fmtNum(12.3456), '12.35');
});

test('routeToBridgeJson converts headings to degrees and keeps points', () => {
  const r = sampleRouteObj(), b = P.routeToBridgeJson(r, 2);
  assert.equal(b.step, 2); assert.equal(b.start.h, 90); assert.equal(b.segments.length, 3);
  assert.equal(b.segments[0].heading.end, 180); assert.equal(b.segments[1].type, 'curve'); assert.equal(b.segments[1].points.length, 3);
  assert.equal(b.segments[1].heading.start, undefined);
});
test('moveSegment reorders and re-chains', () => {
  const r = P.newRoute({ start: { x: 0, y: 0, h: 0 } }); P.addLine(r, 10, 0); P.addLine(r, 10, 10);
  assert.equal(P.moveSegment(r, 1, -1), true);
  assert.deepEqual(r.segments[0].points[0], { x: 0, y: 0 }); assert.deepEqual(r.segments[1].points[0], r.segments[0].points.at(-1));
  assert.equal(P.moveSegment(r, 0, -1), false);
});
test('JSON keeps colour / hidden / locked; toJava documents >4-point curves', () => {
  const r = sampleRouteObj(); r.segments[0].color = '#abcdef'; r.segments[0].hidden = true; r.segments[0].locked = true;
  const b = P.fromJSON(P.toJSON(r)); assert.equal(b.segments[0].color, '#abcdef'); assert.equal(b.segments[0].hidden, true); assert.equal(b.segments[0].locked, true);
  const c = P.newRoute(); c.segments.push(P.newSegment('curve', [{ x: 0, y: 0 }, { x: 10, y: 30 }, { x: 20, y: 0 }, { x: 30, y: 30 }, { x: 40, y: 0 }, { x: 50, y: 10 }]));
  const j = P.toJava(c); assert.match(j, /6-point Bezier \(degree 5\)/); assert.match(j, /new BezierCurve\(startPose, control1_1, control1_2, control1_3, control1_4, pose1\)/);
  assert.ok(!/Paths\.|PoseFactory|Interpolator\.piecewise/.test(j));
});

// ---- DOM-free smoke test of the canvas editor ----
import { createPathEditor } from '../web/js/pathui.js';
const offline = async () => { throw new Error('offline'); };
function editor(extra = {}) {
  const ed = createPathEditor({ mx: x => x * 4, my: y => 576 - y * 4, getK: () => 4, toast: () => {}, fetchImpl: offline, debounceMs: 0, ...extra });
  ed.active = true; return ed;
}
const ctxStub = () => new Proxy({ measureText: () => ({ width: 40 }) }, { get: (t, k) => (k in t ? t[k] : () => {}), set: () => true });
test('pathui: click, shift-click, drag, undo, panelHtml', () => {
  const ed = editor();
  assert.equal(ed.pointer('down', 60, 60, { button: 0 }), true); ed.pointer('up', 60, 60, {});
  assert.equal(ed.route.segments.length, 1); assert.equal(ed.route.segments[0].type, 'curve', 'default tool appends a waypoint with Bezier handles'); assert.equal(ed.route.segments[0].points.length, 4);
  ed.pointer('down', 90, 60, { button: 0, shiftKey: true }); ed.pointer('up', 90, 60, {});
  assert.equal(ed.route.segments[1].type, 'line', 'Shift flips to a straight segment');
  ed.pointer('move', 60, 60, {}); ed.pointer('down', 60, 60, { button: 0 }); ed.pointer('move', 70, 70, {}); ed.pointer('up', 70, 70, {});
  assert.deepEqual(ed.route.segments[0].points.at(-1), { x: 70, y: 70 });
  assert.equal(ed.undo(), true); assert.deepEqual(ed.route.segments[0].points.at(-1), { x: 60, y: 60 });
  ed.open.add('settings'); const html = ed.panelHtml(); assert.match(html, /Waypoints/); assert.match(html, /Event Markers/); assert.match(html, /data-tool="curve"/);
  ed.open.add('export'); assert.match(ed.panelHtml(), /Export \.pp/); assert.match(ed.panelHtml(), /data-f="mode"/);
});
test('pathui: keys (Esc, Space, Ctrl+Z, Delete), tools, locked paths, draw', async () => {
  const ed = editor({ getObstacles: () => [{ minX: 55, maxX: 65, minY: 55, maxY: 65, name: 'blk' }], getFootprint: () => ({ w: 18, l: 18 }) });
  ed.pointer('down', 30, 30, { button: 0 }); ed.pointer('down', 100, 30, { button: 0 });
  assert.match(ed.panelHtml(), /passes through obstacle|Waypoints/);
  assert.equal(ed.key({ key: 'Escape' }), true); assert.equal(ed.sel, -1);
  assert.equal(ed.key({ key: ' ' }), true); assert.equal(ed.playing, true); ed.step(0.1); assert.ok(ed.playPose); ed.key({ key: ' ' }); assert.equal(ed.playing, false);
  ed.scrub(0.5); assert.ok(ed.playPose.x > 30 && ed.playPose.x < 100);
  ed.draw(ctxStub());
  ed.setTool('delete'); ed.setTool('line'); ed.route.segments[1].locked = true;
  ed.pointer('down', 100, 30, { button: 0 }); assert.equal(ed.drag, null);
  ed.route.segments[1].locked = false; ed.sel = 1; assert.equal(ed.key({ key: 'Delete' }), true); assert.equal(ed.route.segments.length, 1);
  assert.equal(ed.key({ key: 'z', ctrlKey: true }), true); assert.equal(ed.route.segments.length, 2);
  assert.equal(ed.key({ key: 'z', ctrlKey: true, target: { tagName: 'INPUT' } }), false);
  ed.active = false; assert.equal(ed.key({ key: ' ' }), false);
});
test('pathui: draws and plays from the REAL sampler result, badge reflects source', async () => {
  const bridge = { total: 50, segments: [{ type: 'line', mode: 'linear', length: 50, offset: 0, samples: [{ x: 20, y: 20, h: 0, t: 0, s: 0 }, { x: 45, y: 20, h: 0, t: .5, s: 25 }, { x: 70, y: 20, h: 0, t: 1, s: 50 }] }] };
  const ed = editor({ fetchImpl: async () => ({ ok: true, json: async () => bridge }) });
  ed.route.start.x = 20; ed.route.start.y = 20; ed.pointer('down', 70, 20, { button: 0 });
  assert.equal(ed.sourceInfo().source, null);
  await ed.sampler.request(ed.route);
  assert.equal(ed.sourceInfo().label, 'Pedro 2.1.0 core'); assert.equal(ed.samples().length, 3);
  ed.open.add('export'); assert.match(ed.panelHtml(), /Pedro 2\.1\.0 core/);
  const e2 = editor(); e2.pointer('down', 50, 50, { button: 0 }); await e2.sampler.request(e2.route);
  assert.equal(e2.sourceInfo().label, 'JS fallback'); e2.open.add('export'); assert.match(e2.panelHtml(), /JS fallback/);
});
test('pathui: importPP / layer ops via route model', () => {
  const ed = editor(); ed.route.start = { x: 24, y: 24, h: Math.PI / 2 }; ed.pointer('down', 30, 30, { button: 0 }); ed.pointer('down', 60, 30, { button: 0 });   // start pinned: the default blue start would put (60,30) on the path
  const pp = JSON.stringify({ startPoint: { x: 5, y: 5, headingDeg: 90 }, lines: [{ endPoint: { x: 40, y: 5 }, controlPoints: [] }] });
  const w = ed.importPP(pp); assert.ok(Array.isArray(w)); assert.equal(ed.route.segments.length, 1); assert.equal(ed.route.start.x, 5);
  assert.equal(ed.undo(), true); assert.equal(ed.route.segments.length, 2);
});

test('event markers: add/move/remove, order by travel, Java calls the subsystem', () => {
  const r = sampleRouteObj(); r.segments.forEach(s => { s.callbacks = []; });
  assert.equal(P.addEvent(r, 1, 0.5, 'intakeOn'), 0); P.addEvent(r, 0, 0.25, 'setRpm'); P.addEvent(r, 2, 1.5, 'x');
  const ev = P.routeEvents(r); assert.deepEqual(ev.map(e => e.name), ['setRpm', 'intakeOn']);
  assert.deepEqual(ev[0].args, { rpm: 3000 }); assert.ok(ev[0].frac < ev[1].frac && ev[0].preset);
  assert.ok(P.moveEvent(r, 1, 0, 2) && r.segments[1].callbacks[0].t === 1);
  const j = P.toJava(r, {}); assert.match(j, /shooter\.setRpm\(3000\.0\); \/\* setRpm \*\//); assert.match(j, /intake\.on\(\);/);
  assert.ok(P.removeEvent(r, 1, 0) && !P.removeEvent(r, 1, 0));
  const back = P.fromJSON(P.toJSON(r)); assert.deepEqual(back.segments[0].callbacks[0].args, { rpm: 3000 });
});
test('snapPoint: magnet beats grid, grid rounds, clamps; snapAngle', () => {
  const r = sampleRouteObj();
  assert.deepEqual(P.snapPoint(r, 24.8, 72.9, { grid: 6 }), { x: 24, y: 72, snapped: 'point' });
  assert.deepEqual(P.snapPoint(r, 50.4, 10.2, { grid: 6, magnet: false }), { x: 48, y: 12, snapped: 'grid' });
  assert.deepEqual(P.snapPoint(r, 200, -5, { grid: 0, magnet: false }), { x: 144, y: 0, snapped: null });
  near(P.snapAngle(0.3, 15), 15 * Math.PI / 180, 1e-9);
});
test('setEndHeading edits waypoint heading; tangent refuses', () => {
  const r = sampleRouteObj();
  assert.ok(P.setEndHeading(r, 0, 1)); near(P.resolveHeadings(r)[0].h1, 1);
  assert.ok(P.setEndHeading(r, 2, 0.7)); near(P.resolveHeadings(r)[2].h0, 0.7);
  assert.equal(P.setEndHeading(r, 1, 1), false);
});
test('trajectory: local fallback and pose lookup; planTrajectory falls back without pedrofollow', async () => {
  const r = sampleRouteObj(); const tr = P.localTrajectory(r, { dt: 0.05 });
  assert.ok(tr.length > 10 && tr[0].t === 0); const e = tr.at(-1); near(e.x, 100, 0.5);
  const mid = P.trajectoryPoseAt(tr, e.t / 2); assert.ok(mid.x > 24 && mid.x < 100);
  const { traj, source } = await P.planTrajectory(r, { dt: 0.05 }); assert.ok(traj.length > 10); assert.ok(['local', 'pedro-follower'].includes(source));
});
test('nearestT finds the closest parameter', () => { const r = sampleRouteObj(); near(P.nearestT(r, 0, 24, 48), 0.5, 0.02); });
