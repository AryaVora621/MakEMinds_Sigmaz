// t16: heading interpolation export fidelity. Headings are radians; export prints Math.toRadians(deg).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, fmtAngleRad } from '../web/js/autoexport.js';
import { mirrorAuto } from '../web/js/autos.js';

const auto = (alliance = 'blue') => ({
  name: 'T16 Heading', alliance, start: { x: 9, y: 84, h: Math.PI / 2 },
  paths: { p1: { markers: [], segments: [
    { type: 'line', points: [{ x: 9, y: 84 }, { x: 24, y: 84 }], heading: { mode: 'constant', start: Math.PI / 2 } },
    { type: 'curve', points: [{ x: 24, y: 84 }, { x: 36, y: 60 }, { x: 48, y: 36 }], heading: { mode: 'linear', start: 0, end: Math.PI / 2 } },
    { type: 'line', points: [{ x: 48, y: 36 }, { x: 60, y: 36 }], heading: { mode: 'tangent', reversed: true } },
  ] } },
  root: { id: 'n1', type: 'sequential', children: [{ id: 'n2', type: 'follow', path: 'p1' }] },
});

test('fmtAngleRad converts radians to Math.toRadians(degrees)', () => {
  assert.equal(fmtAngleRad(Math.PI / 2), 'Math.toRadians(90)');
  assert.equal(fmtAngleRad(0), 'Math.toRadians(0)');
  assert.equal(fmtAngleRad(-Math.PI / 2), 'Math.toRadians(-90)');
  assert.equal(fmtAngleRad(Math.PI), 'Math.toRadians(180)');
});

test('poses keep inch coordinates and radian headings; geometry and no NaN', () => {
  const java = exportAutoJava(auto());
  assert.match(java, /new Pose\(9\.0, 84\.0, Math\.toRadians\(90\)\)/);
  assert.match(java, /new Pose\(24\.0, 84\.0\)/);
  assert.match(java, /new BezierLine\(pose_p1_0_0, pose_p1_0_1\)/);
  assert.match(java, /new BezierCurve\(pose_p1_1_0, pose_p1_1_1, pose_p1_1_2\)/);
  assert.doesNotMatch(java, /NaN|undefined|Infinity/);
});

test('heading modes emit the matching Pedro interpolation call', () => {
  const java = exportAutoJava(auto());
  assert.match(java, /\.setConstantHeadingInterpolation\(Math\.toRadians\(90\)\)/);
  assert.match(java, /\.setLinearHeadingInterpolation\(Math\.toRadians\(0\), Math\.toRadians\(90\)\)/);
  const tangent = java.indexOf('.setTangentHeadingInterpolation()');
  assert.ok(tangent > 0, 'tangent call present');
  assert.ok(java.indexOf('.setReversed()', tangent) > tangent, 'setReversed follows tangent');
});

test('linear heading with end omitted falls back to start', () => {
  const a = auto();
  a.paths.p1.segments[1].heading = { mode: 'linear', start: 1 };
  assert.match(exportAutoJava(a), /\.setLinearHeadingInterpolation\(Math\.toRadians\(57\.3\), Math\.toRadians\(57\.3\)\)/);
});

test('alliance statement: blue -> BLUE; mirrored red auto emits BLUE without crashing', () => {
  assert.match(exportAutoJava(auto('blue')), /Robot\.ALLIANCE = Alliance\.BLUE;/);
  assert.match(exportAutoJava(auto('red')), /Robot\.ALLIANCE = Alliance\.RED;/);
  const m = mirrorAuto(auto('red'));
  assert.equal(m.alliance, 'blue');
  assert.equal(m.start.x, 144 - 9);
  const java = exportAutoJava(m);
  assert.match(java, /Robot\.ALLIANCE = Alliance\.BLUE;/);
  assert.match(java, /new Pose\(135\.0, 84\.0, /);
});
