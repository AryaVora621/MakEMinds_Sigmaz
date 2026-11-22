import test from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, exportPathsOnly, listCommandStubs, checkJavaStructure, sanitizeClassName, fmtAngleRad, IVY_GRADLE_LINE } from '../web/js/autoexport.js';

const P = (x, y) => ({ x, y });
const cmd = (name, args) => ({ type: 'command', name, args });
const base = (root, extra = {}) => ({
  name: 'Test Auto', alliance: 'blue', start: { x: 24, y: 120, h: Math.PI / 2 },
  paths: {
    a: { segments: [{ type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'linear', start: Math.PI / 2, end: Math.PI } }], markers: [] },
    b: { segments: [{ type: 'curve', points: [P(48, 96), P(60, 60), P(40, 40)], heading: { mode: 'tangent', reversed: true } }, { type: 'line', points: [P(40, 40), P(40, 30)], heading: { mode: 'constant', start: 0 } }], markers: [] },
  },
  root, ...extra,
});
const gen = (root, opts, extra) => exportAutoJava(base(root, extra), { className: 'TestAuto', ...opts });
const ok = j => { const r = checkJavaStructure(j); assert.deepEqual(r.problems, []); assert.equal(r.ok, true); };

test('simple sequential auto', () => {
  const j = gen({ type: 'sequential', children: [{ type: 'follow', path: 'a' }, cmd('intakeOn'), { type: 'wait', seconds: 0.5 }, { type: 'follow', path: 'b' }] });
  ok(j);
  assert.match(j, /public class TestAuto extends LinearOpMode/);
  assert.match(j, /robot\.auto\.followPath\(path_a, true, FOLLOW_TIMEOUT_S\)/);
  assert.match(j, /isBusyOrTimeout\(\)/);
  assert.match(j, /\} finally \{\n\s+robot\.stop\(\);/);
  assert.match(j, /robot\.intake\.forward\(\)/);
  assert.match(j, /new BezierLine\(pose_a_0_0, pose_a_0_1\)/);
  assert.match(j, /new BezierCurve\(pose_b_0_0, pose_b_0_1, pose_b_0_2\)/);
});

test('parallel group uses a status array and waits for all', () => {
  const j = gen({ type: 'parallel', children: [{ type: 'follow', path: 'a' }, cmd('intakeOn')] });
  ok(j);
  assert.match(j, /private final boolean\[\] n0_flags = new boolean\[2\]/);
  assert.match(j, /boolean all = true;/);
});

test('race cancels losers', () => {
  const j = gen({ type: 'race', children: [{ type: 'follow', path: 'a' }, { type: 'wait', seconds: 2 }] });
  ok(j);
  assert.match(j, /int winner = -1;/);
  assert.match(j, /if \(winner != 0\)/);
  assert.match(j, /robot\.auto\.cancelMove\(\)/);
});

test('deadline: designated child decides, others cancelled', () => {
  const j = gen({ type: 'deadline', deadline: 1, children: [cmd('intakeOn'), { type: 'wait', seconds: 1 }] });
  ok(j);
  assert.match(j, /deadline \(branch 1 decides/);
  assert.match(j, /if \(run2\(\)\)/); // child index 1 is node 2
});

test('deadline defaults to child 0 and tolerates bad index', () => {
  const j = gen({ type: 'deadline', deadline: 9, children: [{ type: 'wait', seconds: 1 }] });
  ok(j);
  assert.match(j, /branch 0 decides/);
});

test('waitUntil maps known conditions to real robot queries', () => {
  const j = gen({ type: 'sequential', children: [
    { type: 'waitUntil', condition: 'flywheelReady', timeout: 2 },
    { type: 'waitUntil', condition: 'ballCount>=3' },
    { type: 'waitUntil', condition: 'gateOpen' },
  ] });
  ok(j);
  assert.match(j, /robot\.shooter != null && robot\.shooter\.ready\(\)/);
  assert.match(j, /robot\.tracker\.getCount\(\) >= 3/);
  assert.match(j, /robot\.gate\.isOpen\(\)/);
  assert.match(j, />= 2\.0;   \/\/ timeout/);
  assert.deepEqual(listCommandStubs(base({ type: 'waitUntil', condition: 'flywheelReady' })), []);
});

test('unknown waitUntil condition becomes a TODO stub', () => {
  const root = { type: 'waitUntil', condition: 'sensorSees', timeout: 1 };
  const j = gen(root);
  ok(j);
  assert.match(j, /private boolean condition_sensorSees\(\)/);
  assert.match(j, /TODO: implement the wait condition/);
  assert.deepEqual(listCommandStubs(base(root)), ['condition:sensorSees']);
});

test('markers are polled in the follow state, never addParametricCallback (broken in Pedro 2.1.0)', () => {
  const a = base({ type: 'follow', path: 'a' });
  a.paths.a.markers = [{ t: 0.4, command: cmd('intakeOn') }, { t: 0.9, command: { type: 'sequential', children: [cmd('gateOpen'), cmd('flowerDeploy')] } }];
  const j = exportAutoJava(a, { className: 'M' });
  ok(j);
  assert.doesNotMatch(j, /addParametricCallback|addTemporalCallback|addPoseCallback/);
  assert.match(j, /getChainIndex\(\)/);
  assert.match(j, /tNow >= 0\.4/);
  assert.match(j, /tNow >= 0\.9/);
  assert.match(j, /robot\.gate\.open\(\)/);
  assert.match(j, /robot\.flower\.deploy\(\)/);
});

test('marker on a multi-segment path lands on the right segment with local t', () => {
  const a = base({ type: 'follow', path: 'b' });
  a.paths.b.markers = [{ t: 1, command: cmd('intakeOff') }, { t: 0, command: cmd('intakeOn') }];
  const j = exportAutoJava(a, { className: 'M' });
  ok(j);
  assert.match(j, /chainIdx == 0 && tNow >= 0\.0\)/);
  assert.match(j, /chainIdx == 1 && tNow >= 1\.0\)/);
  assert.doesNotMatch(j, /addParametricCallback/);
});

test('timed command inside a marker is flagged, not silently dropped', () => {
  const a = base({ type: 'follow', path: 'a' });
  a.paths.a.markers = [{ t: 0.5, command: cmd('shoot') }];
  const j = exportAutoJava(a, { className: 'M' });
  assert.match(j, /TODO: marker command "shoot" is timed/);
  assert.deepEqual(listCommandStubs(a), ['marker:shoot']);
});

test('mirror-free: coordinates are emitted exactly as given for both alliances', () => {
  const r = exportAutoJava(base({ type: 'follow', path: 'a' }, { alliance: 'red' }), { className: 'R' });
  const bl = exportAutoJava(base({ type: 'follow', path: 'a' }), { className: 'R' });
  assert.match(r, /new Pose\(24\.0, 120\.0, Math\.toRadians\(90\)\)/);
  assert.match(r, /Robot\.ALLIANCE = Alliance\.RED/);
  assert.match(bl, /Robot\.ALLIANCE = Alliance\.BLUE/);
  assert.equal(r.replace(/RED/g, 'BLUE').replace(/red/g, 'blue'), bl);
});

test('custom commands and unknown names produce stubs', () => {
  const root = { type: 'sequential', children: [cmd('custom:spinTwice'), cmd('park'), cmd('setRpm', { rpm: 3200 }), cmd('mystery')] };
  const j = gen(root);
  ok(j);
  assert.match(j, /private void custom_spinTwice\(\)/);
  assert.match(j, /private void cmdPark\(\)/);
  assert.match(j, /private void cmdSetRpm\(double rpm\)/);
  assert.match(j, /cmdSetRpm\(3200\.0\);/);
  assert.match(j, /private void custom_mystery\(\)/);
  assert.deepEqual(listCommandStubs(base(root)), ['custom:mystery', 'custom:spinTwice', 'park', 'setRpm']);
});

test('real robot commands emit existing APIs and no stubs', () => {
  const root = { type: 'sequential', children: ['intakeOn', 'intakeOff', 'aimOn', 'aimOff', 'flowerDeploy', 'flowerStow', 'gateOpen', 'gateClose', 'shoot', 'shootAll'].map(n => cmd(n)) };
  const j = gen(root);
  ok(j);
  for (const s of ['robot.intake.forward()', 'robot.intake.stop()', 'setSpinning(true)', 'setSpinning(false)', 'robot.flower.stow()', 'robot.gate.close()', 'robot.feedIfReady(true)']) assert.ok(j.includes(s), s);
  assert.deepEqual(listCommandStubs(base(root)), []);
});

test('shoot duration comes from args.seconds, defaults differ for shootAll', () => {
  const j = gen({ type: 'sequential', children: [cmd('shoot', { seconds: 1.5 }), cmd('shootAll')] });
  assert.match(j, />= 1\.5\)/);
  assert.match(j, />= 3\.0\)/);
});

test('class names are sanitised', () => {
  assert.equal(sanitizeClassName('My Auto 9!'), 'My_Auto_9');
  assert.equal(sanitizeClassName('9lives'), 'Auto_9lives');
  assert.equal(sanitizeClassName('class'), 'classAuto');
  assert.equal(sanitizeClassName(''), 'GeneratedAuto');
  assert.equal(sanitizeClassName('Robot'), 'RobotAuto');
  assert.equal(sanitizeClassName('../../evil'), 'evil');
  for (const bad of ['a b', '1x', 'public', '', 'x-y', '}{']) {
    const j = exportAutoJava(base({ type: 'wait', seconds: 1 }), { className: bad });
    ok(j);
  }
});

test('class name falls back to the auto name', () => {
  const j = exportAutoJava(base({ type: 'wait', seconds: 1 }));
  assert.match(j, /public class Test_Auto extends/);
});

test('structure checker catches broken output', () => {
  const j = gen({ type: 'follow', path: 'a' });
  assert.equal(checkJavaStructure(j.replace(/\}\s*$/, '')).ok, false);
  assert.equal(checkJavaStructure(j.replace('followPath(path_a, true, FOLLOW_TIMEOUT_S);', 'followPath(path_a, true, FOLLOW_TIMEOUT_S;')).ok, false);
  assert.equal(checkJavaStructure(j.replace('"Test Auto"', '"Test Auto')).ok, false);
  assert.equal(checkJavaStructure(j.replace('private PathChain path_a, path_b;', '')).ok, false);
  assert.equal(checkJavaStructure(j.replace('private boolean run0()', 'private boolean runX()')).ok, false);
  assert.equal(checkJavaStructure(j.replace('class TestAuto', 'class void')).ok, false);
  assert.equal(checkJavaStructure('').ok, false);
  assert.ok(checkJavaStructure(j.replace('Pose(24.0', 'Pose(NaN')).problems.some(p => /NaN/.test(p)));
});

test('structure checker flags a case that falls through', () => {
  const j = gen({ type: 'sequential', children: [cmd('intakeOn'), cmd('intakeOff')] });
  const broken = j.replace(/(case 0:[\s\S]*?)break;/, '$1');
  const r = checkJavaStructure(broken);
  assert.equal(r.ok, false);
  assert.ok(r.problems.some(p => /falls through/.test(p)));
});

test('structure checker ignores braces inside strings and comments', () => {
  const r = checkJavaStructure('public class A { String s = "}"; // {\n /* ( */ }');
  assert.deepEqual(r.problems, []);
});

test('no NaN or Infinity even with bad numbers', () => {
  const a = base({ type: 'sequential', children: [{ type: 'wait', seconds: NaN }, { type: 'waitUntil', condition: 'gateOpen', timeout: Infinity }, cmd('setRpm', { rpm: NaN })] });
  a.start = { x: NaN, y: Infinity, h: NaN };
  a.paths.a.segments[0].points[0] = P(NaN, undefined);
  const j = exportAutoJava(a, { className: 'N' });
  assert.doesNotMatch(j, /NaN|Infinity|undefined/);
  ok(j);
});

test('headings formatted cleanly', () => {
  assert.equal(fmtAngleRad(Math.PI / 2), 'Math.toRadians(90)');
  assert.equal(fmtAngleRad(-Math.PI / 2), 'Math.toRadians(-90)');
  assert.equal(fmtAngleRad(0), 'Math.toRadians(0)');
  assert.equal(fmtAngleRad(-0), 'Math.toRadians(0)');
  assert.equal(fmtAngleRad(Math.PI), 'Math.toRadians(180)');
  assert.equal(fmtAngleRad(Math.PI / 6), 'Math.toRadians(30)');
  assert.equal(fmtAngleRad(Math.PI * 7 / 36 + 1e-12), 'Math.toRadians(35)');
  assert.equal(fmtAngleRad(0.1), 'Math.toRadians(5.73)');
  const j = gen({ type: 'follow', path: 'a' });
  assert.match(j, /setLinearHeadingInterpolation\(Math\.toRadians\(90\), Math\.toRadians\(180\)\)/);
  assert.match(j, /setConstantHeadingInterpolation\(Math\.toRadians\(0\)\)/);
  assert.match(j, /\.setTangentHeadingInterpolation\(\)\n\s+\.setReversed\(\)/);
});

test('pose numbers formatted cleanly', () => {
  const a = base({ type: 'follow', path: 'a' });
  a.paths.a.segments[0].points = [P(24.000000001, 119.99999999), P(48.126, -0.0001)];
  const j = exportAutoJava(a, { className: 'F' });
  assert.match(j, /new Pose\(24\.0, 120\.0\)/);
  assert.match(j, /new Pose\(48\.13, 0\.0\)/);
  assert.doesNotMatch(j, /\d\.\d{3,}/.source ? /Pose\([^)]*\d\.\d{3,}/ : /x/);
});

test('deterministic output', () => {
  const root = { type: 'sequential', children: [{ type: 'follow', path: 'b' }, { type: 'parallel', children: [cmd('shoot'), { type: 'follow', path: 'a' }] }, cmd('custom:z'), cmd('custom:y')] };
  const a1 = exportAutoJava(base(root), { className: 'D' }), a2 = exportAutoJava(JSON.parse(JSON.stringify(base(root))), { className: 'D' });
  assert.equal(a1, a2);
  const reordered = base(root); reordered.paths = { b: reordered.paths.b, a: reordered.paths.a };
  assert.equal(exportAutoJava(reordered, { className: 'D' }), a1);
});

test('undeclared path reference is caught by the checker', () => {
  const j = gen({ type: 'follow', path: 'missing' });
  const r = checkJavaStructure(j);
  assert.equal(r.ok, false);
  assert.ok(r.problems.some(p => /path_missing/.test(p)));
});

test('nested groups compile to consistent states', () => {
  const root = { type: 'sequential', children: [
    { type: 'parallel', children: [
      { type: 'sequential', children: [{ type: 'follow', path: 'a' }, cmd('shoot')] },
      { type: 'race', children: [{ type: 'waitUntil', condition: 'ballsFull', timeout: 3 }, { type: 'wait', seconds: 2 }] },
    ] },
    { type: 'deadline', children: [{ type: 'follow', path: 'b' }, cmd('intakeOn')] },
  ] };
  const j = gen(root);
  ok(j);
  const runs = [...j.matchAll(/private boolean run(\d+)\(\)/g)].map(m => Number(m[1]));
  assert.deepEqual(runs, [...runs].sort((x, y) => x - y));
  assert.equal(new Set(runs).size, runs.length);
});

test('empty and missing root', () => {
  ok(exportAutoJava({ name: 'E' }, { className: 'E' }));
  ok(exportAutoJava({ name: 'E', root: { type: 'sequential', children: [] } }, { className: 'E' }));
  ok(exportAutoJava({ name: 'E', root: { type: 'race', children: [] } }, { className: 'E' }));
});

test('ivy style prints the gradle line and uses documented names', () => {
  const j = gen({ type: 'sequential', children: [{ type: 'follow', path: 'a' }, { type: 'parallel', children: [cmd('intakeOn'), { type: 'wait', seconds: 1 }] }] }, { style: 'ivy' });
  assert.ok(j.includes(IVY_GRADLE_LINE));
  assert.match(j, /UNVERIFIED/);
  assert.match(j, /sequential\(/);
  assert.equal(checkJavaStructure(j).ok, true);
});

test('default style is state machine with no Ivy import', () => {
  const j = gen({ type: 'follow', path: 'a' });
  assert.doesNotMatch(j, /ivy/i);
});

test('exportPathsOnly has poses and builders but no robot/markers', () => {
  const a = base({ type: 'follow', path: 'a' });
  a.paths.a.markers = [{ t: 0.5, command: cmd('intakeOn') }];
  const j = exportPathsOnly(a);
  assert.match(j, /follower\.pathBuilder\(\)/);
  assert.match(j, /private PathChain path_a, path_b;/);
  assert.doesNotMatch(j, /robot\./);
  assert.doesNotMatch(j, /addParametricCallback/);
  assert.equal(checkJavaStructure('class X {' + j + '}').ok, true);
});

test('package name and group options', () => {
  const j = exportAutoJava(base({ type: 'wait', seconds: 1 }), { className: 'P', packageName: 'foo.bar', group: 'Close' });
  assert.match(j, /^package foo\.bar;/);
  assert.match(j, /group = "Close"/);
  assert.doesNotMatch(exportAutoJava(base({ type: 'wait', seconds: 1 }), { className: 'P', packageName: '' }), /^package/);
});

test('hostile strings cannot break out of comments or string literals', () => {
  const a = base({ type: 'sequential', children: [cmd('custom:x*/}System.exit(0);//'), { type: 'waitUntil', condition: '"; evil(); //' }] });
  a.name = 'Evil" */ int x; /*';
  const j = exportAutoJava(a, { className: 'Ev' });
  assert.doesNotMatch(j, /evil\(\);/);
  assert.doesNotMatch(j, /System\.exit\(0\);/);
  ok(j);
});

test('path-planner event markers (segment callbacks) are exported as polled markers', () => {
  const a = base({ type: 'follow', path: 'a' });
  a.paths.a.segments[0].callbacks = [{ t: 0.5, name: 'gateOpen' }, { t: 0.7, name: 'someNote' }];
  const j = exportAutoJava(a, { className: 'EvtAuto' });
  ok(j);
  assert.doesNotMatch(j, /addParametricCallback/);
  assert.match(j, /tNow >= 0\.5/);
  assert.match(j, /robot\.gate\.open\(\)/);
  assert.doesNotMatch(j, /someNote/);
  assert.equal(a.paths.a.markers.length, 0, 'input is not mutated');
});
