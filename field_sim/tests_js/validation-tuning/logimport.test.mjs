import test from 'node:test';
import assert from 'node:assert/strict';
import { parseCsv, importCsvLog, dashToSim, importDashboardCsv, importWpilog, detectFormat, importAny, toPoseTrack, toFeedforwardSamples }
  from '../../web/js/physics/validation-tuning/logimport.js';
import { writeWpilog } from '../../web/js/physics/validation-tuning/wpilog.js';

const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

test('parseCsv auto-detects comma, semicolon and tab', () => {
  for (const d of [',', ';', '\t']) {
    const r = parseCsv(['a', 'b', 'c'].join(d) + '\n' + ['1', '2', '3'].join(d) + '\n' + ['4', '5', '6'].join(d));
    assert.deepEqual(r.columns, ['a', 'b', 'c']);
    assert.deepEqual(r.rows, [[1, 2, 3], [4, 5, 6]]);
  }
});
test('header aliases map to canonical names, case-insensitively', () => {
  const log = importCsvLog('Timestamp,Pose_X,POSE_Y,Theta,U,Vel\n0,1,2,0.1,0.5,3\n1,2,3,0.2,0.6,4\n', { timeUnit: 's' });
  for (const k of ['x', 'y', 'heading', 'power', 'velocity']) assert.ok(log.series[k], `series ${k}`);
  assert.deepEqual(log.series.x.v, [1, 2]);
  assert.deepEqual(log.series.x.t, [0, 1]);
  assert.deepEqual(log.series.power.v, [0.5, 0.6]);
  assert.equal(log.series.t, undefined);
});
test('fwd, strafe, turn keep their names', () => {
  const log = importCsvLog('t,fwd,strafe,turn\n0,1,0,0\n1,1,0,0.5\n', { timeUnit: 's' });
  assert.deepEqual(Object.keys(log.series).sort(), ['fwd', 'strafe', 'turn']);
});
test('time unit auto-detect: ms (median dt > 1)', () => {
  const log = importCsvLog('time,x\n0,0\n20,1\n40,2\n60,3\n');
  near(log.series.x.t[1], 0.02); near(log.series.x.t[3], 0.06);
});
test('time unit auto-detect: us (median dt > 1000)', () => {
  const log = importCsvLog('time,x\n0,0\n20000,1\n40000,2\n60000,3\n');
  near(log.series.x.t[1], 0.02); near(log.series.x.t[3], 0.06);
});
test('time unit auto-detect: seconds', () => {
  const log = importCsvLog('time,x\n0,0\n0.02,1\n0.04,2\n');
  near(log.series.x.t[2], 0.04);
});
test('explicit time unit overrides auto-detect', () => {
  const log = importCsvLog('time,x\n0,0\n1000,1\n2000,2\n', { timeUnit: 'ms' });
  near(log.series.x.t[2], 2);
});
test('dashToSim applies the 72 in offset and zero rotation', () => {
  const p = dashToSim([10, 20, 0.3]);
  near(p[0], 82); near(p[1], 92); near(p[2], 0.3);
  const q = dashToSim([0, 0, 0]);
  near(q[0], 72); near(q[1], 72);
});
test('dashToSim honours a custom rotation', () => {
  const p = dashToSim([10, 0, 0], { offsetIn: [72, 72], rotRad: Math.PI / 2 });
  near(p[0], 72, 1e-9); near(p[1], 82, 1e-9); near(p[2], Math.PI / 2, 1e-9);
});
test('importDashboardCsv converts every pose', () => {
  const log = importDashboardCsv('time,x,y,heading\n0,-72,-72,0\n1,0,0,0.5\n', { timeUnit: 's' });
  near(log.series.x.v[0], 0); near(log.series.y.v[0], 0);
  near(log.series.x.v[1], 72); near(log.series.heading.v[1], 0.5);
});
test('toPoseTrack and toFeedforwardSamples', () => {
  const log = importCsvLog('t,x,y,heading,power,velocity\n0,0,0,0,0.2,1\n1,1,2,0.1,0.4,2\n', { timeUnit: 's' });
  const tr = toPoseTrack(log);
  assert.deepEqual(tr.t, [0, 1]); assert.deepEqual(tr.x, [0, 1]); assert.deepEqual(tr.y, [0, 2]); near(tr.h[1], 0.1);
  const ff = toFeedforwardSamples(log);
  assert.deepEqual(ff.u, [0.2, 0.4]); assert.deepEqual(ff.v, [1, 2]);
});
test('detectFormat', () => {
  const wl = writeWpilog([], []);
  assert.equal(detectFormat(wl, 'x.wpilog'), 'wpilog');
  assert.equal(detectFormat(wl), 'wpilog');
  assert.equal(detectFormat('t,x\n0,1\n', 'a.csv'), 'csv');
  assert.equal(detectFormat('{"a":1}', 'a.json'), 'json');
});
test('importWpilog and importAny produce the Log shape', () => {
  const buf = writeWpilog([{ name: '/vel', type: 'double' }], [{ name: '/vel', tUs: 0, value: 1 }, { name: '/vel', tUs: 1000000, value: 3 }]);
  const log = importWpilog(buf);
  const key = Object.keys(log.series).find((k) => k.includes('vel'));
  assert.ok(key, JSON.stringify(Object.keys(log.series)));
  assert.deepEqual(log.series[key].v, [1, 3]);
  assert.deepEqual(log.series[key].t, [0, 1]);
  assert.equal(typeof log.source, 'string');
  const any = importAny(buf, 'run.wpilog');
  assert.deepEqual(Object.keys(any.series), Object.keys(log.series));
  const csv = importAny('t,x\n0,1\n1,2\n', 'run.csv');
  assert.ok(csv.series.x);
});
