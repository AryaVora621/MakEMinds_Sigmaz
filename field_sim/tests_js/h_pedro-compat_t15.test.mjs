import { test } from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava } from '../web/js/autoexport.js';

// Pedro 2.1.0 chain callbacks never fire (docs/OVERNIGHT_RULES.md), so markers must be polled in the follow state.
const P = (x, y) => ({ x, y });
const twoSeg = (extraMarkers = [], callbacks0 = []) => ({
  name: 'Compat T15', alliance: 'blue', start: { x: 24, y: 120, h: 0 },
  paths: { p1: {
    segments: [
      { type: 'line', points: [P(24, 120), P(48, 96)], heading: { mode: 'tangent' }, callbacks: callbacks0 },
      { type: 'line', points: [P(48, 96), P(72, 96)], heading: { mode: 'tangent' } },
    ],
    markers: [
      { t: 0.0, command: { type: 'command', name: 'intakeOn' } },
      { t: 0.5, command: { type: 'command', name: 'aimOn' } },
      { t: 1.0, command: { type: 'command', name: 'intakeOff' } },
      ...extraMarkers,
    ],
  } },
  root: { type: 'sequential', children: [{ type: 'follow', path: 'p1' }] },
});
const gen = (auto) => exportAutoJava(auto, { className: 'CompatT15Auto' });

test('no chain callback API is emitted for markers or event callbacks', () => {
  const j = gen(twoSeg([], [{ t: 0.25, name: 'intakeOff' }]));
  assert.doesNotMatch(j, /addParametricCallback|addCallback|setCallback/);
});

test('marker statements are guarded by a fire-once flag and set it', () => {
  const j = gen(twoSeg());
  const flags = [...new Set([...j.matchAll(/if \(!(n\d+_m\d+) &&/g)].map(m => m[1]))];
  assert.ok(flags.length >= 3, 'expected a flag per marker, got ' + flags.length);
  for (const f of flags) {
    assert.match(j, new RegExp(`${f} = true;`), f + ' must be set when it fires');
  }
});

test('markers are polled via getChainIndex and getCurrentTValue', () => {
  const j = gen(twoSeg());
  assert.match(j, /robot\.auto\.getFollower\(\)\.getChainIndex\(\)/);
  assert.match(j, /robot\.auto\.getFollower\(\)\.getCurrentTValue\(\)/);
});

test('marker at t=1.0 is still emitted and fires (end-of-path flush)', () => {
  const j = gen(twoSeg());
  assert.match(j, /robot\.intake\.stop\(\);/, 'intakeOff marker statement missing');
  // the t=1.0 marker is also flushed after the follow ends, so it cannot be skipped when t never reaches 1.0 exactly
  const flushes = j.match(/if \(!n\d+_m\d+\) \{/g) || [];
  assert.ok(flushes.length >= 3, 'expected end-of-path flush guards for every marker');
});

test('a timed command (shoot) as a marker yields a TODO comment, not blocking code', () => {
  const j = gen(twoSeg([{ t: 0.5, command: { type: 'command', name: 'shoot' } }]));
  assert.match(j, /\/\/ TODO: marker command "shoot" is timed/);
  // the timed shoot is not emitted as an inline call inside the path poll; only the TODO line is present
  assert.doesNotMatch(j, /robot\.feedIfReady\(true\)/);
});
