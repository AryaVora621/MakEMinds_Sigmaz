import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const mk = () => createPathEditor({ mx: x => x * 5, my: y => y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0 });
const click = (ed, x, y, e = {}) => { ed.pointer('down', x, y, { button: 0, ...e }); ed.pointer('up', x, y, e); };
const withAlliance = (a, fn) => { const prev = P.PLAN_VIEW.alliance; P.PLAN_VIEW.alliance = a; try { return fn(); } finally { P.PLAN_VIEW.alliance = prev; } };

test('a new route on a blue PLAN_VIEW starts in the blue half at defaultStart(blue)', () => {
  const ed = withAlliance('blue', mk);
  assert.equal(ed.route.alliance, 'blue');
  assert.ok(ed.route.start.x > P.FIELD / 2, `start x ${ed.route.start.x}`);
  assert.deepEqual({ x: ed.route.start.x, y: ed.route.start.y }, { x: P.defaultStart('blue').x, y: P.defaultStart('blue').y });
  assert.equal(P.startInView(ed.route), true);
});

test('a new route on a red PLAN_VIEW starts in the red half', () => {
  const ed = withAlliance('red', mk);
  assert.equal(ed.route.alliance, 'red');
  assert.ok(ed.route.start.x < P.FIELD / 2);
  assert.equal(P.startInView(ed.route), true);
});

test('setAlliance(red) moves the start into the red half and undo restores blue', () => {
  const ed = withAlliance('blue', mk);
  const blueStart = JSON.parse(JSON.stringify(ed.route.start));
  assert.ok(ed.route.start.x > 72);
  assert.equal(ed.setAlliance('red'), true);
  assert.equal(ed.route.alliance, 'red');
  assert.equal(P.PLAN_VIEW.alliance, 'red');
  assert.ok(ed.route.start.x < 72, `start x ${ed.route.start.x}`);
  assert.equal(P.startInView(ed.route), true);
  assert.equal(ed.undo(), true);
  assert.equal(ed.route.alliance, 'blue');
  assert.equal(P.PLAN_VIEW.alliance, 'blue');
  assert.deepEqual(ed.route.start, blueStart);
  assert.equal(ed.redo(), true);
  assert.equal(ed.route.alliance, 'red');
});

test('setAlliance is one undo entry and a no-op for the same alliance', () => {
  const ed = withAlliance('blue', mk);
  click(ed, 100, 24);
  const n = ed.route.segments.length;
  assert.equal(ed.setAlliance('blue'), false);
  ed.setAlliance('red');
  ed.undo();
  assert.equal(ed.route.alliance, 'blue');
  assert.equal(ed.route.segments.length, n, 'undo of setAlliance keeps the path');
});

test('mirror then mirror returns the original route', () => {
  const ed = withAlliance('blue', mk);
  click(ed, 100, 24); click(ed, 110, 60);
  const original = JSON.stringify(ed.route);
  ed.mirror();
  assert.equal(ed.route.alliance, 'red');
  assert.equal(P.PLAN_VIEW.alliance, 'red');
  assert.equal(P.startInView(ed.route), true);
  assert.ok(ed.route.start.x < 72);
  ed.mirror();
  assert.equal(ed.route.alliance, 'blue');
  assert.equal(P.startInView(ed.route), true);
  assert.equal(JSON.stringify(ed.route), original);
});

test('clear keeps the start in view for the current alliance', () => {
  const ed = withAlliance('blue', mk);
  ed.setAlliance('red');
  click(ed, 30, 40);
  ed.clear();
  assert.equal(ed.route.segments.length, 0);
  assert.equal(ed.route.alliance, 'red');
  assert.equal(P.startInView(ed.route), true);
});
