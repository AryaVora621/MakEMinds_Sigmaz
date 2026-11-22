import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';
import { routeToPP, ppToRoute } from '../web/js/ppformat.js';

/** One straight segment with an autoAim callback at t=0.5 carrying the given args. */
function routeWithAutoAim(args) {
  const r = P.newRoute({ name: 'AutoAim', start: { x: 24, y: 24, h: 0 } });
  P.addLine(r, 96, 24);
  const cb = { t: 0.5, name: 'autoAim' };
  if (args !== undefined) cb.args = args;
  r.segments[0].callbacks = [cb];
  return r;
}
/** Route -> .pp object -> route, both directly and through JSON text. */
function roundTrips(r) {
  const direct = ppToRoute(routeToPP(r)).route;
  const viaJson = ppToRoute(JSON.stringify(JSON.parse(JSON.stringify(routeToPP(r))))).route;
  return [direct, viaJson];
}

test('autoAim shootAfter:true survives routeToPP -> ppToRoute as a strict boolean', () => {
  const r = routeWithAutoAim({ shootAfter: true });
  for (const back of roundTrips(r)) {
    const cb = back.segments[0].callbacks[0];
    assert.equal(cb.name, 'autoAim');
    assert.equal(cb.t, 0.5);
    assert.deepEqual(cb.args, { shootAfter: true });
    assert.equal(typeof cb.args.shootAfter, 'boolean');
    assert.equal(cb.args.shootAfter, true);
  }
});

test('autoAim shootAfter:false also round-trips as a strict boolean', () => {
  const r = routeWithAutoAim({ shootAfter: false });
  for (const back of roundTrips(r)) {
    assert.deepEqual(back.segments[0].callbacks[0].args, { shootAfter: false });
  }
});

test('autoAim callback without args keeps no args key after round trip', () => {
  const r = routeWithAutoAim(undefined);
  for (const back of roundTrips(r)) {
    const cb = back.segments[0].callbacks[0];
    assert.equal(cb.name, 'autoAim');
    assert.equal(Object.prototype.hasOwnProperty.call(cb, 'args'), false);
  }
});

test('autoAim callback in exported .pp object carries no args key when args are absent', () => {
  const o = routeToPP(routeWithAutoAim(undefined));
  const x = o.lines[0].biobuzz;
  assert.ok(x && Array.isArray(x.callbacks), 'biobuzz.callbacks present');
  assert.equal(Object.prototype.hasOwnProperty.call(x.callbacks[0], 'args'), false);
});

test('shootAfter:"true" string in a .pp object normalises to boolean true', () => {
  const pp = routeToPP(routeWithAutoAim({ shootAfter: true }));
  pp.lines[0].biobuzz.callbacks[0].args = { shootAfter: 'true' };
  const cb = ppToRoute(pp).route.segments[0].callbacks[0];
  assert.deepEqual(cb.args, { shootAfter: true });
  assert.equal(cb.args.shootAfter, true);
});

test('shootAfter string other than "true" normalises to boolean false', () => {
  const pp = routeToPP(routeWithAutoAim({ shootAfter: true }));
  pp.lines[0].biobuzz.callbacks[0].args = { shootAfter: 'yes' };
  const cb = ppToRoute(pp).route.segments[0].callbacks[0];
  assert.deepEqual(cb.args, { shootAfter: false });
});

test('junk args keys are dropped from autoAim callbacks', () => {
  const pp = routeToPP(routeWithAutoAim({ shootAfter: true }));
  pp.lines[0].biobuzz.callbacks[0].args = { shootAfter: true, rpm: 3000, evil: 'x', nested: { a: 1 } };
  const cb = ppToRoute(pp).route.segments[0].callbacks[0];
  assert.deepEqual(cb.args, { shootAfter: true });
  assert.equal(Object.prototype.hasOwnProperty.call(cb.args, 'rpm'), false);
  assert.equal(Object.prototype.hasOwnProperty.call(cb.args, 'evil'), false);
  assert.equal(Object.prototype.hasOwnProperty.call(cb.args, 'nested'), false);
});

test('autoAim args without a shootAfter key leave no args on import', () => {
  const pp = routeToPP(routeWithAutoAim({ shootAfter: true }));
  pp.lines[0].biobuzz.callbacks[0].args = { rpm: 3000 };
  const cb = ppToRoute(pp).route.segments[0].callbacks[0];
  assert.equal(Object.prototype.hasOwnProperty.call(cb, 'args'), false);
});
