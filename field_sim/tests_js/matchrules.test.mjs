import test from 'node:test';
import assert from 'node:assert/strict';
import { RULES, isOfficial, MatchScorer, NectarRelease } from '../web/js/matchrules.js';

test('hold limit and preload constants', () => {
  assert.equal(RULES.holdLimit, 4); assert.equal(RULES.preloadPollen, 4);
});
test('timing constants', () => {
  assert.deepEqual([RULES.timing.auto, RULES.timing.transition, RULES.timing.teleop], [30, 8, 120]);
  assert.equal(RULES.timing.endgameFlowerUnlockRemaining, 60);
});
test('robot size limits', () => {
  assert.deepEqual(RULES.robotLimits.start, [18, 18, 18]); assert.deepEqual(RULES.robotLimits.expanded, [18, 24, 29]);
});
test('provenance flags present and endgame label is an assumption', () => {
  assert.equal(RULES.source, 'BIOBUZZ manual TU08');
  assert.equal(isOfficial('holdLimit'), true);
  assert.equal(isOfficial('points.tipAuto'), true);
  assert.equal(isOfficial('timing.endgame'), false);
  assert.equal(isOfficial('timing.endgameFlowerUnlockRemaining'), true);
  assert.equal(isOfficial('nothing.here'), false);
  for (const k of Object.values(RULES.provenance)) assert.equal(typeof k.official, 'boolean');
});
test('tip points by phase; transition counts as auto', () => {
  const m = new MatchScorer();
  m.onTip('red', 'auto'); m.onTip('red', 'transition'); m.onTip('red', 'teleop'); m.onTip('red', 'endgame');
  const f = m.finalize(null, {});
  assert.equal(f.red.tips, 4); assert.equal(f.red.tipPoints, 80); assert.equal(f.blue.total, 0);
});
test('park once per phase and leave once', () => {
  const m = new MatchScorer();
  assert.equal(m.onPark('blue', 'auto'), true); assert.equal(m.onPark('blue', 'auto'), false);
  assert.equal(m.onPark('blue', 'teleop'), true);
  assert.equal(m.onLeave('blue'), true); assert.equal(m.onLeave('blue'), false);
  const f = m.finalize();
  assert.equal(f.blue.parkPoints, 10); assert.equal(f.blue.leavePoints, 3); assert.equal(f.blue.total, 13);
});
test('flower window closed before last 60 s, open after', () => {
  const m = new MatchScorer();
  assert.equal(m.onBallInFlower('red', true, 61), false);
  assert.equal(m.onBallInFlower('red', true, 60), true);
  assert.equal(m.onBallInFlower('red', true, 10), true);
  assert.equal(m.onBallInFlower('red', false, 10), false);   // not owned
  assert.equal(m.finalize().red.flowerPoints, 4);
});
test('bottom nectar bonus is gated by the flower window', () => {
  const m = new MatchScorer();
  assert.equal(m.onBottomNectar('red', 90), false); assert.equal(m.onBottomNectar('red', 30), true);
  assert.equal(m.finalize().red.flowerPoints, 5);
});
test('garden balls score 1 each at any time', () => {
  const m = new MatchScorer(); m.onBallInGarden('blue'); m.onBallInGarden('blue');
  assert.equal(m.finalize().blue.gardenPoints, 2);
});
test('end-of-match breakdown and winner / RP', () => {
  const m = new MatchScorer();
  m.onLeave('red'); m.onPark('red', 'auto'); m.onTip('red', 'auto'); m.onTip('red', 'teleop');
  m.onBallInFlower('red', true, 30); m.onBallInGarden('red'); m.onPark('red', 'teleop');
  m.onTip('blue', 'teleop');
  const f = m.finalize({ red: 'down', blue: 'up' }, { red: 3, blue: 1 });
  assert.deepEqual(f.red, { tips: 2, tipPoints: 40, upCellBalls: 3, upCellPoints: 6, parkPoints: 10, leavePoints: 3, flowerPoints: 2, gardenPoints: 1, total: 62 });
  assert.equal(f.blue.total, 22);
  const s = m.summary();
  assert.equal(s.winner, 'red'); assert.deepEqual(s.rp, { red: 3, blue: 0 }); assert.ok(s.finalized);
});
test('tie gives 1 RP each', () => {
  const m = new MatchScorer(); m.onTip('red', 'auto'); m.onTip('blue', 'auto'); m.finalize();
  const s = m.summary(); assert.equal(s.winner, 'tie'); assert.deepEqual(s.rp, { red: 1, blue: 1 });
});
test('history records events and reset clears', () => {
  const m = new MatchScorer(); m.onTip('red', 'auto'); m.onBallInFlower('red', true, 100);
  assert.equal(m.history.length, 2); assert.equal(m.history[1].type, 'flower_rejected');
  m.reset(); assert.equal(m.history.length, 0); assert.equal(m.summary().red.total, 0);
});
test('unknown alliance throws', () => {
  assert.throws(() => new MatchScorer().onTip('green', 'auto'));
});
test('nectar release: one per own tip', () => {
  const n = new NectarRelease();
  assert.equal(n.update(100, 0).release, 0);
  assert.equal(n.update(100, 1).release, 1);
  assert.equal(n.update(100, 1).release, 0);
  assert.equal(n.update(90, 3).release, 2);
});
test('nectar release: all remaining at 60 s left, never more than the pool', () => {
  const n = new NectarRelease();
  n.update(100, 2);
  assert.equal(n.update(61, 2).release, 0);
  assert.equal(n.update(60, 2).release, 3);
  assert.equal(n.update(30, 9).release, 0);
});
