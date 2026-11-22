// cellSensors/insideCell/crossesOpening on a synthetic cell (x 0..10, opening y=0, back y=10, z 0..5).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { cellSensors, insideCell, crossesOpening, SENSOR_INSET, OPEN_INSET } from '../web/js/hivemesh.js';

function makeHive(shift = [0, 0, 0]) {
  const S = p => [p[0] + shift[0], p[1] + shift[1], p[2] + shift[2]];
  const cell = {
    opening_corners: [[0, 0, 0], [10, 0, 0], [10, 0, 5], [0, 0, 5]].map(S),
    floor_corners: [[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0]].map(S),
    back_wall_corners: [[0, 10, 0], [10, 10, 0], [10, 10, 5], [0, 10, 5]].map(S),
    opening_normal: [0, -1, 0],   // outward, through the opening
    aim_point: S([5, 0, 2.5]),
  };
  return { pivot: shift, cells: { c1: cell } };
}
const s = cellSensors(makeHive()).c1;

test('documented inset constants and sensor extent', () => {
  assert.equal(SENSOR_INSET, 0.5);
  assert.equal(OPEN_INSET, 1.0);
  assert.equal(s.x0, 0.5);
  assert.equal(s.x1, 9.5);
  assert.equal(insideCell([5, 5, 2.5], s), true);
});

test('insideCell boundaries follow the inset planes (inclusive at the inset)', () => {
  const cases = [
    [[5, 1.0, 2.5], true], [[5, 0.5, 2.5], false],   // OPEN_INSET behind opening
    [[5, 5, 0.5], true], [[5, 5, 0.4], false],       // floor
    [[5, 5, 4.5], true], [[5, 5, 4.6], false],       // lid
    [[5, 9.5, 2.5], true], [[5, 9.6, 2.5], false],   // back wall
    [[0.5, 5, 2.5], true], [[0.4, 5, 2.5], false],   // side x0
    [[9.6, 5, 2.5], false],                          // side x1
  ];
  for (const [p, want] of cases) assert.equal(insideCell(p, s), want, `point ${p}`);
});

test('segment through the opening (outside to inside) is crossesOpening', () => {
  assert.equal(crossesOpening([5, -5, 2.5], [5, 5, 2.5], s), true);
});

test('exiting segment, wall segment and x-offset segment are not crossesOpening', () => {
  assert.equal(crossesOpening([5, 5, 2.5], [5, -5, 2.5], s), false);  // inside to outside
  assert.equal(crossesOpening([5, -5, 8], [5, 5, 8], s), false);      // wall above opening
  assert.equal(crossesOpening([15, -5, 2.5], [15, 5, 2.5], s), false); // beside opening in x
});

test('segments entirely outside the opening plane are not crossesOpening', () => {
  assert.equal(crossesOpening([20, -5, 2.5], [30, -5, 2.5], s), false);
  assert.equal(crossesOpening([-5, -5, 2.5], [-5, -10, 2.5], s), false);
});

test('sensor geometry is invariant under pivot shift', () => {
  const t = cellSensors(makeHive([10, 20, 3])).c1;
  assert.deepEqual(t.planes, s.planes);
  assert.deepEqual(t.P, s.P);
  assert.deepEqual(t.op, s.op);
});
