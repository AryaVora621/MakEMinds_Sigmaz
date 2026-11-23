// calibrate.js: recover plant time constants from a synthetic trace. Units: s, in, rad.
import test from 'node:test';
import assert from 'node:assert/strict';
import { resolveParams } from '../../web/js/physics/pedro-fidelity/params.js';
import { MecanumPlant } from '../../web/js/physics/pedro-fidelity/plant.js';
import { fitFromTrace, compareTraces, parseTraceCsv, fitAxis } from '../../web/js/physics/pedro-fidelity/calibrate.js';

// effort (cx, cy, cw) -> wheel powers [LF, LR, RF, RR] (inverse of plant.efforts())
const powersOf = (cx, cy, cw) => [cx - cy - cw, cx + cy - cw, cx + cy + cw, cx - cy + cw];

// Excitation: 1.5 s driving ONE axis at +-0.5 effort, then 1.5 s FLOAT coast (so the Coulomb term is observable), repeated per axis.
function makeTrace(truth) {
  const plant = new MecanumPlant(truth, { x: 0, y: 0, h: 0 }), dt = truth.loop.dt, trace = [];
  const sched = [];
  for (let r = 0; r < 4; r++) for (const ax of [0, 1, 2]) for (const sg of [1, -1]) sched.push([ax, 0.5 * sg]);
  const blk = Math.round(1.5 / dt);
  let p = [0, 0, 0, 0];
  for (let k = 0; k < sched.length * 2 * blk; k++) {
    const b = Math.floor(k / blk);
    if (k % blk === 0) {
      if (b % 2 === 0) { const [ax, a] = sched[b / 2], c = [0, 0, 0]; c[ax] = a; p = powersOf(...c); } else p = [0, 0, 0, 0];
    }
    trace.push({ t: k * dt, x: plant.x, y: plant.y, h: plant.h, p: p.slice() });
    plant.setPowers(p); plant.step(dt);
  }
  return trace;
}

test('fitFromTrace recovers tau within 5% from a synthetic trace', () => {
  const truth = resolveParams({ plant: { tauX: 0.25, tauY: 0.3, tauW: 0.15 } });
  const fit = fitFromTrace(makeTrace(truth), resolveParams());   // fit starts from the default params
  for (const [k, want] of [['tauX', 0.25], ['tauY', 0.3], ['tauW', 0.15]]) {
    assert.ok(Math.abs(fit[k] - want) / want < 0.05, `${k} ${fit[k]} vs ${want}`);
  }
  assert.ok(Math.abs(fit.xVelocity - truth.mecanum.xVelocity) / truth.mecanum.xVelocity < 0.05, `xVelocity ${fit.xVelocity}`);
});

test('fitAxis on an exact first-order response returns tau and vFree', () => {
  const tau = 0.2, vFree = 100, s = []; let v = 0;
  for (let k = 0; k < 200; k++) { s.push({ t: k * 0.01, e: 1, v }); v += (vFree - v) / tau * 0.01; }
  const f = fitAxis(s, { coulomb: 0 });
  assert.ok(Math.abs(f.tau - tau) / tau < 0.05, `tau ${f.tau}`);
  assert.ok(Math.abs(f.vFree - vFree) / vFree < 0.05, `vFree ${f.vFree}`);
});

test('compareTraces: identical traces give zero error; a 1 in offset gives 1 in RMS', () => {
  const a = makeTrace(resolveParams()).slice(0, 300);
  const same = compareTraces(a, a);
  assert.equal(same.rmsPosIn, 0); assert.equal(same.maxPosIn, 0);
  const b = a.map(s => ({ ...s, x: s.x + 1 }));
  const c = compareTraces(a, b);
  assert.ok(Math.abs(c.rmsPosIn - 1) < 1e-6, `rms ${c.rmsPosIn}`);
  assert.ok(Math.abs(c.maxPosIn - 1) < 1e-6);
  assert.ok(compareTraces(a, b, { alignPose: true }).rmsPosIn < 1e-9, 'alignPose removes the start offset');
});

test('parseTraceCsv reads t,x,y,h,p0..p3[,V]', () => {
  const rows = parseTraceCsv('t,x,y,h,p0,p1,p2,p3,V\n0,1,2,0.5,1,1,1,1,12.1\n0.01,1.1,2,0.5,0,0,0,0,12.0\n');
  assert.equal(rows.length, 2);
  assert.deepEqual(rows[0], { t: 0, x: 1, y: 2, h: 0.5, p: [1, 1, 1, 1], V: 12.1 });
  assert.deepEqual(rows[1].p, [0, 0, 0, 0]);
  const noV = parseTraceCsv('t,x,y,h,p0,p1,p2,p3\n0,1,2,0.5,1,1,1,1\n');
  assert.equal(noV[0].V, undefined);
});
