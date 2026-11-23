import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import * as C from '../../web/js/collision.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const colliders = JSON.parse(fs.readFileSync(path.join(here, '..', '..', 'web', 'data', 'colliders.json'), 'utf8'));
const obs = C.buildObstacles(colliders);

// Expected values were computed from the pre-refactor collision.js logic (copied verbatim to legacy.js)
// before the facade landed: 20 seeded poses (LCG seed 12345) near/inside walls and hive frame. Units: in, rad.
const CASES = 
[{"pose":{"x":-1.545,"y":-2.136,"h":0.271},"prev":{"x":-1.95,"y":-3.367,"h":0.271},"r":{"x":12.406888814323992,"y":12.406888814323992,"h":0.271,"hit":true,"normals":[[1,0],[0,1]],"pushed":28.49477762864798}},{"pose":{"x":12.572,"y":71.37,"h":0.304},"prev":{"x":12.283,"y":70.519,"h":0.304},"r":{"x":12.607473523346897,"y":71.37,"h":0.304,"hit":true,"normals":[[1,0]],"pushed":0.03547352334689814}},{"pose":{"x":1.438,"y":109.386,"h":2.927},"prev":{"x":2.717,"y":108.788,"h":2.927},"r":{"x":12.036213960181486,"y":109.386,"h":2.927,"hit":true,"normals":[[1,0]],"pushed":10.598213960181486}},{"pose":{"x":65.911,"y":56.126,"h":2.887},"prev":{"x":66.193,"y":54.768,"h":2.887},"r":{"x":65.911,"y":56.126,"h":2.887,"hit":false,"normals":[],"pushed":0}},{"pose":{"x":147.705,"y":97.103,"h":-0.484},"prev":{"x":147.705,"y":97.741,"h":-0.484},"r":{"x":127.45396226817039,"y":98.71468739468713,"h":-0.484,"hit":true,"normals":[[-1,0],[-0.8851407150528379,0.4653234515428497]],"pushed":20.648862645012773}},{"pose":{"x":60.239,"y":98.885,"h":-2.166},"prev":{"x":61.472,"y":97.827,"h":-2.166},"r":{"x":60.239,"y":98.885,"h":-2.166,"hit":false,"normals":[],"pushed":0}},{"pose":{"x":43.104,"y":-3.59,"h":-0.235},"prev":{"x":43.728,"y":-3.809,"h":-0.235},"r":{"x":43.104,"y":12.174315941605538,"h":-0.235,"hit":true,"normals":[[0,1]],"pushed":15.764315941605538}},{"pose":{"x":142.889,"y":-2.302,"h":1.758},"prev":{"x":143.482,"y":-1.766,"h":1.758},"r":{"x":132.15613404495878,"y":11.843865955041222,"h":1.758,"hit":true,"normals":[[-1,0],[0,1]],"pushed":24.87873191008245}},{"pose":{"x":141.771,"y":79.181,"h":1.051},"prev":{"x":141.301,"y":78.811,"h":1.051},"r":{"x":130.39228635014658,"y":79.181,"h":1.051,"hit":true,"normals":[[-1,0]],"pushed":11.378713649853406}},{"pose":{"x":20.053,"y":137.796,"h":1.68},"prev":{"x":20.182,"y":137.119,"h":1.68},"r":{"x":20.053,"y":132.7466304100585,"h":1.68,"hit":true,"normals":[[0,-1]],"pushed":5.049369589941502}},{"pose":{"x":46.203,"y":104.61,"h":0.478},"prev":{"x":45.418,"y":104.781,"h":0.478},"r":{"x":46.203,"y":104.61,"h":0.478,"hit":false,"normals":[],"pushed":0}},{"pose":{"x":26.91,"y":65.399,"h":1.08},"prev":{"x":26.847,"y":64.28,"h":1.08},"r":{"x":26.91,"y":65.399,"h":1.08,"hit":false,"normals":[],"pushed":0}},{"pose":{"x":55.378,"y":137.271,"h":1.522},"prev":{"x":56.449,"y":136.716,"h":1.522},"r":{"x":55.378,"y":128.20162006102208,"h":1.522,"hit":true,"normals":[[0,-1]],"pushed":9.069379938977907}},{"pose":{"x":49.023,"y":50.259,"h":1.842},"prev":{"x":48.133,"y":51.231,"h":1.842},"r":{"x":51.17465116075763,"y":42.52076289711023,"h":1.842,"hit":true,"normals":[[0.267891308388401,-0.9634491407904989]],"pushed":8.031806532663087}},{"pose":{"x":13.571,"y":57.289,"h":2.353},"prev":{"x":12.901,"y":57.181,"h":2.353},"r":{"x":14.05395711866982,"y":57.289,"h":2.353,"hit":true,"normals":[[1,0]],"pushed":0.48295711866981983}},{"pose":{"x":33.313,"y":55.43,"h":2.325},"prev":{"x":32.373,"y":55.814,"h":2.325},"r":{"x":33.313,"y":55.43,"h":2.325,"hit":false,"normals":[],"pushed":0}},{"pose":{"x":122.97,"y":101.481,"h":0.178},"prev":{"x":121.784,"y":101.087,"h":0.178},"r":{"x":122.97,"y":101.481,"h":0.178,"hit":false,"normals":[],"pushed":0}},{"pose":{"x":136.45,"y":54.197,"h":1.78},"prev":{"x":135.474,"y":55.408,"h":1.78},"r":{"x":132.00100152424855,"y":54.197,"h":1.78,"hit":true,"normals":[[-1,0]],"pushed":4.448998475751432}},{"pose":{"x":90.01,"y":89.023,"h":-2.436},"prev":{"x":89.774,"y":87.856,"h":-2.436},"r":{"x":83.15412974375747,"y":97.07079648058595,"h":-2.436,"hit":true,"normals":[[-0.6484850875222241,0.7612273584556017]],"pushed":10.5721324794652}},{"pose":{"x":9.441,"y":37.718,"h":1.759},"prev":{"x":8.75,"y":38.088,"h":1.759},"r":{"x":11.955049886973246,"y":37.17183442839401,"h":1.759,"hit":true,"normals":[[1,0],[0.18709459070102316,-0.9823419028680476]],"pushed":2.9660116186627237}}]
;

test('collision.js still exports the legacy API', () => {
  for (const n of ['ROBOT', 'buildObstacles', 'robotCorners', 'resolveRobot', 'slideVelocity', 'isFree']) {
    assert.ok(n in C, `missing export ${n}`);
  }
  assert.deepEqual({ ...C.ROBOT }, { halfX: 9, halfY: 9 });
});

test('resolveRobot matches pre-refactor behaviour on 20 fixed poses', () => {
  assert.equal(CASES.length, 20);
  const T = 1e-9;
  CASES.forEach(({ pose, prev, r: e }, i) => {
    const r = C.resolveRobot({ ...pose }, { ...prev }, obs);
    assert.ok(Math.abs(r.x - e.x) < T, `case ${i} x ${r.x} vs ${e.x}`);
    assert.ok(Math.abs(r.y - e.y) < T, `case ${i} y ${r.y} vs ${e.y}`);
    assert.ok(Math.abs(r.h - e.h) < T, `case ${i} h`);
    assert.equal(r.hit, e.hit, `case ${i} hit`);
    assert.ok(Math.abs(r.pushed - e.pushed) < T, `case ${i} pushed`);
    assert.equal(r.normals.length, e.normals.length, `case ${i} normal count`);
    r.normals.forEach((n, k) => {
      assert.ok(Math.abs(n[0] - e.normals[k][0]) < T && Math.abs(n[1] - e.normals[k][1]) < T, `case ${i} normal ${k}`);
    });
  });
});

test('slideVelocity removes the into-wall component', () => {
  const v = C.slideVelocity(10, 0, [[-1, 0]]);
  assert.deepEqual([v.vx, v.vy], [0, 0]);
});

test('isFree: true at field centre, false overlapping a hive_frame box', () => {
  assert.equal(C.isFree(72, 72, 0, obs, 9, 9), true);
  const hive = obs.boxes.find((b) => b.kind === 'hive_frame');
  assert.ok(hive);
  assert.equal(C.isFree((hive.minX + hive.maxX) / 2, (hive.minY + hive.maxY) / 2, 0, obs, 9, 9), false);
});
