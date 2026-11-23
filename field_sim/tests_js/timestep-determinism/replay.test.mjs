import test from 'node:test';
import assert from 'node:assert/strict';
import { TS } from '../../web/js/physics/timestep-determinism/params.js';
import { createRng } from '../../web/js/physics/timestep-determinism/prng.js';
import { approach, integrateBodyTwist } from '../../web/js/physics/timestep-determinism/integrators.js';
import {
  quantAxis, encodeInput, decodeInput, quantizeInput, InputRecorder, InputLog,
} from '../../web/js/physics/timestep-determinism/inputlog.js';
import { runReplay, recordRun } from '../../web/js/physics/timestep-determinism/replay.js';

const schema = { axes: ['fwd', 'turn'], buttons: ['a', 'b'] };

// Toy sim: 2D robot (ramped body velocity + exact SE(2) pose) and a ball nudged by seeded noise each control tick.
function createSim(seed) {
  const rng = createRng(seed).stream('shoot');
  const pose = { x: 0, y: 0, h: 0 };
  const st = { vx: 0, om: 0, ball: { x: 0, y: 0 } };
  let vt = 0, ot = 0;
  return {
    stepTick({ h, isControl, input }) {
      if (isControl) {
        vt = (input?.axes?.fwd ?? 0) * 50; // [in/s]
        ot = (input?.axes?.turn ?? 0) * 2; // [rad/s]
        st.ball.x += rng.gaussian() * 0.01; // [in]
        st.ball.y += rng.gaussian() * 0.01;
      }
      st.vx = approach(st.vx, vt, 100, 200, h);
      st.om = approach(st.om, ot, 20, 40, h);
      integrateBodyTwist(pose, st.vx, 0, st.om, h);
    },
    snapshot() { return { pose: { ...pose }, vx: st.vx, om: st.om, ball: { ...st.ball } }; },
  };
}

const inputForTick = (tick) => ({
  axes: { fwd: tick < 600 ? 0.5 : -0.3, turn: tick < 300 ? 0.2 : 0.7 },
  buttons: { a: tick >= 900, b: false },
});

const TICKS = 1200;
const record = (seed = 42) => recordRun({ createSim, seed, schema, inputForTick, ticks: TICKS });

test('decode(encode(x)) equals quantizeInput(x) and is idempotent', () => {
  const x = { axes: { fwd: 0.123456789, turn: -2 }, buttons: { a: true } };
  const enc = encodeInput(x, schema);
  const dec = decodeInput(enc, schema);
  assert.deepEqual(dec, quantizeInput(x, schema));
  assert.equal(dec.axes.fwd, Math.round(0.123456789 * TS.AXIS_QUANT) / TS.AXIS_QUANT);
  assert.equal(dec.axes.turn, -1);
  assert.equal(dec.buttons.a, true);
  assert.equal(dec.buttons.b, false);
  assert.deepEqual(quantizeInput(dec, schema), dec);
  assert.equal(quantAxis(2), TS.AXIS_QUANT);
  assert.equal(quantAxis(-2), -TS.AXIS_QUANT);
});

test('recordRun then runReplay is ok', () => {
  const log = record();
  assert.ok(log instanceof InputLog);
  assert.equal(log.endTick, TICKS);
  const res = runReplay(log, { createSim });
  assert.equal(res.ok, true, JSON.stringify(res.firstDesync));
  assert.equal(res.firstDesync, null);
  assert.equal(res.ticks, TICKS);
  const resQ = runReplay(log, { createSim, compare: 'quant' });
  assert.equal(resQ.ok, true);
});

test('JSON round-trip of the log still replays ok', () => {
  const log = record();
  const text = JSON.stringify(log.toJSON());
  const log2 = InputLog.fromJSON(text);
  assert.equal(log2.seed, log.seed);
  assert.equal(log2.endTick, log.endTick);
  const res = runReplay(log2, { createSim });
  assert.equal(res.ok, true, JSON.stringify(res.firstDesync));
});

test('log is change-only: constant input between changes yields few entries', () => {
  const log = record();
  const json = log.toJSON();
  assert.ok(json.inputs.length <= 6, `entries ${json.inputs.length}`);
  for (const [tick] of json.inputs) assert.equal(tick % TS.CONTROL_DIVIDER, 0);
  for (let i = 1; i < json.inputs.length; i++) assert.ok(json.inputs[i][0] > json.inputs[i - 1][0]);
  assert.equal(json.version, TS.REPLAY_VERSION);
  assert.equal(json.hz, TS.PHYSICS_HZ);
});

test('altering one input entry desyncs at the first hash tick after it', () => {
  const json = JSON.parse(JSON.stringify(record().toJSON()));
  const idx = json.inputs.findIndex((e) => e[0] >= 600);
  assert.ok(idx > 0);
  const tick = json.inputs[idx][0];
  json.inputs[idx][1][0] = 20000; // different fwd axis
  const res = runReplay(InputLog.fromJSON(json), { createSim });
  assert.equal(res.ok, false);
  const expectedTick = Math.floor(tick / TS.HASH_EVERY_TICKS) * TS.HASH_EVERY_TICKS + TS.HASH_EVERY_TICKS;
  assert.equal(res.firstDesync.tick, expectedTick);
  assert.ok(['exact', 'quant'].includes(res.firstDesync.kind));
});

test('changing the seed desyncs', () => {
  const json = JSON.parse(JSON.stringify(record(42).toJSON()));
  json.seed = 43;
  const res = runReplay(InputLog.fromJSON(json), { createSim });
  assert.equal(res.ok, false);
  assert.equal(res.firstDesync.tick, TS.HASH_EVERY_TICKS);
});

test('recording is deterministic: two runs give identical logs', () => {
  assert.deepEqual(record().toJSON(), record().toJSON());
});

test('InputRecorder is change-only and inputAt returns held values (binary search)', () => {
  const rec = new InputRecorder({ schema, seed: 1 });
  const i0 = { axes: { fwd: 0.5, turn: 0 }, buttons: { a: false } };
  const i1 = { axes: { fwd: -0.5, turn: 0 }, buttons: { a: true } };
  rec.record(4, i0);
  rec.record(8, i0); // unchanged -> not appended
  rec.record(12, i0);
  rec.record(16, i1);
  rec.record(20, i1);
  assert.equal(rec.entries.length, 2);
  rec.recordHash(240, { exact: 'deadbeef', quant: 'cafebabe' });
  const log = rec.finish(240);
  assert.deepEqual(log.inputAt(0), { axes: { fwd: 0, turn: 0 }, buttons: { a: false, b: false } }); // before first entry
  assert.deepEqual(log.inputAt(3), log.inputAt(0));
  const held = quantizeInput(i0, schema);
  assert.deepEqual(log.inputAt(4), held);
  assert.deepEqual(log.inputAt(15), held);
  assert.deepEqual(log.inputAt(16), quantizeInput(i1, schema));
  assert.deepEqual(log.inputAt(10000), quantizeInput(i1, schema));
  assert.deepEqual(log.hashAt(240), { exact: 'deadbeef', quant: 'cafebabe' });
  assert.equal(log.hashAt(241), null);
});

test('fromJSON rejects wrong version unless allowMismatch', () => {
  const json = JSON.parse(JSON.stringify(record().toJSON()));
  json.version = 99;
  assert.throws(() => InputLog.fromJSON(json));
  const json2 = JSON.parse(JSON.stringify(record().toJSON()));
  json2.hz = 120;
  assert.throws(() => InputLog.fromJSON(json2));
  assert.doesNotThrow(() => InputLog.fromJSON(json2, { allowMismatch: true }));
});
