// FAILING BY DESIGN (real mismatch, kept on purpose): Java RobotSettings.AutoAim.TIMEOUT_S (was 3.0) but
// web/js/autoaim_cmd.js AUTOAIM_DEFAULTS.timeoutSec = 4. The "timeout parity" test below keeps its
// assertion and fails until the owner of one side changes its value. Do not weaken it.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const javaPath = fileURLToPath(new URL('../../TeamCode/src/main/java/robot/RobotSettings.java', import.meta.url));
const java = readFileSync(javaPath, 'utf8');
const num = name => {
  const m = java.match(new RegExp(`public static double ${name} = ([\\d.]+);`));
  return m ? Number(m[1]) : NaN;
};
const TIMEOUT_S = num('TIMEOUT_S');
const SHOOT_AFTER_MAX_S = num('SHOOT_AFTER_MAX_S');
const transferMatch = java.match(/public static boolean TRANSFER_ON_START = (true|false);/);
const TRANSFER_ON_START = transferMatch ? transferMatch[1] === 'true' : null;

const { AUTOAIM_DEFAULTS, makeAutoAim } = await import(new URL('../web/js/autoaim_cmd.js', import.meta.url).href);

test('Java TIMEOUT_S is a positive finite number', () => {
  assert.ok(Number.isFinite(TIMEOUT_S) && TIMEOUT_S > 0, `got ${TIMEOUT_S}`);
});

test('Java SHOOT_AFTER_MAX_S is a positive finite number', () => {
  assert.ok(Number.isFinite(SHOOT_AFTER_MAX_S) && SHOOT_AFTER_MAX_S > 0, `got ${SHOOT_AFTER_MAX_S}`);
});

test('Java TRANSFER_ON_START is a boolean', () => {
  assert.equal(typeof TRANSFER_ON_START, 'boolean');
});

test('JS timeoutSec default is a positive finite number', () => {
  assert.ok(Number.isFinite(AUTOAIM_DEFAULTS.timeoutSec) && AUTOAIM_DEFAULTS.timeoutSec > 0);
});

test('timeout parity: JS AUTOAIM_DEFAULTS.timeoutSec equals Java TIMEOUT_S', () => {
  // EXPECTED TO FAIL: JS 4 vs Java 3.0 (see header comment).
  assert.equal(AUTOAIM_DEFAULTS.timeoutSec, TIMEOUT_S);
});

test('transfer parity: JS autoAim default for transfer-on-start equals Java TRANSFER_ON_START', () => {
  const calls = [];
  const cmd = makeAutoAim({ host: { transfer: on => calls.push(on), current: () => null, target: () => null, apply() {} } });
  cmd.start();
  assert.equal(AUTOAIM_DEFAULTS.transferOnStart, TRANSFER_ON_START, 'sim default must match Java RobotSettings.AutoAim.TRANSFER_ON_START');
  assert.deepEqual(calls, TRANSFER_ON_START ? [true] : []);
});

test('FINDING: JS has no shootAfter max-duration default matching Java SHOOT_AFTER_MAX_S', (t) => {
  // JS has no exported field for this. The firing cap is a non-exported const FIRE_MAX_SEC = 6
  // in autoaim_cmd.js, while Java SHOOT_AFTER_MAX_S = 5.0. Documented only; no assertion on JS.
  t.diagnostic(`Java SHOOT_AFTER_MAX_S=${SHOOT_AFTER_MAX_S}; JS FIRE_MAX_SEC=6 (not exported, not in AUTOAIM_DEFAULTS)`);
  assert.ok(!('shootAfterMaxSec' in AUTOAIM_DEFAULTS));
});
