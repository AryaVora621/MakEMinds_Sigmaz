// t18: robustness/invalid input to the auto exporter. Real crashes or wrong output are {todo:'bug: ...'} (suite stays green).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { exportAutoJava, exportPathsOnly, listCommandStubs, sanitizeClassName, checkJavaStructure } from '../web/js/autoexport.js';

const JAVA_ID = /^[A-Za-z_$][\w$]*$/;
const STYLES = ['statemachine', 'ivy'];
const fatal = src => checkJavaStructure(src).problems.filter(p => /unbalanced|unclosed|unterminated|NaN|Infinity|undefined/.test(p));
const wait = s => ({ type: 'wait', seconds: s }), seq = (...children) => ({ type: 'sequential', children }), cmd = name => ({ type: 'command', name });

test('empty root and empty input do not throw', () => {
  for (const style of STYLES) for (const auto of [{}, { root: null }]) assert.deepEqual(fatal(exportAutoJava(auto, { style })), []);
  assert.doesNotThrow(() => exportPathsOnly({}));
  assert.deepEqual(listCommandStubs(undefined), []);
});

test('follow to a missing path id: braces balanced', () => {
  assert.deepEqual(fatal(exportAutoJava({ root: { type: 'follow', path: 'ghost' }, paths: {} })), []);
});

test('follow to a missing path id: no undeclared fields', { todo: 'bug: missing path emits undeclared path_ghost field' }, () => {
  assert.deepEqual(checkJavaStructure(exportAutoJava({ root: { type: 'follow', path: 'ghost' }, paths: {} })).problems, []);
});

test('unknown command name becomes a TODO stub', () => {
  const auto = { root: seq(cmd('zapTheRobot')) };
  assert.ok(listCommandStubs(auto).includes('custom:zapTheRobot'));
  assert.match(exportAutoJava(auto), /TODO: implement custom command "zapTheRobot"/);
});

test('nested groups depth 5 export and stay balanced', () => {
  let node = wait(0.5);
  for (let i = 0; i < 5; i++) node = seq(node, wait(0.1 * (i + 1)));
  for (const style of STYLES) assert.deepEqual(fatal(exportAutoJava({ root: node }, { style })), []);
});

test('huge wait value stays valid Java', { todo: 'bug: num() turns 1e30 into "1e+3" (trailing-zero strip), silently shortening the wait' }, () => {
  assert.deepEqual(fatal(exportAutoJava({ root: wait(1e30) })), []);
  assert.doesNotMatch(exportAutoJava({ root: wait(1e30) }), />= 1e\+3;/);
});

test('class names with spaces, digits, keywords and empty string are valid identifiers', () => {
  for (const name of ['class', '1st auto', '', 'Red Far 2+3', 'int']) {
    const id = sanitizeClassName(name);
    assert.match(id, JAVA_ID, `name "${name}" -> "${id}"`);
    const src = exportAutoJava({ name, root: wait(1) }, { className: name });
    assert.deepEqual(fatal(src), []);
    assert.match(src, new RegExp(`public class ${id} `));
  }
});

test('child array with null entries does not throw', { todo: 'bug: walk() reads node.id on null children (TypeError)' }, () => {
  assert.doesNotThrow(() => exportAutoJava({ root: seq(null, wait(1), null) }));
});

test('both styles export robust input without fatal structure problems', () => {
  const auto = { name: 'Both', alliance: 'red', root: seq(wait(1), cmd('shoot'), cmd('park')) };
  for (const style of STYLES) assert.deepEqual(fatal(exportAutoJava(auto, { style })), []);
});
