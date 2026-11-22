// h_teamcode-java_t18: no Pedro 2.1.0 chain callbacks in robot Java or in planner-emitted Java.
// Chain callbacks never fire (OVERNIGHT_RULES.md), so any hit below is a real defect.
// Failing-by-design findings: none declared at write time. Offending path:line is printed in the assert message.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync, statSync } from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const CALLBACK = /addParametricCallback|addTemporalCallback|addPoseCallback|setCallbacks|\.addCallback/;
const isComment = line => /^\s*(\/\/|\*)/.test(line);

// Recursively collect files under dir whose name ends with ext.
function walk(dir, ext, out = []) {
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    if (statSync(p).isDirectory()) walk(p, ext, out);
    else if (name.endsWith(ext)) out.push(p);
  }
  return out;
}

// Return "path:line: text" for every non-comment line that matches the callback pattern.
function scan(files) {
  const hits = [];
  for (const f of files) {
    readFileSync(f, 'utf8').split(/\r?\n/).forEach((line, i) => {
      if (!isComment(line) && CALLBACK.test(line)) hits.push(`${f}:${i + 1}: ${line.trim()}`);
    });
  }
  return hits;
}

const javaRoot = fileURLToPath(new URL('../../TeamCode/src/main/java', import.meta.url));
const webJs = fileURLToPath(new URL('../web/js', import.meta.url));

test('TeamCode Java has no Pedro chain callbacks outside comments', () => {
  const hits = scan(walk(javaRoot, '.java'));
  assert.deepEqual(hits, [], `Pedro chain callback found (chain callbacks never fire):\n${hits.join('\n')}`);
});

test('planner-generated Java (web/js template strings) has no Pedro chain callbacks', () => {
  // Scan every web/js/*.js file (top level only), matching the raw names in template strings.
  const files = readdirSync(webJs).filter(n => n.endsWith('.js')).map(n => join(webJs, n));
  const hits = scan(files);
  assert.deepEqual(hits, [], `Pedro chain callback emitted by planner (chain callbacks never fire):\n${hits.join('\n')}`);
});
