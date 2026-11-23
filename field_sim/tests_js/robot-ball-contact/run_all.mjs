// Run: node tests_js/robot-ball-contact/run_all.mjs   (exit code 1 on any failure)
const files = ['./solver.test.mjs', './roller_hold.test.mjs', './reaction_pinch.test.mjs', './facade.test.mjs'];
let pass = 0, fail = 0;
for (const f of files) {
  let mod;
  try { mod = await import(f); } catch (e) {
    fail++; console.log(`FAIL ${f}: import error: ${(e && e.stack) || e}`); continue;
  }
  for (const t of mod.tests) {
    try { await t.fn(); pass++; console.log(`ok   ${f} - ${t.name}`); } catch (e) {
      fail++; console.log(`FAIL ${f} - ${t.name}\n     ${(e && e.message) || e}`);
    }
  }
}
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
