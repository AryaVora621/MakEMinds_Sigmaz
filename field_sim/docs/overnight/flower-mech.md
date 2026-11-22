# flower-mech: integration report (step 3)

## Changed files (all owned)
- web/js/flowerarm.js: three fixes (below).
- tests_js/flower_regress.test.mjs (new): negative-dt and nested-cfg-isolation regressions.
- tests_js/h_flower-mech_t16.test.mjs, h_flower-mech_t17.test.mjs: un-skipped the two BUG tests; they now pass and guard the fixes.

## Fixes applied (verified against code, reproduced by the Haiku skip tests)
1. Double feed: feedFromFlower now skips balls already in arm._pending. Before, the top ball could be re-selected on the next tick and fedCount burned on one ball (t20 probe: released ids A,B,C,C).
2. Shared config: merge() returned DEFAULT_FLOWER_CFG nested objects by reference, so mutating one arm's cfg leaked into all later arms. Now deep-cloned (structuredClone).
3. update(dt): negative or NaN dt is clamped to 0 so the arm cannot run backwards.

## Evidence (ran)
- node --test tests_js/flower*.test.mjs tests_js/h_flower-mech_*.test.mjs: 56 tests, 56 pass, 0 fail, 0 skip.
- node --test tests_js/*.test.mjs: 946 tests, 922 pass, 6 fail, 3 skipped. The 6 failures are not in my files: validateSpec missing frame string, FAILING-BY-DESIGN hood muzzle height, tunnel/speed sweep x2 (dt 0.0167 and 0.0333), AUTOAIM timeout JS vs Java parity, hold-limit literal 4 in Java and web/js. I did not investigate them.
- Before my edits the full suite had 9 fails: my 3 new/un-skipped tests (expected red before the fix) plus these same 6.

## Haiku findings, judged
- Accepted: double feed, shared cfg, negative dt.
- Not applied (design decisions): fedCount resets on every deploy(), including deploy from mid-stow, so toggling can exceed maxBallsPerDeploy; the hold limit 4 still caps it. Kept as is and documented here. reset() is silent (no state callback), kept. onState callback exceptions are not isolated, kept.
- Doc-only: the "tip = bottom edge of the paddle" comment really means the rubber bottom (3.05 in); behaviour is correct.
- Ownership of a flower is dynamic (top nectar, manual 10.5.2, per docs/research_manual.md:50); code does not hardcode an owner.

## Unverified
- Hinge, arm length and angles in flower_arm.json are ASSUMED; no CAD measurement checked. The Onshape CAD in the request was not reachable or read by me, so no geometry came from it.
- Nothing visual was re-run in step 3 (the t20 Haiku headless-Chrome run is theirs, not re-checked by me).
- flowerarm3d.js was not changed or re-audited in this step; only the Haiku parity tests (tip vs tipRobot, wrist angle) cover it, and they pass.
- No Java/Pedro behavior checked here.
- Haiku claim that JSON and DEFAULT_FLOWER_CFG can drift: no parity test exists. Suggested, not added.

## Patches for other owners
- app.js: none required. The arm API is unchanged. Callers that pass a negative dt will now see a no-op instead of reverse motion.
- Owners of the 6 failing non-flower tests: see the list above.
