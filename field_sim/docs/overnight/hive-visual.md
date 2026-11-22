# hive-visual report (Sonnet lead, step 3 integrate)

## Finding that matters most
hivemesh.js is collision-only: it had NO tip animation and NO drop logic. The visible tip lives in hivecycle.js
(smoothstep, 0.6 s) + app.js (group.rotation.x = rad(angleDeg - cadAngle)). Haiku audits (a01, a14, a20) confirm the
rotation math (pivot, axis +x, sign, 60 deg total) is consistent for red and blue.
Balls DO NOT drop out when they were CAPTURED (a17, a19): `state:'scored'` balls are re-placed at a cell slot every frame
and never return to `free` on tip (releaseOnTip=false, onTip never calls world.release). Only FREE balls resting in the up
cell fall out through the opening (a19: 16/16 exits, 0 via wall, 0 NaN, run in node, not by me).
So the user's "balls actually drop out of the hive" is NOT satisfied by the current code for scored balls.

## Changed files (all owned)
- web/js/hivemesh.js
  - parseHiveCollider: rejects short buffer, implausible counts, truncated data, tri index >= nv (a05 #1,#2).
  - TriMesh ctor: rejects out-of-range index (a bad index used to make bounds NaN and silently kill all collisions).
  - sphereContacts: `!(d2 < r2)` so a NaN contact from a degenerate triangle is dropped (a05 #3).
  - `_nextStamp()`: Int32 stamp wrap guard (a05 #4); used by sphereContacts and sweepSegment.
  - hiveTipLabel(null) no longer throws; stale doc example fixed (5 pollen + 1 nectar = 6.8, not 6.6).
  - NEW pure helpers: TIP_VISUAL, dampedEase, hiveTipVisual, dropSchedule, dropsDue (spec from a18, re-run in tests).
    Constants (dampK 6, releaseStartFrac 0.5, staggerS 0.1 s) are ASSUMED cosmetic, TODO tune by eye.
- tests_js/hive_visual_tip.test.mjs (new, 7 tests).
- tests_js/h_hive-visual_a12.test.mjs: the todo "null stats throws" is now a passing test.
- Kept as is: h_hive-visual_a07..a12 tests (all pass). Not applied: a05 #5 (sphereVsMesh prev; unused by balls.js, behaviour change).

## Evidence (commands I ran)
- `node --test tests_js/hive_visual_tip.test.mjs tests_js/h_hive-visual_*.test.mjs` -> 35 pass, 0 fail.
- `node --test tests_js/*.test.mjs` -> 953 tests, 930 pass, 6 fail, 14 todo. The 6 failures are in other owners' files:
  validateSpec missing frame string; FAILING-BY-DESIGN hood muzzle; h_shoot-physics_t12 tunnel sweep x2 (hive 200-534 in/s);
  autoAim timeout parity; hold-limit literal parity. My edits are behaviour-neutral for valid meshes (validation, NaN guard,
  stamp guard), but I did NOT bisect the t12 tunnel failures against the pre-edit file (cannot stash); treat as unverified.
- Haiku headless runs (read from reports, not re-run by me): a13 screenshot showed red hive visibly changed after
  `__sim.tip('red')`; a14 rotX tracks angleDeg exactly and settles at 1.0472 rad (60 deg). SwiftShader ran ~0.6 fps so
  the 0.6 s animation took ~20 s wall (dt clamped 0.05/frame). I took no screenshot myself in this step.

## Unverified
- Real-GPU look of the tip and any drop (no screenshot of balls leaving was taken by anyone).
- Blue tip direction vs real hardware (a01).
- Whether hive skin transparency (alpha 0.2 panels, a16) lets viewers see balls inside; inferred only.
- Drop timing constants; nectar weight 1.75 is INFERRED (hivetip_model.js).

## Patches for other owners
1. balls.js (hive-physics owner), to make scored balls really drop: on tip start, snapshot scored balls of the
   hive's UP cell (the cell that flips to down is the one whose opening faces the field floor; use hivecycle `counts`),
   compute `sched = dropSchedule({nBalls, tipDurationS: HIVE_PARAMS.animationDuration})`, and each frame
   `for (i of dropsDue(sched, prevElapsed, elapsed)) { ball.state='free'; ball._enter=false; ball.captureCooldown=0.5; }`
   so the mesh collider and gravity carry them out of the opening. Add a captureEnabled guard so the ball is not
   re-captured by the cell that is now the down cell. Needs a balls.js test (8 scored -> 8 free, 0 NaN).
2. hivecycle.js: optional `step()` swap smoothstep for `dampedEase` (import from hivemesh.js) for a damper feel.
   Set `releaseOnTip` consistent with the world release so the scorer list does not diverge.
3. app.js: `onTip` should call the new world release hook; HUD "TIPPING" lingers in headless only because of 0.6 fps.
4. Missing `web/data/robot.glb`: the Onshape robot CAD is not loaded (a14); out of scope here.
