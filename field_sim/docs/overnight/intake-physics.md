# intake-physics report (BRIEF 2, items 1 and 4 only)

## Item 1: intake capture realism
Files: `web/js/balls.js` (INTAKE constants + `_intakeStep`), `tests_js/physics3d.test.mjs`.
Cause of vacuuming: 14 in funnel plus 8 in per-side flare, a sideways guide force toward the mouth, mouth half-width = halfY+0.8,
and capture of anything with centre up to 2*halfX under the footprint (scooping through the body).
New model (named constants in `INTAKE`):
- `pullRadius` 2.5 in (surface gap to the roller face where the roller pulls, about one ball diameter), `pull` 500 in/s^2, straight at the face only.
- `mouthInset` 0.5: ball centre must be within halfY-0.5 laterally; no flare, no sideways guide. Balls beside the mouth only get ordinary robot collision.
- `captureGap` 0.6: surface within this of the face (touching roller) = loaded.
- `maxOverlap` 2.0: centre deeper than this inside the face (under the body) is never loaded; kept above 0 only as swept-robot slack.
- Unchanged: `maxLift` 3.2, `maxSpeed` 220, shot balls ignored, capacity (`space`), front+rear sides (MaxBot has both; "behind" read as
  behind the roller face / through the body, not the rear roller).
Removed `funnelLen`, `flare`, `guide`, `mouthExtra`. Nothing outside balls.js/tests referenced them (grep of web/js, tests_js, tools_js).
Tests (physics3d.test.mjs, replaced the old "wide mouth + funnel" test which asserted the bug): touching/near-roller balls taken front+rear;
balls past the pull radius or beside the mouth are neither taken nor dragged (<1 in); side-hugging balls not scooped; ball under the body
not loaded; constants sanity. The far/side/dragged tests fail on the old code (it took balls 22 in away and from beside).
The existing swept-robot, capacity and sleeping-start-ball intake tests still pass unchanged.

## Item 4: shot / hive / ball physics audit
Read the capture path (`_hiveCapture`, `_capture`, `crossesOpening`, `_enter`): only a ball that crossed the opening polygon from outside
(or a preloaded start ball resting inside) scores. No code defect found, so no change to balls.js capture, collision.js, hive*.js, shot*.js.
New `tests_js/shotaudit.test.mjs` (seeded fuzz, deterministic):
1. 120 shots at each hive: all balls finite, inside field+margin; every scored ball sits inside its own cell sensor.
2. 160 shots, 6 s settle: no free ball left resting inside a cell sensor (no stuck-uncounted balls).
3. Back-door check: every scored ball was observed on the outside side of its own cell opening plane while free before scoring
   (>=3 scores per hive required so the test is not vacuous).
4. 5000 in/s balls fired through the hive: speed clamp holds, no NaN.
holdLimit untouched (not in my files; still 4).
Note: a first "thrown from behind the opening" test wrongly flagged legit scores: behind cell A is in front of cell B (opposite opening),
and balls spawned inside the hive body can pop out and fall back in. Rewrote as check 3.

## Evidence
`node --test tests_js/*.test.mjs`: 464 pass, 0 fail, 1 skipped (was 456 pass before). Note `node --test tests_js` (bare directory)
reports a spurious single failure on this Node; use the glob `tests_js/*.test.mjs`.

## Not verified / follow-ups
- No headless-Chrome check of intake feel; physics-only tests. Real robot dims in app (DIMS passed to `setRobot`) were not inspected;
  tests use 9x9 in. If the robot is narrower than the actual roller, tune `mouthInset`.
- Numbers (pullRadius 2.5, pull 500) are sim-feel choices, not measured; TODO measure on the real MaxBot intake.
- Integrator: app.js calls `world.setIntake({enabled, space})` only, so no change needed there. Autos that relied on far-away pickup
  now need the path to actually drive the roller onto the ball.
- Items 2 and 3 (flower arm, robot model) were not touched, per assignment.
