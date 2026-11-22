# java-pedro (BRIEF 3 items 3 and 4)

Scope: Pedro export compile check, Java review fixes. Did NOT touch app.js/autorunner (item 1/2 are for others), MAX_BALLS, auto-park.
Earlier stopped run: TeamCode has no git history (untracked), so no diff was possible. File timestamps showed ShootWhileDrivingDemoAuto was untouched (Nov 4); no partial edit found there. Existing Turret persist code and TurretZeroTest were already present and intact.

## Evidence
- `gradlew.bat --offline :TeamCode:testDebugUnitTest`: compiles, 168 tests pass (incl. SimJsonTest, 3 new tests). SimJson, SimBridge, Robot compile.
- `node --test tests_js/*.test.mjs`: 469 pass, 0 fail.
- Pedro export: generated a sample (2 paths, curve+line, markers, parallel/race/waitUntil/shoot/flower/gate) with autoexport.js, dropped it into TeamCode/src/main temporarily, `compileDebugJavaWithJavac` against Pedro 2.1.0 + FTC SDK: clean. File removed afterwards. Not run on a robot.

## Pedro export changes (autoexport.js, tests_js/autoexport.test.mjs)
- Removed `addParametricCallback` (chain callbacks never fire in 2.1.0). Markers are now polled in the follow state via `getFollower().getChainIndex()` and `getCurrentTValue()`; unfired markers are flushed when the path ends.
- Follow uses `followPath(path, true, FOLLOW_TIMEOUT_S)` + `isBusyOrTimeout()`, cancels and moves on if timed out. `FOLLOW_TIMEOUT_S = 15` is a safety bound, not a tuned value.
- Generated runOpMode wraps the loop in try/finally robot.stop().
- No tuned constants invented.

## Review findings (opus / sonnet_a / sonnet_b)
FIXED
- Opus 1 turret zero lost auto->teleop: `Robot.continuingFromAuto` forces `Turret.ZERO_MODE = PERSIST_FROM_LAST_OPMODE` when a saved pose exists (cold teleop unchanged; 300 s age guard still applies).
- Opus 2 LQR: Kalman predictor now gets the clamped applied voltage (FlywheelLQR).
- Opus 3 / turret limit overlap: `LIMIT_HYSTERESIS_DEG = 10` (PLACEHOLDER, FULL_360 only) lets aim overshoot a limit and clamp rather than unwind 360. Test added. BACKUP_170 behaviour unchanged.
- Opus 4 / sonnet_a C2 MIN_RPM: solution with rpm < Flywheel.MIN_RPM is now reachable=false with a reason. Test added. MIN_RPM value itself unchanged.
- Opus 6 / sonnet_a C5 reverse count: intake falling edge while reversing decrements; shooter falling edge while reversing is not a shot. Test added.
- Opus 14 / sonnet_b 2: demo autos use followPath timeout + isBusyOrTimeout, cancelMove on timeout, try/finally robot.stop(). `LEG_TIMEOUT_S = 10` safety bound.
- sonnet_b 6: `Robot.stop()` calls `follower.breakFollowing()` before zeroing the drivetrain.
- sonnet_a C1 FAR_CELL_UP: added `Robot.toggleFarCellUp()` and gamepad2 D-pad up/down toggle in TwoPlayerTeleOp.
REJECTED / not changed
- MAX_BALLS and preload (sonnet_a C6): needs the user's answer; left at 3.
- Opus 15 / sonnet_b 1,3,5: Pedro constants are placeholders; cannot invent tuned values (needs on-robot tuning).
DEFERRED (not clearly correct or needs hardware/design)
- Opus 5 vision trim, 7 ADRC/LADRC observer input, 8/C4 HoodReadyTimer restart, 9 FlowerMech move timeout, 10/C7 IntakeTransfer resume, 11 TABLE aim from pivot, 12 per-instance Axon tunables, 13 FlywheelPIDF integral, nits; sonnet_a A1-A8 (sim parity), C3 hood settle, C8 gate on feed; sonnet_b 4, 7, 8, 10, 11, 12.
- Opus S1-S5 sim bridge items.

## Follow-ups for other owners
- `field_sim/web/js/pathing.js:440` (planner-ui) still emits `.addParametricCallback(...)` in its own Java export; switch to the polled approach or drop callbacks there.
- Ivy-style export no longer carries path markers (they were callbacks); only statemachine style polls them.
- Not verified: markers firing on a real robot (polling logic compiled, not run); `getChainIndex()` semantics on a single-Path chain assumed 0.
