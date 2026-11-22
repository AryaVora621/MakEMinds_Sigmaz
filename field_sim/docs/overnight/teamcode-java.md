# teamcode-java report

## Verdict on command-group compatibility
- The codebase has NO command framework (no NextFTC/FTCLib/Ivy dependency; checked build.dependencies.gradle and TeamCode/src by grep, haiku t03/t04 agree). Pedro 2.1.0 has no command groups either.
- `AutoAim.run` is BLOCKING (two `while` loops, calls `robot.update()` itself). Inside a sequential group it works (it just occupies the OpMode thread). Inside a parallel group it is unsafe: it would double-call `robot.update()` and starve the other branch. It has no cancel token; only `opModeActive`.
- Cancel/interrupt: the flywheel stays spinning on every exit (by design, so the next path keeps speed); only `robot.stop()` or `setSpinning(false)` clears it. Before my change an exception left a latched `intake.feed()` running.
- No Pedro chain callbacks anywhere in TeamCode (grep, t13, plus a JS test).

## Changed files (all mine)
- NEW `TeamCode/src/main/java/commands/autonomous/AutoAimStep.java`: non-blocking command form (`initialize/execute/isFinished/end(interrupted)`), injected clock and `Port` seam, `forRobot(robot, shootAfter)` factory. Never calls `robot.update()` (owner calls it once per loop). `end()` always stops intake, leaves flywheel spinning. Shoot-after phase is bounded by `SHOOT_AFTER_MAX_S`.
- NEW `TeamCode/src/test/java/commands/autonomous/AutoAimStepTest.java`: 8 tests (aim, timeout, one-tick non-blocking, cancel, shoot-after bound, not-ready stops intake, no shooter, re-init).
- `AutoAim.java`: wrapped both loops in try/finally that stops the intake (flywheel still left spinning).
- `AutoAimLogic.java`: timeout check is now `!(elapsed < timeout)` so a NaN clock times out instead of hanging. Test added to `AutoAimLogicTest.java`.
- `AutoAimTunables.TIMEOUT_S` 3.0 -> 4.0 to match sim `AUTOAIM_DEFAULTS.timeoutSec` (still a PLACEHOLDER, not robot-tuned; the value is a judgment call, revert if you prefer 3 and change the sim).
- `tests_js/h_teamcode-java_t17.test.mjs`: removed `ammoCap` from the scan regex (app.js `ammoCap=99` is the practice-mode unlimited fallback, a false positive).

## Evidence (ran)
- `..\gradlew.bat --offline :TeamCode:testDebugUnitTest`: BUILD SUCCESSFUL (twice, last after all edits). First run XML totals: 202 tests, 0 fail, 0 error, 0 skipped (AutoAimLogicTest 14, AutoAimStepTest 8). Test XML not re-totalled after the TIMEOUT_S edit; build was green.
- `node --test tests_js/*.test.mjs` final: tests 973, pass 954, fail 0 (the rest skipped/todo). Earlier mid-run it showed 8 failures owned by other workers (hivetip, validateSpec, hood, tunnel sweep); they were gone on the final run, so those owners changed things concurrently. I did not touch them.
- MAX_BALLS: `grep MAX_BALLS = ` in TeamCode/src/main finds only BallTracker.java:12 = 4. Haiku t06/t07 and `h_teamcode-java_t17` test confirm Java and JS hold limit 4.

## Haiku findings: accepted / rejected
- Accepted: t02 (try/finally intake stop), t05 (NaN clock gap, non-blocking seam), t12 (feed latch hazard, drove the end() design), t16/t17/t18/t19 tests kept (all green).
- Not acted on, outside my files or not verified: t10 flower parity (JS lacks DWELL/FAULTED), t14 export issues, t20 (sim does not drop balls from hive), t11 (VEX loader rule conflict SG9 vs VUG3, unverified).
- t12 H5: `AutoAim` calls `intake.feed()` on `transferOn` without `readyToFire()`; this matches the sim's "transfer on at start" semantics, left as is.

## Unverified
- `AutoAimStep` is tested only through the fake `Port`; `forRobot` is not exercised on a Robot or hardware, and no scheduler consumes it yet.
- No on-robot run of any of this. Real Pedro 2.1.0 API names used by the planner export were not compiled here.

## Patches for other owners
- autoexport.js (planner export): Ivy style makes `autoAim` a silent no-op `instant(() -> {})` (autoexport.js:396); the default statemachine style could instead emit `AutoAimStep.forRobot(robot, shootAfter)` and step it (`initialize/execute/isFinished/end`) in the loop, which is the supported non-blocking path. Ivy is not a dependency; label it "not compilable".
- Robot.java owner: `Robot.stop()` does not touch gate/flower/hood; dangling javadoc at Robot.java:325-326.
- Sim owner (hive drop): t20 says captured balls stay attached to the hive pivot and never drop out on tip.
- Sim owner (flower): add DWELL auto-stow and FAULTED parity to flowerarm.js if parity is wanted.
