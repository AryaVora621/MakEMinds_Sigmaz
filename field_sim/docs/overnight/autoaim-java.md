# autoaim-java: robot-side AutoAim command (integration report)

Goal: one autonomous command that aligns flywheel RPM, turret and hood for the current pose, turns transfer on,
finishes when aimed or on a timeout (never hangs), and can optionally keep feeding. Matches the sim `autoAim`
command (type `autoAim`, boolean `shootAfter`; `field_sim/web/js/autoaim_cmd.js`).

## Changes (all under TeamCode/src, Java 8)
- `main/.../commands/autonomous/AutoAim.java`: blocking `run(robot, opModeActive, [timeoutS,] shootAfter [, perLoop])`
  and `statusText`. Sets `shooter.setSpinning(true)`, loops `robot.update()` (which already runs
  `ShooterBase.update`, i.e. the existing TurretAiming/ShotSolver; no solver duplicated), feeds via
  `robot.intake.feed()`, finishes on `shooter.ready()` (flywheel + turret + hood or hood-timeout, existing gating).
  Reviewed and fixed: NaN timeout now falls back to `TIMEOUT_S` (was passed through, contradicting the javadoc).
- `AutoAimLogic.java`: hardware-free state machine (time passed in): done on ready, timeout, or no shooter.
- `AutoAimTunables.java`: `TIMEOUT_S`=3.0, `TRANSFER_ON_START`=true, `SHOOT_AFTER_MAX_S`=5.0, all marked
  PLACEHOLDER / TODO(tune on robot). Removed the unused `REQUIRE_READY_TO_FEED` field (dead, referenced nowhere).
- `AutoAimDemoAuto.java`: "Demo 3: AutoAim" opmode (placeholder start pose).
- Tests: `AutoAimLogicTest` (13), `AutoAimStatusTextTest` (6).

## Evidence
- Verified callee APIs exist: `Robot.update/feedIfReady/readyToFire/stop/addTelemetry`, `ShooterBase.setSpinning/ready/
  configurationName`, `IntakeTransfer.feed/stop`, `AutoDrivebase.followPath/isBusyOrTimeout` (used in javadoc example).
- `gradlew.bat --offline :TeamCode:testDebugUnitTest`: BUILD SUCCESSFUL; AutoAimLogicTest 13/13, AutoAimStatusTextTest 6/6,
  0 failures (re-run after the final edits below).
- `node --test tests_js/*.test.mjs`: 705 tests, 704 pass, 0 fail, 1 skipped.

## Unverified
- Nothing run on a robot or in the FTC runtime; `AutoAim.run` itself (needs a real Robot/hardwareMap) has no unit test,
  only the logic and status text do.
- Timeout and transfer behaviour values are placeholders, not tuned.

## Notes / follow-ups
- With `TRANSFER_ON_START` true the transfer runs while still aligning (same as sim), so a ball can be pushed toward the
  shooter before ready; set false on the robot if that is not wanted. `shootAfter` gates feeding on `Robot.readyToFire()`.
- `shootAfter=false` leaves the flywheel spinning and does not call `robot.stop()`; the OpMode must stop it.
- Sim planner (other agent) should keep `shootAfter` semantics aligned: true = bounded feed phase after aim.
