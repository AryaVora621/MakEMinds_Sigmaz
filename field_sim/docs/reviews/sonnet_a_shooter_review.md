# Shooter / subsystem cross-check review (sonnet_a)

Scope: web sim `shot_core.js`, `shotrules.js` vs Java `ShotSolver`, `ShotTable`, `TurretAiming`, `FieldConstants`; `FIELD_CONSTANTS.md`; subsystems intake, transfer, gate, flower_mech, hood, flywheel, ball_sensing, IntakeTransfer, ShooterBase; TeleOps, Robot.
Read-only review. No source files edited. `web/js` files were read at one point in time and may have changed since. Nothing was executed (no Python on PATH, and only static reading was done).

Paths: Java root = `TeamCode/src/main/java/commands/...`; JS root = `field_sim/web/js/`.

## Verdict

The core solver is a faithful port. `shot_core.js` and `ShotSolver.java` agree on every formula and branch: kinematics, the gamma/theta/speed/minspeed quartics, rising filter, branch note, forward check, and wrap. I found no numeric mismatch there. The mismatches are in the layer around the solver: what the robot does before calling it (latency, motion gate) versus what the sim does, plus several hardware and logic defects.

## A. Sim vs robot mismatches

### A1. Sim omits latency compensation and the shoot-on-the-move gate (MEDIUM)
- Java `TurretAiming.java:130-134` advances the pose by `v*LATENCY_S` (0.05 s) and heading by `omega*LATENCY_S`. It then zeroes vx, vy and omega when `hypot(vx,vy) < MIN_MOVE_SPEED_IN_S (4)` (`:159-165`).
- Sim `app.js:315-330` (`solveShot`) and `shotrules.js:158-174` (`solveAuto`) use the live pose and the full velocity and omega every time. The comment at `app.js:312` says "same maths as ShotSolver.java".
- Scenario: at 60 in/s the robot aims about 3 in further ahead than the sim does. At 4 in/s the Java solution jumps by about `v*T`, roughly 3 in. The sim has no such step.
- A pure spin below 4 in/s translation is ignored by Java but compensated by the sim. This only matters when `PIVOT_OFFSET` is not 0 (default 0).
- Fix: either add the same latency and gate to `solveAuto` and `solveShot`, driven by spec fields, or reword the comment and expose "latency" and "min move speed" in the spec. Better still, make Java blend velocity in smoothly instead of using a hard 4 in/s step.

### A2. Sim has no `SHOT_SOURCE = TABLE`, no vision trim and no `reachable` rpm rule (LOW)
- `ShotTable.java` and the TABLE branch (`TurretAiming.java:138-156`) have no sim counterpart. `ShotTable` values are PLACEHOLDERS (`ShotTable.java:18-23`), so the sim cannot be used to validate table mode.
- TABLE mode never sets `fromBehind`, so a shot from behind the opening reports `reachable=true` (`TurretAiming.java:138-156`, `:265`). The PHYSICS path checks it.

### A3. Hive aim x differs by 0.017 in (NIT)
- Java `FieldConstants.java:34-35` uses 59.25 / 84.75. The CAD (doc line 39) and the sim JSON aim_point use 59.267 / 84.767. `legalzone.js:8` uses 59.25 / 84.75.
- Negligible for aiming, but it is two sources of truth. Pick one. The CAD value is the better one, since the doc says the centre is 59.267 and Fig 9-10 gives 59.25.

### A4. Landing test rectangle is not the real opening (MEDIUM, info-only in the sim)
- `shotrules.js:141-155` (`landing`) tests `|a| <= halfW-R` and `|v| <= halfH-R`, with defaults `halfW = 10` and `halfH = 7`, centred on the aim point. No call site sets them (grep found none).
- The aim point is the pentagon centroid. The doc (line 27 and the aim table at line 39) puts the bbox centre 1.4 in above it in the plane (z 59.53 vs 58.31). Opening: 8 in of vertical walls plus a roof triangle.
- The centred rectangle therefore extends about 1.4 in below the real floor and cuts off the roof peak. A ball whose centre is on the floor edge counts as landing, although its radius 1.39 hits the front lip.
- `forward.lands` is the "Ball would enter the opening" message in assist and manual mode. It will be optimistic at the bottom edge and pessimistic near the roof peak.
- Fix: read the pentagon polygon from `field_constants.json` and do a point-in-polygon test with the ball radius inset. At minimum use `halfH = 5.6` below and 8.4 above the centroid.

### A5. `ranges()` measures `d` from robot centre, `autoPhysics` from the pivot (LOW)
- `stationaryOk` (`shotrules.js:179-184`) places the robot at `tg.x - d`, heading 0. With a non-zero `pivotX/pivotY` the pivot-to-target distance is not `d`. Meanwhile `autoPhysics` (`:225-229`) compares the pivot distance with `rg.far/near`.
- The OUT_OF_RANGE messages can be off by about `pivotX`, or can choose the wrong branch near the limit.
- `rangeKey` (`:177`) omits `pivotX`, `pivotY` and the normal, so the cache can go stale when the spec changes.
- Fix: place the pivot at `tg.x - d` and add `pivotX/pivotY` to the key.

### A6. `ranges()` assumes one contiguous feasible interval (LOW)
- `:192-197` take the first ok distance as near and the last as far. If hood clamping makes the feasible set two intervals, the gap is reported as in range.

### A7. `evaluateShot` assist-mode NaN comparisons (LOW)
- `shotrules.js:367-368`: if `ctx.hoodDeg` or `ctx.turretDeg` is `undefined`, the comparisons are false and the limit check is skipped. Triggering the message would also throw on `.toFixed`. `forwardShot` defaults them to 0; these lines do not.
- Fix: use `(ctx.hoodDeg || 0)`.

### A8. `st.intakeOn = st.feeding` in the sim bridge (LOW)
- `Robot.java:235`. The bridge reports the intake as on only while feeding, so ordinary bumper intaking never shows in the sim.

## B. Field constants vs `FIELD_CONSTANTS.md`

Checked and matching: tag ID order for all four clusters (low-x first: 33,32,31,30 / 34-37 / 38-41 / 45,44,43,42, doc lines 50-53), hive z 43.95, opening z 53.5 / 65.6, aim z 58.313, y offset 16.483 (doc 55.517 / 88.483), opening normal (0, -/+0.866, 0.5), loading zones, gardens, flower XY (documented reorder), ball sizes, `DEFAULT_BOUNDS` equal to glass faces (1.326 / 142.674).

Differences, all documented as intentional in Java comments, listed for completeness:
- Flower opening height is 21.5 in Java vs 21.404 in CAD (`FieldConstants.java:81`).
- Loading zone is 23 vs 22.69, garden 23 vs 22.69 (`:89`, `:95`).
- `FIELD_SIZE_IN = 144` vs the CAD interior of 141.35 (doc line 82 and "Unresolved", line 113). Nothing in the shooter uses it. The sim bounds use 141.35, and Pedro/robot bounds use 144. A shot taken at x < 1.3 in the sim is OUT_OF_FIELD but legal on the robot. The doc says the team must decide. Still open.
- `FieldConstants.java:72-75` has a dangling comment fragment ("// on a built field, in BOTH tip states)."). Cosmetic.
- Doc-only issue: line 8 of the doc says "no Haiku workers were spawned". That is process noise in a reference doc. Remove it.

## C. Robot-side defects

### C1. `FAR_CELL_UP` can never be changed during a match (HIGH)
- `Robot.java:69` is a static default of `true`. Neither TeleOp, nor Robot, nor any sensor path updates it. The comment says "Change it when the hive tips", but there is no input.
- Scenario: the hive is bistable. When the near cell is the raised one, the aim point is 33 in away in y (`FieldConstants.java:133`). Every shot goes to the wrong cell.
- Fix: add an operator toggle (for example gamepad2 D-pad) in `TwoPlayerTeleOp`, or vision/tag-based detection. At minimum show `FAR_CELL_UP` in telemetry and let the Dashboard edit it, with a loud warning if it is never set.

### C2. `Flywheel.MIN_RPM = 2500` overrides low physics RPM (MEDIUM)
- `Flywheel.java:515, 554-556`. Any non-zero target is raised to 2500 RPM. Near shots at the preferred 50 degree entry can need less. By my estimate, about 200 in/s launch speed gives about 2245 RPM with the placeholder constants.
- `atTarget()` compares against the raised target, so `ready()` is true and the ball goes out too fast.
- Fix: lower `MIN_RPM` to a true floor (for example 300 to match the sim `MIN_SPIN_RPM`), or have `ShooterBase` flag solutions below `MIN_RPM` as unreachable instead of silently raising them.

### C3. Hood reports "at target" immediately when slew and feedback are off (MEDIUM)
- `Hood.atTarget()` (`Hood.java:471-475`) calls `AxonServo.atTarget`. With the defaults `FEEDBACK_CALIBRATED = false` and `SLEW_DEG_PER_S = 0` (`AxonServo.java:33, 50`), position is the commanded angle, and the command jumps to target on the first `update()` (`:168-169`).
- `ready()` therefore never waits for the real hood motion. The sim models a 120 deg/s hood (`aimcontrol.js:8`).
- Result: the first shot after a large hood change can be fed while the hood is still moving. `HOOD_READY_TIMEOUT_S` never matters.
- Fix: give Hood its own slew or a minimum settle time (`|delta| / HOOD_RATE`) that applies when feedback is untrusted.

### C4. `HoodReadyTimer` never times out while the target keeps moving (LOW)
- `HoodReadyTimer.java:356-360` restarts the timer whenever the target differs by more than 1e-9. During shoot-on-the-move the target changes every loop, so a dead hood servo never trips the timeout. ready() then waits forever, which is the safe direction but defeats the purpose.
- Fix: restart only when the target moves by more than the tolerance (about 1.5 deg).

### C5. Intake outtake never decrements the ball count (MEDIUM)
- `BallTrackerLogic.java:187-192`: intake rising edges are ignored while reversing, and `consumeFalling()` is thrown away. A ball pushed back out through the intake sensor never lowers the count.
- Scenario: the driver ejects 2 balls, the count stays high, and with `AUTO_STOP_WHEN_FULL` the intake locks until the operator presses BACK (the recovery path exists, but it is only a workaround).
- Fix: while `intakeReversing`, a falling intake edge should decrement the counter (the ball left through the front).

### C6. Ball cap and preload (MEDIUM)
- `BallTracker.MAX_BALLS = 3` (`BallTracker.java:12`) but `FieldConstants.PRELOAD_POLLEN = 4` (`FieldConstants.java:111`). A 4-ball preload clamps to 3.
- The tracker starts at 0 and nothing seeds it with the preload or an autonomous leftover (`Robot.java:145`). The `Robot` placeholder list warns about `MAX_BALLS` (line 365) but not about this.
- Fix: confirm the real hopper capacity, and seed the count at teleop start from the auto hand-off (as the pose already is via `savedPose`).

### C7. `IntakeTransfer` never resumes after a full-stop clears by itself (LOW)
- `IntakeTransfer.update` (`:84-87`) only stops motors, never restarts them. Both TeleOps call `forward()` every loop while the bumper is held, so it recovers there. Any caller that calls `forward()` once (an autonomous command) would stay stopped after the count drops. Document it, or restart when `modeMayRun` flips back to true.

### C8. Feeding ignores the gate (MEDIUM, needs a design decision)
- `Robot.feedIfReady` (`Robot.java:262-275`) and `ShooterBase.ready()` never check `Gate.isOpen()` or `gate.atTarget()`. The gate is toggled only by operator Y. If the gate is closed, a "ready" feed pushes balls into a closed gate.
- Fix: open the gate automatically on feed (and require `gate.atTarget()` in `readyToFire`), or document that the operator must open it first.

### C9. Smaller items
- `Gate.java:158`: when `ENABLED` is false, `atTarget()` is true and `isOpen()` still toggles. Harmless, but telemetry will show a gate state that does not exist.
- `FlowerMechLogic.java:466` compares `progress == goal` on doubles. It is safe here because the slew snaps to goal, but fragile if refactored.
- `TwoPlayerTeleOp.java:164-169`: gamepad2 A/B act while Start is held (Start+A/B is used to assign gamepads on the Driver Station). The init loop guards against this, the run loop does not.
- `TurretAiming.java:262-267`: RPM above `MAX_RPM` is only a warning (`reachable=false`) and the Flywheel clamps to max. That is correct and consistent with the sim `RPM_TOO_HIGH`.
- `Flywheel.AT_TARGET_TOLERANCE = 50` ticks/s is about 107 RPM at 28 ticks/rev. That is about 3 to 4 percent at 3000 RPM, in line with the sim `toleranceRpmFrac` of 0.04. OK.
- `OnePlayerTeleOp` never calls `intake.update(tracker)`, so it has no full-stop. Expected, since it has no tracker.

## D. Priority order for fixes

1. C1 `FAR_CELL_UP` input (HIGH).
2. C2 `MIN_RPM`, C3 hood settle, C5 outtake decrement, C6 ball cap/preload, C8 gate (MEDIUM, each is a small change).
3. A1 latency/gate parity, A4 landing polygon (MEDIUM).
4. The rest (LOW/NIT).
