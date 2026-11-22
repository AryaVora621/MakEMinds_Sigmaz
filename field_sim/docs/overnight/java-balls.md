# java-balls (ball capacity, tip counts, robot ball-type handling)

Scope: read-only audit of `..\TeamCode` plus edits made by the java tasks e1-e6. No git history exists for TeamCode (untracked), so changes below are inferred from file timestamps (all after `docs/OVERNIGHT_RULES.md`) and by reading the code. No diff was possible.

## 1. Facts from the user (authoritative)
- Max balls held is 4. Applied: `BallTracker.java:12` `MAX_BALLS = 4`; `FieldConstants.java:111` `PRELOAD_POLLEN = 4` (manual G304).
- A hive tips at 8 POLLEN. The manual does not say this. Applied: `FieldConstants.java:112-113` `HIVE_TIP_POLLEN = 8`, tagged USER-CONFIRMED.
- Shooter must fire both POLLEN and NECTAR. Mixed nectar+pollen loads must be modelled.

## 2. Changes made (Java/test files, from timestamps and content)
- `TeamCode/src/main/java/commands/subsystems/ball_sensing/BallTracker.java:12`: `MAX_BALLS` 3 -> 4.
- `TeamCode/src/main/java/commands/subsystems/ball_sensing/BallTrackerLogic.java:67-72`: intake edges during reversing decrement the count (falling edge). Documented at header lines 8-15.
- `TeamCode/src/main/java/commands/field/FieldConstants.java:98-114`: game-piece diameters (2.8 / 3.6 in, OFFICIAL 9.8), `PRELOAD_POLLEN = 4`, `HIVE_TIP_POLLEN = 8`, and a comment-only placeholder for nectar tip count (line 114, no constant).
- `TeamCode/src/main/java/commands/subsystems/flywheel/FlywheelPIDF.java:34-38`: integral clamp uses `|kI|` (java-pedro deferred item 13).
- `TeamCode/src/main/java/commands/subsystems/flywheel/FlywheelLQR.java`: Kalman predictor gets clamped voltage (java-pedro Opus 2).
- `TeamCode/src/main/java/commands/subsystems/turret/TurretAiming.java`: turret limit hysteresis (`LIMIT_HYSTERESIS_DEG`, PLACEHOLDER) (java-pedro Opus 3).
- `TeamCode/src/main/java/robot/Robot.java:327`: `toggleFarCellUp()`; `Robot.java:374` MAX_BALLS placeholder warning is gone; `continuingFromAuto` turret persist (java-pedro Opus 1).
- `TeamCode/src/main/java/commands/teleop/TwoPlayerTeleOp.java:102`: gamepad2 D-pad toggles far cell up.
- `TeamCode/src/main/java/commands/autonomous/ProfileDemoAuto.java`, `ShootWhileDrivingDemoAuto.java`: followPath timeout, try/finally stop (java-pedro Opus 14).
- Tests: `BallTrackerLogicTest.java` and `RobustnessTest.java` now build with capacity 4 (15 and 3 call sites); `BallCounterTest.java:52` asserts `MAX_BALLS == 4`; new `commands/field/FieldConstantsBallsTest.java` (`HIVE_TIP_POLLEN == 8`, `PRELOAD_POLLEN == 4`); `flywheel/FlywheelPIDFTest.java` (negative kI clamp); `turret/TurretAimingTest.java`.
- Not verified: I did not run `gradlew.bat :TeamCode:testDebugUnitTest`. java-pedro.md reports 168 passing before the MAX_BALLS and ball-test edits; current status is unknown.

## 3. Nectar tip count
- The manual gives no nectar tip count (`docs/overnight/research_manual.md:34`, `:39-40`). Nectar diameter 3.6 in (manual 9.8) is in `FieldConstants.java:105`.
- The only researched number is the mass ratio 0.091 lb / 0.055 lb (about 1.65), cited from the AndyMark BIOBUZZ game-set listing (https://andymark.com/products/ftc-2026-27-biobuzz-game-sets, third party) and the FIRST game postcard (https://www.firstinspires.org/hubfs/web/program/ftc/biobuzz-gamepostcard.pdf). Per `field_sim/web/js/hivetip_model.js:7-36`. A third-party video summary (https://youtubesummary.com/summary/sUH3z5a5S9I) is cited there only for "tip is proportional to mass", which it does not state. Confidence: low for the tip effect; mass ratio itself medium.
- ASSUMPTION (recorded): nectar tip count is unknown. The sim exposes it as one tunable, `nectarWeight` (hivetip_model.js:67, default about 1.65 pollen-equivalent, so 5 nectar alone tip a hive). Mixed rule: tip when `pollen*1 + nectar*nectarWeight >= 8`. Java has no nectar constant and needs one only when the robot logic is written.
- Reasoning: the user says the rule is 8 pollen; the manual is silent on counts; mass is the only sourced physical proxy available.

## 4. Robot-code findings (confirmed)
- Java has no per-ball type or tip logic. `BallTracker` and `BallTrackerLogic` count integers only (`BallTrackerLogic.java:18-30`). `BallColor` (`NONE, POLLEN, NECTAR, UNKNOWN`) exists (`BallColor.java:5`) but is only used by `BallSensor` (`BallSensor.java:69,181,199,213`) and telemetry (`BallTracker.java:120`). `DETECT_COLOR = false` (`BallTracker.java:15`), so color is off by default.
- No `ballsToTip` or tip-count constant exists in Java. Hive code is aiming geometry only: `FieldConstants.java:124-137` (aimY, opening normal), `Robot.java:68` (far-cell-up flag).
- Shooter is not type-gated. A grep for `BallColor|POLLEN|NECTAR` across `commands/subsystems/` (outside ball_sensing) finds nothing, so `ShooterBase.ready()` never reads a ball type.
- The `TeamCode` tests use only integer ball counts and never touch a ball type.

## 5. Deferred java-pedro items
- FIXED: item 13 FlywheelPIDF integral clamp (now `|kI|`, `FlywheelPIDF.java:34-38`, tested).
- FIXED (earlier): Opus 1, 2, 3, 4, 6, 14, sonnet_a C1, C2, C5, sonnet_b 6 (see java-pedro.md).
- SKIPPED, still open: Opus 5 vision trim (`TurretAiming.java:106-109, 237-239`), needs robot test of the sign (VISION_TX_SIGN).
- SKIPPED, still open: 8/C4 HoodReadyTimer restart (`HoodReadyTimer.java:20-24`, `ShooterBase.java:98-99`). The solver output changes continuously, so the hood timer restarts while shooting on the move. Needs a restart threshold; not changed.
- Not re-checked in this pass (status per java-pedro.md): items 7, 9, 10, 11, 12, sonnet_a A1-A8, C3, C8, sonnet_b 4, 7, 8, 10, 11, 12, Opus S1-S5.

## 6. Follow-ups for field_sim owners
- ballsToTip must be 8 for pollen. Current state: `field_sim/web/js/hivecycle.js:53` now sets `ballsToTip: HIVE_TIP.threshold / pollenWeight` (= 8), and `mode: 'weight'` is the default (line 54). Stale test: `field_sim/tests_js/hivecycle.test.mjs:35-36` still asserts `ballsToTip === 4`, which will fail. Owner must update it. Test lines 49, 57, 74, 98, 107, 110, 146, 171, 188 also use explicit ballsToTip values and need review.
- Nectar+pollen mixed rule: implemented as a linear weight sum in `hivetip_model.js:34`; the nectar weight is an ASSUMPTION (see section 3).
- Shooting both types: Java shooter does not gate on type, so the sim should let the shooter fire either type and count each toward the tip. Verify `field_sim` shooter paths take a ball type.
- `field_sim/web/js/pathing.js:440` still emits `.addParametricCallback(...)` (from java-pedro follow-ups). Not a ball issue, but unresolved.
- Java: when `BallTracker.MAX_BALLS` changes or a nectar constant is added, update `FieldConstantsBallsTest.java` accordingly.

Evidence: `ls` of `TeamCode/src/main/java/commands/subsystems/ball_sensing/`; grep for `ballsToTip|HIVE_TIP|POLLEN|NECTAR|BallColor|MAX_BALLS` in `TeamCode/src`; reads of the files above. JS tests not run (no JS edited).
