# Pedro Pathing 2.1.0 review (sonnet_b)

Scope: pedroPathing/Constants.java, Tuning.java, commands/autonomous (AutoDrivebase, motion_profiling/*, ProfileDemoAuto, ShootWhileDrivingDemoAuto), Drivebase, OdomPods, Robot.java.
Method: read-only; compared against the 2.1.0 `core` and `ftc` sources jars (Follower, VectorCalculator, FollowerConstants, PathConstraints, Mecanum, PinpointLocalizer). Nothing was compiled or run on a robot. Paths are relative to TeamCode/src/main/java.

## Verified OK (no action)
- Motor names and directions in Constants match Drivebase (Pedro Mecanum.updateConstants re-applies directions every loop, same values).
- Pinpoint offset convention: `forwardPodY` goes to goBILDA `setOffsets(x=forwardPodY, y=strafePodX)`. OdomPods doc (parallel pod: left positive; perpendicular pod: forward positive) matches.
- `PathConstraints(0.99, 100, 1, 1)` resolves to the 4-arg ctor (t=0.99, timeout=100 ms, braking 1/1, other thresholds default 0.1/0.1/0.007). Valid.
- ProfilePlanner math (trapezoid, S-curve, segment integration, plateau test `vp*j >= a^2`, `accelDistance`, binary search for short moves, 7 segments <= MAX_SEGMENTS) is correct. Hand-checked a 24 in move: vp ~ 42 in/s, ~1.14 s.
- `Vector.setComponents` flips theta for negative magnitude, so `getMagnitude()` is never negative. The "reversed vector = braking" test in ProfiledVectorCalculator is the right approach.
- Replacing `follower.vectorCalculator` is legal (public field). `breakFollowing()` is called on the new calculator (needed to init vectors/lists) and it shares `Constants.followerConstants` with the Follower.
- No PathCallbacks, `turn`, or `HeadingInterpolator` calls exist in the reviewed autonomous code, so none to review. Demos use `setLinearHeadingInterpolation(h, h)` correctly.
- Tuning.java is the stock Pedro quickstart; no local defects beyond finding 9.

## Findings (ranked)

### 1. HIGH - Follower runs on untuned library defaults, and predictive braking is off
- Where: pedroPathing/Constants.java:25 (`new FollowerConstants()`), also :46.
- Scenario: in 2.1.0 `usePredictiveBraking` defaults to false and is only turned on by `.predictiveBrakingCoefficients(...)`. Constants never calls it, so:
  - the legacy drive-PIDF path is used (`VectorCalculator.getDriveVector`), which depends on the untuned `forwardZeroPowerAcceleration`;
  - running Tuning > "Predictive Braking Tuner" has no effect on the robot (its kQuadratic/kLinear output has nowhere to go);
  - comments saying "Pedro's predictive braking" (PlannedProfile.java:11, MotionProfile.java) do not describe what runs;
  - mass, centripetalScaling, translational/heading/drive PIDs, and `xVelocity` (81.34 in/s default) are all unmeasured.
- Impact: end-of-path overshoot or oscillation; `K_V = 1/xVelocity` is wrong if the real free speed differs; the S-curve is only trackable if `V_MAX` (60) is below the real free speed.
- Fix: pick one braking scheme. Either run the predictive tuner and add `.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(...))`, plus `.xVelocity(...)`/`.yVelocity(...)` on MecanumConstants; or tune forward/lateral zero-power acceleration for the legacy path. Add `.mass(...)`. Add a Robot.placeholderWarnings() entry that fires while `followerConstants` is default (for example `xVelocity == 81.34056`).

### 2. HIGH (demos), MEDIUM (library) - waiting loops have no stuck/timeout handling
- Where: ProfileDemoAuto.java:80 `while (opModeIsActive() && robot.auto.isBusy())`; ShootWhileDrivingDemoAuto.java:167 same.
- Scenario: wrong motor direction, a blocked robot, a dead Pinpoint or an unplugged wheel before t=0.8 means Pedro's stuck detector never fires (`stuckTValue` = 0.8). `isBusy()` stays true and the OpMode hangs until STOP.
- AutoDrivebase already provides `followPath(..., timeoutS)` and `isBusyOrTimeout()` (AutoDrivebase.java:92, 178); the demos do not use them.
- Fix: `followPath(path, true, expectedTime * 2 + 2)` and `while (opModeIsActive() && robot.auto.isBusyOrTimeout())`; on `timedOut()` call `cancelMove()` and abort or skip the leg.

### 3. MEDIUM - planned profile hands back full power when the time-based plan expires
- Where: motion_profiling/PlannedProfile.java:58-60, 75.
- Scenario: the reference is purely time-based. If the robot cannot track it (V_MAX 60 above real free speed, low battery, wheel slip, or `MAX_DT_S` = 0.1 clamping at AutoDrivebase.java:125 under-counting elapsed), then at `elapsed >= totalTime` the profile returns `stockPower` (up to 1.0). The robot, still behind the reference, gets a full-power surge: the launch kick the profile exists to remove, now at the end. Also `command` is clamped to 1.0, so feedforward saturates and kP has no authority.
- Fix: do not release until position error is small and speed is low (for example `ref[0]-traveled < 1 in`, or ramp the cap down over ~0.2 s). Cap `V_MAX` at ~0.85 x `Constants.driveConstants.getXVelocity()` when planning (`freeSpeed()` already reads it).

### 4. MEDIUM - `elapsed` includes the pre-move loop time and stops during braking loops
- Where: PlannedProfile.java:56 (`elapsed += dt` before `sample`), AutoDrivebase.java:125.
- Scenario: the first `apply()` after `followPath` adds the previous loop's dt (up to 0.1 s), so the reference starts about 0.1 s ahead (a small free step at launch). `apply()` is skipped on loops where the vector is reversed or zero, so the clock stops and the reference drifts later than wall time.
- Fix: set `elapsed = 0` at `start()` and ignore the first dt, or drive the clock from `moveElapsedS()`.

### 5. MEDIUM - holdEnd=false path completes up to ~1 in short, still moving
- Where: Constants.java:46 (`tValueConstraint 0.99`, `timeout 100`) with core Follower.update lines 536-539.
- Scenario: completion is `t >= 0.99` AND (`100 ms elapsed` OR velocity/translational/heading constraints met); precedence is `A || (B && C && D)`, so 100 ms after reaching t=0.99 the path completes regardless of velocity or error. With `holdEnd=false` the robot leaves the path up to 0.01 x length short (~1 in on 96 in), moving, and `breakFollowing()` floats the motors.
- Fix: fine for intermediate waypoints. For precise stops use `0.995` (library default) or a longer timeout (250-500 ms) and prefer `holdEnd=true` (as the demos do).

### 6. MEDIUM - Robot.stop() does not cancel the follower
- Where: robot/Robot.java:420-425.
- Scenario: `drivebase.stop()` writes zero power directly, but the follower may still be following or holding, and the caller may keep calling `robot.update()`; Pedro re-applies power. Pedro's motor cache (`lastMotorPowers`, `motorCachingThreshold`) does not know the motors were zeroed externally and can skip identical re-sends. Two owners of the same motors.
- Fix: in `stop()` read the pose, then call `auto.cancelMove()` / `follower.breakFollowing()` before zeroing; never call `update()` after `stop()`.

### 7. LOW/MEDIUM - static `savedPose` can carry a stale pose into a later teleop
- Where: Robot.java `savedPose` (static), `continuingFromAuto`, `stop()` at :421.
- Scenario: any OpMode calling `stop()` (including an earlier TeleOp) overwrites it, so a practice TeleOp inherits the previous pose instead of `FALLBACK_X/Y`. A Pinpoint power cycle also invalidates it.
- Fix: store a timestamp and ignore after a few minutes, or save only from autonomous OpModes; show "using saved pose" in telemetry.

### 8. LOW - AutoDrivebase details
- AutoDrivebase.java:128-140: `moving`, `tangentTheta`, `speed` come from the previous loop's follower state because `beginLoop` runs before `follower.update()`. At a sharp (>90 degree) chain corner the first loop on the new path may classify a forward vector as "reversed" (ProfiledVectorCalculator.java:60-63) and skip the profile for one loop.
- `getFollower()` (:192) is an escape hatch: calling `follower.followPath` directly bypasses `beginMove`, so the profile never starts and the path runs unprofiled.
- ProfiledVectorCalculator.java:65-68: output is cached per `beginLoop`; anything calling `follower.update()` without `AutoDrivebase.update()` would freeze the cached power (Tuning uses its own vanilla follower, so not an issue today).
- `timedOut` (:56, 167) is cleared only by `beginMove`; after `cancelMove()` it stays true until the next move.

### 9. LOW - Tuning.java (stock quickstart)
- Tuning builds a vanilla follower via `Constants.createFollower`, not the profiled calculator. PID values transfer, but "press A to set temporarily" values (`setXVelocity` etc.) must be copied into Constants by hand. `driveConstants` is a shared object, so `PlannedProfile.freeSpeed()` sees a temporary xVelocity during that session only.
- PredictiveBrakingTuner (:776): the DONE state re-runs every loop and prints twice (stock); it only matters once finding 1 is addressed.
- Zero-power-acceleration tuners end on `currentVelocity < getVelocityConstraint()` (0.1 in/s default); a drag-limited robot may never get below it and hang (stock).

### 10. LOW - OdomPods / Constants coupling
- commands/subsystems/odom_pods/OdomPods.java is used only for its static constants; its constructor, which reconfigures the same Pinpoint, is never called. If anyone constructs it next to the follower, the device is configured twice. Consider making it constants-only.
- Offsets are 0/0 (OdomPods.java:17-19) and encoder directions are guesses (FORWARD/FORWARD; Pedro defaults are REVERSED/FORWARD). Robot.placeholderWarnings flags the offsets but not the directions. Verify with Tuning > Localization Test (push forward and sideways; x/y must increase).

### 11. INFO - Drivebase
- Drivebase.java:28 sets BRAKE in the constructor, but Pedro `Mecanum.breakFollowing()` sets FLOAT on every `followPath` and at path end. In an OpMode that runs a path and then drives manually BRAKE is lost. Teleop-only OpModes are unaffected. The comment at Robot.java:136 ("its motor setup wins") is true for direction, not for zero-power behavior after a path.
- Mixer signs and clockwise-positive turn are consistent with `AIM_TURN_SIGN = -1` (Pedro heading is CCW positive).

### 12. INFO - MotionProfile contract vs reality
- The `stockPower < 0` branches in PowerRamp/SlewLimit/PlannedProfile and the "negative = braking" contract in MotionProfile.java:7-9 are dead code: ProfiledVectorCalculator only calls `apply` with a non-negative magnitude. As a result SlewLimitProfile's `previous` (:31-34) is stale after a braking interval, so the slew limit is not re-applied after braking.

## Suggested priority
Findings 1 and 2 before any on-robot run; then 3, 5, 6.
