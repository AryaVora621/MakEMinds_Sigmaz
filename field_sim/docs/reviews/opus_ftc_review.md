# TeamCode review (Opus, read-only)

Scope: `TeamCode/src/main/java` (Pedro Pathing 2.1.0, FTC SDK). No source files were edited. I checked the Pedro 2.1.0
sources (Follower, VectorCalculator, PinpointLocalizer, PathConstraints) against how they are used here.

Math I checked and found correct (so nobody re-checks it): the ShotSolver ENTRY_ANGLE, HOOD_ANGLE, SPEED and
MIN_SPEED quartics, the depressed-cubic critical points, the Cauchy root bound, the descending-arrival filter
(`gT^2/2 > h`) and the forward-sim check. The TurretAiming `fromBehind` arrival-velocity test. The FlywheelLQR
Kalman predict and update algebra (F P F^T + Q, (I-KH)M) and the scalar DARE. The ProfilePlanner trapezoid and
S-curve. The Pinpoint offset mapping (`forwardPodY`/`strafePodX` match `setOffsets(x,y)`). Pinpoint velocity
frame: PedroCoordinates is the identity, so `follower.getVelocity()` is in the field frame and the shoot-on-the-move
input is in the right frame. The mecanum mixing and the turn sign. `AIM_TURN_SIGN = -1`.
`PathConstraints(0.99, 100, 1, 1)` is the 4-argument (tValue, timeout, brakingStrength, brakingStart)
overload, not the velocity/translational/heading one, so it is fine.

Findings are ranked most severe first.

---

## 1. HIGH: the turret zero is lost between autonomous and teleop under the default settings

Files: `commands/subsystems/turret/Turret.java:40-42, 91-122, 165-171`, `robot/Robot.java:420-432`,
`commands/teleop/TwoPlayerTeleOp.java:54`

- `ZERO_MODE = ZERO_AT_INIT` and `USE_PERSISTED_ANGLE = false` are the defaults.
- At the end of auto, `Robot.stop()` calls `Turret.stop()`, which only saves the angle and sets power 0 (BRAKE).
  Nothing drives the turret back to 0 before auto ends.
- The teleop `Turret` constructor runs `STOP_AND_RESET_ENCODER` and, in ZERO_AT_INIT mode, takes wherever the turret
  is as 0 deg.

Failure scenario: `ShootWhileDrivingDemoAuto` (or any real auto) ends with the turret tracking the hive at, say,
+70 deg. In teleop:
- Every aim command is off by 70 deg.
- The soft limits (`Turret360Limits` +/-180) are now centred on the wrong place, so the turret can be driven
  250 deg from the real mechanical centre. On a cable-limited turret that can rip the wiring.
- The init telemetry tells the driver to "place turret at mechanical centre, then press BACK". Nobody does that
  between auto and teleop in a match.

Fix (any one):
- Make `ZERO_MODE = PERSIST_FROM_LAST_OPMODE` the default. The code path already exists and is unit tested.
- Have the autos command the turret to 0 and wait for `atTarget()` before `robot.stop()`.
- At least have `Robot.continuingFromAuto` force persist mode when `savedPose != null`, because that is exactly the
  auto-to-teleop case.

## 2. MEDIUM-HIGH: the FlywheelLQR Kalman filter is fed the unsaturated voltage, so the disturbance estimate winds up during spin-up

File: `commands/subsystems/flywheel/FlywheelLQR.java:63, 86-90`

`lastVoltsModel = voltsModel` stores the command before the `+kS`, before `/batteryVoltage` and before the [-1, 1]
clamp. With the default constants:
- ad = 0.842 and bd = 36.7 at dt = 0.02. The DARE gives k of about 0.022 V per (tick/s).
- A spin-up from 0 to 4000 RPM (1867 ticks/s) asks for about 41 + 8 = 49 V. The motor gets at most about 12 V.

Every loop during spin-up:
- The predicted velocity is far above the measured one, so the innovation is strongly negative.
- `k1 > 0`, so `xDist` is driven down to the -4 V clamp (`MAX_DISTURBANCE`).
- When the wheel nears the target, the controller adds `-xDist = +4 V`, which is about 930 ticks/s of extra
  steady-state push.
- The result is a large overshoot and slow settling, and `atTarget()` (50 ticks/s) is delayed on every shot recovery.

Fix: feed the predictor the voltage actually applied, `clamp(volts, -V_batt, V_batt) - kS*sgn(target)`. Optionally
freeze the disturbance update (`k1 = 0`) while the output is saturated. This is not the default controller (PIDF is),
so it only bites when `FLYWHEEL_CONTROLLER = LQR`.

## 3. MEDIUM: +/-180 deg turret limits leave no overlap, so the turret makes a full 360 deg unwind whenever the target crosses directly behind

Files: `commands/subsystems/turret/TurretAiming.java:238-257`, `Turret360Limits.java:6-7`, `TurretLimits.java:8`

With `MODE = FULL_360` the window is exactly [-180, 180]. For any phi, only one of `phi + 360k` lies inside it, so the
"closest to current" search never has a choice.

Failure scenario:
- The robot drives around the hive, and the bearing to the target goes from 179 deg to -179 deg.
- The only valid candidate is -179, so the turret swings 358 deg at `MAX_TARGET_VELOCITY` (360 deg/s). That is about
  1 s with `ready()` false.
- If odometry noise or vision trim jitters phi around +/-180, the turret whips back and forth repeatedly.
- With BACKUP_170 there is a 20 deg dead zone where the aim is clamped and flagged unreachable, which is fine.

Fix:
- If the cable allows it, make the window wider than 360 (for example -200..+200) so the closest-candidate logic has
  real hysteresis.
- Otherwise add a rule to hold at the current limit while the requested angle is within N deg of the other limit,
  and only unwind when the robot is not about to shoot.

## 4. MEDIUM: Flywheel MIN_RPM silently raises the shot RPM, but the solution still reports reachable and ready()

Files: `commands/subsystems/flywheel/Flywheel.java:21, 59-63`, `ShooterBase.java:90-93, 105-117`,
`TurretAiming.java:262-267`

`setTargetVelocity` raises any nonzero target to `MIN_RPM = 2500`. `TurretAiming` only flags rpm > MAX_RPM.

Worked example with the defaults (exit 12 in, target z 58.3 in, R 1.89 in, efficiency 0.45, 1 in/s = 11.23 RPM):
- A 50 deg entry shot at d = 36 in needs about 206 in/s, which is about 2310 RPM.
- A hood-clamped 65 deg shot at d = 72 in needs about 2560 RPM.

So near and mid shots land in or close to the clamp band. The flywheel runs at 2500 instead of 2310, then
`flywheel.atTarget()` is true, `ready()` is true, and the ball overshoots.

Fix: in `TurretAiming.solve` set `reachable = false` with a reason when `rpm < Flywheel.MIN_RPM`. Or treat MIN_RPM
only as an idle-spin floor and never apply it to a shot setpoint.

## 5. MEDIUM: the vision trim fights the shoot-on-the-move lead, and as a P-only trim it removes only 1/3 of a static error

File: `commands/subsystems/turret/TurretAiming.java:233-237`

The trim is `phi += clamp(-1 * 0.5 * tx, +/-5)` on top of the physics azimuth.

- With the camera boresighted on the turret, tx = turret - lineOfSight. The steady state is
  `turret = (2*phi + LOS)/3`.
- Moving: phi deliberately leads away from the line of sight (the solver's lateral-velocity correction). The trim
  drags the turret about a third of the way back toward the tag (capped at 5 deg), so the trim makes moving shots
  worse.
- Static: if the model is off by e, only e/3 is removed.
- The tags sit on the underside of the cells, not at the opening centroid the solver aims at, so tx = 0 is not even
  the right aim at short range.

Fix:
- Apply the trim only when `!s.compensating`.
- Compare tx with the expected bearing of the tag the camera is locked on (`phi_expected_tag - turret`) instead of 0.
- Integrate a slow bias (I-only, rate-limited) instead of adding P*tx every loop.

Not active by default (`Robot.USE_LIMELIGHT = false`), but `USE_VISION_TRIM = true`.

## 6. MEDIUM: the ball count drifts when ejecting, which can latch the intake full-stop

Files: `commands/subsystems/ball_sensing/BallTrackerLogic.java:57-93`, `Robot.java:199`, `IntakeTransfer.java:80-88`

- While the intake runs backward, intake rising edges are ignored (good), but nothing ever decrements. Balls pushed
  out through the intake leave the count unchanged.
- A ball parked at the shooter sensor that is pulled back by the reverse gives a shooter falling edge, which is
  counted as a shot (decrement plus shot event, so it is also logged as a shot). That ball is still in the robot.

Scenario with `AUTO_STOP_INTAKE_WHEN_FULL`: the driver intakes 3 (count 3, full), sees a wrong-colour ball, reverses
and spits out 2. The count stays 3 (or 2 if the shooter ball moved), and forward intake stays blocked until gp2 BACK.

Fix:
- While reversing, decrement on intake falling edges.
- Ignore shooter falling edges unless the transfer is moving forward (feeding).

## 7. MEDIUM-LOW: TurretADRC (and FlywheelLADRC at low battery) feed the observer a command the motor never received

Files: `commands/subsystems/turret/TurretADRC.java:61-65`, `commands/subsystems/flywheel/FlywheelLADRC.java:47-50`

Turret: `lastU` is clamped to `MAX_VOLTS = 12`, but the output is then clamped to `MAX_POWER = 0.8`, which is about
10 V at 12.5 V. During large slews the ESO thinks it applied 12 V and gets only about 10 V. `z3` absorbs a false
-2*B0 disturbance, so the turret overshoots when it comes out of saturation. The class doc says "the observer is fed
the saturated command (anti-windup)", but that is not true here.

Flywheel LADRC: the same thing happens whenever the battery sags below 12 V, which is common during spin-up.

Fix: `lastU = clamp(u, -MAX_POWER*V_batt, MAX_POWER*V_batt)` (turret), or `clamp(u, -V_batt, V_batt)` (flywheel).

## 8. LOW-MEDIUM: the HoodReadyTimer never times out while the hood setpoint is continuously re-solved

Files: `commands/subsystems/grouped_subsystems/shooter/HoodReadyTimer.java:20-24`, `ShooterBase.java:86, 99`

The timer restarts whenever the target changes by more than 1e-9 deg. In PHYSICS mode the hood target changes every
loop while the robot moves or odometry drifts. So with a stuck or unreachable hood, `ready()` waits forever, which
defeats the stated purpose of `HOOD_READY_TIMEOUT_S`.

Fix: restart only when the target moves more than a few times `AT_TARGET_TOL_DEG` from the target that started the
timer.

## 9. LOW-MEDIUM: FlowerMech can stay in DEPLOYING or STOWING forever with trusted feedback

Files: `commands/subsystems/flower_mech/FlowerMechLogic.java:117-134`, `AxonServo.java:36, 190-200`,
`FlowerMech.java:38`

`reached` needs `atTarget` (3 deg), but a stall needs an error above `STALL_TOLERANCE_DEG` (10 deg). If a servo
settles 3 to 10 deg off (load, a slightly wrong calibration, the arm resting on a flower), the state never advances,
never faults, and `isBusy()` stays true, so any auto waiting on it hangs. This is only reachable once
`FEEDBACK_CALIBRATED = true`.

Fix: add a move timeout (for example span/slew + 0.5 s) that either advances the state or faults.

## 10. LOW: IntakeTransfer halts forward when full but never resumes by itself

File: `IntakeTransfer.java:80-88`

`update()` stops the motors but leaves `mode = FORWARD`. When the count drops (a ball shot), the motors stay off
unless the caller calls `forward()` again. TwoPlayerTeleOp calls `forward()` every loop, so it works there. Any auto
that calls `forward()` once and then waits will stall.

Fix: in `update()`, call `intake.forward()/transfer.forward()` when `mode == FORWARD` and running is allowed again.

## 11. LOW: TABLE shot source aims from the robot centre, ignores velocity and skips the from-behind check

File: `TurretAiming.java:138-156`

- The distance and bearing use (x, y) of the robot centre, not the turret pivot, but the ShotTable doc says "distance
  from the turret". With a nonzero `PIVOT_OFFSET` the turret angle is off by about offset/d (for example 3 deg at
  6 in / 108 in).
- Robot velocity is ignored (`compensating` stays false). That is acceptable for a table, but undocumented.
- `fromBehind` is never evaluated.

Fix: rotate the pivot offset into the field frame the way ShotSolver does, and evaluate the normal test with a
zero-velocity model.

## 12. LOW: one static tunable set is shared by every Axon servo

File: `commands/subsystems/servo/AxonServo.java:23-50`

`FEEDBACK_CALIBRATED`, `FEEDBACK_V_MIN/MAX`, `SLEW_DEG_PER_S` and the stall thresholds are static, so they apply to
the hood, gate, flower A and flower B together. Calibrating the hood and setting `FEEDBACK_CALIBRATED = true` starts
trusting the uncalibrated gate and flower feedback too. That can create false stalls or sync faults, or the stuck
state from finding 9.

Fix: make these per-instance (constructor arguments or per-subsystem statics).

## 13. LOW: FlywheelPIDF integral bookkeeping

File: `FlywheelPIDF.java:33-36`

- The integral accumulates without bound while `kI == 0`. Setting kI live from the Dashboard therefore applies an
  instant +/-`I_MAX_VOLTS` step.
- With kI != 0 there is no conditional integration, so the integral winds up to the clamp during every spin-up and
  then overshoots.

Fix: integrate only when `kI != 0` and the output is not saturated.

## 14. LOW: demo autos do not stop the robot in a finally block

Files: `ShootWhileDrivingDemoAuto.java:53-77`, `ProfileDemoAuto.java:45-66`

`robot.stop()` runs only if the loop exits normally. An exception means no pose save and no turret-angle save (see
finding 1), and the flywheel keeps its last power until the SDK stops the OpMode. TwoPlayerTeleOp already uses
try/finally, so do the same here.

## 15. LOW: Pedro Constants are library defaults

File: `pedroPathing/Constants.java:25`

- `FollowerConstants` is all defaults: mass 10.65 kg (used by the centripetal correction), xVelocity and yVelocity,
  zero-power accelerations and all PIDFs.
- The `OdomPods` offsets are 0, and pod directions are FORWARD/FORWARD.
- The class comment and `placeholderWarnings()` already say this. Listed so it is not forgotten: until the robot is
  weighed and tuned, the planned profiles' `kV = 1/xVelocity` uses the default xVelocity too
  (`PlannedProfile.java:150-152`).

## 16. Nits

- `ShotSolver.wrap` (`ShotSolver.java:192-199`) returns [-pi, pi) (pi maps to -pi), but the docs say (-pi, pi].
  This is harmless with symmetric limits.
- No controller guards `batteryVoltage <= 0`. A 0 V read makes `output * 12 / 0` either NaN (0/0) or +/-Inf, and NaN
  passes straight through `Math.max/min` into `setPower`. A one-line `Math.max(V, 6)` guard costs nothing (all turret
  and flywheel controllers).
- `ShotTable.interpolate` assumes ascending `DISTANCE_IN`. Unsorted rows silently return the wrong values.
- `BallSensor` debounce: two balls closer than `FALL_MS` (60 ms) at a sensor merge into one edge, so a rapid feed
  under-counts. Check this when tuning the feed rate.

---

## SimBridge and Robot sim wiring (reviewed last: another worker is editing these files now, so the line numbers may already be stale)

Files: `commands/sim/SimBridge.java`, `commands/sim/SimJson.java`, `robot/Robot.java:135, 217-246`

- S1 (low): the sim pose override is applied after `shooter.update()` (`Robot.java:214` vs `:219-221`), so the
  shooter always aims from the previous loop's pose. Move the override before `auto.update()`, or straight after it.
- S2 (low): `st.intakeOn = st.feeding` (`Robot.java:235`) reports the intake as off whenever the driver intakes with
  the bumpers. IntakeTransfer needs a mode accessor (FORWARD/BACKWARD/FEED).
- S3 (low): the reverse-channel `simIn.balls` is only echoed back in the published state. The tracker never receives
  it, so "hardware-less ball testing" does not exercise the tracker or full-stop logic.
- S4 (low): `SimJson.parseInput` looks up `"balls"` anywhere after `"input"`, not inside the input object, so a reply
  that carries a later `"balls"` field (for example an echoed state) is accepted as input.
- S5 (low): `publishPath` skips the BUSY gate, so while the sim is slow the single-thread executor queue grows by one
  path post per `followPath`. It is bounded by the number of moves, so this is not dangerous.
- S6 (info): `latestInput` and `latestInputMs` are two separate volatile writes. A reader can pair a new input with
  the old timestamp, which at worst drops one sample. Harmless.
- S7 (info): the safety gating (USE_SIM_BRIDGE && ARM_FOR_PRACTICE && HOST), the timeouts, the reply cap, the backoff
  and the daemon thread all look right. `auto.setPose` on a real robot writes the Pinpoint over I2C every loop while
  the reverse channel is on. That is acceptable for practice only, which matches the comments.
