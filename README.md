# MakEMinds_Sigmaz

FTC robot code for the BIOBUZZ (2026-27) season, built on the FTC SDK and Pedro Pathing. The robot has a
mecanum drivebase (Pedro Pathing, Pinpoint odometry), an intake motor, a transfer motor, a flower mechanism,
a lane gate with a colour sensor, and a shooter (two-motor flywheel, turret on two Axon servos, fixed launch
angle, no hood) that aims with a ballistic solver instead of a lookup table (a measured table is kept as an
alternative). The repo also contains `field_sim/`, a 3D field simulator that uses the same shot physics as the
robot code.

## Requirements

| Tool | Version |
|---|---|
| FTC SDK | 11.0.0 |
| Pedro Pathing | 2.1.0 (`com.pedropathing:ftc`) |
| Panels | 1.0.6 (`com.bylazar:fullpanels`) |
| Android Gradle Plugin | 8.7.0 |
| Gradle (wrapper) | 8.9 |
| JDK | 17 (to run Gradle) |
| IDE | Android Studio (version not pinned in the repo) |

Other build settings (`build.common.gradle`): compileSdk 34, minSdk 24, targetSdk 28, Java 1.8 source level.
Python 3 is only needed for `field_sim`.

## Setup, build, deploy

1. Clone the repo and open the root folder in Android Studio. Let Gradle sync (needs network the first time for
   the FTC, Pedro, Panels and JUnit artifacts).
2. Build: `gradlew assembleDebug` (Windows) or `./gradlew assembleDebug`.
3. Deploy: connect the Control Hub (USB or Wi-Fi ADB), select the `TeamCode` run configuration, and press Run.
4. On the Driver Station, pick an OpMode from the list below.

## Unit tests

Plain JUnit 4 tests in `TeamCode/src/test/java` (no robot or Android runtime needed):

```
gradlew :TeamCode:testDebugUnitTest        # Windows
./gradlew :TeamCode:testDebugUnitTest      # macOS / Linux
```

They cover the shot solver (including golden values from the simulator's JavaScript solver), motion profiles,
field constants, turret aiming math, ball sensing, intake/transfer logic and the Axon servo math.

## OpModes

| Name | Group | Class |
|---|---|---|
| Two Player TeleOp | TeleOp | `commands.teleop.TwoPlayerTeleOp` |
| Drivetrain Test | Test | `commands.teleop.DrivetrainTest` (spin one wheel at a time to check motor directions) |
| Tuning | Pedro Pathing | `pedroPathing.Tuning` (Pedro tuning menu) |

## Settings

Every value you would normally edit (hardware config names, motor and servo directions, powers, servo angles,
soft limits, thresholds, gains, timings and feature switches such as `USE_SHOOTER` or `USE_GATE`) lives in one
file: `TeamCode/src/main/java/robot/RobotSettings.java`.

- The file has one nested class per subsystem, named after it (`RobotSettings.Robot`, `Drivetrain`, `Odometry`,
  `Flywheel`, `Turret`, `BallSensor`, `LaneSorter`, `Limelight`, ...). The header comment lists every section in
  order; search for `class <Name>` to jump to one. Each field has a comment with its units, and `PLACEHOLDER`
  marks values that still have to be measured on the robot.
- The code reads them as `RobotSettings.<Section>.<FIELD>`. Changing a value needs a rebuild and redeploy.
- `RobotSettings.Robot` holds the robot lineup (alliance, which optional hardware is fitted, intake auto-stop).
- Not in this file: Pedro Pathing follower tuning (PIDF, mass, braking) stays in `pedroPathing/Constants.java`,
  where Pedro's tuning OpModes tell you to paste it (motor names, pod offsets and path constraints come from
  `RobotSettings`). Field geometry stays in `commands/field/FieldConstants.java`, the Pedro tuning OpModes in
  `pedroPathing/Tuning.java`, and the motion-profile classes in `commands/autonomous/motion_profiling/` keep
  their own constants. The flywheel safety ceiling `Flywheel.HARD_MAX_RPM` is final in code on purpose.

## Field simulator (`field_sim/`)

A browser-based 3D/2D model of the field and robot. Setup (first time):

```
cd field_sim
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt     # Windows
.venv/bin/python -m pip install -r requirements.txt             # macOS / Linux
```

Run:

```
.venv\Scripts\python.exe run_sim.py     # Windows
.venv/bin/python run_sim.py             # macOS / Linux
```

This serves `http://127.0.0.1:8780` and opens the browser. Flags: `--port` (default 8780), `--host` (default
127.0.0.1), `--no-browser`, and `--export` (rebuild `web/data` from `data/field_model`). See `field_sim/README.md`
for controls. The robot-to-sim bridge has been removed from TeamCode, so the robot no longer posts its state to the sim.

## Project structure (`TeamCode/src/main/java`)

- `commands.autonomous`: `AutoDrivebase` (path following with a motion profile) and `AutoAim` / `AutoAimStep`
  (aim-and-shoot command for autonomous routines).
- `commands.autonomous.motion_profiling`: `MotionProfile` strategies (`NoProfile`, `PowerRampProfile`,
  `SlewLimitProfile`, `TrapezoidProfile`, `SCurveProfile`) and `ProfilePlanner`.
- `commands.field`: `Alliance` and `FieldConstants` (field geometry, hive aim points, AprilTag IDs).
- `commands.subsystems.ball_sensing`: ball color/count tracking logic and sensors.
- `commands.subsystems.drivebase`, `intake`, `transfer`, `gate`, `flower_mech`: mechanism subsystems.
- `commands.subsystems.sorting`: `LaneSorter` (colour sensor drives the lane gate).
- `commands.subsystems.flywheel`: `Flywheel` with a PIDF controller.
- `commands.subsystems.servo`: Axon servo helpers.
- `commands.subsystems.turret`: `Turret` (two Axon servos), soft limits, and `TurretAiming` (setpoints for turret angle, launch elevation, RPM).
- `commands.subsystems.grouped_subsystems`: combined subsystems (`intake_transfer`, `shooter` with `ShotSolver`
  and `ShotTable`).
- `commands.subsystems.limelight`: Limelight 3A vision trim (odometry pod settings are in `RobotSettings.Odometry`).
- `commands.teleop`: driver-controlled OpModes (`TwoPlayerTeleOp`, `DrivetrainTest`).
- `pedroPathing`: Pedro `Constants` (follower tuning; drive and localizer values read from `RobotSettings`) and
  the `Tuning` OpMode.
- `robot`: `RobotSettings` (all editable values, see Settings above), `Robot` (wires the subsystems together;
  the feature switches `USE_SHOOTER`, `USE_LIMELIGHT`, `USE_BALL_TRACKER`, `USE_FLOWER_MECH`, `USE_GATE`,
  `USE_COLOR_SORT` are in `RobotSettings.Robot`) and `HardwareCheck`. `Robot.stop()` at the end of an autonomous
  saves the pose; `TwoPlayerTeleOp` starts from it once (else from `RobotSettings.TeleOp.FALLBACK_*`).

## License

See `LICENSE`.
