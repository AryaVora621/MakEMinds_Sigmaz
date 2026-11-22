package robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import commands.field.Alliance;
import commands.subsystems.ball_sensing.BallSensor.SensorType;
import commands.subsystems.turret.Turret.ZeroMode;
import commands.subsystems.turret.TurretAiming.ShotSource;

/**
 * ALL robot settings in ONE file. Edit values here (config names, motor/servo directions, powers, speeds,
 * servo angles, limits, thresholds, gains, timings, feature switches); the subsystem classes read them from here.
 *
 * HOW TO USE
 *   - Find the section for the subsystem (Drivetrain, Flywheel, Turret, ...). Each field has a doc comment with
 *     its units. "PLACEHOLDER" means the value is a guess that still has to be measured on the robot.
 *   - Fields are public static and NOT final on purpose: a few are changed while the robot runs (for example the
 *     alliance chosen at init) and the unit tests change some.
 *   - Changes need a rebuild and redeploy (these are not live-editable from a dashboard).
 *   - Pedro Pathing's own follower tuning values (PIDF, mass, braking) stay in pedroPathing/Constants.java
 *     because Pedro's tuning OpModes tell you to paste them there; everything else for Pedro (motor names and
 *     directions, odometry pods, path constraints) is read from this file.
 *   - Safety ceilings that must not be edited stay in code: Flywheel.HARD_MAX_RPM (5000).
 *   - Every setting exists once. Notes on two that used to be copied: the shot source is TurretAiming.SHOT_SOURCE;
 *     intake auto-stop is Robot.AUTO_STOP_INTAKE_WHEN_FULL.
 *     Teleop started after autonomous also sets Turret.ZERO_MODE = PERSIST_FROM_LAST_OPMODE.
 *   - Kept in their own files on purpose: field geometry (commands/field/FieldConstants.java), Pedro tuning
 *     OpModes (pedroPathing/Tuning.java) and the motion-profile constants (commands/autonomous/motion_profiling/).
 *
 * SECTIONS: Robot, TeleOp, Drivetrain, Odometry, PathFollowing, AutoAim, Intake, Transfer, IntakeTransfer,
 *           Flywheel, FlywheelPIDF, Shooter, ShotTable, TurretAiming, Turret, TurretLimits, AxonServo,
 *           FlowerMech, Gate, BallSensor, HueClassifier, BallTracker, LaneSorter, Limelight.
 */
public final class RobotSettings {
    private RobotSettings() {}

    // ======================================================================
    // Robot
    // ======================================================================
    /** Top-level robot lineup: alliance, which optional hardware exists, shot-source choice, firing steadiness and logging. */
    public static final class Robot {
        private Robot() {}

        // ---- lineup ----
        /** Which alliance we shoot for (BLUE or RED). Changed at runtime by the INIT X/B buttons. */
        public static Alliance ALLIANCE = Alliance.BLUE;
        /** false = drive-only robot (no shooter hardware is touched). */
        public static boolean USE_SHOOTER = true;
        /** true = use the Limelight tx to trim the turret. */
        public static boolean USE_LIMELIGHT = false;
        /** Hive tip state: true = the far-side cell of our hive is raised (the target). Toggled at runtime by gamepad2 D-pad. */
        public static boolean FAR_CELL_UP = true;

        // ---- fire steadiness ----
        /** Only feed a ball while the robot's filtered acceleration is below this (in/s^2); 0 to disable. */
        public static double FIRE_MAX_ACCEL_IN_S2 = 40;
        /** Smoothing of the acceleration estimate (0..1, higher = faster but noisier). */
        public static double ACCEL_FILTER_ALPHA = 0.25;

        // ---- diagnostics ----
        /** Show the list of unmeasured constants still at their defaults in telemetry. */
        public static boolean SHOW_PLACEHOLDER_WARNINGS = true;
        /** Append one CSV line per shot to FIRST_FOLDER/shotlog/shots_DATE.csv (disables itself on any error). With the tracker on it logs on shooter-sensor shot events (needs BallTracker.USE_SHOOTER_SENSOR); without it, on the rising edge of feeding, at most once per SHOT_LOG_MIN_INTERVAL_MS. */
        public static boolean SHOT_LOG_ENABLED = false;
        /** Without the ball tracker: minimum milliseconds between two shot-log lines (feed rising edges closer than this are not logged). */
        public static long SHOT_LOG_MIN_INTERVAL_MS = 300;

        // ---- optional hardware (all default false because hardwareMap.get throws if it is not configured) ----
        /** Construct a BallTracker (colour sensors) and feed it to the intake each loop. It expects two sensors (intakeColor, shooterColor); with only one, set BallTracker.USE_INTAKE_SENSOR / USE_SHOOTER_SENSOR to false for the missing one. */
        public static boolean USE_BALL_TRACKER = false;
        /** Requires USE_BALL_TRACKER: stop the intake when the tracker reports full (needs BallTracker.USE_SHOOTER_SENSOR to engage). */
        public static boolean AUTO_STOP_INTAKE_WHEN_FULL = false;
        /** Construct the flower mechanism. */
        public static boolean USE_FLOWER_MECH = true;
        /** Construct the gate. */
        public static boolean USE_GATE = false;
        /** Construct the lane colour sorter (the left-wall colour sensor drives the gate). Needs USE_GATE. */
        public static boolean USE_COLOR_SORT = false;
    }

    // ======================================================================
    // TeleOp
    // ======================================================================
    /** Two-player teleop: fallback start pose and gamepad thresholds. */
    public static final class TeleOp {
        private TeleOp() {}

        // ---- start pose (used if autonomous did not run first) ----
        /** Fallback start X in inches (Pedro coordinates). PLACEHOLDER: set to your teleop start. */
        public static double FALLBACK_X_IN = 72;   // PLACEHOLDER
        /** Fallback start Y in inches (Pedro coordinates). PLACEHOLDER: set to your teleop start. */
        public static double FALLBACK_Y_IN = 12;   // PLACEHOLDER
        /** Fallback start heading in radians (Pedro coordinates). PLACEHOLDER. */
        public static double FALLBACK_HEADING_RAD = Math.PI / 2;   // PLACEHOLDER

        // ---- gamepad ----
        /** Gamepad2 right trigger value (0..1) above which it counts as pressed (fire / feed). */
        public static double TRIGGER_PRESS_THRESHOLD = 0.5;
    }

    // ======================================================================
    // Drivetrain
    // ======================================================================
    /** Four-motor mecanum drivetrain (also feeds Pedro's driveConstants in pedroPathing.Constants). */
    public static final class Drivetrain {
        private Drivetrain() {}

        // ---- hardware names ----
        /** Config name of the front-left drive motor. */
        public static String FRONT_LEFT_NAME = "frontLeft";
        /** Config name of the front-right drive motor. */
        public static String FRONT_RIGHT_NAME = "frontRight";
        /** Config name of the back-left drive motor. */
        public static String BACK_LEFT_NAME = "backLeft";
        /** Config name of the back-right drive motor. */
        public static String BACK_RIGHT_NAME = "backRight";

        // ---- directions (flip one if that wheel spins backwards; check with the Drivetrain Test OpMode) ----
        /** Front-left motor direction. */
        public static DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        /** Front-right motor direction. */
        public static DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        /** Back-left motor direction. */
        public static DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        /** Back-right motor direction. */
        public static DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

        // ---- behaviour ----
        /** What the wheels do at zero power: BRAKE holds position, FLOAT coasts. Set by Drivebase at init; note Pedro's follower switches the drive motors to FLOAT whenever it starts or breaks a path (followPath, holdPoint, Robot.stop). */
        public static DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

        // ---- driver input ----
        /** Log stick curve strength (must be > 0, unitless). Higher = MORE sensitive near the centre (0.1 stick gives about 0.28 power at 9). */
        public static double LOG_STRENGTH = 9;

        // ---- Drivetrain Test OpMode ----
        /** Power (0..1) applied to a single wheel while A/B/X/Y is held in the Drivetrain Test OpMode. */
        public static double TEST_POWER = 0.3;
    }

    // ======================================================================
    // Odometry
    // ======================================================================
    /**
     * goBILDA Pinpoint with two odometry pods (read by pedroPathing.Constants). Parallel pod points along the
     * robot's forward axis; perpendicular pod points along the strafe axis. Offsets are measured from the
     * robot's tracking point (center of rotation).
     */
    public static final class Odometry {
        private Odometry() {}

        // ---- hardware ----
        /** Config name of the Pinpoint computer. */
        public static String HARDWARE_NAME = "pinpoint";
        /** Pod hardware used (sets ticks per mm). */
        public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        // ---- pod offsets ----
        /** Parallel pod's sideways offset from the tracking point, inches. Left of center = positive. PLACEHOLDER: unmeasured. */
        public static double PARALLEL_POD_OFFSET_IN = 0;
        /** Perpendicular pod's forward offset from the tracking point, inches. Ahead of center = positive. PLACEHOLDER: unmeasured. */
        public static double PERPENDICULAR_POD_OFFSET_IN = 0;

        // ---- encoder directions ----
        /** Parallel pod encoder direction (must increase when the robot drives forward). */
        public static GoBildaPinpointDriver.EncoderDirection PARALLEL_POD_DIRECTION =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        /** Perpendicular pod encoder direction (must increase when the robot strafes left). */
        public static GoBildaPinpointDriver.EncoderDirection PERPENDICULAR_POD_DIRECTION =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    // ======================================================================
    // PathFollowing
    // ======================================================================
    /** Pedro path constraints and the autonomous drivebase loop guard (follower gains stay in pedroPathing.Constants). */
    public static final class PathFollowing {
        private PathFollowing() {}

        // ---- Pedro PathConstraints ----
        /** Path completion t-value (0..1) at which Pedro considers a path finished. */
        public static double TVALUE_CONSTRAINT = 0.99;
        /** Milliseconds Pedro keeps correcting after a path is parametrically finished (Pedro's timeoutConstraint). */
        public static double TIMEOUT_CONSTRAINT_MS = 100;
        /** Braking strength multiplier (unitless, 1 = default). */
        public static double BRAKING_STRENGTH = 1;
        /** Braking start multiplier (unitless, 1 = default). */
        public static double BRAKING_START = 1;

        // ---- AutoDrivebase ----
        /** Longest loop time (seconds) the motion profile will believe; guards against a one-off hiccup. */
        public static double MAX_DT_S = 0.1;
    }

    // ======================================================================
    // AutoAim
    // ======================================================================
    /** Tunables for the autoAim autonomous command (alignment tolerances stay in Flywheel and Turret). */
    public static final class AutoAim {
        private AutoAim() {}

        // ---- timing ----
        /** Upper bound, seconds, on how long autoAim waits for the flywheel and turret to be ready, so auto never hangs. PLACEHOLDER: not tuned on robot. */
        public static double TIMEOUT_S = 4.0;   // matches sim AUTOAIM_DEFAULTS.timeoutSec (was 3.0)
        /** With shootAfter, the maximum seconds the command keeps tracking and feeding before it returns. PLACEHOLDER: not tuned on robot. */
        public static double SHOOT_AFTER_MAX_S = 5.0;

        // ---- behaviour ----
        /** If true, transfer turns on as soon as the command starts (matches the sim); if false, only once aligned. PLACEHOLDER: behaviour choice, not tuned on robot. */
        public static boolean TRANSFER_ON_START = false;
    }

    // ======================================================================
    // Intake
    // ======================================================================
    /** Intake motor (single DC motor, runs forward/backward). */
    public static final class Intake {
        private Intake() {}

        // ---- hardware ----
        /** Config name of the intake motor in the Robot Controller configuration. */
        public static String MOTOR_NAME = "intake";
        /** True reverses the motor direction (default FORWARD). */
        public static boolean MOTOR_REVERSED = false;
        /** True = BRAKE when power is 0; false = FLOAT (default). */
        public static boolean ZERO_POWER_BRAKE = false;

        // ---- power ----
        /** Motor power used by forward()/backward(), fraction 0..1. */
        public static double POWER = 1.0;
    }

    // ======================================================================
    // Transfer
    // ======================================================================
    /** Transfer motor (single DC motor, runs forward/backward). */
    public static final class Transfer {
        private Transfer() {}

        // ---- hardware ----
        /** Config name of the transfer motor in the Robot Controller configuration. */
        public static String MOTOR_NAME = "transfer";
        /** True reverses the motor direction (default FORWARD). */
        public static boolean MOTOR_REVERSED = false;
        /** True = BRAKE when power is 0; false = FLOAT (default). */
        public static boolean ZERO_POWER_BRAKE = false;

        // ---- power ----
        /** Motor power used by forward()/backward(), fraction 0..1. */
        public static double POWER = 1.0;
    }

    // ======================================================================
    // IntakeTransfer
    // ======================================================================
    /** Intake and transfer kept in sync (full-stop and feed-confirmation switches). */
    public static final class IntakeTransfer {
        private IntakeTransfer() {}

        // ---- feature switches ----
        // Full-stop of the intake is chosen by RobotSettings.Robot.AUTO_STOP_INTAKE_WHEN_FULL (needs the ball tracker).
        /** When true, isFeedConfirmed() only returns true while feeding and ballAtShooter() was seen since the feed began. */
        public static boolean CONFIRM_FEED_WITH_SHOOTER_SENSOR = false;
    }

    // ======================================================================
    // Flywheel
    // ======================================================================
    /** Two-motor flywheel: config names, directions, speed limits and encoder scaling (PIDF gains are in FlywheelPIDF). */
    public static final class Flywheel {
        private Flywheel() {}

        // ---- hardware ----
        /** Config name of flywheel motor A (its encoder is read). */
        public static String MOTOR_NAME = "flywheel";
        /** Reverse motor A's direction. */
        public static boolean MOTOR_REVERSED = false;
        /** Config name of the second flywheel motor. */
        public static String MOTOR_B_NAME = "flywheel2";          // PLACEHOLDER
        /** Reverse motor B so both motors push the wheel the same way for the same power. Check on the robot. */
        public static boolean MOTOR_B_REVERSED = true;            // PLACEHOLDER
        /** true = motors FLOAT (coast) at zero power; false = BRAKE. */
        public static boolean ZERO_POWER_FLOAT = true;

        // ---- sensing ----
        /** true = average both encoders (only if motor B's encoder is plugged in); false = motor A's encoder only. */
        public static boolean VELOCITY_FROM_BOTH = false;         // PLACEHOLDER
        /** Encoder ticks per flywheel revolution (include any gearing). */
        public static double TICKS_PER_REV = 28;

        // ---- limits / tolerance ----
        /** Any nonzero target is raised to at least this (RPM). */
        public static double MIN_RPM = 2500;
        /** Flywheel counts as at speed when within this of the target (ticks/s). */
        public static double AT_TARGET_TOLERANCE = 50;
    }

    // ======================================================================
    // FlywheelPIDF
    // ======================================================================
    /** Flywheel velocity PIDF gains: V = kS*sgn(r) + kV*r + kA*r_dot + kP*e + kI*integral(e) + kD*de/dt, power = V / battery volts. */
    public static final class FlywheelPIDF {
        private FlywheelPIDF() {}

        // ---- feedforward ----
        /** Volts to just overcome static friction. */
        public static double kS = 0.10;
        /** Volts per (tick/s) of target velocity. */
        public static double kV = 0.0043;
        /** Volts per (tick/s^2) of target acceleration. */
        public static double kA = 0.0005;

        // ---- feedback ----
        /** Volts per (tick/s) of error. */
        public static double kP = 0.005;
        /** Volts per (tick/s * s); leave 0 unless needed. */
        public static double kI = 0.0;
        /** Volts per (tick/s^2); usually 0 (velocity is noisy). */
        public static double kD = 0.0;
        /** Clamp on the integral term's voltage contribution (V). */
        public static double I_MAX_VOLTS = 2.0;

        // ---- voltage ----
        /** Used instead of the battery reading when that reading is missing (<= 1 V or NaN), volts. */
        public static double NOMINAL_VOLTAGE = 12.0;
    }

    // ======================================================================
    // Shooter
    // ======================================================================
    /** Shared shooter logic (ShooterBase). */
    public static final class Shooter {
        private Shooter() {}

        // ---- tolerances ----
        /** Chassis heading must be within this of the aim direction to count as aimed (turretless robots), degrees. */
        public static double HEADING_TOLERANCE_DEG = 3;
    }

    // ======================================================================
    // ShotTable
    // ======================================================================
    /** Measured distance -> flywheel RPM table (used when TurretAiming.SHOT_SOURCE = TABLE). Linear interpolation, flat outside. */
    public static final class ShotTable {
        private ShotTable() {}

        // ---- table (same number of rows in both arrays, nearest first) ----
        /** Distance from the turret to the aim point (inches), nearest first. PLACEHOLDER. */
        public static double[] DISTANCE_IN = {36, 108};
        /** Flywheel RPM at each distance. PLACEHOLDER. */
        public static double[] RPM = {2800, 3800};
    }

    // ======================================================================
    // TurretAiming
    // ======================================================================
    /** Shot aiming physics: geometry, launch elevation, speed-to-RPM calibration, latency, shoot-on-the-move, vision trim. */
    public static final class TurretAiming {
        private TurretAiming() {}

        // ---- shot source ----
        /** PHYSICS = ballistic solver, TABLE = measured distance table (ShotTable). This is the ONE place to choose it. */
        public static ShotSource SHOT_SOURCE = ShotSource.PHYSICS;

        // ---- geometry ----
        /** Turret pivot offset from robot centre, robot frame +x forward (inches). */
        public static double PIVOT_OFFSET_X_IN = 0;
        /** Turret pivot offset from robot centre, robot frame +y left (inches). */
        public static double PIVOT_OFFSET_Y_IN = 0;
        /** Ball exit height above the tiles (inches). PLACEHOLDER: measure. */
        public static double EXIT_HEIGHT_IN = 12;                 // PLACEHOLDER

        // ---- launch elevation ----
        /** Fixed launch elevation of the shooter (degrees above horizontal); there is no hood. */
        public static double FIXED_LAUNCH_ELEVATION_DEG = 45;
        /** Accept shots that reach the target height still rising. Normally false. */
        public static boolean ALLOW_RISING_ARRIVAL = false;

        // ---- speed -> RPM (drag-free model, calibrate RPM_SCALE on the robot) ----
        /** Flywheel wheel radius (inches). PLACEHOLDER: measure. */
        public static double WHEEL_RADIUS_IN = 1.89;              // PLACEHOLDER
        /** Fraction of wheel surface speed that ends up as ball speed (0..1). PLACEHOLDER: calibrate. */
        public static double TRANSFER_EFFICIENCY = 0.45;          // PLACEHOLDER
        /** Multiplier on the computed RPM; tune until shots land. */
        public static double RPM_SCALE = 1.0;
        /** Highest RPM the flywheel may run; a shot needing more is flagged unreachable. Values above Flywheel.HARD_MAX_RPM (5000, in the Flywheel class, not tunable) are ignored: see Flywheel.maxRpm(). */
        public static double MAX_RPM = 5000;

        // ---- latency compensation ----
        /** Advance pose by velocity * LATENCY_S before solving. */
        public static boolean USE_LATENCY_COMPENSATION = true;
        /** Odometry loop + turret lag (seconds). Measure. */
        public static double LATENCY_S = 0.05;

        // ---- shoot on the move ----
        /** Off = aim as if the robot were standing still. */
        public static boolean USE_SHOOT_ON_THE_MOVE = true;
        /** Below this speed the robot counts as stationary (in/s). */
        public static double MIN_MOVE_SPEED_IN_S = 4;

        // ---- vision trim (camera on the turret, boresighted: tx IS the turret error) ----
        /** Apply camera tx trim to the turret angle. */
        public static boolean USE_VISION_TRIM = true;
        /** Degrees of turret trim per degree of tx. */
        public static double VISION_KP = 0.5;
        /** Sign applied to tx; flip to +1 if a robot test shows the opposite. */
        public static double VISION_TX_SIGN = -1;
        /** A bad read can never move the turret more than this (degrees). */
        public static double MAX_VISION_TRIM_DEG = 5;
    }

    // ======================================================================
    // Turret
    // ======================================================================
    /** Two-Axon-servo turret: hardware names, directions, centre, estimated speed and start-angle (zero) mode. */
    public static final class Turret {
        private Turret() {}

        // ---- hardware ----
        /** Config name of turret servo A. */
        public static String SERVO_A_NAME = "turretServoA";          // PLACEHOLDER
        /** Config name of turret servo B. */
        public static String SERVO_B_NAME = "turretServoB";          // PLACEHOLDER
        /** Config name of servo A's analog feedback input (optional; missing from config = no feedback). */
        public static String FEEDBACK_A_NAME = "turretFeedbackA";    // PLACEHOLDER
        /** Config name of servo B's analog feedback input (optional; missing from config = no feedback). */
        public static String FEEDBACK_B_NAME = "turretFeedbackB";    // PLACEHOLDER

        // ---- directions ----
        /** Reverse servo A. Both servos turn the same way when the turret turns. Check on the robot. */
        public static boolean SERVO_A_REVERSED = false;              // PLACEHOLDER
        /** Reverse servo B. Normally NOT reversed (same pinion as A). Check on the robot. */
        public static boolean SERVO_B_REVERSED = false;              // PLACEHOLDER
        /** True if a positive (counter-clockwise) turret angle needs a LOWER servo angle. */
        public static boolean TURRET_REVERSED = false;               // PLACEHOLDER

        // ---- positions ----
        /** Servo angle (degrees, 0..AxonServo.TRAVEL_DEG) that points the turret straight ahead; default is mid-travel. */
        public static double CENTER_SERVO_DEG = 127.5;               // PLACEHOLDER: tune so 0 deg is straight ahead

        // ---- speed estimate / timing ----
        /**
         * Without trusted Axon feedback the reported angle is an estimate moving toward the target at this speed
         * (deg/s). atTarget() and the shooter's ready() wait for it. Set a bit BELOW the real loaded speed.
         * 0 = no estimate (angle jumps to the target at once).
         */
        public static double ASSUMED_SPEED_DEG_PER_S = 250;          // PLACEHOLDER: measure on the robot
        /** Cap on the loop time step (seconds) so the first update after init or a pause cannot jump the estimate. */
        public static double MAX_DT_S = 0.1;
        /** Turret counts as at target within this many degrees. */
        public static double AT_TARGET_TOLERANCE_DEG = 1.5;

        // ---- start angle (zero) ----
        /** Legacy switch: true = same as ZERO_MODE = PERSIST_FROM_LAST_OPMODE. */
        public static boolean USE_PERSISTED_ANGLE = false;
        /** How the starting angle is chosen: ZERO_AT_INIT (0 deg) or PERSIST_FROM_LAST_OPMODE (angle the last OpMode ended at). */
        public static ZeroMode ZERO_MODE = ZeroMode.ZERO_AT_INIT;
        /** A persisted angle older than this (seconds) is ignored (turret or robot may have been moved). */
        public static double PERSIST_MAX_AGE_S = 300;                // PLACEHOLDER: choose for your match flow
    }

    // ======================================================================
    // TurretLimits
    // ======================================================================
    /** Turret soft limits (degrees, 0 = straight ahead, counter-clockwise positive), kept inside the real servo travel. */
    public static final class TurretLimits {
        private TurretLimits() {}

        // ---- limits ----
        /** Lowest allowed turret angle in degrees (the drive is 1:1, servo travel is about +-127.5). */
        public static double MIN_ANGLE_DEG = -125; // PLACEHOLDER: keep inside the real servo travel
        /** Highest allowed turret angle in degrees. */
        public static double MAX_ANGLE_DEG = 125;  // PLACEHOLDER
    }

    // ======================================================================
    // AxonServo
    // ======================================================================
    /** Axon servo settings shared by the turret, flower mech and gate servos: feedback scaling, travel, PWM, slew, stall, calibration. */
    public static final class AxonServo {
        private AxonServo() {}

        // ---- feedback (analog absolute encoder) ----
        /** Feedback voltage at 0 degrees. */
        public static double FEEDBACK_V_MIN = 0.0; // PLACEHOLDER
        /** Feedback voltage at full scale. */
        public static double FEEDBACK_V_MAX = 3.3; // PLACEHOLDER
        /** Feedback angle in degrees at FEEDBACK_V_MAX. */
        public static double FEEDBACK_FULL_SCALE_DEG = 360; // PLACEHOLDER
        /**
         * Set true only after measuring feedback range, direction and offset on the robot. While false,
         * getPositionDeg()/atTarget() use the commanded angle and isStalled() is always false.
         */
        public static boolean FEEDBACK_CALIBRATED = false; // PLACEHOLDER

        // ---- travel / PWM ----
        /** Degrees the servo moves for position 0..1 (255 for 500-2500 us). */
        public static double TRAVEL_DEG = 255; // PLACEHOLDER
        /** PWM pulse width in microseconds at servo position 0. */
        public static double PWM_MIN_US = 500;
        /** PWM pulse width in microseconds at servo position 1. */
        public static double PWM_MAX_US = 2500;

        // ---- slew ----
        /** Max commanded speed in deg/s; 0 = no slew limit. */
        public static double SLEW_DEG_PER_S = 0; // PLACEHOLDER
        /** Max time step (seconds) used by slew math, so the first update after init or a pause does not jump. */
        public static double MAX_DT_S = 0.1;

        // ---- stall detection ----
        /** Stalled if the command is at target but feedback error stays above this many degrees for the timeout. */
        public static double STALL_TOLERANCE_DEG = 10; // PLACEHOLDER
        /** Stall timeout in milliseconds. */
        public static double STALL_TIMEOUT_MS = 750; // PLACEHOLDER

        // ---- calibration ----
        /** Record the feedback offset once, after command and feedback have been still for CALIBRATE_SETTLE_MS. */
        public static boolean FEEDBACK_CALIBRATE = false; // PLACEHOLDER
        /** Time in milliseconds command and feedback must be still before the offset is recorded. */
        public static double CALIBRATE_SETTLE_MS = 600; // PLACEHOLDER
        /** Feedback counts as still when it moves less than this many degrees between loops. */
        public static double CALIBRATE_STABLE_DEG = 1.0; // PLACEHOLDER
    }

    // ======================================================================
    // FlowerMech
    // ======================================================================
    /** Flower-emptying arm: one Axon servo (SERVO_COUNT = 1) or two in sync (B mirrored); deploy, dwell, stow. */
    public static final class FlowerMech {
        private FlowerMech() {}

        // ---- feature switches ----
        /** Master switch; false = mechanism absent, all calls are no-ops. */
        public static boolean ENABLED = true;
        /** 2 = both servos, 1 = only A. */
        public static int SERVO_COUNT = 1;

        // ---- hardware ----
        /** Config name of servo A. */
        public static String SERVO_A_NAME = "flowerServoA";     // PLACEHOLDER
        /** Config name of servo B (only used when SERVO_COUNT >= 2). */
        public static String SERVO_B_NAME = "flowerServoB";     // PLACEHOLDER
        /** Config name of servo A's analog feedback input. */
        public static String FEEDBACK_A_NAME = "flowerFeedbackA"; // PLACEHOLDER
        /** Config name of servo B's analog feedback input. */
        public static String FEEDBACK_B_NAME = "flowerFeedbackB"; // PLACEHOLDER

        // ---- directions ----
        /** True if servo B is mounted mirrored and must be reversed. */
        public static boolean SERVO_B_REVERSED = true;          // PLACEHOLDER

        // ---- positions ----
        /** Servo A stowed angle, degrees. */
        public static double A_STOW_DEG = 0;      // PLACEHOLDER
        /** Servo A deployed angle, degrees. */
        public static double A_DEPLOY_DEG = 120;  // PLACEHOLDER
        /** Servo B stowed angle, degrees. */
        public static double B_STOW_DEG = 0;      // PLACEHOLDER
        /** Servo B deployed angle, degrees. */
        public static double B_DEPLOY_DEG = 120;  // PLACEHOLDER

        // ---- timing / speed ----
        /** Time held at the deployed position before stowing, ms. */
        public static double DWELL_MS = 500;      // PLACEHOLDER
        /** This mechanism's own slew rate (servo A degrees per second); 0 = none (jump to goal). */
        public static double SLEW_DEG_PER_S = 300; // PLACEHOLDER
        /** Cap on the time step so the first update after init or a pause cannot jump, seconds. */
        public static double MAX_DT_S = 0.1;

        // ---- tolerances / faults ----
        /** Allowed disagreement between A and B (in A degrees) before the sync timer runs. */
        public static double SYNC_TOL_DEG = 10;   // PLACEHOLDER
        /** A/B disagreement longer than this latches a fault, ms. */
        public static double SYNC_FAULT_MS = 250; // PLACEHOLDER
        /** Servo counts as at target within this many degrees. */
        public static double AT_TARGET_TOL_DEG = 3; // PLACEHOLDER
        /** After clearFault() no fault can re-latch for this long, seconds. */
        public static double FAULT_GRACE_S = 1.0; // PLACEHOLDER
        /** Trusted feedback only: if servos are not at target this long after the command arrived, carry on anyway, ms. 0 = wait forever. */
        public static double ARRIVE_TIMEOUT_MS = 1000; // PLACEHOLDER
    }

    // ======================================================================
    // Gate
    // ======================================================================
    /** Lane gate on one Axon servo; starts closed. */
    public static final class Gate {
        private Gate() {}

        // ---- feature switches ----
        /** Master switch; false = gate absent, all calls are no-ops. */
        public static boolean ENABLED = true;

        // ---- hardware ----
        /** Config name of the gate servo. */
        public static String SERVO_NAME = "gate";               // PLACEHOLDER
        /** Config name of the servo's analog feedback input. */
        public static String FEEDBACK_NAME = "gateFeedback";    // PLACEHOLDER

        // ---- directions ----
        /** True to reverse the servo direction. */
        public static boolean SERVO_REVERSED = false;           // PLACEHOLDER

        // ---- positions ----
        /** Servo angle when open, degrees. */
        public static double OPEN_DEG = 90;                     // PLACEHOLDER
        /** Servo angle when closed, degrees. */
        public static double CLOSED_DEG = 0;                    // PLACEHOLDER

        // ---- tolerances / timing ----
        /** Servo counts as at target within this many degrees. */
        public static double AT_TARGET_TOL_DEG = 3;             // PLACEHOLDER
        /** Cap on the time step so the first update after init or a pause cannot jump, seconds. */
        public static double MAX_DT_S = 0.1;
    }

    // ======================================================================
    // BallSensor
    // ======================================================================
    /** Shared tuning for every ball sensor (lane sorter sensor and the two ball-tracker sensors). REV Color Sensor V3 by default. */
    public static final class BallSensor {
        private BallSensor() {}

        // ---- sensor type / what to read ----
        /** Which device(s) to read: REV_COLOR_V3 (colour + distance), COLOR_ONLY, or DISTANCE_ONLY. */
        public static SensorType SENSOR_TYPE = SensorType.REV_COLOR_V3;
        /** Use the distance reading for ball presence. */
        public static boolean USE_DISTANCE = true;
        /** Also use the alpha (brightness) reading for ball presence (OR'd with distance). */
        public static boolean USE_COLOR_THRESHOLD = false;
        /** Classify ball colour (hue) on every sensor; the lane sorter forces this on for itself. */
        public static boolean DETECT_COLOR = false;
        /** Colour sensor gain written to the sensor, unitless. */
        public static float GAIN = 2.0F;

        // ---- presence thresholds ----
        /** Distance presence: ball enters at or below this distance, cm. PLACEHOLDER. */
        public static double DISTANCE_ENTER_CM = 4.0;   // PLACEHOLDER
        /** Distance presence: ball leaves at or above this distance, cm. PLACEHOLDER. */
        public static double DISTANCE_EXIT_CM = 6.0;    // PLACEHOLDER
        /** Alpha presence (needs USE_COLOR_THRESHOLD): enter at or above this alpha, 0..1. PLACEHOLDER. */
        public static double ALPHA_ENTER = 0.15;        // PLACEHOLDER
        /** Alpha presence: leave at or below this alpha, 0..1. PLACEHOLDER. */
        public static double ALPHA_EXIT = 0.08;         // PLACEHOLDER

        // ---- timing ----
        /** Debounce: presence must hold this long before it turns on, ms. */
        public static long RISE_MS = 30L;
        /** Debounce: absence must hold this long before presence turns off, ms. */
        public static long FALL_MS = 60L;
        /** Minimum time between I2C reads, ms; 0 = read every update. */
        public static double MIN_READ_INTERVAL_MS = 0.0;

        // ---- fault handling ----
        /** Invalid distance readings in a row before presence is dropped (until then the last state is held). */
        public static int MAX_INVALID_READS = 5;
        /** Read exceptions in a row before the sensor is marked faulted. */
        public static int MAX_READ_ERRORS = 5;
    }

    // ======================================================================
    // HueClassifier
    // ======================================================================
    /** Ball colour classification from sensor hue (pollen = yellow, nectar = red or blue). All ranges are PLACEHOLDERS: tune with the real balls. */
    public static final class HueClassifier {
        private HueClassifier() {}

        // ---- pollen (yellow) ----
        /** Pollen hue range minimum, degrees 0..360. */
        public static double POLLEN_HUE_MIN = 40.0;      // PLACEHOLDER
        /** Pollen hue range maximum, degrees 0..360. */
        public static double POLLEN_HUE_MAX = 75.0;      // PLACEHOLDER

        // ---- nectar, blue ----
        /** Blue nectar hue range minimum, degrees. */
        public static double NECTAR_HUE_MIN = 180.0;     // PLACEHOLDER
        /** Blue nectar hue range maximum, degrees. */
        public static double NECTAR_HUE_MAX = 260.0;     // PLACEHOLDER

        // ---- nectar, red ----
        /** Red nectar hue range minimum, degrees; the range wraps through 0 when MIN > MAX (e.g. 330..25). */
        public static double NECTAR_RED_HUE_MIN = 330.0; // PLACEHOLDER
        /** Red nectar hue range maximum, degrees. */
        public static double NECTAR_RED_HUE_MAX = 25.0;  // PLACEHOLDER

        // ---- reliability ----
        /** Below this HSV saturation (0..1) the reading is too grey to trust (no ball, or the lane wall). */
        public static double MIN_SATURATION = 0.15;
    }

    // ======================================================================
    // BallTracker
    // ======================================================================
    /** Two-sensor ball counter: intake-mouth sensor counts balls in, shooter-side sensor counts shots out. */
    public static final class BallTracker {
        private BallTracker() {}

        // ---- feature switches ----
        /** Master switch for the ball tracker. */
        public static boolean ENABLED = true;
        /** Use the sensor at the intake mouth. */
        public static boolean USE_INTAKE_SENSOR = true;
        /** Use the sensor just before the shooter (needed for shot counting / auto-stop when full). */
        public static boolean USE_SHOOTER_SENSOR = true;
        /** Classify ball colour on the tracker sensors too. */
        public static boolean DETECT_COLOR = false;

        // ---- hardware ----
        /** Config name of the intake-mouth colour sensor. */
        public static String INTAKE_SENSOR_NAME = "intakeColor";
        /** Config name of the shooter-side colour sensor. */
        public static String SHOOTER_SENSOR_NAME = "shooterColor";

        // ---- limits ----
        /** Most balls the robot can hold, count (at least 1 is used). */
        public static int MAX_BALLS = 4;
    }

    // ======================================================================
    // LaneSorter
    // ======================================================================
    /** Colour sorter: the lane-wall colour sensor reads each ball and drives the lane gate. */
    public static final class LaneSorter {
        private LaneSorter() {}

        // ---- hardware ----
        /** Config name of the lane-wall colour sensor. */
        public static String SENSOR_NAME = "laneColor";   // PLACEHOLDER

        // ---- sorting rules ----
        /**
         * Colour order to let through, repeating: P = pollen, N = nectar. Empty = order does not matter:
         * every ball is let through and the sorter only reports each ball's colour and carriage position.
         */
        public static String PATTERN = "";
        /** true = a ball whose colour cannot be told is let through; false = it is held. */
        public static boolean PASS_UNKNOWN = true;

        // ---- timing ----
        /** How long a ball must be gone from the sensor before the gate resets, ms. */
        public static long CLEAR_MS = 150;                // PLACEHOLDER
    }

    // ======================================================================
    // Limelight
    // ======================================================================
    /** Limelight 3A used only for a small horizontal aiming trim (tx) on the hive AprilTag. */
    public static final class Limelight {
        private Limelight() {}

        // ---- hardware ----
        /** Config name of the Limelight 3A. */
        public static String HARDWARE_NAME = "limelight";
        /** Limelight pipeline index to switch to at start. */
        public static int PIPELINE = 0;

        // ---- timing ----
        /** Ignore results older than this, ms. */
        public static double MAX_STALENESS_MS = 100;
    }
}
