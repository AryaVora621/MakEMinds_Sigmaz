package robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import commands.autonomous.AutoDrivebase;
import commands.field.Alliance;
import commands.subsystems.drivebase.Drivebase;
import commands.subsystems.turret.Turret;
import commands.subsystems.flower_mech.FlowerMech;
import commands.subsystems.gate.Gate;
import commands.subsystems.grouped_subsystems.intake_transfer.IntakeTransfer;
import commands.subsystems.grouped_subsystems.shooter.FlywheelTurret;
import commands.subsystems.grouped_subsystems.shooter.ShooterBase;
import commands.subsystems.limelight.Limelight;
import commands.subsystems.turret.TurretAiming;

/**
 * THE main robot file; both autonomous and teleop build the robot through it. The lineup is picked in
 * {@link RobotSettings.Robot}:
 *
 *   ALLIANCE      which cell we shoot at
 *   USE_SHOOTER   false = drive-only robot (no shooter hardware is touched)
 *   USE_LIMELIGHT true = use the Limelight tx to trim the turret
 *
 * The drive motion profile is chosen in commands.autonomous.AutoDrivebase.
 *
 * Both subsystems share one pose source: the Pedro follower inside AutoDrivebase. The shooter reads
 * the follower's pose, velocity and yaw rate, so it keeps aiming correctly while autonomous paths
 * (profiled) or teleop driving move the robot. Call {@link #update()} once per loop.
 */
public class Robot {
    // All tunable constants (ALLIANCE, USE_*, ...) live in RobotSettings.Robot.

    /** Pose saved by {@link #stop()} (autonomous), so teleop can start where autonomous ended. */
    private static Pose savedPose = null;

    public final AutoDrivebase auto;
    public final Drivebase drivebase;
    public final IntakeTransfer intake;
    /** Null when USE_SHOOTER is false. */
    public final ShooterBase shooter;
    /** Null when USE_LIMELIGHT is false. */
    public final Limelight limelight;
    /** Null when USE_FLOWER_MECH is false. */
    public final FlowerMech flower;
    /** Null when USE_GATE is false. */
    public final Gate gate;

    private double filteredAccel;
    /** Loop timing for telemetry: last, smoothed average and worst loop time in ms. */
    private long lastLoopNanos = 0;
    private double loopMs = 0, loopAvgMs = 0, loopMaxMs = 0;
    private BufferedWriter shotLog;
    private boolean shotLogFailed;
    private boolean wasFeeding;
    private long lastShotLogMs;
    private final StringBuilder missing = new StringBuilder();

    /** @param startPose where the robot starts (Pedro coordinates). */
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        // Bulk reads: one hub transaction per loop instead of one per encoder/sensor read (Pedro and the flywheel
        // read often). AUTO clears the cache itself, so no manual clearing is needed.
        for (com.qualcomm.hardware.lynx.LynxModule hub : hardwareMap.getAll(com.qualcomm.hardware.lynx.LynxModule.class)) {
            hub.setBulkCachingMode(com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO);
        }
        // Odometry enabled: AutoDrivebase builds the Pedro follower, which uses the Pinpoint.
        auto = new AutoDrivebase(hardwareMap, startPose);
        drivebase = new Drivebase(hardwareMap);   // created after the follower so its motor setup wins
        // ====== DRIVE-ONLY MODE (requested): every subsystem except the drivetrain is commented out. ======
        // To bring one back: uncomment its block below AND delete its "= null" line. Other code in this file and in
        // TwoPlayerTeleOp already treats these as optional (null-safe), so they can be restored one at a time.
        intake = null;
        shooter = null;
        limelight = null;
        flower = null;
        gate = null;
        // ---- original code, kept for restoring (intake, shooter, limelight, flower, gate) ----
        // intake = new IntakeTransfer(hardwareMap);
        // shooter = RobotSettings.Robot.USE_SHOOTER ? createShooter(hardwareMap) : null;
        // limelight = RobotSettings.Robot.USE_LIMELIGHT ? new Limelight(hardwareMap) : null;
        // // Optional hardware is fail-soft: a missing device gives null plus a "missing" telemetry line.
        // FlowerMech fl = null;
        // if (RobotSettings.Robot.USE_FLOWER_MECH) {
        //     try { fl = new FlowerMech(hardwareMap); } catch (RuntimeException e) { missing.append(" flower mech"); }
        // }
        // flower = fl;
        // Gate g = null;
        // if (RobotSettings.Robot.USE_GATE) {
        //     try { g = new Gate(hardwareMap); } catch (RuntimeException e) { missing.append(" gate"); }
        // }
        // gate = g;
    }

    /**
     * Starts at the pose autonomous ended on, or at fallbackPose if none was saved. The saved pose is used
     * once: a second teleop run (robot moved by hand in between) starts from fallbackPose, not a stale pose.
     */
    public static Robot continuingFromAuto(HardwareMap hardwareMap, Pose fallbackPose) {
        Pose start = takeSavedPose();
        if (start != null) {
            // Auto -> teleop: keep the turret zero from autonomous (review finding: zero lost between opmodes).
            RobotSettings.Turret.ZERO_MODE = Turret.ZeroMode.PERSIST_FROM_LAST_OPMODE;
        }
        return new Robot(hardwareMap, start != null ? start : fallbackPose);
    }

    /** Returns the pose saved by autonomous (or null) and clears it. */
    static Pose takeSavedPose() {
        Pose p = savedPose;
        savedPose = null;
        return p;
    }

    /** Test seam: what {@link #stop()} would save. */
    static void setSavedPoseForTest(Pose p) {
        savedPose = finitePoseOrNull(p);
    }

    private static ShooterBase createShooter(HardwareMap hardwareMap) {
        return new FlywheelTurret(hardwareMap);
    }

    /** Call once per loop, in autonomous and teleop. */
    public void update() {
        long nowNanos = System.nanoTime();
        if (lastLoopNanos != 0) {
            loopMs = (nowNanos - lastLoopNanos) / 1e6;
            loopAvgMs = loopAvgMs == 0 ? loopMs : loopAvgMs + 0.05 * (loopMs - loopAvgMs);
            loopMaxMs = Math.max(loopMaxMs, loopMs);
        }
        lastLoopNanos = nowNanos;
        if (auto != null) {   // null in drive-motors-only mode (no odometry)
            auto.update();

            double ax = auto.getAcceleration().getXComponent();
            double ay = auto.getAcceleration().getYComponent();
            filteredAccel = filterAccel(filteredAccel, ax, ay, RobotSettings.Robot.ACCEL_FILTER_ALPHA);
        }

        if (limelight != null) {
            limelight.update();
        }
        if (flower != null) {
            flower.update();
        }
        if (gate != null) {
            gate.update();
        }
        if (shooter != null) {
            shooter.update(auto.getPose(), auto.getVelocity(), auto.getAngularVelocity(), RobotSettings.Robot.ALLIANCE, RobotSettings.Robot.FAR_CELL_UP,
                    limelight != null && limelight.isValid(), limelight != null ? limelight.getTx() : 0);
        }
    }

    /** True when the shooter is ready and the robot is steady enough to fire accurately. */
    public boolean readyToFire() {
        if (shooter == null || !shooter.ready()) {
            return false;
        }
        return RobotSettings.Robot.FIRE_MAX_ACCEL_IN_S2 <= 0 || filteredAccel <= RobotSettings.Robot.FIRE_MAX_ACCEL_IN_S2;
    }

    /**
     * Feeds a ball (intake and transfer forward) while wantFire is held and the shot is ready.
     *
     * @return true if it is feeding this loop; the caller should then not command the intake itself.
     */
    public boolean feedIfReady(boolean wantFire) {
        boolean feedingBefore = wasFeeding;
        wasFeeding = false;
        if (wantFire && intake != null && readyToFire()) {
            intake.feed();
            if (!feedingBefore && System.currentTimeMillis() - lastShotLogMs >= RobotSettings.Robot.SHOT_LOG_MIN_INTERVAL_MS) {
                lastShotLogMs = System.currentTimeMillis();
                logShot();   // one line per feed start (rising edge)
            }
            wasFeeding = true;
            return true;
        }
        return false;
    }

    /**
     * One step of the acceleration low-pass filter. A non-finite sample (or a non-finite previous value) is
     * not mixed in, so one bad follower reading cannot latch the filter at NaN/Infinity and block firing forever.
     */
    static double filterAccel(double previous, double ax, double ay, double alpha) {
        double sample = Math.hypot(ax, ay);
        if (Double.isNaN(sample) || Double.isInfinite(sample)) {
            return (Double.isNaN(previous) || Double.isInfinite(previous)) ? 0 : previous;
        }
        if (Double.isNaN(previous) || Double.isInfinite(previous)) {
            return sample;
        }
        return previous + alpha * (sample - previous);
    }

    /** Pose worth saving for teleop: null if missing or not finite (a NaN pose must not seed the next opmode). */
    static Pose finitePoseOrNull(Pose p) {
        if (p == null) {
            return null;
        }
        double[] v = {p.getX(), p.getY(), p.getHeading()};
        for (double d : v) {
            if (Double.isNaN(d) || Double.isInfinite(d)) {
                return null;
            }
        }
        return p;
    }

    /** Selects the alliance to shoot for. Robot reads ALLIANCE every loop, so this takes effect immediately. */
    public void setAlliance(Alliance alliance) {
        if (alliance != null) {
            RobotSettings.Robot.ALLIANCE = alliance;
        }
    }

    /** Toggles the flower mechanism (null-safe); clears a fault first if it is faulted. */
    public void toggleFlower() {
        if (flower == null) {
            return;
        }
        if (flower.isFaulted()) {
            flower.clearFault();
        } else {
            flower.toggle();
        }
    }

    /** Toggles the gate that lets balls in to, or blocks them from, the shooter (null-safe). */
    public void toggleGate() {
        if (gate != null) {
            gate.toggle();
        }
    }

    /** Flips which hive cell is raised (the aim target). Wire to an operator button; the hive is bistable. */
    public void toggleFarCellUp() {
        RobotSettings.Robot.FAR_CELL_UP = !RobotSettings.Robot.FAR_CELL_UP;
    }

    /** Makes the turret's current position 0 deg (null-safe: does nothing without a turret). */
    public void rezeroTurret() {
        if (shooter != null) {
            shooter.rezeroTurret();
        }
    }

    /** Where the turret zero came from, for telemetry. */
    public String turretZeroSource() {
        return shooter == null ? "no shooter" : shooter.turretZeroSource();
    }

    /** One line per unmeasured constant that is still at its default (for telemetry; testable). */
    public static List<String> placeholderWarnings() {
        List<String> w = new ArrayList<>();
        if (RobotSettings.TurretAiming.EXIT_HEIGHT_IN == 12) w.add("TurretAiming.EXIT_HEIGHT_IN (ball exit height)");
        if (RobotSettings.TurretAiming.WHEEL_RADIUS_IN == 1.89) w.add("TurretAiming.WHEEL_RADIUS_IN");
        if (RobotSettings.TurretAiming.TRANSFER_EFFICIENCY == 0.45) w.add("TurretAiming.TRANSFER_EFFICIENCY (calibrate)");
        if (RobotSettings.TurretAiming.PIVOT_OFFSET_X_IN == 0 && RobotSettings.TurretAiming.PIVOT_OFFSET_Y_IN == 0) w.add("TurretAiming.PIVOT_OFFSET (turret pivot)");
        if (RobotSettings.TurretAiming.LATENCY_S == 0.05) w.add("TurretAiming.LATENCY_S");
        if (RobotSettings.Odometry.PARALLEL_POD_OFFSET_IN == 0) w.add("Odometry.PARALLEL_POD_OFFSET_IN");
        if (RobotSettings.Odometry.PERPENDICULAR_POD_OFFSET_IN == 0) w.add("Odometry.PERPENDICULAR_POD_OFFSET_IN");
        if (Arrays.equals(RobotSettings.ShotTable.DISTANCE_IN, new double[]{36, 108})
                && Arrays.equals(RobotSettings.ShotTable.RPM, new double[]{2800, 3800})) w.add("ShotTable (distance/rpm rows)");
        if (RobotSettings.Robot.USE_SHOOTER && RobotSettings.Turret.CENTER_SERVO_DEG == 127.5) w.add("Turret.CENTER_SERVO_DEG (servo angle for straight ahead)");
        if (RobotSettings.Flywheel.TICKS_PER_REV == 28) w.add("Flywheel.TICKS_PER_REV (confirm)");
        if (RobotSettings.TeleOp.FALLBACK_X_IN == 72 && RobotSettings.TeleOp.FALLBACK_Y_IN == 12) w.add("TeleOp.FALLBACK pose");
        if (!RobotSettings.AxonServo.FEEDBACK_CALIBRATED) w.add("AxonServo.FEEDBACK_CALIBRATED=false (feedback not trusted)");
        if (RobotSettings.Robot.USE_SHOOTER && RobotSettings.Turret.PERSIST_MAX_AGE_S == 300) w.add("Turret.PERSIST_MAX_AGE_S");
        if (RobotSettings.Robot.USE_FLOWER_MECH && RobotSettings.FlowerMech.FAULT_GRACE_S == 1.0) w.add("FlowerMech.FAULT_GRACE_S");
        if (RobotSettings.Robot.USE_SHOOTER && RobotSettings.Flywheel.MOTOR_B_NAME.equals("flywheel2") && RobotSettings.Flywheel.MOTOR_B_REVERSED) w.add("Flywheel.MOTOR_B_NAME / MOTOR_B_REVERSED (second flywheel motor)");
        if (RobotSettings.Robot.USE_FLOWER_MECH && RobotSettings.FlowerMech.A_DEPLOY_DEG == 120) w.add("FlowerMech deploy angles/dwell");
        if (RobotSettings.Robot.USE_GATE && RobotSettings.Gate.OPEN_DEG == 90) w.add("Gate.OPEN_DEG/CLOSED_DEG");
        if (RobotSettings.Robot.USE_SHOOTER && RobotSettings.Turret.ASSUMED_SPEED_DEG_PER_S == 250) w.add("Turret.ASSUMED_SPEED_DEG_PER_S (measure turret speed)");
        return w;
    }

    /** Appends one CSV line for a feed that was just performed. Never throws; disables itself after one failure. */
    private void logShot() {
        if (!RobotSettings.Robot.SHOT_LOG_ENABLED || shotLogFailed || shooter == null) {
            return;
        }
        try {
            if (shotLog == null) {
                File dir = new File(AppUtil.FIRST_FOLDER, "shotlog");
                if (!dir.exists() && !dir.mkdirs() && !dir.exists()) {
                    throw new java.io.IOException("cannot create " + dir);
                }
                File f = new File(dir, "shots_" + new SimpleDateFormat("yyyyMMdd", Locale.US).format(new Date()) + ".csv");
                boolean header = !f.exists() || f.length() == 0;
                shotLog = new BufferedWriter(new FileWriter(f, true));
                if (header) {
                    shotLog.write("time_ms,x_in,y_in,heading_deg,distance_in,target_rpm,measured_rpm,launch_elev_deg,turret_deg,reachable,alliance\n");
                }
            }
            Pose p = auto.getPose();
            TurretAiming.Solution s = shooter.getSolution();
            shotLog.write(String.format(Locale.US, "%d,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.2f,%.2f,%b,%s\n",
                    System.currentTimeMillis(), p.getX(), p.getY(), Math.toDegrees(p.getHeading()), s.distanceIn,
                    s.rpm, shooter.getFlywheel().getRpm(), s.launchElevDeg,shooter.getTurretAngleDeg(),
                    s.reachable, RobotSettings.Robot.ALLIANCE));
            shotLog.flush();
        } catch (Exception | LinkageError e) {
            shotLogFailed = true;
            closeShotLog();
        }
    }

    private void closeShotLog() {
        if (shotLog != null) {
            try {
                shotLog.close();
            } catch (Exception ignored) {
                // nothing more to do
            }
            shotLog = null;
        }
    }

    /** Filtered robot acceleration magnitude (in/s^2). */
    public double getFilteredAcceleration() {
        return filteredAccel;
    }

    /** Stops everything and remembers the pose for teleop. Call this at the end of autonomous. */
    public void stop() {
        stop(true);
    }

    /**
     * Stops everything. savePose = true (autonomous) remembers the pose for {@link #continuingFromAuto};
     * teleop passes false so a later teleop run does not silently start from where the previous one ended.
     */
    public void stop(boolean savePose) {
        if (savePose) {
            try { savedPose = auto == null ? null : finitePoseOrNull(auto.getPose()); } catch (RuntimeException e) { savedPose = null; }
        }
        closeShotLog();
        // Release the follower first so Pedro stops re-applying power after we zero the motors.
        if (auto != null) {
            try { auto.getFollower().breakFollowing(); } catch (RuntimeException ignored) { }
        }
        // Each subsystem stops independently so one failure cannot skip the others.
        try { drivebase.stop(); } catch (RuntimeException ignored) { }
        if (intake != null) {
            try { intake.stop(); } catch (RuntimeException ignored) { }
        }
        if (shooter != null) {
            try { shooter.stop(); } catch (RuntimeException ignored) { }
        }
        if (limelight != null) {
            try { limelight.stop(); } catch (RuntimeException ignored) { }
        }
    }

    /** Adds the standard status lines to telemetry. */
    public void addTelemetry(Telemetry t) {
        t.addData("alliance", RobotSettings.Robot.ALLIANCE);
        if (auto != null) {
            Pose p = auto.getPose();
            t.addData("drive profile", auto.getProfile().name());
            t.addData("pose (in, in, deg)", "%.1f, %.1f, %.1f", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            t.addData("speed (in/s)", "%.1f", auto.getVelocity().getMagnitude());
            t.addData("accel filtered (in/s^2)", "%.1f", filteredAccel);
        } else {
            t.addLine("No odometry: drive-motors-only mode (no pose / speed shown)");
        }
        double[] cmd = drivebase.getLastCommand();
        double[] wp = drivebase.getLastPowers();
        t.addData("drive cmd fwd / strafe / turn", "%.2f / %.2f / %.2f", cmd[0], cmd[1], cmd[2]);
        t.addData("wheel power FL / FR / BL / BR", "%.2f / %.2f / %.2f / %.2f", wp[0], wp[1], wp[2], wp[3]);
        t.addData("loop ms (last / avg / max)", "%.1f / %.1f / %.1f", loopMs, loopAvgMs, loopMaxMs);
        if (auto != null && !Double.isNaN(auto.getProfile().referenceVelocity())) {
            t.addData("profile ref pos / vel / acc", "%.1f / %.1f / %.1f",
                    auto.getProfile().referencePosition(), auto.getProfile().referenceVelocity(),
                    auto.getProfile().referenceAcceleration());
        }
        if (shooter != null) {
            TurretAiming.Solution s = shooter.getSolution();
            t.addData("shooter", shooter.configurationName());
            t.addData("shot source", "%s (flywheel PIDF, turret on servos)", RobotSettings.TurretAiming.SHOT_SOURCE);
            t.addData("shot status", s.statusText());
            if (s.ok) {
                t.addData(RobotSettings.TurretAiming.SHOT_SOURCE + " setpoint", "dist %.1f in, flight %.2f s%s", s.distanceIn, s.flightTimeS,
                        s.compensating ? " (moving)" : "");
                t.addData("setpoint turret / launch elev / rpm", "%.1f deg / %.1f deg / %.0f", s.turretAngleDeg, s.launchElevDeg, s.rpm);
                t.addData("entry angle", "%.1f deg", s.entryAngleDeg);
                if (s.fromBehind) {
                    t.addData("BLOCKED", "ball would arrive from behind the opening: move to the open side of the raised cell");
                }
            } else {
                t.addData("shot", "NO SOLUTION: " + s.reason);
            }
            t.addData("flywheel rpm", "%.0f", shooter.getFlywheel().getRpm());
            t.addData("reachable / ready / fire-ok", "%b / %b / %b", s.reachable, shooter.ready(), readyToFire());
            if (shooter.hasTurret()) {
                t.addData("turret zero", shooter.turretZeroSource());
            }
        }
        if (!RobotSettings.AxonServo.FEEDBACK_CALIBRATED) {
            t.addLine("Axon feedback: UNCALIBRATED (using commanded angles)");
        }
        if (missing.length() > 0) {
            t.addData("missing", missing.toString().trim());
        }
        if (flower != null) {
            t.addData("flower mech", "%s%s", flower.getState(), flower.isFaulted() ? " FAULTED (gp2 X clears)" : "");
        }
        if (gate != null) {
            t.addData("gate", "%s%s", gate.isOpen() ? "OPEN" : "CLOSED", gate.isStalled() ? " FAULTED (stalled)" : "");
        }
        if (RobotSettings.Robot.SHOW_PLACEHOLDER_WARNINGS) {
            List<String> warnings = placeholderWarnings();
            if (!warnings.isEmpty()) {
                t.addData("PLACEHOLDER WARNINGS", warnings.size() + " unmeasured constants at default");
                for (String w : warnings) {
                    t.addLine("  " + w);
                }
            }
        }
    }
}
