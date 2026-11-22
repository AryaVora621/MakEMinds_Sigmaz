package commands.subsystems.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.servo.AxonServo;
import robot.RobotSettings;

/**
 * Turret driven by two Axon servos in positional mode, both on the same pinion (60T each into a 20T pinion,
 * then 40T into the 120T turret gear: net 1:1, so the turret turns exactly as far as the servos).
 * Angle is in degrees, 0 = straight ahead (servo mid-travel), counter-clockwise positive.
 *
 * The servos hold their own position, so there is no angle controller: setTargetAngle() just commands them
 * (clamped to TurretLimits and to the servo travel).
 * There is no turret encoder yet, so the angle read back is an estimate that moves toward the target at
 * RobotSettings.Turret.ASSUMED_SPEED_DEG_PER_S, or the Axon analog feedback once RobotSettings.AxonServo.FEEDBACK_CALIBRATED is set.
 *
 * ZERO / PERSIST
 *   Servo position is absolute, so 0 deg is always the servo mid-travel. {@link ZeroMode#PERSIST_FROM_LAST_OPMODE}
 *   starts the next OpMode commanding the angle the previous one ended at (so the turret does not snap to centre
 *   between autonomous and teleop); it falls back to 0 after an app restart. rezeroHere() sends the turret to 0.
 *
 * The old motor-turret controllers (PIDF, cascaded PIDF, ADRC) have been deleted.
 */
public class Turret {
    public enum ZeroMode { ZERO_AT_INIT, PERSIST_FROM_LAST_OPMODE }

    // Tunable values (names, directions, centre, speeds, zero mode) live in robot.RobotSettings.Turret.

    static final String SRC_PERSISTED = "persisted from last OpMode";
    static final String SRC_ZERO = "zero at init";
    static final String SRC_NONE = "zero at init (no persisted angle)";
    static final String SRC_STALE = "zero at init (persisted angle too old)";

    /** Turret angle saved by stop(); survives OpMode changes but not a power cycle or app restart. */
    private static double persistedAngleDeg = 0;
    /** True once stop() has saved an angle in this app process (and clearPersistedAngle() was not called since). */
    private static boolean persistedValid = false;
    /** System.nanoTime() when the angle was saved. */
    private static long persistedSavedNanos = 0;

    /**
     * Pure decision (no hardware): returns SRC_PERSISTED to use the saved angle, otherwise one of the
     * zero-at-init texts saying why not.
     */
    static String decideZero(boolean wantPersist, boolean valid, double ageS, double maxAgeS) {
        if (!wantPersist) return SRC_ZERO;
        if (!valid) return SRC_NONE;
        if (!(ageS >= 0 && ageS <= maxAgeS)) return SRC_STALE;   // NaN age or NaN max fails here
        return SRC_PERSISTED;
    }

    /** True if decideZero would use the persisted angle. */
    static boolean usePersisted(boolean wantPersist, boolean valid, double ageS, double maxAgeS) {
        return SRC_PERSISTED.equals(decideZero(wantPersist, valid, ageS, maxAgeS));
    }

    /** Forgets the saved angle so the next Turret starts at 0 (call it after moving the turret or robot by hand). */
    public static void clearPersistedAngle() {
        persistedValid = false;
        persistedAngleDeg = 0;
    }

    private final AxonServo servoA;
    private final AxonServo servoB;
    private String zeroSource;

    private double targetAngle;
    private double angle;
    private final ElapsedTime dtTimer = new ElapsedTime();
    private boolean started;
    /** Seconds of updates left before atTarget() may trust the estimate (see settleTimeS). */
    private double settleRemainingS;

    /** Pure: moves est toward target by at most speed*dt; speed <= 0 or NaN jumps straight to the target. */
    static double estimateStep(double est, double target, double speedDegPerS, double dtS) {
        if (!(speedDegPerS > 0)) return target;
        if (!(dtS > 0)) return est;
        double step = speedDegPerS * dtS;
        double d = target - est;
        return Math.abs(d) <= step ? target : est + Math.signum(d) * step;
    }

    public Turret(HardwareMap hardwareMap) {
        servoA = new AxonServo(hardwareMap, RobotSettings.Turret.SERVO_A_NAME, RobotSettings.Turret.FEEDBACK_A_NAME);
        servoA.setReversed(RobotSettings.Turret.SERVO_A_REVERSED);
        servoB = new AxonServo(hardwareMap, RobotSettings.Turret.SERVO_B_NAME, RobotSettings.Turret.FEEDBACK_B_NAME);
        servoB.setReversed(RobotSettings.Turret.SERVO_B_REVERSED);

        boolean wantPersist = RobotSettings.Turret.USE_PERSISTED_ANGLE || RobotSettings.Turret.ZERO_MODE == ZeroMode.PERSIST_FROM_LAST_OPMODE;
        double ageS = persistedValid ? (System.nanoTime() - persistedSavedNanos) / 1e9 : 0;
        zeroSource = decideZero(wantPersist, persistedValid, ageS, RobotSettings.Turret.PERSIST_MAX_AGE_S);
        double start = 0;
        if (zeroSource.startsWith(SRC_PERSISTED)) {
            start = persistedAngleDeg;
            zeroSource = SRC_PERSISTED + String.format(" (age %.0f s)", ageS);
        }
        start = Double.isNaN(start) ? 0 : clampTarget(start);
        angle = start;
        targetAngle = start;
        // Where the servos physically are at init is unknown unless an earlier OpMode saved it, so atTarget()
        // stays false until they have had time to reach the start angle from there (or from the worst case).
        boolean known = usePersisted(true, persistedValid, ageS, RobotSettings.Turret.PERSIST_MAX_AGE_S) && !Double.isNaN(persistedAngleDeg);
        settleRemainingS = settleTimeS(start, known, persistedAngleDeg, minAngle(), maxAngle(), RobotSettings.Turret.ASSUMED_SPEED_DEG_PER_S);
        command(targetAngle);
    }

    /**
     * Pure: seconds before the estimate may be trusted at init. If the physical position is known it is the
     * travel time from there to start, otherwise the time from the farthest end of [lo, hi]. 0 if speed <= 0.
     */
    static double settleTimeS(double start, boolean known, double knownDeg, double lo, double hi, double speedDegPerS) {
        if (!(speedDegPerS > 0)) return 0;
        double dist = known ? Math.abs(start - knownDeg) : Math.max(Math.abs(start - lo), Math.abs(hi - start));
        return Double.isNaN(dist) ? 0 : dist / speedDegPerS;
    }

    /** Pure: clamps a turret angle to [lo, hi]; NaN stays NaN. */
    static double clamp(double deg, double lo, double hi) {
        return Math.max(lo, Math.min(hi, deg));
    }

    /** Pure: turret-angle range the servos can physically reach (servo 0..travelDeg around centerServoDeg). */
    static double[] servoRange(double centerServoDeg, double travelDeg, boolean turretReversed) {
        double a = -centerServoDeg, b = travelDeg - centerServoDeg;
        return turretReversed ? new double[]{-b, -a} : new double[]{a, b};
    }

    /** Lowest reachable turret angle: the soft limit or the servo travel, whichever is tighter. */
    static double minAngle() {
        return Math.max(TurretLimits.min(), servoRange(RobotSettings.Turret.CENTER_SERVO_DEG, RobotSettings.AxonServo.TRAVEL_DEG, RobotSettings.Turret.TURRET_REVERSED)[0]);
    }

    /** Highest reachable turret angle: the soft limit or the servo travel, whichever is tighter. */
    static double maxAngle() {
        return Math.min(TurretLimits.max(), servoRange(RobotSettings.Turret.CENTER_SERVO_DEG, RobotSettings.AxonServo.TRAVEL_DEG, RobotSettings.Turret.TURRET_REVERSED)[1]);
    }

    private static double clampTarget(double deg) {
        return clamp(deg, minAngle(), maxAngle());
    }

    /**
     * Sets the turret angle, clamped to the soft limits and the servo travel so the estimate and atTarget()
     * never report an angle the servos cannot reach. NaN is ignored (it would poison the estimate for good).
     */
    public void setTargetAngle(double degrees) {
        if (Double.isNaN(degrees)) return;
        targetAngle = clampTarget(degrees);
    }

    /** The clamped angle the turret is being sent to. */
    public double getTargetAngle() {
        return targetAngle;
    }

    /** Call once per loop. */
    public void update() {
        update(0);
    }

    /** Call once per loop. The chassis rate is ignored: the servos hold position themselves. */
    public void update(double chassisRateDegPerSec) {
        command(targetAngle);
        servoA.update();
        servoB.update();
        double dt = Math.min(dtTimer.seconds(), RobotSettings.Turret.MAX_DT_S);
        dtTimer.reset();
        if (!started) {        // the servos only start moving on this first update (AxonServo writes nothing before)
            started = true;
            dt = 0;
        }
        settleRemainingS = Math.max(0, settleRemainingS - dt);
        if (servoA.feedbackTrusted()) {
            angle = servoToAngle(servoA.getPositionDeg());
        } else {
            angle = estimateStep(angle, targetAngle, RobotSettings.Turret.ASSUMED_SPEED_DEG_PER_S, dt);
        }
    }

    /** Sends the turret to 0 deg (servo mid-travel). There is no encoder to re-zero, so this is a "go to centre". */
    public void rezeroHere() {
        targetAngle = 0;
        zeroSource = "re-zeroed by driver (servo centre)";
    }

    /** Where the current zero came from, for telemetry. */
    public String zeroSource() {
        return zeroSource;
    }

    /** Same as {@link #zeroSource()}: which zero path was taken (persisted, stale, none, re-zeroed). */
    public String getZeroSourceText() {
        return zeroSource;
    }

    /** Saves the current angle for the next OpMode. The servos keep holding their last position. */
    public void stop() {
        persistedAngleDeg = angle;
        persistedValid = true;
        persistedSavedNanos = System.nanoTime();
    }

    public double getAngle() {
        return angle;
    }

    /** True when the angle (feedback or estimate) is within tolerance of the target and the init settle time is over. */
    public boolean atTarget() {
        if (settleRemainingS > 0 && !servoA.feedbackTrusted()) return false;
        return Math.abs(targetAngle - angle) <= RobotSettings.Turret.AT_TARGET_TOLERANCE_DEG;
    }

    private void command(double turretDeg) {
        double servoDeg = RobotSettings.Turret.CENTER_SERVO_DEG + (RobotSettings.Turret.TURRET_REVERSED ? -turretDeg : turretDeg);
        servoA.setTargetDeg(servoDeg);
        servoB.setTargetDeg(servoDeg);
    }

    private static double servoToAngle(double servoDeg) {
        double d = servoDeg - RobotSettings.Turret.CENTER_SERVO_DEG;
        return RobotSettings.Turret.TURRET_REVERSED ? -d : d;
    }
}
