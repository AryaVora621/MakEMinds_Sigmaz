package commands.subsystems.grouped_subsystems.intake_transfer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import commands.subsystems.ball_sensing.BallTracker;
import commands.subsystems.intake.Intake;
import commands.subsystems.transfer.Transfer;
import robot.RobotSettings;

/** Intake and transfer kept in sync: the transfer always runs in the same direction as the intake. */
public class IntakeTransfer {
    enum Mode { STOPPED, FORWARD, BACKWARD, FEED }

    private final Intake intake;
    private final Transfer transfer;

    /** Per-instance override; null means follow fullStopDefault. */
    private Boolean fullStopOverride = null;
    /** What setFullStop() last asked for (set from RobotSettings.Robot.AUTO_STOP_INTAKE_WHEN_FULL by Robot); false until then. */
    private boolean fullStopDefault = false;
    private boolean full = false;
    private Mode mode = Mode.STOPPED;
    private boolean feedSeenAtShooter = false;

    public IntakeTransfer(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
    }

    public void forward() {
        endFeed();
        mode = Mode.FORWARD;
        if (!modeMayRun(mode, isFullStopActive(), full, shooterSensorUsable())) {
            intake.stop();
            transfer.stop();
            return;
        }
        intake.forward();
        transfer.forward();
    }

    public void backward() {
        endFeed();
        mode = Mode.BACKWARD;
        intake.backward();
        transfer.backward();
    }

    public void stop() {
        endFeed();
        mode = Mode.STOPPED;
        intake.stop();
        transfer.stop();
    }

    /**
     * Runs intake+transfer forward to feed the shooter, ignoring the full-stop. Safe to call every loop.
     * The feed lasts until the caller calls stop()/forward()/backward().
     */
    public void feed() {
        if (mode != Mode.FEED) feedSeenAtShooter = false;
        mode = Mode.FEED;
        intake.forward();
        transfer.forward();
    }

    /**
     * Call once per loop. Samples isFull()/ballAtShooter() only; never consumes tracker events.
     * Null tracker is a no-op. A requested forward() is halted when the tracker becomes full
     * (if full-stop is active and the shooter sensor is usable); not while feeding.
     */
    public void update(BallTracker tracker) {
        if (tracker == null) return;
        full = tracker.isFull();
        feedSeenAtShooter = feedConfirmed(feedSeenAtShooter, mode == Mode.FEED, tracker.ballAtShooter());
        if (!modeMayRun(mode, isFullStopActive(), full, shooterSensorUsable())) {
            intake.stop();
            transfer.stop();
        }
    }

    /** Sets the full-stop request (and remembers it as the default that resetFullStop() returns to). */
    public void setFullStop(boolean enabled) { fullStopOverride = enabled; fullStopDefault = enabled; }

    /**
     * Driver recovery: clears the latched full flag and the per-instance override (back to following
     * the last setFullStop() value). If forward intake was requested and had been halted, it resumes. The caller
     * should also reset the tracker count, otherwise the next update() may latch full again.
     */
    public void resetFullStop() {
        full = false;
        fullStopOverride = null;
        if (mode == Mode.FORWARD) {
            intake.forward();
            transfer.forward();
        }
    }

    /** True while the intake is being driven backward (outtake). */
    public boolean isReversing() { return isReversingMode(mode); }

    /** Goes back to the last setFullStop() value. */
    public void clearFullStopOverride() { fullStopOverride = null; }

    /** Requested full-stop setting (override or the default); it still needs the shooter sensor to engage. */
    public boolean isFullStopActive() {
        return fullStopOverride != null ? fullStopOverride : fullStopDefault;
    }

    /** True if the last update() saw the tracker full. */
    public boolean isFull() { return full; }

    public boolean isFeeding() { return mode == Mode.FEED; }

    /** Toggle off: always true. Toggle on: true only while feeding and ballAtShooter() was seen since the feed began. */
    public boolean isFeedConfirmed() {
        return !RobotSettings.IntakeTransfer.CONFIRM_FEED_WITH_SHOOTER_SENSOR || (mode == Mode.FEED && feedSeenAtShooter);
    }

    private void endFeed() {
        if (mode == Mode.FEED) feedSeenAtShooter = false;
    }

    private static boolean shooterSensorUsable() {
        return RobotSettings.BallTracker.ENABLED && RobotSettings.BallTracker.USE_SHOOTER_SENSOR;
    }

    // ---- pure, hardware-free decision logic (unit-tested) ----

    /** Forward intake is allowed unless full-stop is requested, the shooter sensor is usable, and the robot is full. */
    static boolean shouldRunForward(boolean fullStopActive, boolean full, boolean shooterSensorUsable) {
        return !(fullStopActive && full && shooterSensorUsable);
    }

    /** Only forward intake is ever gated by the full-stop; backward, feed and stopped modes always may run. */
    static boolean modeMayRun(Mode mode, boolean fullStopActive, boolean full, boolean shooterSensorUsable) {
        return mode != Mode.FORWARD || shouldRunForward(fullStopActive, full, shooterSensorUsable);
    }

    static boolean isReversingMode(Mode mode) { return mode == Mode.BACKWARD; }

    /** Latches a shooter-sensor sighting, only while feeding. */
    static boolean feedConfirmed(boolean alreadyConfirmed, boolean feeding, boolean ballAtShooter) {
        return alreadyConfirmed || (feeding && ballAtShooter);
    }
}
