package commands.subsystems.ball_sensing;

import robot.RobotSettings;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Two-sensor ball counter: one sensor at the intake mouth, one just before the shooter. A ball arriving
 * at the intake counts up; a ball leaving the shooter sensor counts as a shot and counts down. Counting
 * logic lives in BallTrackerLogic (hardware-free, unit tested). Either sensor can be switched off with
 * RobotSettings.BallTracker.USE_INTAKE_SENSOR / RobotSettings.BallTracker.USE_SHOOTER_SENSOR; a missing sensor never throws, it just reports a fault.
 * Call setIntakeReversing() and then update() once per loop.
 */
public class BallTracker {
    private final BallSensor intakeSensor;
    private final BallSensor shooterSensor;
    private final BallTrackerLogic logic = new BallTrackerLogic(maxBalls());
    private boolean intakeReversing = false;

    public BallTracker(HardwareMap hw) {
        intakeSensor = new BallSensor(hw, RobotSettings.BallTracker.INTAKE_SENSOR_NAME);
        shooterSensor = new BallSensor(hw, RobotSettings.BallTracker.SHOOTER_SENSOR_NAME);
    }

    private static int maxBalls() {
        return Math.max(1, RobotSettings.BallTracker.MAX_BALLS);
    }

    public void update() {
        if (!RobotSettings.BallTracker.ENABLED) {
            return;
        }
        long nowMs = System.nanoTime() / 1000000L;
        // Re-apply the tunables every loop so runtime changes to RobotSettings.BallTracker take effect.
        logic.setMax(maxBalls());
        logic.setUse(RobotSettings.BallTracker.USE_INTAKE_SENSOR, RobotSettings.BallTracker.USE_SHOOTER_SENSOR);
        intakeSensor.setDetectColor(RobotSettings.BallTracker.DETECT_COLOR);
        shooterSensor.setDetectColor(RobotSettings.BallTracker.DETECT_COLOR);
        intakeSensor.setEnabled(RobotSettings.BallTracker.USE_INTAKE_SENSOR);
        shooterSensor.setEnabled(RobotSettings.BallTracker.USE_SHOOTER_SENSOR);
        if (RobotSettings.BallTracker.USE_INTAKE_SENSOR) {
            intakeSensor.update(nowMs);
        }
        if (RobotSettings.BallTracker.USE_SHOOTER_SENSOR) {
            shooterSensor.update(nowMs);
        }
        logic.update(intakeSensor.isBallPresent(), intakeSensor.isValid(),
                shooterSensor.isBallPresent(), shooterSensor.isValid(), intakeReversing);
    }

    /** While true, balls passing the intake sensor are leaving: they count down instead of up, and no shot is counted. */
    public void setIntakeReversing(boolean reversing) {
        intakeReversing = reversing;
    }

    /** True if a sensor that is in use is missing or faulted. */
    public boolean isFaulted() {
        return RobotSettings.BallTracker.ENABLED && ((RobotSettings.BallTracker.USE_INTAKE_SENSOR && !intakeSensor.isValid())
                || (RobotSettings.BallTracker.USE_SHOOTER_SENSOR && !shooterSensor.isValid()));
    }

    /** E.g. "intake(intakeColor) missing, shooter(shooterColor) faulted"; empty when healthy or disabled. */
    public String getFaultText() {
        if (!RobotSettings.BallTracker.ENABLED) {
            return "";
        }
        StringBuilder sb = new StringBuilder();
        appendFault(sb, "intake", RobotSettings.BallTracker.USE_INTAKE_SENSOR, intakeSensor);
        appendFault(sb, "shooter", RobotSettings.BallTracker.USE_SHOOTER_SENSOR, shooterSensor);
        return sb.toString();
    }

    private static void appendFault(StringBuilder sb, String label, boolean used, BallSensor s) {
        if (!used || s.isValid()) {
            return;
        }
        if (sb.length() > 0) {
            sb.append(", ");
        }
        sb.append(label).append('(').append(s.getName()).append(')')
                .append(s.isAvailable() ? " faulted" : " missing");
    }

    public int getCount() {
        logic.setMax(maxBalls());
        return logic.getCount();
    }

    public boolean ballAtIntake() {
        return RobotSettings.BallTracker.ENABLED && logic.ballAtIntake();
    }

    public boolean ballAtShooter() {
        return RobotSettings.BallTracker.ENABLED && logic.ballAtShooter();
    }

    /** Returns true once per counted shot (pending shots are dropped while disabled). */
    public boolean consumeShotEvent() {
        if (!RobotSettings.BallTracker.ENABLED) {
            logic.clearEvents();
            return false;
        }
        return logic.consumeShotEvent();
    }

    /** Returns true once per counted intake (pending events are dropped while disabled). */
    public boolean consumeIntakeEvent() {
        if (!RobotSettings.BallTracker.ENABLED) {
            logic.clearEvents();
            return false;
        }
        return logic.consumeIntakeEvent();
    }

    /** Count back to 0 (or 1 if a ball is sitting at the shooter sensor) and drop pending events. */
    public void reset() {
        logic.reset();
    }

    public void setCount(int n) {
        logic.setMax(maxBalls());
        logic.setCount(n);
    }

    public boolean isFull() {
        logic.setMax(maxBalls());
        return logic.isFull();
    }

    public void addTelemetry(Telemetry t) {
        t.addData("Balls enabled", RobotSettings.BallTracker.ENABLED);
        t.addData("Ball count", getCount() + " / " + maxBalls());
        t.addData("Ball sensor faults", isFaulted() ? getFaultText() : "none");
        addSensor(t, "Intake", intakeSensor);
        addSensor(t, "Shooter", shooterSensor);
    }

    private static void addSensor(Telemetry t, String label, BallSensor s) {
        t.addData(label + " (" + s.getName() + ")",
                "avail=%b fault=%b present=%b dist=%.1fcm rgba=%.3f/%.3f/%.3f/%.3f hue=%.0f color=%s",
                s.isAvailable(), s.isFaulted(), s.isBallPresent(), s.getDistanceCm(),
                s.getRed(), s.getGreen(), s.getBlue(), s.getAlpha(), s.getHue(), s.getBallColor());
    }
}
