package commands.subsystems.servo;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import robot.RobotSettings;

/**
 * Axon servo in positional mode with optional analog absolute-encoder feedback.
 * All angles are in the "commanded frame": degrees of servo travel, 0..RobotSettings.AxonServo.TRAVEL_DEG, matching the
 * servo position 0..1. Call {@link #update()} every loop; nothing is written to the servo until
 * the first update(), so the mechanism moves on that first call.
 *
 * UNVERIFIED assumption: the feedback wire is mounted so its angle increases with commanded
 * position when not reversed; when reversed it is mirrored (raw -> RobotSettings.AxonServo.TRAVEL_DEG - raw). Confirm on the robot.
 *
 * Stall detection only runs once the slewed command has reached the target, so a stall in the
 * middle of a slew is detected only after the slew ends.
 */
public class AxonServo {
    // Tunable values (feedback scaling, travel, PWM, slew, stall, calibration) live in robot.RobotSettings.AxonServo.

    private final Servo servo;
    private final AnalogInput analog; // null = no feedback
    private boolean reversed;
    private double targetDeg;
    private double commandDeg;
    private double lastWritten = Double.NaN;
    private long lastNanos = System.nanoTime();
    private long commandChangedNanos = lastNanos;
    private long targetChangedNanos = lastNanos;
    private boolean started;
    private boolean calibrated;
    private double offsetDeg;
    private double lastFramed = Double.NaN;
    private long stableSinceNanos = lastNanos;
    private double measuredDeg = Double.NaN;
    private double rawDeg = Double.NaN;
    private long outOfTolSinceNanos = -1;
    private boolean stalled;

    public AxonServo(HardwareMap hw, String servoName, String analogName) {
        servo = hw.get(Servo.class, servoName);
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(RobotSettings.AxonServo.PWM_MIN_US, RobotSettings.AxonServo.PWM_MAX_US));
        }
        AnalogInput a = null;
        if (analogName != null) {
            try {
                a = hw.get(AnalogInput.class, analogName);
            } catch (Exception e) {
                a = null; // missing from config: run without feedback
            }
        }
        analog = a;
    }

    public void setReversed(boolean reversed) {
        this.reversed = reversed;
        servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    /** Sets the target in degrees, clamped to [0, RobotSettings.AxonServo.TRAVEL_DEG]. NaN is ignored. */
    public void setTargetDeg(double deg) {
        if (Double.isNaN(deg)) return;
        double t = AxonServoMath.clamp(deg, 0, RobotSettings.AxonServo.TRAVEL_DEG);
        if (t != targetDeg) targetChangedNanos = System.nanoTime();
        targetDeg = t;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    /** Measured angle if feedback is present, otherwise the currently commanded (slewed) angle. */
    public double getPositionDeg() {
        return AxonServoMath.effectivePosition(feedbackTrusted(), measuredDeg, commandDeg);
    }

    /** True only if feedback exists and RobotSettings.AxonServo.FEEDBACK_CALIBRATED is set; otherwise the commanded angle is used. */
    public boolean feedbackTrusted() {
        return hasFeedback() && RobotSettings.AxonServo.FEEDBACK_CALIBRATED;
    }

    public boolean atTarget(double tolDeg) {
        // Plain difference: measured is re-centred on the travel window, so no wrapping is needed.
        return Math.abs(targetDeg - getPositionDeg()) <= tolDeg;
    }

    public boolean hasFeedback() {
        return analog != null;
    }

    /** Raw feedback angle in degrees (before reversal/offset); NaN without feedback. */
    public double getRawFeedbackDeg() {
        return rawDeg;
    }

    /** Feedback offset recorded by calibration (0 until calibrated). */
    public double getFeedbackOffsetDeg() {
        return offsetDeg;
    }

    /** Forgets the recorded offset so calibration runs again at the next settle (if RobotSettings.AxonServo.FEEDBACK_CALIBRATE). */
    public void recalibrate() {
        calibrated = false;
        offsetDeg = 0;
        stableSinceNanos = System.nanoTime();
    }

    public boolean isStalled() {
        return stalled;
    }

    /** Call every loop: reads feedback, slews the command, writes the servo, updates stall state. */
    public void update() {
        long now = System.nanoTime();
        double dt = AxonServoMath.capDt((now - lastNanos) / 1e9, RobotSettings.AxonServo.MAX_DT_S);
        lastNanos = now;

        double framed = Double.NaN;
        if (hasFeedback()) {
            rawDeg = AxonServoMath.voltageToDeg(analog.getVoltage(), RobotSettings.AxonServo.FEEDBACK_V_MIN, RobotSettings.AxonServo.FEEDBACK_V_MAX,
                    RobotSettings.AxonServo.FEEDBACK_FULL_SCALE_DEG);
            framed = reversed ? AxonServoMath.wrap360(RobotSettings.AxonServo.TRAVEL_DEG - rawDeg) : rawDeg;
            measuredDeg = AxonServoMath.recenter(framed, offsetDeg, RobotSettings.AxonServo.TRAVEL_DEG);
        }

        if (!started) { // unknown start position: seed from feedback if slewing, else jump to target
            started = true;
            commandDeg = (feedbackTrusted() && !Double.isNaN(measuredDeg) && RobotSettings.AxonServo.SLEW_DEG_PER_S > 0)
                    ? AxonServoMath.clamp(measuredDeg, 0, RobotSettings.AxonServo.TRAVEL_DEG) : targetDeg;
        }
        double prevCommand = commandDeg;
        commandDeg = RobotSettings.AxonServo.SLEW_DEG_PER_S > 0
                ? AxonServoMath.slewStep(commandDeg, targetDeg, RobotSettings.AxonServo.SLEW_DEG_PER_S * dt) : targetDeg;
        if (commandDeg != prevCommand) commandChangedNanos = now;
        double pos = AxonServoMath.degToPosition(commandDeg, RobotSettings.AxonServo.TRAVEL_DEG);
        if (!Double.isNaN(pos) && pos != lastWritten) {
            servo.setPosition(pos);
            lastWritten = pos;
        }
        if (!hasFeedback()) return;

        // Calibration: command at target and feedback still for the settle time.
        // A NaN reading restarts the settle timer, so the offset can never be recorded as NaN.
        if (!AxonServoMath.feedbackStill(framed, lastFramed, RobotSettings.AxonServo.CALIBRATE_STABLE_DEG)
                || commandDeg != targetDeg || commandDeg != prevCommand) {
            stableSinceNanos = now;
        }
        lastFramed = framed;
        if (RobotSettings.AxonServo.FEEDBACK_CALIBRATE && !calibrated && (now - stableSinceNanos) / 1e6 > RobotSettings.AxonServo.CALIBRATE_SETTLE_MS) {
            offsetDeg = AxonServoMath.wrap180(framed - commandDeg);
            calibrated = true;
            measuredDeg = AxonServoMath.recenter(framed, offsetDeg, RobotSettings.AxonServo.TRAVEL_DEG);
        }

        boolean outOfTol = AxonServoMath.outOfTolerance(feedbackTrusted(), commandDeg, targetDeg,
                measuredDeg, RobotSettings.AxonServo.STALL_TOLERANCE_DEG);
        if (!outOfTol) {
            outOfTolSinceNanos = -1;
            stalled = false;
        } else {
            if (outOfTolSinceNanos < 0) outOfTolSinceNanos = now;
            // The command and target must also have been constant for the whole timeout.
            long since = Math.max(outOfTolSinceNanos, Math.max(commandChangedNanos, targetChangedNanos));
            stalled = (now - since) / 1e6 > RobotSettings.AxonServo.STALL_TIMEOUT_MS;
        }
    }
}
