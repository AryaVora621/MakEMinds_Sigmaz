package commands.subsystems.ball_sensing;

import robot.RobotSettings;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * One ball sensor (REV Color Sensor V3 by default). Presence comes from the distance reading (with
 * hysteresis), optionally OR'd with an alpha (brightness) threshold, then time-debounced. Colour is only
 * classified when RobotSettings.BallSensor.DETECT_COLOR or setDetectColor(true) is on.
 *
 * Fail-soft: a device missing from the configuration gives isAvailable() == false; RobotSettings.BallSensor.MAX_READ_ERRORS
 * consecutive read exceptions latch isFaulted() (cleared by the next good read). Neither ever throws.
 * The tuning values live in RobotSettings.BallSensor and are shared by every BallSensor (the lane sorter and the ball tracker).
 */
public class BallSensor {
    public enum SensorType { REV_COLOR_V3, COLOR_ONLY, DISTANCE_ONLY }

    private final String name;
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private final boolean available;
    private boolean faulted = false;
    private boolean enabled = true;
    private boolean haveRead = false;
    private long lastReadMs = 0L;
    private int readErrors = 0;
    private boolean detectColorOverride = false;
    /** Gain last written to the sensor (NaN = never), so setGain is only called on change. */
    private float appliedGain = Float.NaN;
    private final HysteresisThreshold distHyst;
    private final HysteresisThreshold alphaHyst;
    private final Debouncer debouncer;
    private boolean present = false;
    private double distanceCm = Double.NaN;
    private double red;
    private double green;
    private double blue;
    private double alpha;
    private double hue = Double.NaN;
    private BallColor ballColor = BallColor.NONE;

    public BallSensor(HardwareMap hw, String name) {
        this.distHyst = HysteresisThreshold.lowerIsPresent(RobotSettings.BallSensor.DISTANCE_ENTER_CM, RobotSettings.BallSensor.DISTANCE_EXIT_CM);
        this.alphaHyst = HysteresisThreshold.higherIsPresent(RobotSettings.BallSensor.ALPHA_ENTER, RobotSettings.BallSensor.ALPHA_EXIT);
        this.debouncer = new Debouncer(RobotSettings.BallSensor.RISE_MS, RobotSettings.BallSensor.FALL_MS);
        this.name = name;
        boolean ok = false;
        SensorType type = RobotSettings.BallSensor.SENSOR_TYPE == null ? SensorType.REV_COLOR_V3 : RobotSettings.BallSensor.SENSOR_TYPE;
        try {
            if (type == SensorType.DISTANCE_ONLY) {
                distanceSensor = hw.tryGet(DistanceSensor.class, name);
                ok = distanceSensor != null;
            } else {
                colorSensor = hw.tryGet(NormalizedColorSensor.class, name);
                if (colorSensor != null) {
                    // The V3 driver implements both interfaces; COLOR_ONLY deliberately ignores distance.
                    if (type == SensorType.REV_COLOR_V3 && colorSensor instanceof DistanceSensor) {
                        distanceSensor = (DistanceSensor) colorSensor;
                    }
                    ok = true;
                }
            }
        } catch (RuntimeException lookupFailed) {
            ok = false;
            colorSensor = null;
            distanceSensor = null;
        }
        this.available = ok;
    }

    /** Forces colour classification on for this sensor even when RobotSettings.BallSensor.DETECT_COLOR is false. */
    public void setDetectColor(boolean detect) {
        detectColorOverride = detect;
    }

    /** True if a read should happen now: always before the first read, when rate limiting is off, or the clock went backwards. */
    static boolean isReadDue(boolean haveRead, long lastReadMs, long nowMs, double minIntervalMs) {
        if (!haveRead || !(minIntervalMs > 0.0)) {
            return true;
        }
        long dt = nowMs - lastReadMs;
        return dt < 0L || (double) dt >= minIntervalMs;
    }

    /** Disabling drops presence and the colour at once. */
    public void setEnabled(boolean on) {
        if (enabled && !on) {
            present = false;
            ballColor = BallColor.NONE;
            debouncer.reset();
        }
        enabled = on;
    }

    public boolean isValid() {
        return available && enabled && !faulted;
    }

    /** Call once per loop with a monotonic millisecond clock. Never throws. */
    public void update(long nowMs) {
        if (!available || !enabled) {
            present = false;
            ballColor = BallColor.NONE;
            return;
        }
        if (!isReadDue(haveRead, lastReadMs, nowMs, RobotSettings.BallSensor.MIN_READ_INTERVAL_MS)) {
            return;   // rate limited: keep the last result
        }
        haveRead = true;
        lastReadMs = nowMs;

        try {
            boolean wantDistance = RobotSettings.BallSensor.USE_DISTANCE && distanceSensor != null;
            boolean wantColorPresence = RobotSettings.BallSensor.USE_COLOR_THRESHOLD && colorSensor != null;
            boolean wantHue = (RobotSettings.BallSensor.DETECT_COLOR || detectColorOverride) && colorSensor != null;
            boolean readColor = wantColorPresence || wantHue;
            // Pick up tuning changes made at runtime.
            distHyst.setThresholds(RobotSettings.BallSensor.DISTANCE_ENTER_CM, RobotSettings.BallSensor.DISTANCE_EXIT_CM);
            alphaHyst.setThresholds(RobotSettings.BallSensor.ALPHA_ENTER, RobotSettings.BallSensor.ALPHA_EXIT);
            debouncer.setTimes(RobotSettings.BallSensor.RISE_MS, RobotSettings.BallSensor.FALL_MS);

            boolean distPresent = false;
            if (wantDistance) {
                distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
                // NaN, infinite or non-positive = no usable reading: hold the last state for a few reads.
                boolean valid = !Double.isNaN(distanceCm) && !Double.isInfinite(distanceCm) && distanceCm > 0.0;
                distPresent = distHyst.updateOrHold(distanceCm, valid, RobotSettings.BallSensor.MAX_INVALID_READS);
            } else {
                distanceCm = Double.NaN;
                distHyst.reset();
            }

            boolean colorPresent = false;
            if (readColor) {
                if (Float.compare(appliedGain, RobotSettings.BallSensor.GAIN) != 0) {
                    colorSensor.setGain(RobotSettings.BallSensor.GAIN);
                    appliedGain = RobotSettings.BallSensor.GAIN;
                }
                NormalizedRGBA c = colorSensor.getNormalizedColors();
                red = c.red;
                green = c.green;
                blue = c.blue;
                alpha = c.alpha;
                if (wantColorPresence) {
                    colorPresent = alphaHyst.update(alpha);
                } else {
                    alphaHyst.reset();
                }
            } else {
                alphaHyst.reset();
            }

            present = debouncer.update(distPresent || colorPresent, nowMs);
            if (wantHue) {
                hue = HueClassifier.rgbToHue(red, green, blue);
                ballColor = HueClassifier.classify(present, red, green, blue);
            } else {
                hue = Double.NaN;
                ballColor = present ? BallColor.UNKNOWN : BallColor.NONE;
            }
            faulted = false;
            readErrors = 0;
        } catch (RuntimeException readFailed) {
            // A few failed I2C reads keep the last result; MAX_READ_ERRORS in a row latch the fault.
            readErrors++;
            if (readErrors >= Math.max(1, RobotSettings.BallSensor.MAX_READ_ERRORS)) {
                faulted = true;
                present = false;
                debouncer.reset();
                distHyst.reset();
                alphaHyst.reset();
                distanceCm = Double.NaN;
                red = 0.0;
                green = 0.0;
                blue = 0.0;
                alpha = 0.0;
                hue = Double.NaN;
                ballColor = BallColor.NONE;
            }
        }
    }

    public boolean isBallPresent() {
        return available && enabled && !faulted && present;
    }

    public boolean isAvailable() {
        return available;
    }

    public boolean isFaulted() {
        return faulted;
    }

    public double getDistanceCm() {
        return distanceCm;
    }

    public double getRed() {
        return red;
    }

    public double getGreen() {
        return green;
    }

    public double getBlue() {
        return blue;
    }

    public double getAlpha() {
        return alpha;
    }

    public double getHue() {
        return hue;
    }

    public BallColor getBallColor() {
        return ballColor;
    }

    public String getName() {
        return name;
    }
}
