package commands.subsystems.ball_sensing;

import robot.RobotSettings;

/**
 * Classifies a ball by the hue of its colour-sensor reading.
 *
 * Game pieces (field CAD appearance colours): pollen is yellow (RGB 255,239,63, hue ~56), nectar is
 * alliance RED (hue ~0) or alliance BLUE (hue ~240). Both nectar colours classify as NECTAR. All ranges are
 * PLACEHOLDERS: the REV Color Sensor V3 reads under its own white LED and skews toward green, so read the
 * hue telemetry with the real balls and tune.
 */
public final class HueClassifier {

    private HueClassifier() {
    }

    /** Standard RGB -> HSV hue in [0, 360); NaN for grey (max == min) or NaN input. */
    public static double rgbToHue(double r, double g, double b) {
        if (Double.isNaN(r) || Double.isNaN(g) || Double.isNaN(b)) {
            return Double.NaN;
        }
        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        double d = max - min;
        if (d <= 0.0) {
            return Double.NaN;
        }
        double h;
        if (max == r) {
            h = (g - b) / d % 6.0;
        } else if (max == g) {
            h = (b - r) / d + 2.0;
        } else {
            h = (r - g) / d + 4.0;
        }
        h *= 60.0;
        if (h < 0.0) {
            h += 360.0;
        }
        return h;
    }

    /** HSV saturation in [0, 1]; 0 for black or NaN input. */
    public static double saturation(double r, double g, double b) {
        if (Double.isNaN(r) || Double.isNaN(g) || Double.isNaN(b)) {
            return 0.0;
        }
        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        return max <= 0.0 ? 0.0 : (max - min) / max;
    }

    /** True if hue lies in [min, max]; when min > max the range wraps through 0/360. */
    static boolean inRange(double hue, double min, double max) {
        if (min <= max) {
            return hue >= min && hue <= max;
        }
        return hue >= min || hue <= max;
    }

    public static BallColor classifyHue(double hue) {
        if (Double.isNaN(hue)) {
            return BallColor.UNKNOWN;
        }
        if (inRange(hue, RobotSettings.HueClassifier.POLLEN_HUE_MIN, RobotSettings.HueClassifier.POLLEN_HUE_MAX)) {
            return BallColor.POLLEN;
        }
        if (inRange(hue, RobotSettings.HueClassifier.NECTAR_HUE_MIN, RobotSettings.HueClassifier.NECTAR_HUE_MAX) || inRange(hue, RobotSettings.HueClassifier.NECTAR_RED_HUE_MIN, RobotSettings.HueClassifier.NECTAR_RED_HUE_MAX)) {
            return BallColor.NECTAR;
        }
        return BallColor.UNKNOWN;
    }

    /** NONE without a ball; UNKNOWN if too grey or outside every range. */
    public static BallColor classify(boolean ballPresent, double r, double g, double b) {
        if (!ballPresent) {
            return BallColor.NONE;
        }
        return saturation(r, g, b) < RobotSettings.HueClassifier.MIN_SATURATION ? BallColor.UNKNOWN : classifyHue(rgbToHue(r, g, b));
    }
}
