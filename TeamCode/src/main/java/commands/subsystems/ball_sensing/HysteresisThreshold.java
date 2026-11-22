package commands.subsystems.ball_sensing;

/**
 * Two-threshold (Schmitt) presence detector. lowerIsPresent: a value at or below the lower threshold
 * means present (distance); otherwise a value at or above the upper threshold means present (alpha).
 * enter/exit may be given in either order: the lower and upper thresholds are min/max of the two.
 * When both thresholds are equal, leaving needs a value strictly past them, so a value sitting exactly
 * on the threshold does not chatter.
 */
public class HysteresisThreshold {
    private double enter;
    private double exit;
    private final boolean lowerIsPresent;
    private boolean present = false;
    /** Consecutive invalid readings seen by updateOrHold. */
    private int invalidCount = 0;

    public HysteresisThreshold(double enter, double exit, boolean lowerIsPresent) {
        this.enter = enter;
        this.exit = exit;
        this.lowerIsPresent = lowerIsPresent;
    }

    public static HysteresisThreshold lowerIsPresent(double enter, double exit) {
        return new HysteresisThreshold(enter, exit, true);
    }

    public static HysteresisThreshold higherIsPresent(double enter, double exit) {
        return new HysteresisThreshold(enter, exit, false);
    }

    public void setThresholds(double enter, double exit) {
        this.enter = enter;
        this.exit = exit;
    }

    /** Feeds a valid reading. A NaN value or NaN threshold forces "not present". */
    public boolean update(double value) {
        invalidCount = 0;
        if (Double.isNaN(value) || Double.isNaN(enter) || Double.isNaN(exit)) {
            present = false;
            return false;
        }
        double lo = Math.min(enter, exit);
        double hi = Math.max(enter, exit);
        boolean degenerate = lo == hi;
        if (lowerIsPresent) {
            if (!present) {
                present = value <= lo;
            } else {
                // leave at or above hi (strictly above when the thresholds are equal)
                boolean leave = degenerate ? value > hi : value >= hi;
                if (leave) present = false;
            }
        } else {
            if (!present) {
                present = value >= hi;
            } else {
                // leave at or below lo (strictly below when the thresholds are equal)
                boolean leave = degenerate ? value < lo : value <= lo;
                if (leave) present = false;
            }
        }
        return present;
    }

    /**
     * Like update() for valid readings. An invalid reading holds the last state until maxInvalid
     * (at least 1) invalid readings in a row, then reports "not present".
     */
    public boolean updateOrHold(double value, boolean valid, int maxInvalid) {
        if (valid && !Double.isNaN(value)) {
            return update(value);
        }
        invalidCount++;
        if (invalidCount >= Math.max(1, maxInvalid)) {
            present = false;
        }
        return present;
    }

    public boolean isPresent() {
        return present;
    }

    public void reset() {
        present = false;
        invalidCount = 0;
    }
}
