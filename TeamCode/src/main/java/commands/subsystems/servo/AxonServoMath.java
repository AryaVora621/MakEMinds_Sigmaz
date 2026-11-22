package commands.subsystems.servo;

/** Pure math helpers for {@link AxonServo}. No FTC imports so it can be unit-tested on a PC. */
final class AxonServoMath {
    private AxonServoMath() {}

    /** Wraps an angle into [0, 360). */
    static double wrap360(double deg) {
        double r = deg % 360.0;
        if (r < 0) r += 360.0;
        if (r >= 360.0) r = 0.0; // guards against -1e-17 + 360 rounding up to 360
        return r;
    }

    /** Wraps an angle into (-180, 180]. */
    static double wrap180(double deg) {
        double r = wrap360(deg);
        return r > 180.0 ? r - 360.0 : r;
    }

    /** Shortest signed error from measured to target, in (-180, 180]. */
    static double angleError(double targetDeg, double measuredDeg) {
        return wrap180(targetDeg - measuredDeg);
    }

    /** Clamps volts into [vMin, vMax] and maps linearly to [0, fullScaleDeg]. */
    static double voltageToDeg(double volts, double vMin, double vMax, double fullScaleDeg) {
        if (vMax <= vMin) return 0.0;
        double v = clamp(volts, vMin, vMax);
        return (v - vMin) / (vMax - vMin) * fullScaleDeg;
    }

    /**
     * Re-centres a feedback angle on the travel window: removes the offset, then wraps into
     * [travel/2 - 180, travel/2 + 180] so a reading just below 0 is small and negative, not ~360.
     */
    static double recenter(double framedDeg, double offsetDeg, double travelDeg) {
        double mid = travelDeg / 2;
        return mid + wrap180(framedDeg - offsetDeg - mid);
    }

    /**
     * Moves current toward target by at most maxStepDeg. maxStepDeg <= 0 returns current (the
     * caller decides whether "no slew limit" means jump). A NaN current returns target.
     */
    static double slewStep(double currentDeg, double targetDeg, double maxStepDeg) {
        if (Double.isNaN(currentDeg)) return targetDeg;
        if (maxStepDeg <= 0) return currentDeg;
        double diff = targetDeg - currentDeg;
        if (Math.abs(diff) <= maxStepDeg) return targetDeg;
        return currentDeg + Math.signum(diff) * maxStepDeg;
    }

    /** Caps a time step to [0, maxDtS] so the first update after init or a pause cannot jump. NaN gives 0. */
    static double capDt(double dtS, double maxDtS) {
        if (Double.isNaN(dtS) || dtS <= 0) return 0.0;
        return maxDtS > 0 ? Math.min(dtS, maxDtS) : dtS;
    }

    /** Position used by callers: measured only when feedback is trusted and valid, else the commanded angle. */
    static double effectivePosition(boolean trusted, double measuredDeg, double commandDeg) {
        return trusted && !Double.isNaN(measuredDeg) ? measuredDeg : commandDeg;
    }

    /** True if the command is at target and the measured error exceeds tolerance; never true when untrusted or NaN. */
    static boolean outOfTolerance(boolean trusted, double commandDeg, double targetDeg,
                                  double measuredDeg, double tolDeg) {
        return trusted && !Double.isNaN(measuredDeg) && commandDeg == targetDeg
                && Math.abs(targetDeg - measuredDeg) > tolDeg;
    }

    /** True if the feedback moved less than stableDeg since the last loop; any NaN reading counts as not still. */
    static boolean feedbackStill(double framedDeg, double lastFramedDeg, double stableDeg) {
        if (Double.isNaN(framedDeg) || Double.isNaN(lastFramedDeg)) return false;
        return Math.abs(wrap180(framedDeg - lastFramedDeg)) < stableDeg;
    }

    /** Converts degrees of travel to a servo position in [0, 1]. */
    static double degToPosition(double deg, double travelDeg) {
        if (travelDeg <= 0) return 0.0;
        return clamp(deg / travelDeg, 0.0, 1.0);
    }

    /** Converts a servo position (0..1) to degrees of travel. */
    static double positionToDeg(double pos, double travelDeg) {
        return pos * travelDeg;
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
