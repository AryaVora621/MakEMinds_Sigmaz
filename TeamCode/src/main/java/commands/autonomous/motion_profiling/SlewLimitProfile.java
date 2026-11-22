package commands.autonomous.motion_profiling;

/**
 * Slew limit: the drive power may rise by at most RISE_PER_S each second, relative to what was sent
 * on the previous loop. Falling power (and Pedro's braking) is never limited.
 *
 * Differs from the power ramp: a ramp is a ceiling that depends on time since the move started; a
 * slew limit follows the command itself, so it also smooths any later jump in Pedro's request
 * (for example when a new path in a chain starts).
 */
public class SlewLimitProfile implements MotionProfile {
    // ---- Tunable constants -------------------------------------------------
    public static double START_POWER = 0.20;  // power assumed on the first loop of a move (0..1)
    public static double RISE_PER_S = 2.50;   // maximum increase in power per second
    // ------------------------------------------------------------------------

    private double previous;

    @Override
    public String name() {
        return "Slew limit";
    }

    @Override
    public void start(double totalDistanceIn) {
        previous = START_POWER;
    }

    @Override
    public double apply(double stockPower, double speedInS, double traveledIn, double dt) {
        if (stockPower < 0) {
            previous = 0;
            return stockPower;
        }
        double step = dt > 0 && !Double.isInfinite(dt) ? RISE_PER_S * dt : 0;   // bad dt: hold, never go negative/NaN
        double out = stockPower > previous ? Math.min(stockPower, previous + step) : stockPower;
        previous = out;
        return out;
    }

    @Override
    public void stop() {
        previous = 0;
    }
}
