package commands.autonomous.motion_profiling;

/**
 * Power ramp: a ceiling on drive power that starts low and rises linearly with time since the move
 * began. cap(t) = START_POWER + RAMP_PER_S * t. It limits only the forward drive power, never
 * Pedro's braking, so stopping behaviour is unchanged.
 *
 * Cheapest option: removes the launch kick, but the ramp is in power, not in speed, so the real
 * acceleration still depends on battery voltage and floor grip.
 */
public class PowerRampProfile implements MotionProfile {
    // ---- Tunable constants -------------------------------------------------
    public static double START_POWER = 0.30;  // ceiling at the instant a move starts (0..1)
    public static double RAMP_PER_S = 1.50;   // how fast the ceiling rises (power per second)
    // ------------------------------------------------------------------------

    private double elapsed;

    @Override
    public String name() {
        return "Power ramp";
    }

    @Override
    public void start(double totalDistanceIn) {
        elapsed = 0;
    }

    @Override
    public double apply(double stockPower, double speedInS, double traveledIn, double dt) {
        if (dt > 0 && !Double.isInfinite(dt)) {
            elapsed += dt;   // a zero, negative, NaN or infinite step must not move (or poison) the ramp clock
        }
        if (stockPower < 0) {
            return stockPower;
        }
        double cap = Math.min(1.0, START_POWER + RAMP_PER_S * elapsed);
        return Math.min(stockPower, cap);
    }

    @Override
    public void stop() {
        elapsed = 0;
    }
}
