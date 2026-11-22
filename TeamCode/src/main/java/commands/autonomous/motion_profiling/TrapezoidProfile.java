package commands.autonomous.motion_profiling;

/**
 * Trapezoidal speed profile: constant acceleration up to a cruise speed, then constant deceleration.
 * Acceleration jumps instantly at the corners (infinite jerk), but the launch is far gentler than
 * stock Pedro. Shared feedforward gains are in {@link PlannedProfile}.
 */
public class TrapezoidProfile extends PlannedProfile {
    // ---- Tunable constants -------------------------------------------------
    public static double V_MAX_IN_S = 60;     // cruise speed; about 0.8 x free speed
    public static double A_MAX_IN_S2 = 90;    // below the wheel-slip limit; about 0.85 x mu * g
    // ------------------------------------------------------------------------

    @Override
    public String name() {
        return "Trapezoid";
    }

    @Override
    protected ProfilePlanner.Plan plan(double distanceIn) {
        return ProfilePlanner.trapezoid(distanceIn, V_MAX_IN_S, A_MAX_IN_S2);
    }
}
