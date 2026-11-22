package commands.autonomous.motion_profiling;

/**
 * Seven-phase S-curve profile: acceleration itself ramps up and down (limited jerk), so the robot
 * never gets a sudden kick. Costs about a_max / j_max extra seconds per move versus a trapezoid.
 * Shared feedforward gains are in {@link PlannedProfile}.
 */
public class SCurveProfile extends PlannedProfile {
    // ---- Tunable constants -------------------------------------------------
    public static double V_MAX_IN_S = 60;     // cruise speed; about 0.8 x free speed
    public static double A_MAX_IN_S2 = 90;    // below the wheel-slip limit; about 0.85 x mu * g
    public static double J_MAX_IN_S3 = 900;   // start near 20 x a_max / (trapezoid move time); lower = smoother
    // ------------------------------------------------------------------------

    @Override
    public String name() {
        return "S-curve";
    }

    @Override
    protected ProfilePlanner.Plan plan(double distanceIn) {
        return ProfilePlanner.sCurve(distanceIn, V_MAX_IN_S, A_MAX_IN_S2, J_MAX_IN_S3);
    }
}
