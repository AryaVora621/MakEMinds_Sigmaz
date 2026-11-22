package commands.autonomous.motion_profiling;

import pedroPathing.Constants;

/**
 * Shared base for profiles that plan a full velocity curve (trapezoid, S-curve) and then follow it
 * with feedforward plus a position correction:
 *
 *   power = kS + kV * v_ref + kA * a_ref + kP * (p_ref - p_actual)
 *
 * The result is only ever allowed to lower stock Pedro's drive power, never raise it, and Pedro's
 * braking is untouched. Once the reference has finished, Pedro's own stopping takes over.
 *
 * Tune order: set the profile limits (subclass), then kV, then kA, then kP.
 */
public abstract class PlannedProfile implements MotionProfile {
    // ---- Tunable constants (shared by trapezoid and S-curve) ---------------
    /** Free speed at full power (in/s). 0 = use Pedro's measured xVelocity from pedroPathing.Constants. */
    public static double FREE_SPEED_IN_S = 0;
    /** Power needed just to start moving (0..1). */
    public static double K_S = 0.05;
    /** Power per (in/s). 0 = automatic: 1 / free speed. */
    public static double K_V = 0;
    /** Power per (in/s^2). Roughly motor time constant / free speed (for example 0.15 s / 80 in/s). */
    public static double K_A = 0.0018;
    /** Power per inch of position error (reference minus actual). */
    public static double K_P = 0.04;
    // ------------------------------------------------------------------------

    private ProfilePlanner.Plan plan;
    private double elapsed;
    private final double[] ref = new double[4];

    /** Builds the plan for a move of the given length. */
    protected abstract ProfilePlanner.Plan plan(double distanceIn);

    private static double freeSpeed() {
        return FREE_SPEED_IN_S > 0 ? FREE_SPEED_IN_S : Constants.driveConstants.getXVelocity();
    }

    @Override
    public void start(double totalDistanceIn) {
        plan = plan(totalDistanceIn);
        elapsed = 0;
        ref[0] = ref[1] = ref[2] = ref[3] = 0;
    }

    @Override
    public double apply(double stockPower, double speedInS, double traveledIn, double dt) {
        if (plan == null || stockPower < 0) {
            return stockPower;
        }
        if (!(dt > 0) || Double.isInfinite(dt)) {
            return stockPower;                      // bad time step: profile effectively off for this loop
        }
        elapsed += dt;
        plan.sample(elapsed, ref);
        if (elapsed >= plan.totalTime) {
            return stockPower;                      // reference finished: Pedro's braking finishes the move
        }
        double kv;
        if (K_V > 0) {
            kv = K_V;
        } else {
            double free = freeSpeed();
            if (!(free > 0) || Double.isInfinite(free)) {
                return stockPower;                  // no valid free speed: profile effectively off (stock power)
            }
            kv = 1.0 / free;
        }
        double command = K_S + kv * ref[1] + K_A * ref[2] + K_P * (ref[0] - traveledIn);
        if (Double.isNaN(command) || Double.isInfinite(command)) {
            return stockPower;
        }
        return Math.min(stockPower, Math.max(0, Math.min(1, command)));
    }

    @Override
    public void stop() {
        plan = null;
        elapsed = 0;
    }

    @Override
    public double referencePosition() { return plan == null ? Double.NaN : ref[0]; }

    @Override
    public double referenceVelocity() { return plan == null ? Double.NaN : ref[1]; }

    @Override
    public double referenceAcceleration() { return plan == null ? Double.NaN : ref[2]; }

    /** Duration of the current plan in seconds, or NaN if none. */
    public double plannedTime() { return plan == null ? Double.NaN : plan.totalTime; }
}
