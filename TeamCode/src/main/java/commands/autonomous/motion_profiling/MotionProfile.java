package commands.autonomous.motion_profiling;

/**
 * A drive-power shaper for Pedro Pathing. One instance is installed in AutoDrivebase; swapping the
 * class named there swaps the whole behaviour.
 *
 * Each loop the profile is shown what stock Pedro wants to drive at (stockPower, 0..1 along the
 * path) and returns the power it allows. A negative stockPower is Pedro braking: profiles must pass
 * it through untouched.
 *
 * Units: inches and seconds.
 */
public interface MotionProfile {
    /** Short name for telemetry. */
    String name();

    /** A new move begins. totalDistanceIn is the length of the whole path or chain. */
    void start(double totalDistanceIn);

    /**
     * Called once per loop while a move is running.
     *
     * @param stockPower the drive power stock Pedro would use (negative = braking)
     * @param speedInS   current speed along the path (in/s)
     * @param traveledIn distance travelled along the whole move so far (in)
     * @param dt         seconds since the last call
     * @return the drive power to use
     */
    double apply(double stockPower, double speedInS, double traveledIn, double dt);

    /** The move ended or was cancelled. */
    void stop();

    /** Reference position along the move (in), or NaN if this profile has none. For logging. */
    default double referencePosition() { return Double.NaN; }

    /** Reference speed (in/s), or NaN. For logging. */
    default double referenceVelocity() { return Double.NaN; }

    /** Reference acceleration (in/s^2), or NaN. For logging. */
    default double referenceAcceleration() { return Double.NaN; }
}
