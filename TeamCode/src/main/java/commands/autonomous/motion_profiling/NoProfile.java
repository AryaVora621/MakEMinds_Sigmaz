package commands.autonomous.motion_profiling;

/**
 * Base version: no profile limit. Stock Pedro Pathing behaviour, including its full-power step at
 * the start of every move. Use this as the baseline to compare every other profile against.
 */
public class NoProfile implements MotionProfile {
    @Override
    public String name() {
        return "None (stock Pedro)";
    }

    @Override
    public void start(double totalDistanceIn) {
    }

    @Override
    public double apply(double stockPower, double speedInS, double traveledIn, double dt) {
        return stockPower;
    }

    @Override
    public void stop() {
    }
}
