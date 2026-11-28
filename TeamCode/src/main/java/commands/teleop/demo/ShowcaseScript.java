package commands.teleop.demo;

/** Pure timeline of the Showcase demo (no hardware, unit tested): what the turret and flywheel should do at time t. */
public final class ShowcaseScript {
    private ShowcaseScript() {}

    /** Total length of the show, seconds. */
    public static final double DURATION_S = 12;
    static final double TURRET_SWEEP_END_S = 10;
    static final double TURRET_PERIOD_S = 4;
    static final double SPIN_START_S = 2, SPIN_FULL_S = 5, SPIN_HOLD_END_S = 8, SPIN_STOP_S = 11;

    /** Turret target (degrees): sweeps left-right-left through the middle until 10 s, then rests at 0. */
    public static double turretDeg(double tS, double spanDeg) {
        if (!(tS >= 0) || tS >= TURRET_SWEEP_END_S) return 0;
        return DemoMath.triangle(tS, -spanDeg, spanDeg, TURRET_PERIOD_S);
    }

    /** Flywheel RPM request: 0 until 2 s, ramps to peak by 5 s, holds to 8 s, ramps down to 0 by 11 s. */
    public static double rpm(double tS, double peakRpm) {
        if (!(tS > SPIN_START_S) || tS >= SPIN_STOP_S) return 0;
        if (tS < SPIN_FULL_S) return peakRpm * (tS - SPIN_START_S) / (SPIN_FULL_S - SPIN_START_S);
        if (tS <= SPIN_HOLD_END_S) return peakRpm;
        return peakRpm * (SPIN_STOP_S - tS) / (SPIN_STOP_S - SPIN_HOLD_END_S);
    }

    public static String phase(double tS) {
        if (tS < SPIN_START_S) return "turret sweep begins";
        if (tS < SPIN_FULL_S) return "turret sweeping, flywheel spinning UP";
        if (tS <= SPIN_HOLD_END_S) return "turret sweeping, flywheel HOLD at peak";
        if (tS < TURRET_SWEEP_END_S) return "turret sweeping, flywheel spinning DOWN";
        if (tS < SPIN_STOP_S) return "turret returning to centre, flywheel spinning DOWN";
        if (tS < DURATION_S) return "settling";
        return "done";
    }
}
