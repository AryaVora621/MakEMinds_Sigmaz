package commands.subsystems.gate;

/** Hardware-free helpers for Gate. */
public final class GateLogic {
    private GateLogic() {}

    /** Clamps a raw time step to [0, maxDtS]; NaN or negative gives 0. */
    public static double clampDt(double dt, double maxDtS) {
        if (!(dt > 0)) return 0;
        return Math.min(dt, maxDtS);
    }
}
