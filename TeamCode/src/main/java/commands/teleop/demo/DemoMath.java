package commands.teleop.demo;

/** Pure helpers shared by the Demo OpModes (no hardware, unit tested in DemoMathTest). */
public final class DemoMath {
    private DemoMath() {}

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Moves cur toward target by at most maxStep (maxStep <= 0 or NaN: stays). */
    public static double stepToward(double cur, double target, double maxStep) {
        if (!(maxStep > 0)) return cur;
        double d = target - cur;
        return Math.abs(d) <= maxStep ? target : cur + Math.signum(d) * maxStep;
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double wrapRad(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /**
     * Triangle wave between lo and hi: starts at the middle going up, one full lo->hi->lo cycle takes periodS.
     * Returns the middle when periodS is not positive.
     */
    public static double triangle(double tS, double lo, double hi, double periodS) {
        double mid = 0.5 * (lo + hi);
        if (!(periodS > 0) || Double.isNaN(tS)) return mid;
        double ph = ((tS / periodS) % 1.0 + 1.0) % 1.0;           // 0..1
        double u = ph < 0.25 ? ph * 4 : ph < 0.75 ? 2 - ph * 4 : ph * 4 - 4;   // 0 -> 1 -> -1 -> 0
        return mid + u * 0.5 * (hi - lo);
    }

    /** Field (start-frame) error to robot-frame {forward, left}. heading is CCW radians. */
    public static double[] toRobotFrame(double ex, double ey, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{ex * c + ey * s, -ex * s + ey * c};
    }

    /**
     * P controller toward a point: returns {forward, strafeRight, turnClockwise} in [-cap, cap], with a small
     * minimum so the wheels actually move near the target. Zero inside tolIn.
     */
    public static double[] driveToward(double x, double y, double h, double tx, double ty, double th,
                                       double cap, double kPos, double kTurn, double minPower, double tolIn) {
        double ex = tx - x, ey = ty - y;
        double dist = Math.hypot(ex, ey);
        double fwd = 0, left = 0;
        if (dist > tolIn) {
            double[] r = toRobotFrame(ex, ey, h);
            double scale = clamp(kPos * dist, minPower, cap) / dist;   // total speed along the error, floor and cap on its length
            fwd = r[0] * scale;
            left = r[1] * scale;
        }
        double turnErr = wrapRad(th - h);                              // CCW positive
        double turn = Math.abs(turnErr) < Math.toRadians(1.5) ? 0 : clamp(-kTurn * turnErr, -cap, cap);   // drivebase turn is clockwise positive
        return new double[]{clamp(fwd, -cap, cap), clamp(-left, -cap, cap), turn};
    }

    /** One stall-detector step: returns the new "stuck seconds" (resets when the pose moved or no power is commanded). */
    public static double stuckStep(double stuckS, double movedIn, double power, double dt, double minMoveIn, double minPower) {
        if (power < minPower || !(movedIn < minMoveIn)) return 0;
        return stuckS + dt;
    }

    /** Next value of a dpad-adjusted setting: up adds step, down subtracts, clamped to [lo, hi]. */
    public static double adjust(double cur, boolean up, boolean down, double step, double lo, double hi) {
        double v = cur;
        if (up) v += step;
        if (down) v -= step;
        return clamp(v, lo, hi);
    }

    /** Square corner i (0..3 then back to 0) of a side-length square starting at the origin, going forward, left, back, right. */
    public static double[] squareCorner(int i, double side) {
        switch (((i % 4) + 4) % 4) {
            case 0: return new double[]{side, 0};
            case 1: return new double[]{side, side};
            case 2: return new double[]{0, side};
            default: return new double[]{0, 0};
        }
    }
}
