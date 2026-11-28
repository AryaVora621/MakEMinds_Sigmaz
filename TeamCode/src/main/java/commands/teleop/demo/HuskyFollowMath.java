package commands.teleop.demo;

/** Steering for Husky Follow, kept pure so it is unit tested off the robot. HuskyLens frames are 320 x 240. */
public final class HuskyFollowMath {
    private HuskyFollowMath() {}

    public static final double FRAME_W = 320;
    public static final double MAX_FWD = 0.4, MAX_TURN = 0.35;

    /**
     * Returns {forward, turn} that centers the target and holds the distance at which its box was targetWidth px wide.
     * Positive turn is clockwise (toward a target on the right). It never drives forward while the target is far off
     * to the side, so it turns to face it first, and it backs up gently when the target comes too close.
     */
    public static double[] command(double x, double width, double targetWidth) {
        double off = (x - FRAME_W / 2) / (FRAME_W / 2);                  // -1 left .. +1 right
        double turn = clamp(0.5 * off, MAX_TURN);
        double err = (targetWidth - width) / Math.max(targetWidth, 1);   // + = too far
        double fwd = Math.abs(err) < 0.12 ? 0 : clamp(0.6 * err, MAX_FWD);
        if (fwd < 0) fwd = Math.max(fwd, -0.2);                          // back away slowly
        if (Math.abs(off) > 0.5) fwd *= 0.25;
        return new double[] {fwd, turn};
    }

    static double clamp(double v, double m) {
        return Math.max(-m, Math.min(m, v));
    }
}
