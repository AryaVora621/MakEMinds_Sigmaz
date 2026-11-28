package commands.teleop.demo;

/**
 * One drive command from the laptop link, parsed from a line "seq forward strafe turn ms" (or "seq stop").
 * Pure Java so it is unit tested off the robot. Values are clamped here, so a typo on the laptop can never
 * ask for more than the caps below.
 */
public final class RemoteCommand {
    public static final double POWER_CAP = 0.15; // owner 11/28: this robot is fast, drive really slow
    public static final int MAX_MS = 1000;

    public final long seq;
    public final double forward, strafe, turn;
    public final int ms;
    public final boolean stop;

    private RemoteCommand(long seq, double f, double s, double t, int ms, boolean stop) {
        this.seq = seq; this.forward = f; this.strafe = s; this.turn = t; this.ms = ms; this.stop = stop;
    }

    /** Returns null for anything malformed: a bad line must never move the robot. */
    public static RemoteCommand parse(String line) {
        if (line == null) return null;
        String[] p = line.trim().split("\\s+");
        try {
            long seq = Long.parseLong(p[0]);
            if (p.length == 2 && p[1].equalsIgnoreCase("stop")) return new RemoteCommand(seq, 0, 0, 0, 0, true);
            if (p.length != 5) return null;
            double f = clamp(Double.parseDouble(p[1])), s = clamp(Double.parseDouble(p[2])), t = clamp(Double.parseDouble(p[3]));
            int ms = Integer.parseInt(p[4]);
            if (ms <= 0) return null;
            return new RemoteCommand(seq, f, s, t, Math.min(ms, MAX_MS), false);
        } catch (RuntimeException e) {
            return null;
        }
    }

    static double clamp(double v) {
        if (Double.isNaN(v)) return 0;
        return Math.max(-POWER_CAP, Math.min(POWER_CAP, v));
    }
}
