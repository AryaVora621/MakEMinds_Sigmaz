package commands.subsystems.grouped_subsystems.shooter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Drag-free ball-trajectory solver. Java port of the solver verified in the Shooter Simulator
 * (core.js / core.py: 120,000 random cases, landing error below 1e-9 in, JS/Python agreement 9e-14).
 *
 * Units: inches, seconds, radians, Pedro frame (x right, y up, heading CCW from +x). No hardware.
 *
 * Model: the ball leaves the turret pivot P at height z0 with launch velocity u relative to the robot,
 * so its ground velocity is v_s + u_h (v_s = pivot velocity, including the spin of the chassis). It
 * flies a parabola under gravity and must pass through the target point (X, Y, Z) at flight time T.
 * Given one extra requirement (entry angle, hood angle, launch speed, or minimum speed) the flight
 * time T is the root of a quartic, and the aim direction follows from T.
 *
 * Air drag is ignored; calibrate with RobotSettings.TurretAiming.RPM_SCALE on the real robot.
 */
public final class ShotSolver {
    private ShotSolver() {
    }

    public static final double G_IN_S2 = 386.0886;

    /** What extra requirement picks the shot. */
    public enum Mode {
        /** Param = descent angle at the target, radians (0..pi/2). Chooses the launch elevation. */
        ENTRY_ANGLE,
        /** Param = fixed launch elevation (hood angle), radians. Solves for launch speed. */
        HOOD_ANGLE,
        /** Param = launch speed relative to the robot, in/s. Solves for elevation (two arcs). */
        SPEED,
        /** Slowest possible launch; param ignored. */
        MIN_SPEED
    }

    public enum Arc { LOW, HIGH }

    /** Robot state and target for one solve. */
    public static final class Input {
        public double x, y, heading;          // robot pose (in, in, rad)
        public double vx, vy, omega;          // robot field velocity (in/s) and yaw rate (rad/s)
        public double pivotOffsetX, pivotOffsetY;   // turret pivot in the robot frame (+x forward, +y left), in
        public double exitHeight;             // ball exit height above the tiles, in
        public double targetX, targetY, targetZ;
        public boolean allowRising;           // accept shots that reach the target height still rising
    }

    /** Result of a solve. Fields other than ok/reason are only valid when ok is true. */
    public static final class Result {
        public boolean ok;
        public String reason = "";
        public String branchNote = "";
        public double flightTimeS;
        public double launchSpeedInS;         // relative to the robot
        public double elevationRad;           // launch elevation above horizontal
        public double azimuthRad;             // field-frame direction of the horizontal launch velocity
        public double turretRad;              // azimuth minus robot heading, wrapped to (-pi, pi]
        public double entryAngleRad;          // descent angle at the target
        public double distanceIn;             // horizontal pivot-to-target distance
        public double pivotX, pivotY;
        public double pivotVx, pivotVy;
        public double landingErrorIn;         // independent forward-simulation check
        public double minSpeedInS = Double.NaN; // set when the requested speed is below the minimum
    }

    public static Result solve(Input p, Mode mode, double param, Arc arc) {
        Result out = new Result();
        double c = Math.cos(p.heading), s = Math.sin(p.heading);
        double rfx = c * p.pivotOffsetX - s * p.pivotOffsetY;
        double rfy = s * p.pivotOffsetX + c * p.pivotOffsetY;
        double px = p.x + rfx, py = p.y + rfy;
        double vsx = p.vx - p.omega * rfy, vsy = p.vy + p.omega * rfx;
        double dx = p.targetX - px, dy = p.targetY - py;
        double d = Math.hypot(dx, dy);
        double h = p.targetZ - p.exitHeight;
        out.pivotX = px;
        out.pivotY = py;
        out.pivotVx = vsx;
        out.pivotVy = vsy;
        out.distanceIn = d;

        double vs2 = vsx * vsx + vsy * vsy;
        double dv = dx * vsx + dy * vsy;
        double d2 = d * d;
        List<Double> cand = new ArrayList<>();

        switch (mode) {
            case ENTRY_ANGLE: {
                if (!(param > 0 && param < Math.PI / 2)) {
                    out.reason = "entry angle must be in (0, 90) deg";
                    return out;
                }
                if (d < 1e-9) {
                    out.reason = "unreachable: target is directly above/below the exit point (d = 0)";
                    return out;
                }
                double w = 2.0 * (h + d * Math.tan(param)) / G_IN_S2;
                if (w <= 0) {
                    out.reason = "unreachable: target too far below for this entry angle";
                    return out;
                }
                cand.add(Math.sqrt(w));
                break;
            }
            case HOOD_ANGLE: {
                if (!(param > 0 && param < Math.PI / 2)) {
                    out.reason = "hood angle must be in (0, 90) deg";
                    return out;
                }
                double t2 = Math.tan(param) * Math.tan(param);
                for (double t : quarticPosRoots(G_IN_S2 * G_IN_S2 / 4.0, G_IN_S2 * h - t2 * vs2,
                        2.0 * t2 * dv, h * h - t2 * d2)) {
                    if (h + 0.5 * G_IN_S2 * t * t > 0) {
                        cand.add(t);
                    }
                }
                if (cand.isEmpty()) {
                    out.reason = "unreachable: no flight time for this fixed hood angle";
                    return out;
                }
                break;
            }
            case SPEED: {
                cand.addAll(quarticPosRoots(G_IN_S2 * G_IN_S2 / 4.0, vs2 + G_IN_S2 * h - param * param,
                        -2.0 * dv, d2 + h * h));
                if (cand.isEmpty()) {
                    double tm = minSpeedT(dx, dy, vsx, vsy, h);
                    double ux = dx / tm - vsx, uy = dy / tm - vsy;
                    out.minSpeedInS = Math.hypot(Math.hypot(ux, uy), (h + 0.5 * G_IN_S2 * tm * tm) / tm);
                    out.reason = "unreachable: launch speed is below the minimum " + out.minSpeedInS + " in/s";
                    return out;
                }
                break;
            }
            case MIN_SPEED:
                cand.add(minSpeedT(dx, dy, vsx, vsy, h));
                break;
            default:
                throw new IllegalArgumentException("bad mode");
        }

        int before = cand.size();
        if (!p.allowRising) {
            List<Double> keep = new ArrayList<>();
            for (double t : cand) {
                if (0.5 * G_IN_S2 * t * t > h) {
                    keep.add(t);
                }
            }
            cand = keep;
        }
        if (cand.isEmpty()) {
            out.reason = "no descending-arrival solution (every solution reaches the target height while still rising)";
            return out;
        }
        if ((mode == Mode.SPEED || mode == Mode.HOOD_ANGLE) && cand.size() == 1) {
            if (before > 1) {
                out.branchNote = "only one arc arrives descending";
            } else if (mode == Mode.SPEED) {
                out.branchNote = "only one arc (speed equals the minimum)";
            }
        }

        double t = (arc == Arc.LOW || cand.size() == 1) ? cand.get(0) : cand.get(cand.size() - 1);
        double uhx = dx / t - vsx, uhy = dy / t - vsy;
        double uz = (h + 0.5 * G_IN_S2 * t * t) / t;
        double uhm = Math.hypot(uhx, uhy);
        out.flightTimeS = t;
        out.launchSpeedInS = Math.hypot(uhm, uz);
        out.elevationRad = Math.atan2(uz, uhm);
        out.azimuthRad = Math.atan2(uhy, uhx);
        out.turretRad = wrap(out.azimuthRad - p.heading);
        out.entryAngleRad = Math.atan2(0.5 * G_IN_S2 * t * t - h, d);

        // Independent forward check: fly the ball and measure the miss.
        boolean rising = (uz - G_IN_S2 * t) > 0;
        double disc = uz * uz - 2.0 * G_IN_S2 * h;
        if (disc >= 0) {
            double tf = rising ? (uz - Math.sqrt(disc)) / G_IN_S2 : (uz + Math.sqrt(disc)) / G_IN_S2;
            double lx = px + (vsx + uhx) * tf, ly = py + (vsy + uhy) * tf;
            out.landingErrorIn = Math.hypot(lx - p.targetX, ly - p.targetY);
        } else {
            out.landingErrorIn = Double.NaN;
        }
        out.ok = true;
        return out;
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double wrap(double a) {
        double m = (a + Math.PI) % (2 * Math.PI);
        if (m < 0) {
            m += 2 * Math.PI;
        }
        return m - Math.PI;
    }

    /** Flight time of the slowest possible launch (unique positive root of a convex quartic). */
    static double minSpeedT(double dx, double dy, double vsx, double vsy, double h) {
        double dv = dx * vsx + dy * vsy;
        double dd = dx * dx + dy * dy + h * h;
        double lo = 0, hi = 1;
        while (qMin(hi, dv, dd) <= 0) {
            hi *= 2;
        }
        for (int i = 0; i < 200; i++) {
            double mid = 0.5 * (lo + hi);
            if (mid <= lo || mid >= hi) {
                break;
            }
            if (qMin(mid, dv, dd) < 0) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return 0.5 * (lo + hi);
    }

    private static double qMin(double t, double dv, double dd) {
        return 0.5 * G_IN_S2 * G_IN_S2 * t * t * t * t + 2.0 * dv * t - 2.0 * dd;
    }

    /** Real roots of x^3 + p x + q = 0. */
    private static double[] cubicRealRoots(double p, double q) {
        if (Math.abs(p) < 1e-300) {
            return new double[]{Math.cbrt(-q)};
        }
        double disc = (q / 2) * (q / 2) + Math.pow(p / 3, 3);
        if (disc > 0) {
            double sq = Math.sqrt(disc);
            return new double[]{Math.cbrt(-q / 2 + sq) + Math.cbrt(-q / 2 - sq)};
        }
        double r = 2 * Math.sqrt(-p / 3);
        double arg = Math.max(-1, Math.min(1, 3 * q / (2 * p) * Math.sqrt(-3 / p)));
        double phi = Math.acos(arg) / 3;
        double[] out = new double[3];
        for (int k = 0; k < 3; k++) {
            out[k] = r * Math.cos(phi - 2 * Math.PI * k / 3);
        }
        return out;
    }

    /** Positive real roots of a T^4 + b T^2 + c T + e = 0 (a > 0), ascending. */
    static List<Double> quarticPosRoots(double a, double b, double c, double e) {
        double bound = 1 + Math.max(Math.abs(b), Math.max(Math.abs(c), Math.abs(e))) / a;
        List<Double> crit = new ArrayList<>();
        for (double t : cubicRealRoots(b / (2 * a), c / (4 * a))) {
            if (t > 0 && t < bound) {
                crit.add(t);
            }
        }
        Collections.sort(crit);
        List<Double> pts = new ArrayList<>();
        pts.add(0.0);
        pts.addAll(crit);
        pts.add(bound);
        List<Double> roots = new ArrayList<>();
        for (int i = 0; i < pts.size() - 1; i++) {
            double lo = pts.get(i), hi = pts.get(i + 1);
            double flo = quartic(a, b, c, e, lo), fhi = quartic(a, b, c, e, hi);
            if (flo == 0 && lo > 0) {
                roots.add(lo);
            } else if (flo * fhi < 0) {
                for (int it = 0; it < 200; it++) {
                    double mid = 0.5 * (lo + hi);
                    if (mid <= lo || mid >= hi) {
                        break;
                    }
                    double fm = quartic(a, b, c, e, mid);
                    if (fm == 0) {
                        lo = hi = mid;
                        break;
                    }
                    if ((fm < 0) == (flo < 0)) {
                        lo = mid;
                        flo = fm;
                    } else {
                        hi = mid;
                    }
                }
                roots.add(0.5 * (lo + hi));
            }
        }
        for (double t : crit) {   // grazing (double) roots
            double scale = a * Math.pow(t, 4) + Math.abs(b) * t * t + Math.abs(c) * t + Math.abs(e);
            if (Math.abs(quartic(a, b, c, e, t)) <= 1e-12 * scale) {
                boolean fresh = true;
                for (double r : roots) {
                    if (Math.abs(t - r) <= 1e-6 * Math.max(1, t)) {
                        fresh = false;
                        break;
                    }
                }
                if (fresh) {
                    roots.add(t);
                }
            }
        }
        Collections.sort(roots);
        return roots;
    }

    private static double quartic(double a, double b, double c, double e, double t) {
        return (((a * t * t) + b) * t + c) * t + e;
    }
}
