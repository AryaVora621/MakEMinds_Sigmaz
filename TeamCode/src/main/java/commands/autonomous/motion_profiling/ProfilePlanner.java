package commands.autonomous.motion_profiling;

/**
 * Plans one-dimensional move profiles (distance D, speed limit, acceleration limit, optional jerk
 * limit). Pure math, no hardware: units are inches and seconds. This is the Java version of the
 * engine verified in the Motion Profile Lab.
 *
 * A plan is a list of segments of constant jerk; position, velocity and acceleration are found by
 * integrating from one segment to the next.
 */
public final class ProfilePlanner {
    private ProfilePlanner() {
    }

    /** A planned move. Sample it with {@link #sample(double, double[])}. */
    public static final class Plan {
        private static final int MAX_SEGMENTS = 8;
        private final double[] dur = new double[MAX_SEGMENTS];
        private final double[] a0 = new double[MAX_SEGMENTS];
        private final double[] jerk = new double[MAX_SEGMENTS];
        private final double[] t0 = new double[MAX_SEGMENTS];
        private final double[] p0 = new double[MAX_SEGMENTS];
        private final double[] v0 = new double[MAX_SEGMENTS];
        private int n;

        public final double distance;
        public double totalTime;
        public double peakVelocity;
        public double peakAcceleration;

        Plan(double distance) {
            this.distance = distance;
        }

        /** Appends a segment of the given duration, starting acceleration, and constant jerk. */
        void add(double duration, double startAccel, double j) {
            if (duration <= 0) {
                return;
            }
            double t = 0, p = 0, v = 0;
            if (n > 0) {
                int k = n - 1;
                t = t0[k] + dur[k];
                p = p0[k] + v0[k] * dur[k] + 0.5 * a0[k] * dur[k] * dur[k] + jerk[k] * dur[k] * dur[k] * dur[k] / 6.0;
                v = v0[k] + a0[k] * dur[k] + 0.5 * jerk[k] * dur[k] * dur[k];
            }
            dur[n] = duration;
            a0[n] = startAccel;
            jerk[n] = j;
            t0[n] = t;
            p0[n] = p;
            v0[n] = v;
            n++;
            totalTime = t + duration;
        }

        /** Writes {position, velocity, acceleration, jerk} at time t into out (length >= 4). */
        public void sample(double t, double[] out) {
            if (n == 0 || t <= 0) {
                out[0] = 0;
                out[1] = 0;
                out[2] = n == 0 ? 0 : a0[0];
                out[3] = n == 0 ? 0 : jerk[0];
                return;
            }
            if (t >= totalTime) {
                out[0] = distance;
                out[1] = 0;
                out[2] = 0;
                out[3] = 0;
                return;
            }
            int k = n - 1;
            for (int i = 0; i < n; i++) {
                if (t < t0[i] + dur[i]) {
                    k = i;
                    break;
                }
            }
            double tau = t - t0[k];
            out[0] = p0[k] + v0[k] * tau + 0.5 * a0[k] * tau * tau + jerk[k] * tau * tau * tau / 6.0;
            out[1] = v0[k] + a0[k] * tau + 0.5 * jerk[k] * tau * tau;
            out[2] = a0[k] + jerk[k] * tau;
            out[3] = jerk[k];
        }
    }

    /** Trapezoidal speed profile: constant acceleration, cruise, constant deceleration. */
    public static Plan trapezoid(double distance, double vmax, double amax) {
        Plan plan = new Plan(distance);
        if (!(distance > 1e-9) || !(vmax > 0) || !(amax > 0)
                || Double.isInfinite(distance) || Double.isInfinite(vmax) || Double.isInfinite(amax)) {
            return plan;   // also rejects NaN / infinite limits: empty plan = profile off
        }
        double vp = vmax;
        double dAcc = vp * vp / (2.0 * amax);
        if (2.0 * dAcc > distance) {            // triangle: never reaches vmax
            vp = Math.sqrt(amax * distance);
            dAcc = distance / 2.0;
        }
        double ta = vp / amax;
        double tc = (distance - 2.0 * dAcc) / vp;
        plan.add(ta, amax, 0);
        plan.add(tc, 0, 0);
        plan.add(ta, -amax, 0);
        plan.peakVelocity = vp;
        plan.peakAcceleration = amax;
        return plan;
    }

    /**
     * Seven-phase S-curve: jerk +j, 0, -j, cruise, -j, 0, +j. Handles all four cases (long move,
     * cruise without an acceleration plateau, short move, very short move) by finding the highest
     * peak speed that fits the distance.
     */
    public static Plan sCurve(double distance, double vmax, double amax, double jmax) {
        Plan plan = new Plan(distance);
        if (!(distance > 1e-9) || !(vmax > 0) || !(amax > 0) || !(jmax > 0) || Double.isInfinite(distance)
                || Double.isInfinite(vmax) || Double.isInfinite(amax) || Double.isInfinite(jmax)) {
            return plan;   // also rejects NaN / infinite limits: empty plan = profile off
        }
        double vp = vmax;
        if (2.0 * accelDistance(vp, amax, jmax) > distance) {
            double lo = 0, hi = vmax;
            for (int i = 0; i < 200; i++) {
                double mid = 0.5 * (lo + hi);
                if (2.0 * accelDistance(mid, amax, jmax) > distance) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            vp = 0.5 * (lo + hi);
        }
        double tj, ta, ap;
        if (vp * jmax >= amax * amax) {
            tj = amax / jmax;
            ta = vp / amax - amax / jmax;
            ap = amax;
        } else {
            tj = Math.sqrt(vp / jmax);
            ta = 0;
            ap = tj * jmax;
        }
        double tc = Math.max(0, (distance - 2.0 * accelDistance(vp, amax, jmax)) / vp);
        plan.add(tj, 0, jmax);
        plan.add(ta, ap, 0);
        plan.add(tj, ap, -jmax);
        plan.add(tc, 0, 0);
        plan.add(tj, 0, -jmax);
        plan.add(ta, -ap, 0);
        plan.add(tj, -ap, jmax);
        plan.peakVelocity = vp;
        plan.peakAcceleration = ap;
        return plan;
    }

    /** Distance covered while accelerating from rest to speed vp with the acceleration and jerk limits. */
    static double accelDistance(double vp, double amax, double jmax) {
        double accelTime;
        if (vp * jmax >= amax * amax) {
            accelTime = vp / amax + amax / jmax;     // 2*tj + ta
        } else {
            accelTime = 2.0 * Math.sqrt(vp / jmax);  // 2*tj
        }
        return 0.5 * vp * accelTime;
    }
}
