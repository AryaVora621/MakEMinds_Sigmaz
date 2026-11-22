import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.SimClock;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.*;

/**
 * Drives the REAL Pedro Pathing 2.1.0 Follower (error/vector calculators, PIDFs, Kalman-filtered drive error,
 * centripetal correction, hold/brake, path-end constraints, callbacks) against a simulated Localizer and a
 * simulated mecanum Drivetrain. Batch mode: JSON request on stdin, JSON trajectory on stdout.
 * Live mode (--live): one JSON command per stdin line ({"cmd":"init"|"step"|"close"}), one JSON line out each.
 * Headings in JSON are DEGREES; positions are inches; time is seconds.
 */
public class PedroFollowSim {
    static final boolean REALTIME = Boolean.getBoolean("pedro.realtime");   // set when the clock is not patched

    // ---------------- simulated localizer (true state; Pinpoint-like, noise-free) ----------------
    static class SimLocalizer implements Localizer {
        double x, y, h, vx, vy, w;   // field frame
        public Pose getPose() { return new Pose(x, y, h); }
        public Pose getVelocity() { return new Pose(vx, vy, w); }
        public Vector getVelocityVector() { Vector v = new Vector(); v.setOrthogonalComponents(vx, vy); return v; }
        public void setStartPose(Pose p) { setPose(p); }
        public void setPose(Pose p) { x = p.getX(); y = p.getY(); h = p.getHeading(); vx = vy = w = 0; }
        public void update() {}
        public double getTotalHeading() { return h; }
        public double getForwardMultiplier() { return 1; }
        public double getLateralMultiplier() { return 1; }
        public double getTurningMultiplier() { return 1; }
        public void resetIMU() {}
        public double getIMUHeading() { return h; }
        public boolean isNAN() { return false; }
    }

    // ---------------- simulated mecanum drivetrain: the real com.pedropathing.ftc Mecanum.calculateDrive math ----------------
    static class SimMecanum extends Drivetrain {
        double xVel, yVel;
        final double[] powers = new double[4];   // LF, LR, RF, RR
        SimMecanum(double xVel, double yVel, double maxPower) {
            this.xVel = xVel; this.yVel = yVel; this.maxPowerScaling = maxPower; this.nominalVoltage = 12;
            double[] polar = Pose.cartesianToPolar(xVel, -yVel);
            Vector fl = new Vector(polar[0], polar[1]).normalize();
            vectors = new Vector[]{
                new Vector(fl.getMagnitude(), fl.getTheta()),
                new Vector(fl.getMagnitude(), 2 * Math.PI - fl.getTheta()),
                new Vector(fl.getMagnitude(), 2 * Math.PI - fl.getTheta()),
                new Vector(fl.getMagnitude(), fl.getTheta())};
        }
        static double norm(Vector a, Vector b, double max) { return com.pedropathing.math.MathFunctions.findNormalizingScaling(a, b, max); }
        public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
            if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
            if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
            if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);
            double[] wp = new double[4];
            Vector[] mv = new Vector[4];
            Vector[] tp = new Vector[2];
            if (correctivePower.getMagnitude() == maxPowerScaling) {
                tp[0] = correctivePower.copy(); tp[1] = correctivePower.copy();
            } else {
                Vector left = correctivePower.minus(headingPower), right = correctivePower.plus(headingPower);
                if (left.getMagnitude() > maxPowerScaling || right.getMagnitude() > maxPowerScaling) {
                    double s = Math.min(norm(correctivePower, headingPower, maxPowerScaling), norm(correctivePower, headingPower.times(-1), maxPowerScaling));
                    tp[0] = correctivePower.minus(headingPower.times(s));
                    tp[1] = correctivePower.plus(headingPower.times(s));
                } else {
                    Vector lp = left.plus(pathingPower), rp = right.plus(pathingPower);
                    if (lp.getMagnitude() > maxPowerScaling || rp.getMagnitude() > maxPowerScaling) {
                        double s = Math.min(norm(left, pathingPower, maxPowerScaling), norm(right, pathingPower, maxPowerScaling));
                        tp[0] = left.plus(pathingPower.times(s));
                        tp[1] = right.plus(pathingPower.times(s));
                    } else { tp[0] = lp.copy(); tp[1] = rp.copy(); }
                }
            }
            tp[0] = tp[0].times(2.0); tp[1] = tp[1].times(2.0);
            for (int i = 0; i < 4; i++) { mv[i] = vectors[i].copy(); mv[i].rotateVector(robotHeading); }
            wp[0] = (mv[1].getXComponent() * tp[0].getYComponent() - tp[0].getXComponent() * mv[1].getYComponent()) / (mv[1].getXComponent() * mv[0].getYComponent() - mv[0].getXComponent() * mv[1].getYComponent());
            wp[1] = (mv[0].getXComponent() * tp[0].getYComponent() - tp[0].getXComponent() * mv[0].getYComponent()) / (mv[0].getXComponent() * mv[1].getYComponent() - mv[1].getXComponent() * mv[0].getYComponent());
            wp[2] = (mv[3].getXComponent() * tp[1].getYComponent() - tp[1].getXComponent() * mv[3].getYComponent()) / (mv[3].getXComponent() * mv[2].getYComponent() - mv[2].getXComponent() * mv[3].getYComponent());
            wp[3] = (mv[2].getXComponent() * tp[1].getYComponent() - tp[1].getXComponent() * mv[2].getYComponent()) / (mv[2].getXComponent() * mv[3].getYComponent() - mv[3].getXComponent() * mv[2].getYComponent());
            double mx = Math.max(Math.max(Math.abs(wp[0]), Math.abs(wp[1])), Math.max(Math.abs(wp[2]), Math.abs(wp[3])));
            if (mx > maxPowerScaling) for (int i = 0; i < 4; i++) wp[i] = wp[i] / mx * maxPowerScaling;
            return wp;
        }
        public void updateConstants() {}
        public void breakFollowing() { Arrays.fill(powers, 0); }
        public void runDrive(double[] p) { for (int i = 0; i < 4; i++) powers[i] = Double.isNaN(p[i]) ? 0 : Math.max(-1, Math.min(1, p[i])); }
        public void startTeleopDrive() {}
        public void startTeleopDrive(boolean brake) {}
        public double xVelocity() { return xVel; }
        public double yVelocity() { return yVel; }
        public void setXVelocity(double v) { xVel = v; }
        public void setYVelocity(double v) { yVel = v; }
        public double getVoltage() { return 12; }
        public String debugString() { return Arrays.toString(powers); }
        Vector[] vecs() { return vectors; }
    }

    // ---------------- the simulation ----------------
    static final int FOLLOW = 0, WAIT = 1, SETTLE = 2, DONE = 3;

    static class Group { List<Path> paths = new ArrayList<>(); List<double[]> cbs = new ArrayList<>(); List<String> cbNames = new ArrayList<>(); List<Integer> cbPath = new ArrayList<>(); int firstSeg; double waitAfter; boolean[] fired = new boolean[0]; }

    final SimLocalizer loc = new SimLocalizer();
    SimMecanum drive;
    Follower follower;
    FollowerConstants fc = new FollowerConstants();
    PathConstraints pc;
    double dt = 0.005, t = 0, maxTime = 60, settle = 0.3, waitEnd = 0, settleEnd = 0;
    int stride = 2, tickNo = 0, phase = FOLLOW, gi = 0, phaseTicks = 0, curSeg = 0;
    boolean holdEnd = true, done = false;
    String endReason = "";
    List<Group> groups = new ArrayList<>();
    double tauX = 0.18, tauY = 0.20, tauW = 0.10, maxAccel = 220, omegaMax, coastW = 8;
    double maxPower = 1;
    StringBuilder events = new StringBuilder();
    int evCount = 0;

    static double[] arr(Object o, int n) {
        List<Object> l = PedroSampler.arr(o); double[] r = new double[n];
        for (int i = 0; i < n && i < l.size(); i++) r[i] = (Double) l.get(i);
        return r;
    }

    void event(String kind, String name, int seg) {
        if (evCount++ > 0) events.append(',');
        events.append("{\"t\":").append(PedroSampler.d(t)).append(",\"kind\":").append(PedroSampler.q(kind))
              .append(",\"name\":").append(PedroSampler.q(name)).append(",\"seg\":").append(seg).append("}");
    }

    PedroFollowSim(Map<String, Object> req) {
        Map<String, Object> robot = req.get("robot") == null ? new HashMap<>() : PedroSampler.obj(req.get("robot"));
        Map<String, Object> cs = req.get("constants") == null ? new HashMap<>() : PedroSampler.obj(req.get("constants"));
        dt = PedroSampler.num(req, "dt", 0.005);
        if (dt < 0.0005 || dt > 0.05) throw new RuntimeException("dt must be within 0.0005..0.05 s");
        stride = (int) Math.max(1, PedroSampler.num(req, "stride", 2));
        maxTime = PedroSampler.num(req, "maxTime", 60);
        settle = PedroSampler.num(req, "settle", 0.3);
        holdEnd = !Boolean.FALSE.equals(req.get("holdEnd"));
        maxPower = PedroSampler.num(req, "maxPower", PedroSampler.num(robot, "maxPower", 1));

        // Follower constants: library defaults (this is what TeamCode pedroPathing/Constants.java currently uses), then overrides.
        fc.mass = PedroSampler.num(cs, "mass", fc.mass);
        fc.forwardZeroPowerAcceleration = PedroSampler.num(cs, "forwardZeroPowerAcceleration", fc.forwardZeroPowerAcceleration);
        fc.lateralZeroPowerAcceleration = PedroSampler.num(cs, "lateralZeroPowerAcceleration", fc.lateralZeroPowerAcceleration);
        fc.centripetalScaling = PedroSampler.num(cs, "centripetalScaling", fc.centripetalScaling);
        fc.holdPointTranslationalScaling = PedroSampler.num(cs, "holdPointTranslationalScaling", fc.holdPointTranslationalScaling);
        fc.holdPointHeadingScaling = PedroSampler.num(cs, "holdPointHeadingScaling", fc.holdPointHeadingScaling);
        if (cs.get("usePredictiveBraking") instanceof Boolean) fc.usePredictiveBraking = (Boolean) cs.get("usePredictiveBraking");
        if (cs.get("automaticHoldEnd") instanceof Boolean) fc.automaticHoldEnd = (Boolean) cs.get("automaticHoldEnd");
        if (cs.get("translationalPIDF") != null) { double[] a = arr(cs.get("translationalPIDF"), 4); fc.coefficientsTranslationalPIDF = new PIDFCoefficients(a[0], a[1], a[2], a[3]); }
        if (cs.get("headingPIDF") != null) { double[] a = arr(cs.get("headingPIDF"), 4); fc.coefficientsHeadingPIDF = new PIDFCoefficients(a[0], a[1], a[2], a[3]); }
        if (cs.get("drivePIDF") != null) { double[] a = arr(cs.get("drivePIDF"), 5); fc.coefficientsDrivePIDF = new FilteredPIDFCoefficients(a[0], a[1], a[2], a[3], a[4]); }
        fc.driveKalmanFilterModelCovariance = PedroSampler.num(cs, "driveKalmanFilterModelCovariance", fc.driveKalmanFilterModelCovariance);
        fc.driveKalmanFilterDataCovariance = PedroSampler.num(cs, "driveKalmanFilterDataCovariance", fc.driveKalmanFilterDataCovariance);

        Map<String, Object> pcm = req.get("pathConstraints") == null ? new HashMap<>() : PedroSampler.obj(req.get("pathConstraints"));
        // TeamCode Constants.pathConstraints = new PathConstraints(0.99, 100, 1, 1)
        pc = new PathConstraints(PedroSampler.num(pcm, "tValue", 0.99), PedroSampler.num(pcm, "timeoutMs", 100),
                                 PedroSampler.num(pcm, "braking", 1), PedroSampler.num(pcm, "brakingStart", 1));

        // Mecanum defaults from com.pedropathing.ftc MecanumConstants (Constants.java does not override them).
        double xv = PedroSampler.num(robot, "xVelocity", 81.34056), yv = PedroSampler.num(robot, "yVelocity", 65.43028);
        tauX = PedroSampler.num(robot, "tauX", tauX); tauY = PedroSampler.num(robot, "tauY", tauY);
        tauW = PedroSampler.num(robot, "tauW", tauW); maxAccel = PedroSampler.num(robot, "maxAccel", maxAccel);
        coastW = PedroSampler.num(robot, "coastW", coastW);
        omegaMax = PedroSampler.num(robot, "omegaMax", xv / 15.0);   // v / (half track + half wheelbase ~ 15 in)
        drive = new SimMecanum(xv, yv, PedroSampler.num(robot, "maxPower", 1));
        follower = new Follower(fc, loc, drive, pc);
        follower.setMaxPower(maxPower);

        Map<String, Object> start = PedroSampler.obj(req.get("start"));
        double sx = PedroSampler.num(start, "x", 0), sy = PedroSampler.num(start, "y", 0), sh = PedroSampler.num(start, "h", 0);
        follower.setStartingPose(new Pose(sx, sy, Math.toRadians(sh)));
        buildGroups(req, sx, sy, sh);
        startGroup();
    }

    void buildGroups(Map<String, Object> req, double sx, double sy, double sh) {
        List<Object> segs = PedroSampler.arr(req.get("segments"));
        if (segs.isEmpty()) throw new RuntimeException("route has no segments");
        double px = sx, py = sy, ph = sh;
        Group g = new Group(); g.firstSeg = 0;
        for (int si = 0; si < segs.size(); si++) {
            Map<String, Object> sg = PedroSampler.obj(segs.get(si));
            String type = String.valueOf(sg.getOrDefault("type", "line"));
            List<Pose> pts = new ArrayList<>();
            for (Object po : PedroSampler.arr(sg.get("points"))) {
                Map<String, Object> pm = PedroSampler.obj(po);
                pts.add(new Pose(PedroSampler.num(pm, "x", 0), PedroSampler.num(pm, "y", 0)));
            }
            if (pts.size() == 1) pts.add(0, new Pose(px, py));
            Curve curve = type.equals("line") ? new BezierLine(pts.get(0), pts.get(pts.size() - 1)) : new BezierCurve(pts);
            Path path = new Path(curve);
            Map<String, Object> hd = sg.get("heading") == null ? new HashMap<>() : PedroSampler.obj(sg.get("heading"));
            String mode = String.valueOf(hd.getOrDefault("mode", "constant"));
            boolean rev = Boolean.TRUE.equals(hd.get("reversed"));
            double hs = PedroSampler.num(hd, "start", ph), he = PedroSampler.num(hd, "end", hs);
            switch (mode) {
                case "tangent": path.setTangentHeadingInterpolation(); if (rev) path.reverseHeadingInterpolation(); break;
                case "linear": path.setLinearHeadingInterpolation(Math.toRadians(hs), Math.toRadians(he), 1.0, rev); break;
                default: path.setConstantHeadingInterpolation(Math.toRadians(hs)); he = hs;
            }
            Pose last = pts.get(pts.size() - 1);
            px = last.getX(); py = last.getY();
            ph = mode.equals("tangent") ? ph : he;
            int local = g.paths.size();
            g.paths.add(path);
            if (sg.get("callbacks") != null) for (Object co : PedroSampler.arr(sg.get("callbacks"))) {
                Map<String, Object> cm = PedroSampler.obj(co);
                g.cbs.add(new double[]{PedroSampler.num(cm, "t", 1.0), PedroSampler.num(cm, "ms", -1)});
                g.cbNames.add(String.valueOf(cm.getOrDefault("name", "callback")));
                g.cbPath.add(local);
            }
            double wa = PedroSampler.num(sg, "waitAfter", 0);
            if (wa > 0 && si < segs.size() - 1) {
                g.waitAfter = wa; groups.add(g);
                g = new Group(); g.firstSeg = si + 1;
            } else if (si == segs.size() - 1) { g.waitAfter = Math.max(0, wa); }
        }
        groups.add(g);
    }

    PathChain buildChain(Group g) {
        PathBuilder b = follower.pathBuilder(pc);
        for (int i = 0; i < g.paths.size(); i++) {
            b.addPath(g.paths.get(i));
            for (int c = 0; c < g.cbs.size(); c++) {
                if (g.cbPath.get(c) != i) continue;
                final String name = g.cbNames.get(c); final int seg = g.firstSeg + i;
                Runnable r = () -> event("pedroCallback", name, seg);
                if (g.cbs.get(c)[1] >= 0) b.addTemporalCallback(g.cbs.get(c)[1], r);
                else b.addParametricCallback(g.cbs.get(c)[0], r);
            }
        }
        PathChain pcn = b.build(); if (System.getProperty("pedro.debug") != null) System.err.println("callbacks in chain: " + pcn.getCallbacks().size()); return pcn;
    }

    double segStartT = 0;
    /** Reliable callback events: fire each callback when its segment reaches t (or ms elapsed) using Pedro getPathCompletion. Pedro 2.1.0 own chain callbacks are broken (see README), these are reported as pedroCallback. */
    void monitorCallbacks() {
        Group g = groups.get(gi);
        for (int c = 0; c < g.cbs.size(); c++) {
            if (g.fired[c]) continue;
            int seg = g.firstSeg + g.cbPath.get(c);
            boolean ready = curSeg > seg || (curSeg == seg && (g.cbs.get(c)[1] >= 0 ? (t - segStartT) * 1000.0 >= g.cbs.get(c)[1] : follower.getPathCompletion() >= g.cbs.get(c)[0] || follower.atParametricEnd()));
            if (ready) { g.fired[c] = true; event("callback", g.cbNames.get(c), seg); }
        }
    }

    void startGroup() {
        Group g = groups.get(gi);
        phase = FOLLOW; phaseTicks = 0; curSeg = g.firstSeg; segStartT = t; g.fired = new boolean[g.cbs.size()];
        event("pathStart", "group" + gi, g.firstSeg);
        follower.followPath(buildChain(g), maxPower, holdEnd);
    }

    void finishGroupOrAdvance() {
        Group g = groups.get(gi);
        int last = g.firstSeg + g.paths.size() - 1;
        for (int c = 0; c < g.cbs.size(); c++) if (!g.fired[c]) { g.fired[c] = true; event("callback", g.cbNames.get(c), g.firstSeg + g.cbPath.get(c)); }
        event("pathEnd", "group" + gi, last);
        if (g.waitAfter > 0) { phase = WAIT; waitEnd = t + g.waitAfter; event("waitStart", "wait" + gi, last); }
        else { phase = SETTLE; settleEnd = t + settle; }
    }

    /** Integrates the simulated robot over h seconds using the wheel powers last commanded by the Follower. */
    void physics(double h) {
        double[] p = drive.powers;
        Vector[] v = drive.vecs();
        double cx = Math.abs(v[0].getXComponent()), sy = Math.abs(v[0].getYComponent());
        double fx = 0, fy = 0;
        for (int i = 0; i < 4; i++) { fx += p[i] * v[i].getXComponent(); fy += p[i] * v[i].getYComponent(); }
        double cmdX = fx / (4 * cx), cmdY = fy / (4 * sy);
        double cmdW = ((p[2] + p[3]) - (p[0] + p[1])) / 4.0;
        double mag = Math.abs(p[0]) + Math.abs(p[1]) + Math.abs(p[2]) + Math.abs(p[3]);
        double c = Math.cos(loc.h), s = Math.sin(loc.h);
        double rvx = loc.vx * c + loc.vy * s, rvy = -loc.vx * s + loc.vy * c, w = loc.w;
        int n = 4; double hh = h / n;
        for (int k = 0; k < n; k++) {
            if (mag < 0.02) {                        // float mode: coast with the tuned zero-power deceleration
                rvx = coast(rvx, Math.abs(fc.forwardZeroPowerAcceleration), hh);
                rvy = coast(rvy, Math.abs(fc.lateralZeroPowerAcceleration), hh);
                w = coast(w, coastW, hh);
            } else {
                rvx += clamp((cmdX * drive.xVel - rvx) / tauX, maxAccel) * hh;
                rvy += clamp((cmdY * drive.yVel - rvy) / tauY, maxAccel) * hh;
                w += clamp((cmdW * omegaMax - w) / tauW, omegaMax / tauW) * hh;
            }
            double hm = loc.h + w * hh * 0.5;
            double cm = Math.cos(hm), sm = Math.sin(hm);
            loc.vx = rvx * cm - rvy * sm; loc.vy = rvx * sm + rvy * cm; loc.w = w;
            loc.x += loc.vx * hh; loc.y += loc.vy * hh; loc.h += w * hh;
        }
    }
    static double clamp(double a, double m) { return Math.max(-m, Math.min(m, a)); }
    static double coast(double v, double decel, double h) { double d = decel * h; return Math.abs(v) <= d ? 0 : v - Math.signum(v) * d; }

    void tick() {
        if (done) return;
        SimClock.advance((long) Math.round(dt * 1e9));
        if (REALTIME) try { Thread.sleep((long) Math.max(1, Math.round(dt * 1000))); } catch (InterruptedException e) {}
        physics(dt);
        t += dt; tickNo++; phaseTicks++;
        follower.update();
        if (phase == FOLLOW) {
            if (follower.getFollowingPathChain()) { int ns = groups.get(gi).firstSeg + follower.getChainIndex(); if (ns != curSeg) { curSeg = ns; segStartT = t; } }
            monitorCallbacks();
            if (!follower.isBusy() && phaseTicks > 2) finishGroupOrAdvance();
        } else if (phase == WAIT) {
            if (t >= waitEnd) {
                event("waitEnd", "wait" + gi, curSeg);
                if (gi < groups.size() - 1) { gi++; startGroup(); } else { phase = SETTLE; settleEnd = t + settle; }
            }
        } else if (phase == SETTLE) {
            if (t >= settleEnd) { phase = DONE; done = true; endReason = "finished"; event("done", "route", curSeg); }
        }
        if (!done && t >= maxTime) { done = true; phase = DONE; endReason = "timeout"; event("timeout", "maxTime", curSeg); }
    }

    void appendTick(StringBuilder o) {
        o.append("{\"t\":").append(PedroSampler.d(t)).append(",\"x\":").append(PedroSampler.d(loc.x)).append(",\"y\":").append(PedroSampler.d(loc.y))
         .append(",\"h\":").append(PedroSampler.d(Math.toDegrees(loc.h))).append(",\"vx\":").append(PedroSampler.d(loc.vx)).append(",\"vy\":").append(PedroSampler.d(loc.vy))
         .append(",\"w\":").append(PedroSampler.d(Math.toDegrees(loc.w))).append(",\"p\":[");
        for (int i = 0; i < 4; i++) o.append(i > 0 ? "," : "").append(PedroSampler.d(drive.powers[i]));
        o.append("],\"seg\":").append(curSeg).append(",\"phase\":").append(phase).append("}");
    }

    String info() {
        return "{\"dt\":" + PedroSampler.d(dt) + ",\"stride\":" + stride + ",\"realtimeClock\":" + REALTIME
            + ",\"constants\":{\"mass\":" + PedroSampler.d(fc.mass) + ",\"forwardZeroPowerAcceleration\":" + PedroSampler.d(fc.forwardZeroPowerAcceleration)
            + ",\"lateralZeroPowerAcceleration\":" + PedroSampler.d(fc.lateralZeroPowerAcceleration) + ",\"centripetalScaling\":" + PedroSampler.d(fc.centripetalScaling)
            + ",\"xVelocity\":" + PedroSampler.d(drive.xVel) + ",\"yVelocity\":" + PedroSampler.d(drive.yVel) + ",\"omegaMax\":" + PedroSampler.d(omegaMax) + "}}";
    }

    /** Runs up to n ticks, appending every stride-th tick; returns when done or n ticks are done. */
    void run(int n, StringBuilder ticks, int[] count) {
        for (int i = 0; i < n && !done; i++) {
            tick();
            if (tickNo % stride == 0 || done) { if (count[0]++ > 0) ticks.append(','); appendTick(ticks); }
        }
    }

    // ---------------- entry points ----------------
    static String errJson(Throwable t) {
        java.io.StringWriter sw = new java.io.StringWriter();
        t.printStackTrace(new java.io.PrintWriter(sw));
        return "{\"error\":" + PedroSampler.q(t.toString()) + ",\"trace\":" + PedroSampler.q(sw.toString()) + "}";
    }

    static String batch(Map<String, Object> req) {
        PedroFollowSim sim = new PedroFollowSim(req);
        StringBuilder ticks = new StringBuilder(); int[] c = {0};
        sim.appendTick(ticks); c[0] = 1;
        int maxTicks = (int) Math.ceil(sim.maxTime / sim.dt) + 10;
        sim.run(maxTicks, ticks, c);
        return "{\"ok\":true,\"source\":\"pedro-core\",\"finished\":" + sim.endReason.equals("finished") + ",\"endReason\":" + PedroSampler.q(sim.endReason)
            + ",\"duration\":" + PedroSampler.d(sim.t) + ",\"info\":" + sim.info() + ",\"events\":[" + sim.events + "],\"ticks\":[" + ticks + "]}";
    }

    public static void main(String[] args) throws Exception {
        PrintStream out = new PrintStream(System.out, true, "UTF-8");
        if (args.length > 0 && args[0].equals("--live")) { live(out); return; }
        String res;
        try {
            ByteArrayOutputStream bo = new ByteArrayOutputStream();
            InputStream in = System.in; byte[] buf = new byte[8192]; int n;
            while ((n = in.read(buf)) > 0) bo.write(buf, 0, n);
            res = batch(PedroSampler.obj(new PedroSampler.P(bo.toString("UTF-8").replace("﻿", "")).val()));
        } catch (Throwable t) { res = errJson(t); }
        out.println(res);
    }

    static void live(PrintStream out) throws Exception {
        BufferedReader r = new BufferedReader(new InputStreamReader(System.in, "UTF-8"));
        PedroFollowSim sim = null; String line;
        while ((line = r.readLine()) != null) {
            if (line.trim().isEmpty()) continue;
            try {
                Map<String, Object> m = PedroSampler.obj(new PedroSampler.P(line).val());
                String cmd = String.valueOf(m.get("cmd"));
                if (cmd.equals("init")) {
                    sim = new PedroFollowSim(m);
                    StringBuilder f = new StringBuilder(); sim.appendTick(f);
                    out.println("{\"ok\":true,\"source\":\"pedro-core\",\"info\":" + sim.info() + ",\"tick\":" + f + "}");
                } else if (cmd.equals("step")) {
                    if (sim == null) throw new RuntimeException("no session: send init first");
                    StringBuilder ticks = new StringBuilder(); int[] c = {0};
                    sim.run((int) Math.max(1, PedroSampler.num(m, "n", 10)), ticks, c);
                    out.println("{\"ok\":true,\"done\":" + sim.done + ",\"endReason\":" + PedroSampler.q(sim.endReason) + ",\"t\":" + PedroSampler.d(sim.t)
                        + ",\"events\":[" + sim.events + "],\"ticks\":[" + ticks + "]}");
                    sim.events.setLength(0); sim.evCount = 0;
                } else if (cmd.equals("close")) { out.println("{\"ok\":true}"); return; }
                else throw new RuntimeException("unknown cmd " + cmd);
            } catch (Throwable t) { out.println(errJson(t).replace("\n", " ")); }
        }
    }
}
