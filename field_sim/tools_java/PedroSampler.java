import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.util.*;

/**
 * Samples a route using the REAL Pedro Pathing 2.1.0 core classes.
 * stdin: JSON route, stdout: JSON samples. Headings in the JSON are DEGREES.
 */
public class PedroSampler {
    // ---------- tiny JSON parser ----------
    static class P {
        final String s; int i = 0;
        P(String s) { this.s = s; }
        void ws() { while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++; }
        Object val() {
            ws();
            char c = s.charAt(i);
            if (c == '{') {
                i++; Map<String, Object> m = new LinkedHashMap<>(); ws();
                if (s.charAt(i) == '}') { i++; return m; }
                while (true) {
                    ws(); String k = str(); ws(); i++; // ':'
                    m.put(k, val()); ws();
                    if (s.charAt(i++) == '}') return m;
                }
            }
            if (c == '[') {
                i++; List<Object> l = new ArrayList<>(); ws();
                if (s.charAt(i) == ']') { i++; return l; }
                while (true) {
                    l.add(val()); ws();
                    if (s.charAt(i++) == ']') return l;
                }
            }
            if (c == '"') return str();
            if (s.startsWith("true", i)) { i += 4; return Boolean.TRUE; }
            if (s.startsWith("false", i)) { i += 5; return Boolean.FALSE; }
            if (s.startsWith("null", i)) { i += 4; return null; }
            int st = i;
            while (i < s.length() && "+-0123456789.eE".indexOf(s.charAt(i)) >= 0) i++;
            if (st == i) throw new RuntimeException("JSON parse error at " + i);
            return Double.parseDouble(s.substring(st, i));
        }
        String str() {
            StringBuilder b = new StringBuilder(); i++; // opening quote
            while (s.charAt(i) != '"') {
                char c = s.charAt(i++);
                if (c == '\\') {
                    char e = s.charAt(i++);
                    switch (e) {
                        case 'n': b.append('\n'); break;
                        case 't': b.append('\t'); break;
                        case 'r': b.append('\r'); break;
                        case 'u': b.append((char) Integer.parseInt(s.substring(i, i + 4), 16)); i += 4; break;
                        default: b.append(e);
                    }
                } else b.append(c);
            }
            i++; return b.toString();
        }
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> obj(Object o) { return (Map<String, Object>) o; }
    @SuppressWarnings("unchecked")
    static List<Object> arr(Object o) { return (List<Object>) o; }
    static double num(Map<String, Object> m, String k, double d) {
        Object o = m == null ? null : m.get(k);
        return o instanceof Double ? (Double) o : d;
    }

    static String q(String s) {
        StringBuilder b = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            if (c == '"' || c == '\\') b.append('\\').append(c);
            else if (c == '\n') b.append("\\n");
            else if (c == '\r') b.append("\\r");
            else if (c < 32) b.append(' ');
            else b.append(c);
        }
        return b.append('"').toString();
    }
    static String d(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "null";
        return String.format(Locale.ROOT, "%.6f", v);
    }

    public static void main(String[] args) throws Exception {
        StringBuilder out = new StringBuilder();
        try {
            InputStream in = System.in;
            ByteArrayOutputStream bo = new ByteArrayOutputStream();
            byte[] buf = new byte[8192]; int n;
            while ((n = in.read(buf)) > 0) bo.write(buf, 0, n);
            Map<String, Object> route = obj(new P(bo.toString("UTF-8").replace("﻿", "")).val());
            run(route, out);
        } catch (Throwable t) {
            java.io.StringWriter sw = new java.io.StringWriter();
            t.printStackTrace(new java.io.PrintWriter(sw));
            out.setLength(0);
            out.append("{\"error\":").append(q(t.toString())).append(",\"trace\":").append(q(sw.toString())).append("}");
        }
        System.out.println(out);
    }

    /**
     * Invert Pedro's own getPathCompletion(t) (a 0..1 arc-length fraction) by bisection.
     * Path.getTFromPathCompletion is NOT used: in 2.1.0 BezierCurve.getT interpolates against
     * absolute distance while BezierLine.getT expects a fraction, so it is inconsistent.
     */
    static double tForFraction(Path path, double f) {
        double lo = 0, hi = 1;
        for (int it = 0; it < 60; it++) {
            double mid = 0.5 * (lo + hi);
            if (path.getPathCompletion(mid) < f) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    static void run(Map<String, Object> route, StringBuilder out) {
        Map<String, Object> start = obj(route.get("start"));
        double sx = num(start, "x", 0), sy = num(start, "y", 0), sh = num(start, "h", 0);
        double step = num(route, "step", 1.0);
        if (step <= 0) step = 1.0;
        List<Object> segs = arr(route.get("segments"));
        double px = sx, py = sy, ph = sh, total = 0;
        out.append("{\"start\":{\"x\":").append(d(sx)).append(",\"y\":").append(d(sy)).append(",\"h\":").append(d(sh)).append("},\"segments\":[");
        for (int si = 0; si < segs.size(); si++) {
            Map<String, Object> sg = obj(segs.get(si));
            String type = String.valueOf(sg.getOrDefault("type", "line"));
            List<Pose> pts = new ArrayList<>();
            for (Object po : arr(sg.get("points"))) {
                Map<String, Object> pm = obj(po);
                pts.add(new Pose(num(pm, "x", 0), num(pm, "y", 0)));
            }
            // a segment given only its end point chains from the previous end
            if (pts.size() == 1) pts.add(0, new Pose(px, py));
            Curve curve;
            if (type.equals("line")) {
                curve = new BezierLine(pts.get(0), pts.get(pts.size() - 1));
            } else {
                curve = new BezierCurve(pts);
            }
            Path path = new Path(curve);
            Map<String, Object> hd = sg.get("heading") == null ? new HashMap<>() : obj(sg.get("heading"));
            String mode = String.valueOf(hd.getOrDefault("mode", "constant"));
            boolean rev = Boolean.TRUE.equals(hd.get("reversed"));
            double hs = num(hd, "start", ph), he = num(hd, "end", hs);
            switch (mode) {
                case "tangent":
                    path.setTangentHeadingInterpolation();
                    if (rev) path.reverseHeadingInterpolation();
                    break;
                case "linear":
                    path.setLinearHeadingInterpolation(Math.toRadians(hs), Math.toRadians(he), 1.0, rev);
                    break;
                default:
                    path.setConstantHeadingInterpolation(Math.toRadians(hs));
            }
            path.init();
            double len = path.length();
            int cnt = Math.max(1, (int) Math.ceil(len / step - 1e-9));
            out.append(si > 0 ? "," : "").append("{\"type\":").append(q(type)).append(",\"mode\":").append(q(mode))
               .append(",\"length\":").append(d(len)).append(",\"offset\":").append(d(total)).append(",\"samples\":[");
            Pose last = null;
            for (int k = 0; k <= cnt; k++) {
                double s = (k == cnt) ? len : k * step;
                double t = (k == 0) ? 0.0 : (k == cnt ? 1.0 : tForFraction(path, s / len));
                Pose p = path.getPose(t);
                last = p;
                out.append(k > 0 ? "," : "").append("{\"x\":").append(d(p.getX())).append(",\"y\":").append(d(p.getY()))
                   .append(",\"h\":").append(d(Math.toDegrees(p.getHeading()))).append(",\"t\":").append(d(t))
                   .append(",\"s\":").append(d(s)).append("}");
            }
            out.append("]}");
            total += len;
            px = last.getX(); py = last.getY(); ph = Math.toDegrees(last.getHeading());
        }
        out.append("],\"total\":").append(d(total)).append("}");
    }
}
