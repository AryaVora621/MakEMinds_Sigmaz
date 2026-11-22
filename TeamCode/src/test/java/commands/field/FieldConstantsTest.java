package commands.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeTrue;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Checks FieldConstants against the field CAD export field_sim/data/field_model/field_constants.json.
 * The JSON is read at test time, so a change to either side that makes them disagree fails here.
 * If the file is absent (a machine without field_sim) the CAD tests are skipped with an assumption message.
 *
 * Tolerance: TOL = 0.05 in. The CAD values are rounded to 3 decimals and FieldConstants rounds some to 2.
 *
 * Intentional divergences, NOT asserted equal to the CAD:
 *   FIELD_SIZE_IN 144 (OFFICIAL 9.2 nominal; CAD interior between glass is 141.35),
 *   FLOWER_OPENING_HEIGHT_IN 21.5 (rule book 9.7; CAD 21.404),
 *   FLOWER_OPENING_DIAMETER_IN 4 (rule book; CAD 3.96),
 *   LOADING_ZONE_LENGTH_IN / GARDEN_LENGTH_IN 23 (rule book; CAD tape 22.69).
 * Those are only checked to be within a sane distance of the CAD value, so a typo is still caught.
 */
public class FieldConstantsTest {
    private static final double TOL = 0.05;
    private static final String REL = "field_sim/data/field_model/field_constants.json";
    private static Map<String, Object> cad;     // null when the file is absent

    @BeforeClass
    public static void loadCad() throws IOException {
        File f = findCadFile();
        if (f != null) {
            cad = Json.asMap(new Json(new String(Files.readAllBytes(f.toPath()), StandardCharsets.UTF_8)).parse());
        }
    }

    /** Tries the working directory (the module dir under Gradle) and its parents (the repo root). */
    private static File findCadFile() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        for (int i = 0; i < 4 && dir != null; i++) {
            File f = new File(dir, REL);
            if (f.isFile()) {
                return f;
            }
            dir = dir.getParentFile();
        }
        return null;
    }

    private static void needCad() {
        assumeTrue("SKIPPED: " + REL + " not found from " + System.getProperty("user.dir")
                + " (no field_sim on this machine)", cad != null);
    }

    // ---- CAD comparisons ----------------------------------------------------

    private static Map<String, Object> hive(String color) {
        return Json.asMap(Json.asMap(cad.get("hives")).get(color));
    }

    private static Map<String, Object> cell(String color, String state, String cell) {
        Map<String, Object> st = Json.asMap(Json.asMap(hive(color).get("states")).get(state));
        return Json.asMap(Json.asMap(st.get("cells")).get(cell));
    }

    @Test
    public void hivePivotMatchesCad() {
        needCad();
        double[] red = Json.nums(hive("red").get("pivot"));
        double[] blue = Json.nums(hive("blue").get("pivot"));
        assertEquals(FieldConstants.RED_HIVE_X_IN, red[0], TOL);
        assertEquals(FieldConstants.BLUE_HIVE_X_IN, blue[0], TOL);
        assertEquals(FieldConstants.HIVE_CENTER_Y_IN, red[1], TOL);
        assertEquals(FieldConstants.HIVE_CENTER_Y_IN, blue[1], TOL);
        assertEquals(FieldConstants.HIVE_PIVOT_HEIGHT_IN, red[2], TOL);
        assertEquals(FieldConstants.HIVE_PIVOT_HEIGHT_IN, blue[2], TOL);
        assertEquals(FieldConstants.HIVE_SEPARATION_IN, blue[0] - red[0], TOL);
        assertEquals(FieldConstants.TIP_ANGLE_DEG, Json.num(hive("red").get("tilt_deg")), TOL);
    }

    @Test
    public void aimPointMatchesCadInBothStatesAndAlliances() {
        needCad();
        for (String color : new String[]{"red", "blue"}) {
            Alliance a = color.equals("red") ? Alliance.RED : Alliance.BLUE;
            // far cell up is state far_up; audience cell up is state audience_up.
            double[] far = Json.nums(cell(color, "far_up", "far").get("aim_point"));
            double[] aud = Json.nums(cell(color, "audience_up", "audience").get("aim_point"));
            assertEquals(color + " far x", FieldConstants.aimX(a), far[0], TOL);
            assertEquals(color + " far y", FieldConstants.aimY(true), far[1], TOL);
            assertEquals(color + " far z", FieldConstants.UP_OPENING_CENTER_Z_IN, far[2], TOL);
            assertEquals(color + " aud x", FieldConstants.aimX(a), aud[0], TOL);
            assertEquals(color + " aud y", FieldConstants.aimY(false), aud[1], TOL);
            assertEquals(color + " aud z", FieldConstants.UP_OPENING_CENTER_Z_IN, aud[2], TOL);
            // Offset from the pivot (y = 72) is the same on both sides.
            assertEquals(FieldConstants.UP_CELL_OFFSET_Y_IN, far[1] - 72, TOL);
            assertEquals(FieldConstants.UP_CELL_OFFSET_Y_IN, 72 - aud[1], TOL);
            // The raised cell flags itself "up" in the JSON.
            assertEquals(Boolean.TRUE, cell(color, "far_up", "far").get("up"));
            assertEquals(Boolean.TRUE, cell(color, "audience_up", "audience").get("up"));
        }
    }

    @Test
    public void upOpeningHeightAndNormalMatchCad() {
        needCad();
        Map<String, Object> c = cell("red", "audience_up", "audience");
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (Object corner : Json.asList(c.get("opening_corners"))) {
            double z = Json.nums(corner)[2];
            lo = Math.min(lo, z);
            hi = Math.max(hi, z);
        }
        // Manual Fig 9-10 gives 53.5 / 65.6; CAD 53.466 / 65.590.
        assertEquals(FieldConstants.UP_OPENING_BOTTOM_Z_IN, lo, TOL);
        assertEquals(FieldConstants.UP_OPENING_TOP_Z_IN, hi, TOL);
        double[] n = Json.nums(c.get("opening_normal"));
        double[] mine = FieldConstants.openingNormal(false);
        for (int i = 0; i < 3; i++) {
            assertEquals("normal " + i, mine[i], n[i], 0.01);   // CAD normal is rounded to 3 decimals
        }
        double[] nf = Json.nums(cell("red", "far_up", "far").get("opening_normal"));
        double[] minef = FieldConstants.openingNormal(true);
        for (int i = 0; i < 3; i++) {
            assertEquals("far normal " + i, minef[i], nf[i], 0.01);
        }
    }

    @Test
    public void flowerPositionsMatchCad() {
        needCad();
        List<Object> flowers = Json.asList(cad.get("flowers"));
        assertEquals(FieldConstants.FLOWER_POSITIONS_IN.length, flowers.size());
        // Order differs between the two (JSON: far, audience, red, blue), so match each constant to a distinct CAD flower.
        Set<Integer> used = new HashSet<>();
        for (double[] p : FieldConstants.FLOWER_POSITIONS_IN) {
            int found = -1;
            for (int i = 0; i < flowers.size(); i++) {
                double[] c = Json.nums(Json.asMap(flowers.get(i)).get("opening_center"));
                if (Math.abs(c[0] - p[0]) <= TOL && Math.abs(c[1] - p[1]) <= TOL) {
                    found = i;
                }
            }
            assertTrue("no CAD flower near " + Arrays.toString(p), found >= 0);
            assertTrue("flower matched twice", used.add(found));
        }
        // Intentional divergences: rule book values kept. Only a sanity bound against the CAD.
        Map<String, Object> f0 = Json.asMap(flowers.get(0));
        assertEquals(21.5, FieldConstants.FLOWER_OPENING_HEIGHT_IN, 0);
        assertEquals(Json.num(f0.get("opening_height")), FieldConstants.FLOWER_OPENING_HEIGHT_IN, 0.2);
        assertEquals(4, FieldConstants.FLOWER_OPENING_DIAMETER_IN, 0);
        assertEquals(2 * Json.num(f0.get("radius")), FieldConstants.FLOWER_OPENING_DIAMETER_IN, 0.1);
    }

    private static double[] box(String zone) {
        Map<String, Object> z = Json.asMap(Json.asMap(cad.get("zones")).get(zone));
        return new double[]{Json.num(z.get("xmin")), Json.num(z.get("ymin")),
                Json.num(z.get("xmax")), Json.num(z.get("ymax"))};
    }

    private static void assertBox(String what, double[] expected, double[] actual) {
        assertEquals(what + " length", expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertEquals(what + "[" + i + "]", expected[i], actual[i], TOL);
        }
    }

    @Test
    public void loadingZonesAndGardensMatchCad() {
        needCad();
        assertBox("red loading zone", box("red_loading_zone"), FieldConstants.RED_LOADING_ZONE_IN);
        assertBox("blue loading zone", box("blue_loading_zone"), FieldConstants.BLUE_LOADING_ZONE_IN);
        assertBox("red garden", box("red_garden"), FieldConstants.RED_GARDEN_IN);
        assertBox("blue garden", box("blue_garden"), FieldConstants.BLUE_GARDEN_IN);
        // The rule book says 23 in, the CAD tape is 22.69 in (intentional: rule book kept).
        double[] r = box("red_garden");
        assertEquals(FieldConstants.GARDEN_LENGTH_IN, r[2] - r[0], 0.5);
        assertEquals(FieldConstants.GARDEN_WIDTH_IN, r[3] - r[1], TOL);
        double[] l = box("red_loading_zone");
        assertEquals(FieldConstants.LOADING_ZONE_LENGTH_IN, l[3] - l[1], 0.5);
        assertEquals(FieldConstants.LOADING_ZONE_WIDTH_IN, l[2] - l[0], TOL);
    }

    @Test
    public void tagIdsMatchCadWorldXOrder() {
        needCad();
        Map<String, Object> tags = Json.asMap(cad.get("apriltags"));
        assertEquals(16, tags.size());
        assertEquals(FieldConstants.TAG_SIZE_IN, Json.num(Json.asMap(tags.get("30")).get("size")), TOL);
        int[][] clusters = {FieldConstants.RED_FAR_TAG_IDS, FieldConstants.RED_AUDIENCE_TAG_IDS,
                FieldConstants.BLUE_AUDIENCE_TAG_IDS, FieldConstants.BLUE_FAR_TAG_IDS};
        String[] alliance = {"red", "red", "blue", "blue"};
        String[] side = {"far", "audience", "audience", "far"};
        for (int k = 0; k < clusters.length; k++) {
            int[] ids = clusters[k];
            double centre = k < 2 ? FieldConstants.RED_HIVE_X_IN : FieldConstants.BLUE_HIVE_X_IN;
            for (int i = 0; i < ids.length; i++) {
                Map<String, Object> tag = Json.asMap(tags.get(String.valueOf(ids[i])));
                assertEquals("id " + ids[i], alliance[k], tag.get("alliance"));
                assertEquals("id " + ids[i], side[k], tag.get("side"));
                // Arrays are ordered by ascending world x, so tag x = cluster centre + offset[i].
                for (String state : new String[]{"audience_up", "far_up"}) {
                    double x = Json.nums(Json.asMap(Json.asMap(tag.get("states")).get(state)).get("center"))[0];
                    assertEquals("id " + ids[i] + " " + state, centre + FieldConstants.TAG_OFFSETS_IN[i], x, 0.1);
                }
            }
        }
    }

    @Test
    public void fieldSizeIsNominalNotCadInterior() {
        needCad();
        // Intentional: nominal 144 (OFFICIAL 9.2), not the CAD interior 141.35. Both are documented in the JSON.
        double[] size = Json.nums(Json.asMap(cad.get("field")).get("size"));
        assertEquals(size[0], FieldConstants.FIELD_SIZE_IN, 0);
        double[] interior = Json.nums(Json.asMap(Json.asMap(cad.get("field")).get("walls")).get("interior_size_cad"));
        assertEquals(interior[0], FieldConstants.FIELD_SIZE_IN, 3.0);   // manual +-1 in plus CAD error
    }

    // ---- Internal consistency (no CAD needed) -------------------------------

    @Test
    public void aimXPerAlliance() {
        assertEquals(FieldConstants.RED_HIVE_X_IN, FieldConstants.aimX(Alliance.RED), 0);
        assertEquals(FieldConstants.BLUE_HIVE_X_IN, FieldConstants.aimX(Alliance.BLUE), 0);
        assertEquals(59.25, FieldConstants.RED_HIVE_X_IN, 1e-12);   // OFFICIAL Fig 9-10 spacing 25.5 about x = 72
        assertEquals(84.75, FieldConstants.BLUE_HIVE_X_IN, 1e-12);
    }

    @Test
    public void aimYIsOffsetFromPivotEitherSide() {
        assertEquals(72 + FieldConstants.UP_CELL_OFFSET_Y_IN, FieldConstants.aimY(true), 1e-12);
        assertEquals(72 - FieldConstants.UP_CELL_OFFSET_Y_IN, FieldConstants.aimY(false), 1e-12);
        assertEquals(2 * FieldConstants.UP_CELL_OFFSET_Y_IN, FieldConstants.aimY(true) - FieldConstants.aimY(false), 1e-12);
    }

    @Test
    public void redAndBlueAimAreSymmetricThroughFieldCentre() {
        double cx = FieldConstants.HIVE_CENTER_X_IN;
        assertEquals(2 * cx, FieldConstants.aimX(Alliance.RED) + FieldConstants.aimX(Alliance.BLUE), 1e-12);
        assertEquals(2 * FieldConstants.HIVE_CENTER_Y_IN, FieldConstants.aimY(true) + FieldConstants.aimY(false), 1e-12);
    }

    @Test
    public void tagArraysAreAPermutationOf30To45() {
        int[][] clusters = {FieldConstants.RED_FAR_TAG_IDS, FieldConstants.RED_AUDIENCE_TAG_IDS,
                FieldConstants.BLUE_AUDIENCE_TAG_IDS, FieldConstants.BLUE_FAR_TAG_IDS};
        Set<Integer> seen = new HashSet<>();
        for (int[] c : clusters) {
            assertEquals(FieldConstants.TAG_OFFSETS_IN.length, c.length);
            for (int id : c) {
                assertTrue("id " + id, id >= 30 && id <= 45);
                assertTrue("duplicate " + id, seen.add(id));
            }
        }
        assertEquals(16, seen.size());
        double[] o = FieldConstants.TAG_OFFSETS_IN;
        for (int i = 0; i < o.length; i++) {
            assertEquals(-o[i], o[o.length - 1 - i], 0);
            if (i > 0) {
                assertTrue(o[i] > o[i - 1]);
            }
        }
    }

    @Test
    public void zonesAndGardensMirrorThroughFieldCentre() {
        double s = FieldConstants.FIELD_SIZE_IN;
        double[][][] pairs = {{FieldConstants.RED_LOADING_ZONE_IN, FieldConstants.BLUE_LOADING_ZONE_IN},
                {FieldConstants.RED_GARDEN_IN, FieldConstants.BLUE_GARDEN_IN}};
        for (double[][] p : pairs) {
            double[] r = p[0], b = p[1];
            assertEquals(s - r[2], b[0], 1e-9);
            assertEquals(s - r[3], b[1], 1e-9);
            assertEquals(s - r[0], b[2], 1e-9);
            assertEquals(s - r[1], b[3], 1e-9);
        }
    }

    @Test
    public void flowersAreRotationallySymmetric() {
        double[][] f = FieldConstants.FLOWER_POSITIONS_IN;
        for (int i = 0; i < 4; i++) {
            double[] p = f[i], q = f[(i + 1) % 4];
            assertEquals(q[0] - 72, p[1] - 72, 1e-9);
            assertEquals(q[1] - 72, -(p[0] - 72), 1e-9);
        }
    }

    @Test
    public void hiveDimensionsAreInternallyConsistent() {
        assertEquals(2 * FieldConstants.CELL_DEPTH_IN + FieldConstants.CELL_GAP_IN, FieldConstants.HIVE_ARM_LENGTH_IN, 0.05);
        assertTrue(FieldConstants.UP_OPENING_CENTER_Z_IN > FieldConstants.UP_OPENING_BOTTOM_Z_IN);
        assertTrue(FieldConstants.UP_OPENING_CENTER_Z_IN < FieldConstants.UP_OPENING_TOP_Z_IN);
        assertTrue(FieldConstants.UP_OPENING_BOTTOM_Z_IN > FieldConstants.HIVE_PIVOT_HEIGHT_IN);
        // The aim point lies on the arm (within half an arm of the pivot) and inside the field.
        assertTrue(FieldConstants.UP_CELL_OFFSET_Y_IN < FieldConstants.HIVE_ARM_LENGTH_IN / 2);
        assertEquals(FieldConstants.FIELD_SIZE_IN / 2, FieldConstants.HIVE_CENTER_X_IN, 0);
    }

    @Test
    public void openingNormalIsUnitAndTilted() {
        for (boolean far : new boolean[]{true, false}) {
            double[] n = FieldConstants.openingNormal(far);
            assertEquals(1.0, Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]), 1e-12);
            assertEquals(0.0, n[0], 0);
            assertEquals(Math.sin(Math.toRadians(FieldConstants.TIP_ANGLE_DEG)), n[2], 1e-12);
            assertEquals(far ? 1 : -1, Math.signum(n[1]), 0);
        }
    }

    // ---- Minimal JSON reader (objects, arrays, numbers, strings, booleans, null) ----

    static final class Json {
        private final String s;
        private int i;

        Json(String s) {
            this.s = s;
        }

        Object parse() {
            Object v = value();
            ws();
            if (i != s.length()) {
                throw new IllegalStateException("trailing JSON at " + i);
            }
            return v;
        }

        @SuppressWarnings("unchecked")
        static Map<String, Object> asMap(Object o) {
            if (!(o instanceof Map)) {
                throw new IllegalStateException("expected object, got " + o);
            }
            return (Map<String, Object>) o;
        }

        @SuppressWarnings("unchecked")
        static List<Object> asList(Object o) {
            if (!(o instanceof List)) {
                throw new IllegalStateException("expected array, got " + o);
            }
            return (List<Object>) o;
        }

        static double num(Object o) {
            return ((Double) o).doubleValue();
        }

        static double[] nums(Object o) {
            List<Object> l = asList(o);
            double[] d = new double[l.size()];
            for (int k = 0; k < d.length; k++) {
                d[k] = num(l.get(k));
            }
            return d;
        }

        private void ws() {
            while (i < s.length() && Character.isWhitespace(s.charAt(i))) {
                i++;
            }
        }

        private Object value() {
            ws();
            char c = s.charAt(i);
            if (c == '{') {
                i++;
                Map<String, Object> m = new LinkedHashMap<>();
                ws();
                if (s.charAt(i) == '}') {
                    i++;
                    return m;
                }
                while (true) {
                    ws();
                    String k = string();
                    ws();
                    expect(':');
                    m.put(k, value());
                    ws();
                    if (s.charAt(i) == ',') {
                        i++;
                    } else {
                        expect('}');
                        return m;
                    }
                }
            }
            if (c == '[') {
                i++;
                List<Object> l = new ArrayList<>();
                ws();
                if (s.charAt(i) == ']') {
                    i++;
                    return l;
                }
                while (true) {
                    l.add(value());
                    ws();
                    if (s.charAt(i) == ',') {
                        i++;
                    } else {
                        expect(']');
                        return l;
                    }
                }
            }
            if (c == '"') {
                return string();
            }
            if (s.startsWith("true", i)) {
                i += 4;
                return Boolean.TRUE;
            }
            if (s.startsWith("false", i)) {
                i += 5;
                return Boolean.FALSE;
            }
            if (s.startsWith("null", i)) {
                i += 4;
                return null;
            }
            int st = i;
            while (i < s.length() && "+-0123456789.eE".indexOf(s.charAt(i)) >= 0) {
                i++;
            }
            if (st == i) {
                throw new IllegalStateException("bad JSON at " + i);
            }
            return Double.valueOf(s.substring(st, i));
        }

        private String string() {
            expect('"');
            StringBuilder b = new StringBuilder();
            while (s.charAt(i) != '"') {
                char c = s.charAt(i++);
                if (c == '\\') {
                    char e = s.charAt(i++);
                    switch (e) {
                        case 'n': b.append('\n'); break;
                        case 't': b.append('\t'); break;
                        case 'u': b.append((char) Integer.parseInt(s.substring(i, i + 4), 16)); i += 4; break;
                        default: b.append(e);
                    }
                } else {
                    b.append(c);
                }
            }
            i++;
            return b.toString();
        }

        private void expect(char c) {
            if (s.charAt(i) != c) {
                throw new IllegalStateException("expected " + c + " at " + i);
            }
            i++;
        }
    }
}
