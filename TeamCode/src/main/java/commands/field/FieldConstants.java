package commands.field;

/**
 * BIOBUZZ (2026-27) field data in Pedro Pathing coordinates: inches, origin at a field corner,
 * +x right, +y up, 0 to 144. Source: Competition Manual TU07 (figures read 2026-11-05) and the field CAD
 * export field_sim/data/field_model/field_constants.json (checked by FieldConstantsTest).
 *
 * Frame: the Pedro frame is the FTC field frame rotated: x_p = y_f + 72, y_p = 72 - x_f. So the audience
 * side is LOW y, the far wall is y = 144, the RED side is LOW x, the BLUE side is HIGH x.
 *
 * Every value is tagged:
 *   [OFFICIAL]   stated in the manual (section or figure in brackets).
 *   [DERIVED]    computed from official numbers or read from a figure (about +-1 to 3 in).
 *   [UNVERIFIED] an estimate. MEASURE ON THE REAL FIELD before trusting it.
 */
public final class FieldConstants {
    private FieldConstants() {
    }

    // ---- Field ---------------------------------------------------------------
    /** [OFFICIAL 9.2] about 144 x 144 in inside the walls, +-1 in. */
    public static final double FIELD_SIZE_IN = 144;
    /** [OFFICIAL 9.2] 36 foam tiles of 24 x 24 in. */
    public static final double TILE_IN = 24;

    // ---- Hives ---------------------------------------------------------------
    // There are TWO hives (red and blue) side by side at the field centre.
    /** [OFFICIAL 9.6] the hive structure is at the field centre. */
    public static final double HIVE_CENTER_X_IN = FIELD_SIZE_IN / 2.0;
    public static final double HIVE_CENTER_Y_IN = FIELD_SIZE_IN / 2.0;
    /** [OFFICIAL Fig 9-10] the two hives are 25.5 in apart, centre to centre. */
    public static final double HIVE_SEPARATION_IN = 25.5;
    /** [DERIVED Fig 9-17] red is on the low-x side, blue on the high-x side. */
    public static final double RED_HIVE_X_IN = HIVE_CENTER_X_IN - HIVE_SEPARATION_IN / 2.0;   // 59.25
    public static final double BLUE_HIVE_X_IN = HIVE_CENTER_X_IN + HIVE_SEPARATION_IN / 2.0;  // 84.75
    /** [OFFICIAL 9.6.1] pivot axis height above the tiles. */
    public static final double HIVE_PIVOT_HEIGHT_IN = 43.95;
    /** [OFFICIAL Fig 9-9] the GAP between the two cells of one hive (NOT centre to centre). */
    public static final double CELL_GAP_IN = 18.84;
    /** [OFFICIAL Fig 9-9] depth of one cell along the arm. */
    public static final double CELL_DEPTH_IN = 12.04;
    /** [OFFICIAL Fig 9-9] the whole hive arm, outer end to outer end (about 12.04 + 18.84 + 12.04 = 42.92; figure rounding). */
    public static final double HIVE_ARM_LENGTH_IN = 42.91;
    /** [DERIVED Fig 9-10 callout, and the CAD hive transforms are exactly 30.0 deg] arm tilt from horizontal. */
    public static final double TIP_ANGLE_DEG = 30;
    /** [OFFICIAL Fig 9-10] z of the bottom and the top of the raised cell's opening. */
    public static final double UP_OPENING_BOTTOM_Z_IN = 53.5;
    public static final double UP_OPENING_TOP_Z_IN = 65.6;
    /**
     * [DERIVED FROM THE FIELD CAD, field_sim/data/field_model/field_constants.json] centroid of the raised cell's
     * opening: 16.48 in from the field centre along y (y = 55.52 audience cell up, 88.48 far cell up) at z = 58.31 in.
     * The opening is a pentagon (20.04 in wide, 14.00 in tall in its plane, area 218 in^2) spanning z 53.47 to 65.59, consistent with the manual (20 x 14 x 12).
     */
    public static final double UP_CELL_OFFSET_Y_IN = 16.483;
    public static final double UP_OPENING_CENTER_Z_IN = 58.313;
    /** [OFFICIAL Fig 9-10] lowest point of the hive above the tiles. */
    public static final double HIVE_LOWEST_Z_IN = 30.6;

    // ---- AprilTags -----------------------------------------------------------
    /** [OFFICIAL 9.9] tag family 36h11, 3.25 in square. IDs 30-45 sit on the underside of the hive cells, facing down. */
    public static final double TAG_SIZE_IN = 3.25;
    /** [OFFICIAL Fig 9-15] tag centres along a cell's width, from the cluster centre. */
    public static final double[] TAG_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};
    /**
     * Tag IDs by hive cell. [DERIVED Fig 9-15 + 9-17] Each row is one cluster: the ID listed at the LOW-x end first.
     * Red far cell, red audience cell, blue audience cell, blue far cell.
     */
    public static final int[] RED_FAR_TAG_IDS = {33, 32, 31, 30};
    public static final int[] RED_AUDIENCE_TAG_IDS = {34, 35, 36, 37};
    public static final int[] BLUE_AUDIENCE_TAG_IDS = {38, 39, 40, 41};
    public static final int[] BLUE_FAR_TAG_IDS = {45, 44, 43, 42};
    // Per-tag centres, normals and heights in both tip states are DERIVED FROM THE FIELD CAD (field_sim/data/field_model/
    // field_constants.json, section apriltags; tags face down, 60 deg below horizontal). Real sticker placement is
    // UNVERIFIED (hole tolerance +-0.25 in not measured): check with a tape measure on a built field, in BOTH
    // tip states. Tags move when the hive tips.

    // ---- Flowers (4 on the perimeter wall) ----------------------------------
    /** [OFFICIAL 9.7] top opening diameter. */
    public static final double FLOWER_OPENING_DIAMETER_IN = 4;
    /** [OFFICIAL 9.7] top opening height above the tiles. CAD (field_sim JSON) says 21.404 in; the rule book value is kept. */
    public static final double FLOWER_OPENING_HEIGHT_IN = 21.5;
    /** [OFFICIAL 9.7] pollen hole in the lower ring. */
    public static final double FLOWER_RING_HOLE_IN = 2.79;
    /** [DERIVED from CAD, field_sim field_constants.json] flower opening centres in Pedro (x, y); order: far, blue side, audience, red side. */
    public static final double[][] FLOWER_POSITIONS_IN = {{48.61, 140.07}, {140.07, 95.39}, {95.39, 3.93}, {3.93, 48.61}};

    // ---- Zones ---------------------------------------------------------------
    /** [OFFICIAL 9.3] loading zone (the only parking zone), 23 x 11 in, alliance specific. CAD tape measures 22.69 x 11.0 in; the rule book value is kept. */
    public static final double LOADING_ZONE_LENGTH_IN = 23;
    public static final double LOADING_ZONE_WIDTH_IN = 11;
    /** [DERIVED from CAD tape placement, field_sim tape_zones.json] loading zone boxes as {xMin, yMin, xMax, yMax}. */
    public static final double[] RED_LOADING_ZONE_IN = {1.90, 95.91, 12.90, 118.60};
    public static final double[] BLUE_LOADING_ZONE_IN = {131.10, 25.40, 142.10, 48.09};
    /** [OFFICIAL 9.3] garden strip, 23 x 2 in. CAD tape measures 22.69 x 2.0 in; the rule book value is kept. */
    public static final double GARDEN_LENGTH_IN = 23;
    public static final double GARDEN_WIDTH_IN = 2;
    /** [DERIVED from CAD tape placement] red garden: the strip along the audience wall at the red corner. Blue mirrors it (point reflection through the field centre). */
    public static final double[] RED_GARDEN_IN = {1.90, 1.90, 24.59, 3.90};
    public static final double[] BLUE_GARDEN_IN = {119.41, 140.10, 142.10, 142.10};
    // There is NO launch zone in the rules, so you can shoot from anywhere.

    // ---- Game pieces ---------------------------------------------------------
    /** [OFFICIAL 9.8] approximate diameters; the pieces are not perfectly round. */
    public static final double POLLEN_DIAMETER_IN = 2.8;
    public static final double NECTAR_DIAMETER_IN = 3.6;

    // ---- Robot rules ---------------------------------------------------------
    /** [OFFICIAL R102] starting configuration fits in 18 x 18 x 18 in. */
    public static final double START_CUBE_IN = 18;
    /** [OFFICIAL G304] exactly four pollen preloaded. */
    public static final int PRELOAD_POLLEN = 4;
    /** [USER-CONFIRMED] a hive tips at 8 pollen; the manual does not state this. */
    public static final int HIVE_TIP_POLLEN = 8;
    // NECTAR_TIP_COUNT: UNKNOWN. Not in the manual. Do not invent a value.

    // ---- Shot target ---------------------------------------------------------
    /**
     * Outward normal of the raised cell's opening as {x, y, z} (field frame): it points away from the hive
     * and 30 deg above horizontal, toward the far wall if the far cell is up, toward the audience otherwise.
     * [DERIVED FROM THE FIELD CAD] A ball only enters if it travels against this normal, so a shot from behind
     * the opening can never score.
     */
    public static double[] openingNormal(boolean farCellUp) {
        double a = Math.toRadians(TIP_ANGLE_DEG);
        return new double[]{0, (farCellUp ? 1 : -1) * Math.cos(a), Math.sin(a)};
    }

    /** Aim x for the alliance's own hive. */
    public static double aimX(Alliance alliance) {
        return alliance == Alliance.RED ? RED_HIVE_X_IN : BLUE_HIVE_X_IN;
    }

    /**
     * Aim y: the centre of the RAISED cell of our hive. The hive is bistable, so the raised cell is either
     * the one on the far side (y > 72) or the one on the audience side (y < 72). The aim point is 16.483 in from the
     * pivot at y = 72 on either side, so the target moves from y = 55.5 to y = 88.5 (about 33 in) when the hive tips.
     * [DERIVED FROM THE FIELD CAD; the real field may differ by a few inches, so check on a built field]
     */
    public static double aimY(boolean farCellUp) {
        return HIVE_CENTER_Y_IN + (farCellUp ? UP_CELL_OFFSET_Y_IN : -UP_CELL_OFFSET_Y_IN);
    }
}
