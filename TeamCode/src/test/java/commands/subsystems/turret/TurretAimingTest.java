package commands.subsystems.turret;

import robot.RobotSettings;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeNoException;

import commands.field.Alliance;
import commands.subsystems.grouped_subsystems.shooter.ShotTable;
import java.lang.reflect.Method;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Pure-math tests of {@link TurretAiming} for the real hardware: fixed launch elevation (no hood) and a
 * two-Axon turret limited to the servo window RobotSettings.TurretLimits (+-125 deg). No hardware is touched.
 *
 * Geometry used: red hive aim point (59.25, 88.483) with the far cell up. A robot at (144, 132) facing the
 * aim point is a clean reachable shot at the fixed 45 deg launch. A robot at (144, 12) is on the wrong side
 * of the opening at 45 deg (fromBehind).
 */
public class TurretAimingTest {
    private static final double X = 144, Y = 132;
    private static final double AIM_X = 59.25, AIM_Y = 88.483;

    private TurretAiming.ShotSource savedSource;
    private boolean savedLatency, savedMove, savedVision, savedRising;
    private double savedMaxRpm, savedScale, savedPivotX, savedPivotY, savedElev;
    private double savedMin, savedMax;

    @Before
    public void setUp() {
        savedSource = RobotSettings.TurretAiming.SHOT_SOURCE;
        savedLatency = RobotSettings.TurretAiming.USE_LATENCY_COMPENSATION;
        savedMove = RobotSettings.TurretAiming.USE_SHOOT_ON_THE_MOVE;
        savedVision = RobotSettings.TurretAiming.USE_VISION_TRIM;
        savedRising = RobotSettings.TurretAiming.ALLOW_RISING_ARRIVAL;
        savedMaxRpm = RobotSettings.TurretAiming.MAX_RPM;
        savedScale = RobotSettings.TurretAiming.RPM_SCALE;
        savedPivotX = RobotSettings.TurretAiming.PIVOT_OFFSET_X_IN;
        savedPivotY = RobotSettings.TurretAiming.PIVOT_OFFSET_Y_IN;
        savedElev = RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG;
        savedMin = RobotSettings.TurretLimits.MIN_ANGLE_DEG;
        savedMax = RobotSettings.TurretLimits.MAX_ANGLE_DEG;

        RobotSettings.TurretAiming.SHOT_SOURCE = TurretAiming.ShotSource.PHYSICS;
        RobotSettings.TurretAiming.USE_LATENCY_COMPENSATION = false;
        RobotSettings.TurretAiming.USE_SHOOT_ON_THE_MOVE = true;
        RobotSettings.TurretAiming.USE_VISION_TRIM = false;
        RobotSettings.TurretAiming.ALLOW_RISING_ARRIVAL = false;
        RobotSettings.TurretAiming.MAX_RPM = 6000;
        RobotSettings.TurretAiming.RPM_SCALE = 1.0;
        RobotSettings.TurretAiming.PIVOT_OFFSET_X_IN = 0;
        RobotSettings.TurretAiming.PIVOT_OFFSET_Y_IN = 0;
        RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG = 45;
        RobotSettings.TurretLimits.MIN_ANGLE_DEG = -125;
        RobotSettings.TurretLimits.MAX_ANGLE_DEG = 125;
    }

    @After
    public void tearDown() {
        RobotSettings.TurretAiming.SHOT_SOURCE = savedSource;
        RobotSettings.TurretAiming.USE_LATENCY_COMPENSATION = savedLatency;
        RobotSettings.TurretAiming.USE_SHOOT_ON_THE_MOVE = savedMove;
        RobotSettings.TurretAiming.USE_VISION_TRIM = savedVision;
        RobotSettings.TurretAiming.ALLOW_RISING_ARRIVAL = savedRising;
        RobotSettings.TurretAiming.MAX_RPM = savedMaxRpm;
        RobotSettings.TurretAiming.RPM_SCALE = savedScale;
        RobotSettings.TurretAiming.PIVOT_OFFSET_X_IN = savedPivotX;
        RobotSettings.TurretAiming.PIVOT_OFFSET_Y_IN = savedPivotY;
        RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG = savedElev;
        RobotSettings.TurretLimits.MIN_ANGLE_DEG = savedMin;
        RobotSettings.TurretLimits.MAX_ANGLE_DEG = savedMax;
    }

    private static TurretAiming.Solution solve(double x, double y, double heading, boolean far, double current,
                                               boolean turret) {
        return TurretAiming.solve(x, y, heading, 0, 0, 0, Alliance.RED, far, current, turret, false, 0);
    }

    /** Field direction of the shot from (x, y) (it does not depend on the heading with no pivot offset). */
    private static double aim(double x, double y, boolean far) {
        return solve(x, y, 0, far, 0, false).aimHeadingRad;
    }

    /** Heading that makes the required turret angle exactly {@code turretDeg} for the reachable shot. */
    private static double headingForTurret(double turretDeg) {
        return aim(X, Y, true) - Math.toRadians(turretDeg);
    }

    @Test
    public void reachableShot() {
        double heading = headingForTurret(-17);
        TurretAiming.Solution s = solve(X, Y, heading, true, 0, true);
        assertTrue(s.reason, s.ok);
        assertTrue(s.reason, s.reachable);
        assertFalse(s.fromBehind);
        assertFalse(s.compensating);
        assertTrue(s.flightTimeS > 0);
        assertTrue(s.rpm > 0 && s.rpm <= RobotSettings.TurretAiming.MAX_RPM);
        assertEquals(RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG, s.launchElevDeg, 1e-9);
        assertEquals(TurretAiming.speedToRpm(s.launchSpeedInS), s.rpm, 1e-9);
        assertEquals(Math.hypot(AIM_X - X, AIM_Y - Y), s.distanceIn, 1e-9);
        assertTrue(s.turretAngleDeg >= TurretLimits.min() && s.turretAngleDeg <= TurretLimits.max());
        // Turret angle is the aim direction relative to the heading.
        assertEquals(-17.0, s.turretAngleDeg, 1e-6);
        assertEquals(Math.toDegrees(TurretAiming.wrapRadians(s.aimHeadingRad - heading)), s.turretAngleDeg, 1e-9);
    }

    @Test
    public void limitsAreTheServoWindow() {
        assertEquals(RobotSettings.TurretLimits.MIN_ANGLE_DEG, TurretLimits.min(), 0);
        assertEquals(RobotSettings.TurretLimits.MAX_ANGLE_DEG, TurretLimits.max(), 0);
        assertEquals(-125, TurretLimits.min(), 0);
        assertEquals(125, TurretLimits.max(), 0);
    }

    @Test
    public void turretlessRobotReportsZeroTurretAndIsReachable() {
        TurretAiming.Solution s = solve(X, Y, 0, true, 55, false);
        assertTrue(s.ok);
        assertTrue(s.reachable);
        assertEquals(0.0, s.turretAngleDeg, 0);
    }

    @Test
    public void turretAngleIsEquivalentAngleClosestToCurrent() {
        double heading = headingForTurret(30);
        TurretAiming.Solution s = solve(X, Y, heading, true, 0, true);
        // Rotate the heading by 360 deg: same aim, so the same turret angle.
        TurretAiming.Solution t = solve(X, Y, heading + 2 * Math.PI, true, 0, true);
        assertEquals(30.0, s.turretAngleDeg, 1e-6);
        assertEquals(s.turretAngleDeg, t.turretAngleDeg, 1e-6);
    }

    @Test
    public void rpmAboveMaxIsUnreachable() {
        RobotSettings.TurretAiming.MAX_RPM = 1000;                     // far below the ~3000 RPM this shot needs
        TurretAiming.Solution s = solve(X, Y, headingForTurret(0), true, 0, true);
        assertTrue(s.ok);
        assertTrue(s.rpm > RobotSettings.TurretAiming.MAX_RPM);
        assertFalse(s.reachable);
    }

    /** A target directly behind the robot is outside the +-125 deg servo window: unreachable, parked at a limit. */
    @Test
    public void aimBehindRobotIsUnreachableAndParksAtNearestLimit() {
        TurretAiming.Solution behind = solve(X, Y, headingForTurret(180), true, 0, true);
        assertTrue(behind.ok);
        assertFalse(behind.reachable);
        assertEquals(125.0, Math.abs(behind.turretAngleDeg), 1e-9);
        assertTrue(behind.reason, behind.reason.contains("aim outside turret limits"));

        // Just left of straight behind (+170): the +125 limit is nearer.
        TurretAiming.Solution left = solve(X, Y, headingForTurret(170), true, 0, true);
        assertFalse(left.reachable);
        assertEquals(125.0, left.turretAngleDeg, 1e-9);
        // Just right of straight behind (-170): the -125 limit is nearer.
        TurretAiming.Solution right = solve(X, Y, headingForTurret(-170), true, 0, true);
        assertFalse(right.reachable);
        assertEquals(-125.0, right.turretAngleDeg, 1e-9);
    }

    @Test
    public void limitEdges() {
        TurretAiming.Solution in = solve(X, Y, headingForTurret(124), true, 0, true);
        assertTrue(in.reason, in.reachable);
        assertEquals(124.0, in.turretAngleDeg, 1e-6);
        TurretAiming.Solution inNeg = solve(X, Y, headingForTurret(-124), true, 0, true);
        assertTrue(inNeg.reason, inNeg.reachable);
        assertEquals(-124.0, inNeg.turretAngleDeg, 1e-6);
        TurretAiming.Solution out = solve(X, Y, headingForTurret(126), true, 0, true);
        assertFalse(out.reachable);
        assertEquals(125.0, out.turretAngleDeg, 1e-9);   // parked at the limit, not wrapped to -234
    }

    /**
     * There is no 360 deg seam: the window is narrower than a turn, so the turret never takes the equivalent
     * angle on the other side. From +120, a target needing -120 is driven the long way through 0 to -120.
     */
    @Test
    public void noSeamTurretNeverWrapsAround() {
        TurretAiming.Solution s = solve(X, Y, headingForTurret(-120), true, 120, true);
        assertTrue(s.reachable);
        assertEquals(-120.0, s.turretAngleDeg, 1e-6);
        TurretAiming.Solution t = solve(X, Y, headingForTurret(120), true, -120, true);
        assertTrue(t.reachable);
        assertEquals(120.0, t.turretAngleDeg, 1e-6);
    }

    /** MAX_RPM above the flywheel's 5000 RPM hard cap is ignored: a 5500 RPM shot is still unreachable. */
    @Test
    public void rpmAboveHardCapIsUnreachableEvenIfMaxRpmIsRaised() {
        double heading = headingForTurret(0);
        double base = solve(X, Y, heading, true, 0, true).rpm;
        RobotSettings.TurretAiming.MAX_RPM = 9000;
        RobotSettings.TurretAiming.RPM_SCALE = 5500.0 / base;               // same shot, now needing 5500 RPM
        TurretAiming.Solution s = solve(X, Y, heading, true, 0, true);
        assertEquals(5500, s.rpm, 1e-6);
        assertFalse(s.reachable);
        assertTrue(s.reason, s.reason.contains("MAX_RPM 5000"));
    }

    /** A NaN RPM (e.g. a bad tunable) must never be reported as reachable. */
    @Test
    public void nanRpmIsUnreachable() {
        RobotSettings.TurretAiming.RPM_SCALE = Double.NaN;
        TurretAiming.Solution s = solve(X, Y, headingForTurret(0), true, 0, true);
        assertTrue(Double.isNaN(s.rpm));
        assertFalse(s.reachable);
    }

    @Test
    public void rpmBelowFlywheelMinIsUnreachable() {
        double saved = RobotSettings.Flywheel.MIN_RPM;
        try {
            RobotSettings.Flywheel.MIN_RPM = 100000;
            TurretAiming.Solution s = solve(X, Y, headingForTurret(0), true, 0, true);
            assertTrue(s.ok);
            assertFalse(s.reachable);
        } finally {
            RobotSettings.Flywheel.MIN_RPM = saved;
        }
    }

    @Test
    public void fromBehindIsFlaggedAndUnreachable() {
        // Facing the aim point, so the turret limits are not the reason.
        TurretAiming.Solution s = solve(144, 12, aim(144, 12, true), true, 0, true);
        assertTrue(s.ok);
        assertTrue(s.fromBehind);
        assertFalse(s.reachable);
        assertEquals(0.0, s.turretAngleDeg, 1e-6);
    }

    @Test
    public void unreachablePhysicsGivesOkFalse() {
        // Fixed 45 deg launch toward a point right under the opening: no flight time exists.
        TurretAiming.Solution s = TurretAiming.solve(AIM_X, AIM_Y, 0, 0, 0, 0, Alliance.RED, true, 0, true, false, 0);
        assertFalse(s.ok);
        assertFalse(s.reachable);
        assertFalse(s.reason.isEmpty());
    }

    @Test
    public void reasonsAndStatusTextPerBranch() {
        TurretAiming.Solution ok = solve(X, Y, headingForTurret(0), true, 0, true);
        assertTrue(ok.statusText(), ok.statusText().startsWith("ok reachable"));
        assertFalse(ok.statusText().contains("clamped"));

        TurretAiming.Solution behind = solve(144, 12, aim(144, 12, true), true, 0, true);
        assertTrue(behind.fromBehind);
        assertFalse(behind.reason.isEmpty());
        assertTrue(behind.statusText(), behind.statusText().contains("from-behind"));
        assertTrue(behind.statusText().contains("UNREACHABLE"));

        TurretAiming.Solution limit = solve(X, Y, headingForTurret(180), true, 0, true);
        assertTrue(limit.statusText(), limit.statusText().contains("UNREACHABLE"));
        assertTrue(limit.statusText(), limit.statusText().contains("aim outside turret limits"));

        RobotSettings.TurretAiming.MAX_RPM = 1000;
        TurretAiming.Solution rpm = solve(X, Y, headingForTurret(0), true, 0, true);
        assertFalse(rpm.reachable);
        assertTrue(rpm.reason, rpm.reason.contains("MAX_RPM"));
        RobotSettings.TurretAiming.MAX_RPM = 6000;

        TurretAiming.Solution none = TurretAiming.solve(AIM_X, AIM_Y, 0, 0, 0, 0, Alliance.RED, true, 0, true, false, 0);
        assertFalse(none.ok);
        assertFalse(none.reason.isEmpty());
        assertTrue(none.statusText(), none.statusText().startsWith("NO SHOT"));
    }

    /** Over the whole field: every solved shot uses the fixed launch angle and keeps the turret in the window. */
    @Test
    public void launchElevationIsFixedAndTurretStaysInWindowEverywhere() {
        for (boolean far : new boolean[]{true, false}) {
            for (double y = 0; y <= 144; y += 12) {
                for (double x = 0; x <= 144; x += 24) {
                    for (double heading = -Math.PI; heading < Math.PI; heading += Math.PI / 3) {
                        TurretAiming.Solution s = solve(x, y, heading, far, 0, true);
                        if (!s.ok) {
                            assertFalse(s.reachable);
                            continue;
                        }
                        String where = "far=" + far + " (" + x + "," + y + ") h=" + heading;
                        assertEquals(where, RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG, s.launchElevDeg, 1e-6);
                        assertTrue(where + " turret=" + s.turretAngleDeg,
                                s.turretAngleDeg >= TurretLimits.min() - 1e-9 && s.turretAngleDeg <= TurretLimits.max() + 1e-9);
                        assertTrue(where, Double.isFinite(s.rpm));
                    }
                }
            }
        }
    }

    @Test
    public void movingRobotIsCompensatedAndStationaryIsNot() {
        double heading = headingForTurret(0);
        TurretAiming.Solution still = TurretAiming.solve(X, Y, heading, 1, 1, 0, Alliance.RED, true, 0, true, false, 0);
        assertFalse(still.compensating);            // below MIN_MOVE_SPEED_IN_S
        TurretAiming.Solution moving = TurretAiming.solve(X, Y, heading, 0, 20, 0, Alliance.RED, true, 0, true, false, 0);
        assertTrue(moving.compensating);
        assertTrue(moving.reason, moving.ok);
        assertTrue(Math.abs(moving.turretAngleDeg - still.turretAngleDeg) > 1e-3);
    }

    @Test
    public void speedToRpmFormula() {
        RobotSettings.TurretAiming.RPM_SCALE = 1.0;
        double expect = 60.0 * 100 / (2 * Math.PI * RobotSettings.TurretAiming.WHEEL_RADIUS_IN * RobotSettings.TurretAiming.TRANSFER_EFFICIENCY);
        assertEquals(expect, TurretAiming.speedToRpm(100), 1e-9);
        RobotSettings.TurretAiming.RPM_SCALE = 1.1;
        assertEquals(1.1 * expect, TurretAiming.speedToRpm(100), 1e-9);
        assertEquals(0.0, TurretAiming.speedToRpm(0), 0);
    }

    @Test
    public void wrapRadiansRange() {
        for (double a = -30; a <= 30; a += 0.41) {
            double w = TurretAiming.wrapRadians(a);
            assertTrue(w >= -Math.PI && w <= Math.PI);
            assertEquals(Math.cos(a), Math.cos(w), 1e-9);
            assertEquals(Math.sin(a), Math.sin(w), 1e-9);
        }
        assertEquals(0.5, TurretAiming.wrapRadians(0.5 + 2 * Math.PI), 1e-9);
        assertEquals(-0.5, TurretAiming.wrapRadians(-0.5 - 4 * Math.PI), 1e-9);
    }

    /** shortestDifference is private; checked by reflection so the (-180, 180] convention stays pinned. */
    @Test
    public void shortestDifferenceHelper() {
        Method m;
        try {
            m = TurretAiming.class.getDeclaredMethod("shortestDifference", double.class, double.class);
            m.setAccessible(true);
        } catch (ReflectiveOperationException | RuntimeException e) {
            assumeNoException(e);
            return;
        }
        try {
            assertEquals(20.0, (Double) m.invoke(null, -170.0, 170.0), 1e-12);
            assertEquals(-20.0, (Double) m.invoke(null, 170.0, -170.0), 1e-12);
            assertEquals(10.0, (Double) m.invoke(null, 15.0, 5.0), 1e-12);
            assertEquals(-10.0, (Double) m.invoke(null, 5.0, 15.0), 1e-12);
            assertEquals(0.0, (Double) m.invoke(null, 720.0, 0.0), 1e-12);
            assertEquals(180.0, (Double) m.invoke(null, 180.0, 0.0), 1e-12);
            assertEquals(180.0, (Double) m.invoke(null, -180.0, 0.0), 1e-12);   // (-180, 180]: -180 maps to +180
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }

    @Test
    public void tableSourceUsesDistanceTableAndFixedLaunchAngle() {
        RobotSettings.TurretAiming.SHOT_SOURCE = TurretAiming.ShotSource.TABLE;
        TurretAiming.Solution s = solve(24, 24, 0, true, 0, true);
        assertTrue(s.ok);
        double dist = Math.hypot(AIM_X - 24, AIM_Y - 24);
        assertEquals(dist, s.distanceIn, 1e-9);
        assertEquals(ShotTable.rpm(dist), s.rpm, 1e-9);
        assertTrue(s.rpm > 0);
        assertEquals(RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG, s.launchElevDeg, 0);
        // The table path uses the same servo-window turret limits.
        TurretAiming.Solution behind = solve(24, 24, s.aimHeadingRad - Math.PI, true, 0, true);
        assertFalse(behind.reachable);
        assertEquals(125.0, Math.abs(behind.turretAngleDeg), 1e-9);
    }
}
