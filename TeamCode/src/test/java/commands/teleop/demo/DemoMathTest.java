package commands.teleop.demo;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DemoMathTest {
    @Test
    public void triangleHitsEndpointsAndStaysInside() {
        assertEquals(0, DemoMath.triangle(0, -30, 30, 8), 1e-9);
        assertEquals(30, DemoMath.triangle(2, -30, 30, 8), 1e-9);
        assertEquals(-30, DemoMath.triangle(6, -30, 30, 8), 1e-9);
        assertEquals(0, DemoMath.triangle(8, -30, 30, 8), 1e-9);
        for (double t = -20; t < 40; t += 0.37) {
            double v = DemoMath.triangle(t, -30, 30, 8);
            assertTrue(v >= -30 - 1e-9 && v <= 30 + 1e-9);
        }
        assertEquals(5, DemoMath.triangle(3, 0, 10, 0), 1e-9);
    }

    @Test
    public void stepTowardNeverOvershoots() {
        assertEquals(1, DemoMath.stepToward(0, 10, 1), 1e-9);
        assertEquals(10, DemoMath.stepToward(9.5, 10, 1), 1e-9);
        assertEquals(-1, DemoMath.stepToward(0, -10, 1), 1e-9);
        assertEquals(3, DemoMath.stepToward(3, 10, Double.NaN), 1e-9);
    }

    @Test
    public void errorAheadDrivesForwardAndRotatedRobotCompensates() {
        double[] r = DemoMath.toRobotFrame(10, 0, 0);
        assertArrayEquals(new double[]{10, 0}, r, 1e-9);
        // robot facing +y (90 deg): a target at +x is to its right, so "left" is negative
        r = DemoMath.toRobotFrame(10, 0, Math.PI / 2);
        assertArrayEquals(new double[]{0, -10}, r, 1e-9);
    }

    @Test
    public void driveTowardRespectsCapFloorAndTolerance() {
        double[] p = DemoMath.driveToward(0, 0, 0, 24, 0, 0, 0.3, 0.05, 1.0, 0.12, 1.5);
        assertEquals(0.3, p[0], 1e-9);
        assertEquals(0, p[1], 1e-9);
        assertEquals(0, p[2], 1e-9);
        p = DemoMath.driveToward(0, 0, 0, 0, 24, 0, 0.3, 0.05, 1.0, 0.12, 1.5);   // target to the left: strafe left = negative
        assertEquals(0, p[0], 1e-9);
        assertEquals(-0.3, p[1], 1e-9);
        p = DemoMath.driveToward(0, 0, 0, 2, 0, 0, 0.3, 0.05, 1.0, 0.12, 1.5);    // close: floor applies
        assertEquals(0.12, p[0], 1e-9);
        p = DemoMath.driveToward(0, 0, 0, 1, 0, 0, 0.3, 0.05, 1.0, 0.12, 1.5);    // inside tolerance
        assertArrayEquals(new double[]{0, 0, 0}, p, 1e-9);
    }

    @Test
    public void headingErrorTurnsTheRightWay() {
        // robot rotated CCW 20 deg, wants 0: must turn clockwise (positive)
        double[] p = DemoMath.driveToward(0, 0, Math.toRadians(20), 0, 0, 0, 0.3, 0.05, 1.0, 0.12, 1.5);
        assertTrue(p[2] > 0);
        p = DemoMath.driveToward(0, 0, Math.toRadians(-20), 0, 0, 0, 0.3, 0.05, 1.0, 0.12, 1.5);
        assertTrue(p[2] < 0);
    }

    @Test
    public void nanPoseGivesNoNaNCommandAboveCap() {
        double[] p = DemoMath.driveToward(Double.NaN, 0, 0, 24, 0, 0, 0.3, 0.05, 1.0, 0.12, 1.5);
        // NaN comparison fails "dist > tol", so no translation; callers also reject non-finite poses
        assertEquals(0, p[0], 0);
    }

    @Test
    public void stuckDetector() {
        double s = 0;
        for (int i = 0; i < 20; i++) s = DemoMath.stuckStep(s, 0.1, 0.3, 0.05, 0.5, 0.1);
        assertEquals(1.0, s, 1e-9);
        assertEquals(0, DemoMath.stuckStep(s, 2, 0.3, 0.05, 0.5, 0.1), 0);
        assertEquals(0, DemoMath.stuckStep(s, 0, 0.0, 0.05, 0.5, 0.1), 0);
    }

    @Test
    public void adjustClamps() {
        assertEquals(0.35, DemoMath.adjust(0.3, true, false, 0.05, 0.1, 0.5), 1e-9);
        assertEquals(0.5, DemoMath.adjust(0.5, true, false, 0.05, 0.1, 0.5), 1e-9);
        assertEquals(0.1, DemoMath.adjust(0.1, false, true, 0.05, 0.1, 0.5), 1e-9);
    }

    @Test
    public void squareReturnsHome() {
        assertArrayEquals(new double[]{24, 0}, DemoMath.squareCorner(0, 24), 0);
        assertArrayEquals(new double[]{24, 24}, DemoMath.squareCorner(1, 24), 0);
        assertArrayEquals(new double[]{0, 24}, DemoMath.squareCorner(2, 24), 0);
        assertArrayEquals(new double[]{0, 0}, DemoMath.squareCorner(3, 24), 0);
    }
}
