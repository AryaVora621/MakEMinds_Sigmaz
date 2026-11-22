package commands.subsystems.drivebase;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

import robot.RobotSettings;

/** Mecanum mix, normalisation and stick curve of {@link Drivebase} (no hardware). Order: FL, FR, BL, BR. */
public class DrivebaseMixTest {
    private final double savedStrength = RobotSettings.Drivetrain.LOG_STRENGTH;

    @After
    public void restore() {
        RobotSettings.Drivetrain.LOG_STRENGTH = savedStrength;
    }

    @Test
    public void pureForwardStrafeAndTurn() {
        assertArrayEquals(new double[]{1, 1, 1, 1}, Drivebase.mix(1, 0, 0), 1e-12);
        // Strafe right: FL and BR forward, FR and BL backward.
        assertArrayEquals(new double[]{1, -1, -1, 1}, Drivebase.mix(0, 1, 0), 1e-12);
        // Turn clockwise: left side forward, right side backward.
        assertArrayEquals(new double[]{1, -1, 1, -1}, Drivebase.mix(0, 0, 1), 1e-12);
    }

    @Test
    public void saturatedCommandsAreScaledTogetherKeepingRatios() {
        double[] p = Drivebase.mix(1, 1, 0);   // raw FL 2, FR 0, BL 0, BR 2
        assertArrayEquals(new double[]{1, 0, 0, 1}, p, 1e-12);
        p = Drivebase.mix(1, 0.5, 0.5);         // raw 2, 0, 1, 1
        assertArrayEquals(new double[]{1, 0, 0.5, 0.5}, p, 1e-12);
        for (double f = -1; f <= 1; f += 0.25) {
            for (double s = -1; s <= 1; s += 0.25) {
                for (double t = -1; t <= 1; t += 0.25) {
                    for (double v : Drivebase.mix(f, s, t)) {
                        assertTrue(Math.abs(v) <= 1 + 1e-12);
                    }
                }
            }
        }
    }

    @Test
    public void smallCommandsAreNotScaledUp() {
        assertArrayEquals(new double[]{0.3, 0.1, 0.1, 0.3}, Drivebase.mix(0.2, 0.1, 0), 1e-12);
    }

    @Test
    public void nonFiniteInputsAreTreatedAsZero() {
        assertArrayEquals(new double[]{0.5, 0.5, 0.5, 0.5}, Drivebase.mix(0.5, Double.NaN, Double.POSITIVE_INFINITY), 1e-12);
        assertArrayEquals(new double[]{0, 0, 0, 0}, Drivebase.mix(Double.NaN, Double.NaN, Double.NaN), 0);
    }

    @Test
    public void logScaleKeepsEndpointsSignAndIsMonotonic() {
        assertEquals(0, Drivebase.logScale(0), 0);
        assertEquals(1, Drivebase.logScale(1), 1e-12);
        assertEquals(-1, Drivebase.logScale(-1), 1e-12);
        assertEquals(-Drivebase.logScale(0.3), Drivebase.logScale(-0.3), 1e-12);
        double prev = -2;
        for (double x = -1; x <= 1; x += 0.05) {
            double y = Drivebase.logScale(x);
            assertTrue(y > prev);
            prev = y;
        }
    }

    @Test
    public void logScaleWithInvalidStrengthIsLinearNotNaN() {
        RobotSettings.Drivetrain.LOG_STRENGTH = 0;
        assertEquals(0.4, Drivebase.logScale(0.4), 0);
        RobotSettings.Drivetrain.LOG_STRENGTH = -3;
        assertEquals(-0.4, Drivebase.logScale(-0.4), 0);
    }
}
