package commands.subsystems.flywheel;

import robot.RobotSettings;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import commands.subsystems.turret.TurretAiming;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Hardware-free tests of the flywheel target clamp and the 5000 RPM ceiling. */
public class FlywheelTargetTest {
    private double savedMax;

    @Before
    public void save() {
        savedMax = RobotSettings.TurretAiming.MAX_RPM;
    }

    @After
    public void restore() {
        RobotSettings.TurretAiming.MAX_RPM = savedMax;
    }

    private static double rpm(double ticks) {
        return ticks * 60.0 / 28;
    }

    private static double ticks(double rpm) {
        return rpm * 28 / 60.0;
    }

    @Test
    public void defaultCapIsAtMost5000() {
        assertTrue(RobotSettings.TurretAiming.MAX_RPM <= 5000);
        assertTrue(Flywheel.maxRpm() <= 5000);
        assertEquals(5000, Flywheel.HARD_MAX_RPM, 0);
    }

    @Test
    public void maxRpmCannotExceedHardCap() {
        RobotSettings.TurretAiming.MAX_RPM = 6000;
        assertEquals(5000, Flywheel.maxRpm(), 0);
        RobotSettings.TurretAiming.MAX_RPM = Double.NaN;
        assertEquals(5000, Flywheel.maxRpm(), 0);
        RobotSettings.TurretAiming.MAX_RPM = -1;
        assertEquals(5000, Flywheel.maxRpm(), 0);
        RobotSettings.TurretAiming.MAX_RPM = 4000;
        assertEquals(4000, Flywheel.maxRpm(), 0);
    }

    @Test
    public void targetIsClampedBetweenMinAndMax() {
        assertEquals(5000, rpm(Flywheel.clampTargetTicks(ticks(9000), 2500, 5000, 28)), 1e-9);
        assertEquals(2500, rpm(Flywheel.clampTargetTicks(ticks(100), 2500, 5000, 28)), 1e-9);
        assertEquals(3500, rpm(Flywheel.clampTargetTicks(ticks(3500), 2500, 5000, 28)), 1e-9);
        assertEquals(5000, rpm(Flywheel.clampTargetTicks(Double.POSITIVE_INFINITY, 2500, 5000, 28)), 1e-9);
    }

    @Test
    public void zeroNegativeOrNanStops() {
        assertEquals(0, Flywheel.clampTargetTicks(0, 2500, 5000, 28), 0);
        assertEquals(0, Flywheel.clampTargetTicks(-50, 2500, 5000, 28), 0);
        assertEquals(0, Flywheel.clampTargetTicks(Double.NaN, 2500, 5000, 28), 0);
    }

    @Test
    public void capWinsOverMinRpm() {
        // MIN_RPM set above the cap must not push the target above the cap.
        assertEquals(5000, rpm(Flywheel.clampTargetTicks(ticks(100), 7000, 5000, 28)), 1e-9);
    }
}
