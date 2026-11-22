package commands.subsystems.turret;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Hardware-free tests of the turret target clamp and the init settle time. */
public class TurretRangeTest {
    private double savedMin, savedMax;
    private double savedCenter;
    private boolean savedRev;
    private double savedTravel;

    @Before
    public void save() {
        savedMin = robot.RobotSettings.TurretLimits.MIN_ANGLE_DEG;
        savedMax = robot.RobotSettings.TurretLimits.MAX_ANGLE_DEG;
        savedCenter = robot.RobotSettings.Turret.CENTER_SERVO_DEG;
        savedRev = robot.RobotSettings.Turret.TURRET_REVERSED;
        savedTravel = robot.RobotSettings.AxonServo.TRAVEL_DEG;
        robot.RobotSettings.AxonServo.TRAVEL_DEG = 255;
    }

    @After
    public void restore() {
        robot.RobotSettings.TurretLimits.MIN_ANGLE_DEG = savedMin;
        robot.RobotSettings.TurretLimits.MAX_ANGLE_DEG = savedMax;
        robot.RobotSettings.Turret.CENTER_SERVO_DEG = savedCenter;
        robot.RobotSettings.Turret.TURRET_REVERSED = savedRev;
        robot.RobotSettings.AxonServo.TRAVEL_DEG = savedTravel;
    }

    @Test
    public void servoRangeAroundCentre() {
        double[] r = Turret.servoRange(127.5, 255, false);
        assertEquals(-127.5, r[0], 1e-9);
        assertEquals(127.5, r[1], 1e-9);
        r = Turret.servoRange(100, 255, false);
        assertEquals(-100, r[0], 1e-9);
        assertEquals(155, r[1], 1e-9);
        r = Turret.servoRange(100, 255, true);   // reversed: +turret = lower servo angle
        assertEquals(-155, r[0], 1e-9);
        assertEquals(100, r[1], 1e-9);
    }

    @Test
    public void limitsAreTheTighterOfSoftLimitAndServoTravel() {
        robot.RobotSettings.Turret.CENTER_SERVO_DEG = 127.5;
        robot.RobotSettings.Turret.TURRET_REVERSED = false;
        robot.RobotSettings.TurretLimits.MIN_ANGLE_DEG = -125;
        robot.RobotSettings.TurretLimits.MAX_ANGLE_DEG = 125;
        assertEquals(robot.RobotSettings.TurretLimits.MIN_ANGLE_DEG, TurretLimits.min(), 0);
        assertEquals(robot.RobotSettings.TurretLimits.MAX_ANGLE_DEG, TurretLimits.max(), 0);
        assertEquals(-125, Turret.minAngle(), 1e-9);
        assertEquals(125, Turret.maxAngle(), 1e-9);
        // Soft limits set wider than the servo travel must not let the target leave the servo travel.
        robot.RobotSettings.TurretLimits.MIN_ANGLE_DEG = -180;
        robot.RobotSettings.TurretLimits.MAX_ANGLE_DEG = 180;
        assertEquals(-127.5, Turret.minAngle(), 1e-9);
        assertEquals(127.5, Turret.maxAngle(), 1e-9);
        // An off-centre servo zero narrows one side.
        robot.RobotSettings.TurretLimits.MIN_ANGLE_DEG = -125;
        robot.RobotSettings.TurretLimits.MAX_ANGLE_DEG = 125;
        robot.RobotSettings.Turret.CENTER_SERVO_DEG = 100;
        assertEquals(-100, Turret.minAngle(), 1e-9);
        assertEquals(125, Turret.maxAngle(), 1e-9);
    }

    @Test
    public void clampKeepsNanAndClampsInfinity() {
        assertEquals(125, Turret.clamp(1e9, -125, 125), 0);
        assertEquals(-125, Turret.clamp(Double.NEGATIVE_INFINITY, -125, 125), 0);
        assertEquals(10, Turret.clamp(10, -125, 125), 0);
        assertTrue(Double.isNaN(Turret.clamp(Double.NaN, -125, 125)));
    }

    @Test
    public void settleTimeUsesKnownPositionOrWorstCase() {
        assertEquals(0.4, Turret.settleTimeS(0, true, 100, -125, 125, 250), 1e-9);
        assertEquals(0, Turret.settleTimeS(30, true, 30, -125, 125, 250), 1e-9);
        assertEquals(0.5, Turret.settleTimeS(0, false, 0, -125, 125, 250), 1e-9);    // unknown: from the far end
        assertEquals(0.8, Turret.settleTimeS(75, false, 0, -125, 125, 250), 1e-9);
        assertEquals(0, Turret.settleTimeS(0, false, 0, -125, 125, 0), 0);            // no estimate: no wait
        assertEquals(0, Turret.settleTimeS(0, true, Double.NaN, -125, 125, 250), 0);
    }
}
