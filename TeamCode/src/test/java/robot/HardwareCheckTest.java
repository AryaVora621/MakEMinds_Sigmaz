package robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class HardwareCheckTest {
    private boolean shooter, flower, gate, sort, limelight, flowerEnabled, gateEnabled;

    @Before
    public void save() {
        shooter = RobotSettings.Robot.USE_SHOOTER;
        flower = RobotSettings.Robot.USE_FLOWER_MECH;
        gate = RobotSettings.Robot.USE_GATE;
        sort = RobotSettings.Robot.USE_COLOR_SORT;
        limelight = RobotSettings.Robot.USE_LIMELIGHT;
        flowerEnabled = RobotSettings.FlowerMech.ENABLED;
        gateEnabled = RobotSettings.Gate.ENABLED;
        RobotSettings.Robot.USE_LIMELIGHT = false;
        RobotSettings.FlowerMech.ENABLED = true;
        RobotSettings.Gate.ENABLED = true;
    }

    @After
    public void restore() {
        RobotSettings.Robot.USE_SHOOTER = shooter;
        RobotSettings.Robot.USE_FLOWER_MECH = flower;
        RobotSettings.Robot.USE_GATE = gate;
        RobotSettings.Robot.USE_COLOR_SORT = sort;
        RobotSettings.Robot.USE_LIMELIGHT = limelight;
        RobotSettings.FlowerMech.ENABLED = flowerEnabled;
        RobotSettings.Gate.ENABLED = gateEnabled;
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only the four drive motors. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void limelightRequiredOnlyWhenUsed() {
        RobotSettings.Robot.USE_SHOOTER = false;
        RobotSettings.Robot.USE_FLOWER_MECH = false;
        RobotSettings.Robot.USE_GATE = false;
        RobotSettings.Robot.USE_COLOR_SORT = false;
        assertFalse(names().contains(RobotSettings.Limelight.HARDWARE_NAME));
        RobotSettings.Robot.USE_LIMELIGHT = true;
        assertTrue(names().contains(RobotSettings.Limelight.HARDWARE_NAME));
        assertEquals(8, names().size());
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only drive + odometry. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void disabledFlowerAndGateNeedNoServo() {
        RobotSettings.Robot.USE_SHOOTER = false;
        RobotSettings.Robot.USE_FLOWER_MECH = true;
        RobotSettings.Robot.USE_GATE = true;
        RobotSettings.Robot.USE_COLOR_SORT = false;
        assertTrue(names().contains(RobotSettings.FlowerMech.SERVO_A_NAME));
        assertTrue(names().contains(RobotSettings.Gate.SERVO_NAME));
        RobotSettings.FlowerMech.ENABLED = false;
        RobotSettings.Gate.ENABLED = false;
        assertEquals(7, names().size());
    }

    @Test
    public void driveOnlyRequiresFourDriveMotors() {
        List<String> n = names();
        assertEquals(4, n.size());   // 4 drive motors only (no odometry in drive-motors-only mode)
        assertFalse(n.contains("pinpoint"));
    }

    private static List<String> names() {
        List<String> n = new ArrayList<>();
        for (HardwareCheck.Need need : HardwareCheck.required()) n.add(need.name);
        return n;
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only drive + odometry. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void driveAndIntakeAlwaysRequired() {
        RobotSettings.Robot.USE_SHOOTER = false;
        RobotSettings.Robot.USE_FLOWER_MECH = false;
        RobotSettings.Robot.USE_GATE = false;
        RobotSettings.Robot.USE_COLOR_SORT = false;
        List<String> n = names();
        assertEquals(7, n.size());   // 4 drive + pinpoint + intake + transfer
        assertTrue(n.contains("intake") && n.contains("transfer") && n.contains("pinpoint"));
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only drive + odometry. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void shooterAddsFlywheelPairAndTurretServos() {
        RobotSettings.Robot.USE_SHOOTER = true;
        RobotSettings.Robot.USE_FLOWER_MECH = false;
        RobotSettings.Robot.USE_GATE = false;
        RobotSettings.Robot.USE_COLOR_SORT = false;
        assertEquals(11, names().size());   // + 2 flywheel motors + 2 turret servos
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only drive + odometry. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void sorterNeedsGateAndSensorEvenIfGateFlagOff() {
        RobotSettings.Robot.USE_SHOOTER = false;
        RobotSettings.Robot.USE_FLOWER_MECH = false;
        RobotSettings.Robot.USE_GATE = false;
        RobotSettings.Robot.USE_COLOR_SORT = true;
        List<String> n = names();
        assertEquals(9, n.size());   // 7 base + gate servo + colour sensor
    }

    @Test
    public void noDuplicateConfigNames() {
        RobotSettings.Robot.USE_SHOOTER = true;
        RobotSettings.Robot.USE_FLOWER_MECH = true;
        RobotSettings.Robot.USE_GATE = true;
        RobotSettings.Robot.USE_COLOR_SORT = true;
        List<String> n = names();
        assertEquals("two devices share one config name: " + n, n.size(), new java.util.HashSet<>(n).size());
        assertFalse(HardwareCheck.checklist().isEmpty());
    }
}
