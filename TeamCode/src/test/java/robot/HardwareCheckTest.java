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
    private boolean shooter, flower, limelight, flowerEnabled;

    @Before
    public void save() {
        shooter = RobotSettings.Robot.USE_SHOOTER;
        flower = RobotSettings.Robot.USE_FLOWER_MECH;
        limelight = RobotSettings.Robot.USE_LIMELIGHT;
        flowerEnabled = RobotSettings.FlowerMech.ENABLED;
        RobotSettings.Robot.USE_LIMELIGHT = false;
        RobotSettings.FlowerMech.ENABLED = true;
    }

    @After
    public void restore() {
        RobotSettings.Robot.USE_SHOOTER = shooter;
        RobotSettings.Robot.USE_FLOWER_MECH = flower;
        RobotSettings.Robot.USE_LIMELIGHT = limelight;
        RobotSettings.FlowerMech.ENABLED = flowerEnabled;
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only the four drive motors. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void limelightRequiredOnlyWhenUsed() {
        RobotSettings.Robot.USE_SHOOTER = false;
        RobotSettings.Robot.USE_FLOWER_MECH = false;
        assertFalse(names().contains(RobotSettings.Limelight.HARDWARE_NAME));
        RobotSettings.Robot.USE_LIMELIGHT = true;
        assertTrue(names().contains(RobotSettings.Limelight.HARDWARE_NAME));
        assertEquals(8, names().size());
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only drive + odometry. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void disabledFlowerNeedsNoServo() {
        RobotSettings.Robot.USE_SHOOTER = false;
        RobotSettings.Robot.USE_FLOWER_MECH = true;
        assertTrue(names().contains(RobotSettings.FlowerMech.SERVO_A_NAME));
        RobotSettings.FlowerMech.ENABLED = false;
        assertEquals(7, names().size());
    }

    @Test
    public void driveOnlyRequiresFourDriveMotorsAndPinpoint() {
        List<String> n = names();
        assertEquals(5, n.size());   // 4 drive motors + pinpoint
        assertTrue(n.contains("pinPoint"));
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
        List<String> n = names();
        assertEquals(7, n.size());   // 4 drive + pinpoint + intake + transfer
        assertTrue(n.contains("intake") && n.contains("transfer") && n.contains("pinPoint"));
    }

    @Ignore("Drive-only mode: HardwareCheck.required() lists only drive + odometry. Remove this @Ignore when the subsystems are restored.")
    @Test
    public void shooterAddsFlywheelPairAndTurretServos() {
        RobotSettings.Robot.USE_SHOOTER = true;
        RobotSettings.Robot.USE_FLOWER_MECH = false;
        assertEquals(11, names().size());   // + 2 flywheel motors + 2 turret servos
    }

    @Test
    public void noDuplicateConfigNames() {
        RobotSettings.Robot.USE_SHOOTER = true;
        RobotSettings.Robot.USE_FLOWER_MECH = true;
        List<String> n = names();
        assertEquals("two devices share one config name: " + n, n.size(), new java.util.HashSet<>(n).size());
        assertFalse(HardwareCheck.checklist().isEmpty());
    }
}
