package commands.subsystems.turret;

import robot.RobotSettings;

/** The soft limits every turret controller uses: the two-Axon servo window in RobotSettings.TurretLimits. */
public class TurretLimits {
    public static double min() {
        return RobotSettings.TurretLimits.MIN_ANGLE_DEG;
    }

    public static double max() {
        return RobotSettings.TurretLimits.MAX_ANGLE_DEG;
    }
}
