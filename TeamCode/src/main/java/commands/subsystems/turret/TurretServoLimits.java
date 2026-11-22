package commands.subsystems.turret;

/**
 * Intentionally empty. The turret soft limits (MIN_ANGLE_DEG, MAX_ANGLE_DEG) now live in
 * robot.RobotSettings.TurretLimits; read them through {@link TurretLimits#min()} / {@link TurretLimits#max()}.
 */
public final class TurretServoLimits {
    private TurretServoLimits() {}
}
