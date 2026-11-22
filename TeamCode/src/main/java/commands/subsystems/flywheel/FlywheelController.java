package commands.subsystems.flywheel;

/** A flywheel velocity controller. Velocity is in encoder ticks/s. */
public interface FlywheelController {
    /**
     * @param target         desired velocity (ticks/s)
     * @param velocity       measured velocity (ticks/s)
     * @param batteryVoltage current battery voltage (V)
     * @param dt             seconds since the last call
     * @return motor power in [-1, 1]
     */
    double calculate(double target, double velocity, double batteryVoltage, double dt);

    void reset();
}
