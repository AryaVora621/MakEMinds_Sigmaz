package commands.subsystems.flywheel;

import robot.RobotSettings;

/**
 * PIDF: V = kS*sgn(r) + kV*r + kA*r' + kP*e + kI*integral(e) + kD*de/dt, then power = V / battery voltage.
 * Characterize kS, kV, kA first, then tune kP. Gains and NOMINAL_VOLTAGE live in RobotSettings.FlywheelPIDF.
 */
public class FlywheelPIDF implements FlywheelController {

    private double integral;
    private double lastError;
    private double lastTarget;
    private boolean first = true;

    @Override
    public double calculate(double target, double velocity, double batteryVoltage, double dt) {
        if (target == 0) {
            reset();
            return 0;
        }
        double error = target - velocity;
        double targetRate = first ? 0 : (target - lastTarget) / dt;
        double errorRate = first ? 0 : (error - lastError) / dt;

        integral += error * dt;
        if (RobotSettings.FlywheelPIDF.kI != 0) {
            // fix: negative kI inverted the clamp bounds and pinned the integral; use |kI| (java-pedro deferred #13)
            double lim = RobotSettings.FlywheelPIDF.I_MAX_VOLTS / Math.abs(RobotSettings.FlywheelPIDF.kI);
            integral = Math.max(-lim, Math.min(lim, integral));
        }

        double volts = RobotSettings.FlywheelPIDF.kS * Math.signum(target) + RobotSettings.FlywheelPIDF.kV * target + RobotSettings.FlywheelPIDF.kA * targetRate
                + RobotSettings.FlywheelPIDF.kP * error + RobotSettings.FlywheelPIDF.kI * integral + RobotSettings.FlywheelPIDF.kD * errorRate;

        lastError = error;
        lastTarget = target;
        first = false;
        // A 0 / NaN voltage read would give Infinity or NaN power; fall back to nominal 12 V.
        double v = batteryVoltage > 1 ? batteryVoltage : RobotSettings.FlywheelPIDF.NOMINAL_VOLTAGE;
        double power = volts / v;
        if (Double.isNaN(power)) return 0;
        return Math.max(-1, Math.min(1, power));
    }

    @Override
    public void reset() {
        integral = 0;
        lastError = 0;
        lastTarget = 0;
        first = true;
    }
}
