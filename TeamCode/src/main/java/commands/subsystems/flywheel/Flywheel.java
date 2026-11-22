package commands.subsystems.flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import robot.RobotSettings;

/**
 * Two-motor flywheel (both motors on the same shaft, given the same power) with a PIDF velocity controller.
 * Velocity is read from motor A's encoder, or the average of both if RobotSettings.Flywheel.VELOCITY_FROM_BOTH is set.
 * The target can never exceed {@link #HARD_MAX_RPM} (5000), whatever RobotSettings.TurretAiming.MAX_RPM says.
 */
public class Flywheel {
    /** Absolute RPM ceiling for this flywheel. final on purpose: deliberately NOT in RobotSettings (safety ceiling). */
    public static final double HARD_MAX_RPM = 5000;


    private final DcMotorEx motor;
    private final DcMotorEx motorB;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private final FlywheelController controller;

    private double targetVelocity;
    private double velocity;

    public Flywheel(HardwareMap hardwareMap) {
        motor = initMotor(hardwareMap, RobotSettings.Flywheel.MOTOR_NAME, RobotSettings.Flywheel.MOTOR_REVERSED);
        motorB = initMotor(hardwareMap, RobotSettings.Flywheel.MOTOR_B_NAME, RobotSettings.Flywheel.MOTOR_B_REVERSED);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        controller = new FlywheelPIDF();
        timer.reset();
    }

    private static DcMotorEx initMotor(HardwareMap hardwareMap, String name, boolean reversed) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(RobotSettings.Flywheel.ZERO_POWER_FLOAT
                ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    /** Effective RPM ceiling: RobotSettings.TurretAiming.MAX_RPM, but never above HARD_MAX_RPM (NaN/negative MAX_RPM -> HARD_MAX_RPM). */
    public static double maxRpm() {
        double m = RobotSettings.TurretAiming.MAX_RPM;
        return (m > 0 && m < HARD_MAX_RPM) ? m : HARD_MAX_RPM;
    }

    /**
     * Pure: the ticks/s target actually used. 0, negative or NaN -> 0 (stop); otherwise raised to RobotSettings.Flywheel.MIN_RPM and
     * capped at maxRpm. The cap wins if MIN_RPM is set above it.
     */
    static double clampTargetTicks(double ticksPerSecond, double minRpm, double maxRpm, double ticksPerRev) {
        if (!(ticksPerSecond > 0)) return 0;
        double maxTicks = maxRpm * ticksPerRev / 60.0;
        double minTicks = Math.min(minRpm * ticksPerRev / 60.0, maxTicks);
        return Math.min(maxTicks, Math.max(ticksPerSecond, minTicks));
    }

    /** Sets the target in RPM. 0 (or less) stops the flywheel; any other value is raised to at least RobotSettings.Flywheel.MIN_RPM and capped at maxRpm(). */
    public void setTargetRpm(double rpm) {
        setTargetVelocity(rpm * RobotSettings.Flywheel.TICKS_PER_REV / 60.0);
    }

    /** Sets the target in ticks/s. 0 stops the flywheel; any other value is raised to at least RobotSettings.Flywheel.MIN_RPM and capped at maxRpm(). */
    public void setTargetVelocity(double ticksPerSecond) {
        targetVelocity = clampTargetTicks(ticksPerSecond, RobotSettings.Flywheel.MIN_RPM, maxRpm(), RobotSettings.Flywheel.TICKS_PER_REV);
    }

    public double getRpm() {
        return velocity * 60.0 / RobotSettings.Flywheel.TICKS_PER_REV;
    }

    /** Call once per loop. */
    public void update() {
        double dt = Math.max(timer.seconds(), 1e-3);
        timer.reset();
        velocity = RobotSettings.Flywheel.VELOCITY_FROM_BOTH ? 0.5 * (motor.getVelocity() + motorB.getVelocity()) : motor.getVelocity();
        setPower(controller.calculate(targetVelocity, velocity, voltageSensor.getVoltage(), dt));
    }

    public void stop() {
        targetVelocity = 0;
        controller.reset();
        setPower(0);
    }

    private void setPower(double power) {
        motor.setPower(power);
        motorB.setPower(power);
    }

    public double getVelocity() {
        return velocity;
    }

    public boolean atTarget() {
        return targetVelocity != 0 && Math.abs(targetVelocity - velocity) <= RobotSettings.Flywheel.AT_TARGET_TOLERANCE;
    }
}
