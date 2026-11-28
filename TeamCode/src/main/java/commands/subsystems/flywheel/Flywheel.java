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
    /** Run-away guard: power at or above this fraction while the wheel barely turns counts as "not spinning up". */
    static final double STALL_POWER = 0.6;
    /** Run-away guard: measured speed below this fraction of the target counts as "barely turning". */
    static final double STALL_SPEED_FRACTION = 0.15;
    /** Run-away guard: seconds of "full power, wheel not turning" before the power is cut (dead or reversed encoder, jammed wheel). */
    static final double STALL_CUT_S = 2.0;

    private final DcMotorEx motor;
    /** Null when the second motor is not in the robot configuration: the flywheel then runs on motor A alone. */
    private final DcMotorEx motorB;
    /** Null if the robot has no voltage sensor; the controller then falls back to NOMINAL_VOLTAGE. */
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private final FlywheelController controller;

    private double targetVelocity;
    private double velocity;
    private double lastPower;
    private double stallS;
    private boolean faulted;

    public Flywheel(HardwareMap hardwareMap) {
        motor = initMotor(hardwareMap, RobotSettings.Flywheel.MOTOR_NAME, RobotSettings.Flywheel.MOTOR_REVERSED);
        // The second motor name is still a PLACEHOLDER: a wrong name must not take the whole shooter down.
        DcMotorEx b = null;
        try {
            b = initMotor(hardwareMap, RobotSettings.Flywheel.MOTOR_B_NAME, RobotSettings.Flywheel.MOTOR_B_REVERSED);
        } catch (RuntimeException e) {
            b = null;
        }
        motorB = b;
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;

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

    /** The RPM target actually in use after the MIN_RPM / maxRpm clamp (0 = stopped). */
    public double getTargetRpm() {
        return targetVelocity * 60.0 / RobotSettings.Flywheel.TICKS_PER_REV;
    }

    /** True if the second flywheel motor was found in the robot configuration. */
    public boolean hasMotorB() {
        return motorB != null;
    }

    /** Motor A's own measured RPM (its encoder), for diagnostics. */
    public double getMotorARpm() {
        return motor.getVelocity() * 60.0 / RobotSettings.Flywheel.TICKS_PER_REV;
    }

    /** Motor B's own measured RPM (its encoder, only meaningful if plugged in); NaN if motor B is missing. */
    public double getMotorBRpm() {
        return motorB == null ? Double.NaN : motorB.getVelocity() * 60.0 / RobotSettings.Flywheel.TICKS_PER_REV;
    }

    /** True once the run-away guard cut the power; cleared by stopping (target 0). */
    public boolean isFaulted() {
        return faulted;
    }

    /**
     * Pure: advances the run-away timer. Counts up while a target is set, the last power was high and the wheel is
     * (nearly) not turning (also when it turns backwards); any other loop resets it.
     */
    static double stallStep(double stallS, double targetTicks, double velocityTicks, double lastPower, double dt) {
        boolean stalled = targetTicks > 0 && lastPower >= STALL_POWER && !(velocityTicks >= STALL_SPEED_FRACTION * targetTicks);
        return stalled ? stallS + dt : 0;
    }

    public double getRpm() {
        return velocity * 60.0 / RobotSettings.Flywheel.TICKS_PER_REV;
    }

    /** Call once per loop. */
    public void update() {
        double dt = Math.max(timer.seconds(), 1e-3);
        timer.reset();
        velocity = RobotSettings.Flywheel.VELOCITY_FROM_BOTH && motorB != null ? 0.5 * (motor.getVelocity() + motorB.getVelocity()) : motor.getVelocity();
        if (targetVelocity == 0) {
            faulted = false;
        }
        stallS = stallStep(stallS, targetVelocity, velocity, lastPower, dt);
        if (stallS > STALL_CUT_S) {
            faulted = true;   // a dead or reversed encoder would otherwise read 0 rpm and the PIDF would hold full power forever
        }
        if (faulted) {
            lastPower = 0;
            setPower(0);
            return;
        }
        double volts = voltageSensor != null ? voltageSensor.getVoltage() : RobotSettings.FlywheelPIDF.NOMINAL_VOLTAGE;
        lastPower = controller.calculate(targetVelocity, velocity, volts, dt);
        setPower(lastPower);
    }

    public void stop() {
        targetVelocity = 0;
        controller.reset();
        faulted = false;
        stallS = 0;
        lastPower = 0;
        setPower(0);
    }

    private void setPower(double power) {
        motor.setPower(power);
        if (motorB != null) {
            motorB.setPower(power);
        }
    }

    public double getVelocity() {
        return velocity;
    }

    public boolean atTarget() {
        return targetVelocity != 0 && Math.abs(targetVelocity - velocity) <= RobotSettings.Flywheel.AT_TARGET_TOLERANCE;
    }
}
