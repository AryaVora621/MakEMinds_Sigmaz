package commands.subsystems.transfer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import robot.RobotSettings;

/** Single-motor transfer that runs forward or backward. */
public class Transfer {
    private final DcMotorEx motor;

    public Transfer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotSettings.Transfer.MOTOR_NAME);
        motor.setDirection(RobotSettings.Transfer.MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(RobotSettings.Transfer.ZERO_POWER_BRAKE
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void forward() {
        motor.setPower(RobotSettings.Transfer.POWER);
    }

    public void backward() {
        motor.setPower(-RobotSettings.Transfer.POWER);
    }

    public void stop() {
        motor.setPower(0);
    }

    /** Sets raw motor power (clamped to [-1, 1]). */
    public void setPower(double power) {
        motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public double getPower() {
        return motor.getPower();
    }

    public boolean isRunning() {
        return Math.abs(motor.getPower()) > 1e-6;
    }
}
