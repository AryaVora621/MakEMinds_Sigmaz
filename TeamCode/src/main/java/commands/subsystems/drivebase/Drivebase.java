package commands.subsystems.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import robot.RobotSettings;

/** Four-motor mecanum drivebase. */
public class Drivebase {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final double[] lastPowers = new double[4];
    private final double[] lastCommand = new double[3];

    public Drivebase(HardwareMap hardwareMap) {
        // Names and directions come from RobotSettings.Drivetrain (single source of truth; pedroPathing.Constants
        // builds Pedro driveConstants from the same values, and Pedro re-applies them on every update()).
        frontLeft = hardwareMap.get(DcMotorEx.class, RobotSettings.Drivetrain.FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotSettings.Drivetrain.FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotSettings.Drivetrain.BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, RobotSettings.Drivetrain.BACK_RIGHT_NAME);

        frontLeft.setDirection(RobotSettings.Drivetrain.FRONT_LEFT_DIRECTION);
        backLeft.setDirection(RobotSettings.Drivetrain.BACK_LEFT_DIRECTION);
        frontRight.setDirection(RobotSettings.Drivetrain.FRONT_RIGHT_DIRECTION);
        backRight.setDirection(RobotSettings.Drivetrain.BACK_RIGHT_DIRECTION);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(RobotSettings.Drivetrain.ZERO_POWER_BEHAVIOR);
        }
    }

    /** Arcade-style mecanum drive. All inputs in [-1, 1]; forward = +, strafe right = +, turn clockwise = +. */
    public void drive(double forward, double strafe, double turn) {
        double[] p = mix(forward, strafe, turn);
        lastCommand[0] = forward;
        lastCommand[1] = strafe;
        lastCommand[2] = turn;
        setWheelPowers(p[0], p[1], p[2], p[3]);
    }

    /** Sets each wheel's power directly (used by the Drivetrain Test OpMode to check motor names and directions). */
    public void setWheelPowers(double fl, double fr, double bl, double br) {
        lastPowers[0] = fl;
        lastPowers[1] = fr;
        lastPowers[2] = bl;
        lastPowers[3] = br;
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    /** Last powers sent to {frontLeft, frontRight, backLeft, backRight} (for telemetry). */
    public double[] getLastPowers() {
        return lastPowers.clone();
    }

    /** Last {forward, strafe, turn} passed to {@link #drive} (for telemetry). */
    public double[] getLastCommand() {
        return lastCommand.clone();
    }

    /** Encoder positions {frontLeft, frontRight, backLeft, backRight}; only meaningful if encoders are plugged in. */
    public int[] getWheelPositions() {
        return new int[]{frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(), backRight.getCurrentPosition()};
    }

    /**
     * Mecanum mix: {frontLeft, frontRight, backLeft, backRight}, scaled down together so none exceeds 1.
     * A NaN or infinite input is treated as 0 so a bad command can never reach a motor.
     */
    public static double[] mix(double forward, double strafe, double turn) {
        forward = finiteOrZero(forward);
        strafe = finiteOrZero(strafe);
        turn = finiteOrZero(turn);
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        return new double[]{fl / max, fr / max, bl / max, br / max};
    }

    private static double finiteOrZero(double x) {
        return Double.isNaN(x) || Double.isInfinite(x) ? 0 : x;
    }

    public void stop() {
        drive(0, 0, 0);
    }

    /** Logarithmic stick curve: sign(x) * ln(1 + k|x|) / ln(1 + k). Maps [-1, 1] to [-1, 1]. */
    public static double logScale(double x) {
        if (!(RobotSettings.Drivetrain.LOG_STRENGTH > 0)) {
            return x;   // invalid strength (0 would divide 0 by 0): linear
        }
        return Math.signum(x) * Math.log(1 + RobotSettings.Drivetrain.LOG_STRENGTH * Math.abs(x)) / Math.log(1 + RobotSettings.Drivetrain.LOG_STRENGTH);
    }
}
