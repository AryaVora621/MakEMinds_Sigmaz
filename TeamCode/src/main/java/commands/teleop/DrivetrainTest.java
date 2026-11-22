package commands.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import commands.subsystems.drivebase.Drivebase;
import robot.RobotSettings;

/**
 * Drivetrain check, no Pedro and no other subsystems. Run it with the robot on blocks (wheels off the floor).
 *
 *   Hold A = front-left wheel, B = front-right, X = back-left, Y = back-right (each at RobotSettings.Drivetrain.TEST_POWER).
 *   Every wheel must spin so the robot would move FORWARD. If one spins backwards or the wrong wheel
 *   moves, fix the name or direction in RobotSettings.Drivetrain (Pedro Constants.java reads it too).
 *
 *   Left stick = drive/strafe, right stick X = turn: the normal TeleOp mix, so you can check that
 *   forward, strafe-right and turn-clockwise do what the labels say.
 */
@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivebase drive = new Drivebase(hardwareMap);
        telemetry.addLine("Drivetrain Test: robot on blocks. A=FL B=FR X=BL Y=BR; sticks = normal drive.");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                boolean single = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;
                if (single) {
                    drive.setWheelPowers(
                            gamepad1.a ? RobotSettings.Drivetrain.TEST_POWER : 0,
                            gamepad1.b ? RobotSettings.Drivetrain.TEST_POWER : 0,
                            gamepad1.x ? RobotSettings.Drivetrain.TEST_POWER : 0,
                            gamepad1.y ? RobotSettings.Drivetrain.TEST_POWER : 0);
                } else {
                    drive.drive(Drivebase.logScale(-gamepad1.left_stick_y),
                            Drivebase.logScale(gamepad1.left_stick_x),
                            Drivebase.logScale(gamepad1.right_stick_x));
                }
                double[] p = drive.getLastPowers();
                int[] e = drive.getWheelPositions();
                telemetry.addData("power FL / FR / BL / BR", "%.2f / %.2f / %.2f / %.2f", p[0], p[1], p[2], p[3]);
                telemetry.addData("encoder FL / FR / BL / BR", "%d / %d / %d / %d", e[0], e[1], e[2], e[3]);
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }
}
