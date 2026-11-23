package commands.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import commands.field.Alliance;
import commands.subsystems.drivebase.Drivebase;
import robot.HardwareCheck;
import robot.Robot;
import robot.RobotSettings;

/**
 * Two drivers.
 *
 * BUTTON MAP
 *   INIT (before PLAY), gamepad1:
 *     X          select BLUE alliance (default is RobotSettings.Robot.ALLIANCE)
 *     B          select RED alliance
 *     BACK       send the turret to its centre (servo mid-travel); there is no encoder to re-zero
 *   RUN, gamepad1 (driver):
 *     left stick       drive / strafe
 *     right stick X    turn
 *     right bumper     intake + transfer forward
 *     left bumper      intake + transfer backward
 *   Notes: intake bumpers are ignored while gamepad2 right trigger is feeding. X/B alliance selection
 *   is ignored while gamepad1 Start is held.
 *   RUN, gamepad2 (operator):
 *     A                spin the flywheel up and keep aiming
 *     B                stop the flywheel
 *     right trigger    (held) feed a ball as soon as the shot is ready (flywheel and turret at
 *                      target, robot steady). Uses intake.feed().
 *     X                toggle the flower mechanism (rising edge; needs RobotSettings.Robot.USE_FLOWER_MECH);
 *                      when the flower mech is faulted, X clears the fault instead
 *     Y                toggle the gate: lets balls in to, or blocks them from, the shooter (rising edge; needs RobotSettings.Robot.USE_GATE)
 *     D-pad up/down    hive tipped: flip which cell is the aim target (RobotSettings.Robot.FAR_CELL_UP; rising edge)
 *
 * The shooter lineup comes from robot.RobotSettings.Robot; the shooter aims continuously from the live pose, so it
 * works while driving. The turret aims; the chassis is never auto-turned. Teleop starts from the pose
 * the last autonomous saved (else RobotSettings.TeleOp.FALLBACK_* pose) and does not save its own end pose.
 */
@TeleOp(name = "Two Player TeleOp", group = "TeleOp")
public class TwoPlayerTeleOp extends LinearOpMode {
    // Tunable constants (fallback start pose, trigger threshold) live in RobotSettings.TeleOp.

    @Override
    public void runOpMode() {
        // Pre-flight: report every missing/misspelled robot-config device at once instead of crashing on the first.
        List<String> missing = HardwareCheck.missing(hardwareMap);
        if (!missing.isEmpty()) {
            while (!isStopRequested()) {
                telemetry.addLine("CONFIG ERROR: these devices are not in the robot configuration:");
                for (String m : missing) {
                    telemetry.addLine("  " + m);
                }
                telemetry.addLine("Fix the names in the config (or the constants), then re-run.");
                telemetry.update();
                sleep(100);
            }
            return;
        }
        Robot robot = Robot.continuingFromAuto(hardwareMap,
                new Pose(RobotSettings.TeleOp.FALLBACK_X_IN, RobotSettings.TeleOp.FALLBACK_Y_IN, RobotSettings.TeleOp.FALLBACK_HEADING_RAD));

        try {
            boolean backWasDown = false;
            boolean xWasDown = false;
            boolean yWasDown = false;
            boolean dpadWasDown = false;
            while (opModeInInit()) {
                if (gamepad1.start) {
                    // Start+A/B assigns gamepads on the Driver Station: ignore X/B while Start is held
                } else if (gamepad1.x) {
                    robot.setAlliance(Alliance.BLUE);
                } else if (gamepad1.b) {
                    robot.setAlliance(Alliance.RED);
                }
                if (gamepad1.back && !backWasDown) {
                    robot.rezeroTurret();
                }
                backWasDown = gamepad1.back;

                telemetry.addData("alliance (X = BLUE, B = RED)", RobotSettings.Robot.ALLIANCE);
                telemetry.addData("turret (BACK = send to centre)", robot.turretZeroSource());
                telemetry.addLine("Config check passed: all required devices found");
                telemetry.update();
            }
            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                robot.update();

                if (gamepad2.x && !xWasDown) {
                    robot.toggleFlower();
                }
                xWasDown = gamepad2.x;
                if (gamepad2.y && !yWasDown) {
                    robot.toggleGate();
                }
                yWasDown = gamepad2.y;
                boolean dpadDown = gamepad2.dpad_up || gamepad2.dpad_down;
                if (dpadDown && !dpadWasDown) {
                    robot.toggleFarCellUp();   // hive tipped: gamepad2 D-pad up/down flips the aim cell
                }
                dpadWasDown = dpadDown;

                if (robot.shooter != null) {
                    if (gamepad2.a) {
                        robot.shooter.setSpinning(true);
                    } else if (gamepad2.b) {
                        robot.shooter.setSpinning(false);
                    }
                }

                robot.drivebase.drive(
                        Drivebase.logScale(-gamepad1.left_stick_y),
                        Drivebase.logScale(gamepad1.left_stick_x),
                        Drivebase.logScale(gamepad1.right_stick_x));

                if (!robot.feedIfReady(gamepad2.right_trigger > RobotSettings.TeleOp.TRIGGER_PRESS_THRESHOLD)) {
                    // DRIVE-ONLY MODE: robot.intake is null (commented out in Robot.java), so skip the intake buttons.
                    if (robot.intake != null) {
                        if (gamepad1.right_bumper) {
                            robot.intake.forward();
                        } else if (gamepad1.left_bumper) {
                            robot.intake.backward();
                        } else {
                            robot.intake.stop();
                        }
                    }
                }

                robot.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            robot.stop(false);   // teleop does not overwrite the pose saved by autonomous
        }
    }
}
