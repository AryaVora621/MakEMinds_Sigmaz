package commands.teleop.demo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import robot.RobotSettings;

/**
 * Live Pinpoint odometry readout. Moves nothing (no motors are touched), so it is safe anywhere.
 *
 *   A = zero the pose here (x, y, heading = 0; IMU is not recalibrated)
 *   Y = recalibrate the IMU and zero (hold the robot STILL for about a second)
 *   B = zero the "distance travelled" counter
 *
 * Quick checks: push the robot 24 in straight forward, x must read about +24 and y about 0. Push it left, y must
 * go positive. Turn it 90 degrees counter-clockwise, heading must read about +90. A wrong sign means the pod
 * encoder direction in RobotSettings.Odometry is flipped; a heading that creeps while still means a bad IMU calibration.
 */
@TeleOp(name = "Demo: Pinpoint Dashboard", group = "Demo")
public class PinpointDashboard extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        GoBildaPinpointDriver pin = DemoPinpoint.open(hardwareMap);
        while (!isStarted() && !isStopRequested()) {
            if (pin == null) {
                telemetry.addData("Pinpoint", "NOT FOUND: config name \"%s\" (RobotSettings.Odometry.HARDWARE_NAME)", RobotSettings.Odometry.HARDWARE_NAME);
            } else {
                pin.update();
                telemetry.addData("Pinpoint", "found, status %s", pin.getDeviceStatus());
            }
            telemetry.addLine("Press PLAY. A = zero pose, Y = recalibrate IMU (hold still), B = zero distance counter");
            telemetry.update();
        }
        if (isStopRequested()) return;

        boolean aWas = false, yWas = false, bWas = false;
        double lastX = Double.NaN, lastY = Double.NaN, dist = 0;
        while (opModeIsActive()) {
            if (pin == null) {
                telemetry.addData("Pinpoint", "NOT FOUND: \"%s\". Check the name in the robot config and RobotSettings.Odometry.HARDWARE_NAME", RobotSettings.Odometry.HARDWARE_NAME);
                telemetry.update();
                sleep(50);
                continue;
            }
            if (gamepad1.a && !aWas) {
                DemoPinpoint.zeroPose(pin);
                lastX = Double.NaN;
            }
            if (gamepad1.y && !yWas) {
                pin.resetPosAndIMU();
                lastX = Double.NaN;
            }
            if (gamepad1.b && !bWas) dist = 0;
            aWas = gamepad1.a; yWas = gamepad1.y; bWas = gamepad1.b;

            pin.update();
            double x = DemoPinpoint.x(pin), y = DemoPinpoint.y(pin);
            double h = pin.getHeading(AngleUnit.DEGREES);
            if (DemoPinpoint.finite(x, y, lastX, lastY)) dist += Math.hypot(x - lastX, y - lastY);
            lastX = x; lastY = y;

            GoBildaPinpointDriver.DeviceStatus st = pin.getDeviceStatus();
            telemetry.addData("STATUS", st == GoBildaPinpointDriver.DeviceStatus.READY ? "READY" : "PROBLEM: " + st);
            telemetry.addData("x / y (in)", "%.2f / %.2f", x, y);
            telemetry.addData("heading (deg)", "%.1f", h);
            telemetry.addData("vel x / y (in/s)", "%.1f / %.1f", pin.getVelX(DistanceUnit.INCH), pin.getVelY(DistanceUnit.INCH));
            telemetry.addData("turn rate (deg/s)", "%.1f", pin.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("distance travelled (in)", "%.1f", dist);
            telemetry.addData("raw pod ticks X (parallel) / Y (perpendicular)", "%d / %d", pin.getEncoderX(), pin.getEncoderY());
            telemetry.addData("pinpoint loop (Hz / us)", "%.0f / %d", pin.getFrequency(), pin.getLoopTime());
            telemetry.addData("pod offsets in use (in)", "parallel %.2f, perpendicular %.2f%s",
                    RobotSettings.Odometry.PARALLEL_POD_OFFSET_IN, RobotSettings.Odometry.PERPENDICULAR_POD_OFFSET_IN,
                    RobotSettings.Odometry.PARALLEL_POD_OFFSET_IN == 0 && RobotSettings.Odometry.PERPENDICULAR_POD_OFFSET_IN == 0 ? "  (both 0 = unmeasured PLACEHOLDER)" : "");
            telemetry.addLine("A zero pose | Y recalibrate IMU (still) | B zero distance");
            telemetry.update();
        }
    }
}
