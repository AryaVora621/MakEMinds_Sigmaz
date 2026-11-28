package commands.teleop.demo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.drivebase.Drivebase;
import robot.RobotSettings;

/**
 * Drives a 24 in square (forward, left, back, right, always facing the same way) using Pinpoint feedback, at low power.
 * Clear a 3 ft x 3 ft area with room around it first.
 *
 *   A = start (zeroes the pose where the robot stands), B = STOP at any time, X = reset after a stop
 *   D-pad up / down = power cap (default 0.25, 0.10 to 0.40)
 *
 * It aborts on its own if odometry is bad: no Pinpoint, a non-finite pose, a pose that does not change while power is
 * applied (wheels slipping, odometry dead, robot blocked) or a leg that takes more than 8 s. The closing error is the
 * distance from the start point after the last corner; with good odometry and tuned wheels it should be a few inches.
 */
@TeleOp(name = "Demo: Drive Square", group = "Demo")
public class DriveSquareDemo extends LinearOpMode {
    private static final double SIDE_IN = 24;
    private static final double TOL_IN = 1.5;
    private static final double K_POS = 0.04;      // power per inch of error, before the cap
    private static final double K_TURN = 0.8;      // power per radian of heading error
    private static final double MIN_POWER = 0.12;  // floor so the wheels still move near the target
    private static final double LEG_TIMEOUT_S = 8;
    private static final double WINDOW_S = 0.5;    // pose must move at least MIN_MOVE_IN every window while driving
    private static final double MIN_MOVE_IN = 0.5;
    private static final double STUCK_ABORT_S = 1.0;
    private static final double SETTLE_S = 0.25;

    private enum State { IDLE, RUNNING, DONE, ABORTED }

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        GoBildaPinpointDriver pin = DemoPinpoint.open(hardwareMap);
        Drivebase drive = null;
        String driveError = null;
        try {
            drive = new Drivebase(hardwareMap);
        } catch (RuntimeException e) {
            driveError = "drive motors not found (" + RobotSettings.Drivetrain.FRONT_LEFT_NAME + ", " + RobotSettings.Drivetrain.FRONT_RIGHT_NAME
                    + ", " + RobotSettings.Drivetrain.BACK_LEFT_NAME + ", " + RobotSettings.Drivetrain.BACK_RIGHT_NAME + ")";
        }
        double cap = 0.25;
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Pinpoint", pin == null ? "NOT FOUND: \"" + RobotSettings.Odometry.HARDWARE_NAME + "\"" : "found");
            telemetry.addData("Drive motors", driveError == null ? "found" : "NOT FOUND: " + driveError);
            telemetry.addLine("Press PLAY, then A to drive a 24 in square. B = stop. Dpad up/down = power cap.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        State state = State.IDLE;
        String reason = "";
        int corner = 0;
        double legS = 0, settleS = 0, stuckS = 0, winS = 0, winX = 0, winY = 0, winPower = 0;
        double finalErr = Double.NaN, finalHeadingErrDeg = Double.NaN;
        double[] cmd = {0, 0, 0};
        boolean aWas = false, xWas = false, upWas = false, downWas = false;
        ElapsedTime loop = new ElapsedTime();

        try {
            while (opModeIsActive()) {
                double dt = Math.min(loop.seconds(), 0.1);
                loop.reset();
                boolean ready = pin != null && drive != null;
                if (ready) pin.update();
                double x = ready ? DemoPinpoint.x(pin) : 0, y = ready ? DemoPinpoint.y(pin) : 0, h = ready ? DemoPinpoint.headingRad(pin) : 0;

                if (gamepad1.dpad_up && !upWas) cap = DemoMath.adjust(cap, true, false, 0.05, 0.10, 0.40);
                if (gamepad1.dpad_down && !downWas) cap = DemoMath.adjust(cap, false, true, 0.05, 0.10, 0.40);
                upWas = gamepad1.dpad_up; downWas = gamepad1.dpad_down;

                if (gamepad1.b && state == State.RUNNING) {
                    state = State.ABORTED;
                    reason = "stopped by driver (B)";
                }
                if (gamepad1.x && !xWas && state != State.RUNNING) {
                    state = State.IDLE;
                    reason = "";
                }
                xWas = gamepad1.x;

                if (gamepad1.a && !aWas && state == State.IDLE && ready) {
                    DemoPinpoint.zeroPose(pin);
                    pin.update();
                    x = DemoPinpoint.x(pin); y = DemoPinpoint.y(pin); h = DemoPinpoint.headingRad(pin);
                    corner = 0; legS = 0; settleS = 0; stuckS = 0; winS = 0; winX = x; winY = y; winPower = 0;
                    finalErr = Double.NaN; finalHeadingErrDeg = Double.NaN;
                    state = State.RUNNING;
                }
                aWas = gamepad1.a;

                cmd[0] = cmd[1] = cmd[2] = 0;
                double[] target = DemoMath.squareCorner(corner, SIDE_IN);
                if (state == State.RUNNING) {
                    if (!DemoPinpoint.finite(x, y, h)) {
                        state = State.ABORTED;
                        reason = "pose is not a number (Pinpoint fault)";
                    } else if (pin.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                        state = State.ABORTED;
                        reason = "Pinpoint status " + pin.getDeviceStatus();
                    } else {
                        double err = Math.hypot(target[0] - x, target[1] - y);
                        legS += dt;
                        if (err <= TOL_IN) {
                            settleS += dt;
                            if (settleS >= SETTLE_S) {
                                if (corner == 3) {
                                    finalErr = err;
                                    finalHeadingErrDeg = Math.toDegrees(DemoMath.wrapRad(0 - h));
                                    state = State.DONE;
                                } else {
                                    corner++;
                                    legS = 0; settleS = 0; stuckS = 0; winS = 0; winX = x; winY = y; winPower = 0;
                                }
                            }
                        } else {
                            settleS = 0;
                            cmd = DemoMath.driveToward(x, y, h, target[0], target[1], 0, cap, K_POS, K_TURN, MIN_POWER, TOL_IN);
                        }
                        double power = Math.max(Math.abs(cmd[0]), Math.abs(cmd[1]));
                        winPower = Math.max(winPower, power);
                        winS += dt;
                        if (winS >= WINDOW_S) {
                            // Only judge while still well away from the target: close in, the floor power moves the robot slowly.
                            if (err > 4) {
                                stuckS = DemoMath.stuckStep(stuckS, Math.hypot(x - winX, y - winY), winPower, winS, MIN_MOVE_IN, MIN_POWER);
                            } else {
                                stuckS = 0;
                            }
                            winS = 0; winX = x; winY = y; winPower = 0;
                        }
                        if (state == State.RUNNING && stuckS >= STUCK_ABORT_S) {
                            state = State.ABORTED;
                            reason = "pose did not change while driving: odometry dead, wheels slipping or robot blocked";
                        } else if (state == State.RUNNING && legS > LEG_TIMEOUT_S) {
                            state = State.ABORTED;
                            reason = "leg " + (corner + 1) + " timed out";
                        }
                    }
                }
                if (state != State.RUNNING) {
                    cmd[0] = cmd[1] = cmd[2] = 0;
                }
                if (drive != null) drive.drive(cmd[0], cmd[1], cmd[2]);

                telemetry.addData("state", state + (reason.isEmpty() ? "" : ": " + reason));
                telemetry.addData("Pinpoint", pin == null ? "NOT FOUND: \"" + RobotSettings.Odometry.HARDWARE_NAME + "\"" : "status " + pin.getDeviceStatus());
                telemetry.addData("Drive motors", driveError == null ? "found" : "NOT FOUND: " + driveError);
                telemetry.addData("power cap (dpad up/down)", "%.2f", cap);
                telemetry.addData("pose x / y (in), heading (deg)", "%.1f / %.1f, %.1f", x, y, Math.toDegrees(h));
                if (state == State.RUNNING) {
                    telemetry.addData("corner", "%d of 4 -> (%.0f, %.0f)", corner + 1, target[0], target[1]);
                    telemetry.addData("error to corner (in)", "%.1f", Math.hypot(target[0] - x, target[1] - y));
                    telemetry.addData("cmd fwd / strafe / turn", "%.2f / %.2f / %.2f", cmd[0], cmd[1], cmd[2]);
                }
                if (state == State.DONE) {
                    telemetry.addData("CLOSING ERROR (in)", "%.1f  (heading %.1f deg)", finalErr, finalHeadingErrDeg);
                }
                telemetry.addLine("A start | B stop | X reset after stop | dpad up/down cap");
                telemetry.update();
            }
        } finally {
            if (drive != null) drive.stop();
        }
    }
}
