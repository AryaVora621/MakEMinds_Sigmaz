package commands.teleop.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.turret.Turret;
import commands.subsystems.turret.TurretLimits;
import robot.RobotSettings;

/**
 * Sweeps the turret left and right under servo control, between +/- a span that always stays inside TurretLimits
 * (which the Turret also clamps to the real servo travel). Turret motion is rate limited, so it never snaps.
 *
 *   A = start sweeping, B = STOP (holds where it is), X = go back to centre (0 deg) and hold
 *   D-pad up / down = sweep half-span (default 20 deg, 5 deg up to the soft limit)
 *   D-pad left / right = sweep speed (default 25 deg/s, 10 to 60)
 *
 * "commanded" is the angle sent to the servos. "feedback" is the angle the Turret reports: until
 * RobotSettings.AxonServo.FEEDBACK_CALIBRATED is true that is only an ESTIMATE that chases the command, so it will
 * follow the command whether or not the servos really moved. Watch the turret itself.
 */
@TeleOp(name = "Demo: Turret Sweep", group = "Demo")
public class TurretSweepDemo extends LinearOpMode {
    private enum Mode { HOLD, SWEEP, CENTER }

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        String missing = "";
        try {
            if (hardwareMap.tryGet(Servo.class, RobotSettings.Turret.SERVO_A_NAME) == null) missing += " \"" + RobotSettings.Turret.SERVO_A_NAME + "\"";
            if (hardwareMap.tryGet(Servo.class, RobotSettings.Turret.SERVO_B_NAME) == null) missing += " \"" + RobotSettings.Turret.SERVO_B_NAME + "\"";
        } catch (RuntimeException e) {
            missing += " (could not query the hardware map)";
        }
        Turret turret = null;
        if (missing.isEmpty()) {
            try {
                turret = new Turret(hardwareMap);
            } catch (RuntimeException e) {
                missing = " (turret could not be built: " + e.getMessage() + ")";
            }
        }
        double limit = Math.min(Math.abs(TurretLimits.min()), Math.abs(TurretLimits.max()));
        double span = Math.min(20, limit), rate = 25;
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Turret servos", turret == null ? "NOT FOUND:" + missing : "found (feedback inputs: " + (turret.hasFeedback() ? "found" : "not found, estimate only") + ")");
            telemetry.addData("soft limits (deg)", "%.0f to %.0f", TurretLimits.min(), TurretLimits.max());
            telemetry.addLine("Nothing moves until PLAY. A = sweep, B = stop, X = centre, dpad = span / speed");
            telemetry.update();
        }
        if (isStopRequested()) return;

        Mode mode = Mode.HOLD;
        double cmd = 0;
        int dir = 1;
        boolean upWas = false, downWas = false, leftWas = false, rightWas = false;
        ElapsedTime loop = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                double dt = Math.min(loop.seconds(), 0.1);
                loop.reset();
                if (gamepad1.dpad_up && !upWas) span = DemoMath.adjust(span, true, false, 5, 5, limit);
                if (gamepad1.dpad_down && !downWas) span = DemoMath.adjust(span, false, true, 5, 5, limit);
                if (gamepad1.dpad_right && !rightWas) rate = DemoMath.adjust(rate, true, false, 5, 10, 60);
                if (gamepad1.dpad_left && !leftWas) rate = DemoMath.adjust(rate, false, true, 5, 10, 60);
                upWas = gamepad1.dpad_up; downWas = gamepad1.dpad_down; leftWas = gamepad1.dpad_left; rightWas = gamepad1.dpad_right;

                if (gamepad1.b) mode = Mode.HOLD;
                else if (gamepad1.a) mode = Mode.SWEEP;
                else if (gamepad1.x) mode = Mode.CENTER;

                if (mode == Mode.SWEEP) {
                    cmd = DemoMath.clamp(cmd, -span, span);   // span may have been reduced
                    cmd = DemoMath.stepToward(cmd, dir * span, rate * dt);
                    if (Math.abs(cmd - dir * span) < 1e-9) dir = -dir;
                } else if (mode == Mode.CENTER) {
                    cmd = DemoMath.stepToward(cmd, 0, rate * dt);
                }

                if (turret != null) {
                    turret.setTargetAngle(cmd);
                    turret.update();
                }

                telemetry.addData("mode", mode + (mode == Mode.SWEEP ? (dir > 0 ? " (heading to +)" : " (heading to -)") : ""));
                if (turret == null) {
                    telemetry.addData("Turret servos", "NOT FOUND:" + missing);
                } else {
                    telemetry.addData("commanded (deg)", "%.1f  (turret clamps it to %.1f)", cmd, turret.getTargetAngle());
                    telemetry.addData("feedback (deg)", "%.1f  source: %s", turret.getAngle(),
                            turret.feedbackTrusted() ? "Axon feedback (calibrated)" : "ESTIMATE, not a measurement");
                    telemetry.addData("error (deg)", "%.1f   atTarget: %b", turret.getTargetAngle() - turret.getAngle(), turret.atTarget());
                    telemetry.addData("raw analog feedback (deg)", turret.hasFeedback() ? String.format("%.1f", turret.getRawFeedbackDeg()) : "not found");
                }
                telemetry.addData("span +/- (dpad up/down)", "%.0f deg of max %.0f", span, limit);
                telemetry.addData("speed (dpad left/right)", "%.0f deg/s", rate);
                telemetry.addLine("A sweep | B stop | X centre");
                telemetry.update();
            }
        } finally {
            if (turret != null) turret.stop();
        }
    }
}
