package commands.teleop.demo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.flywheel.Flywheel;
import commands.subsystems.turret.Turret;
import commands.subsystems.turret.TurretLimits;
import robot.RobotSettings;

/**
 * The "show it off" OpMode: one press of A runs a 12 s scripted show while the Pinpoint pose stays on screen.
 * The robot does not drive. Each piece skips cleanly if its hardware is missing.
 *
 *   0-10 s   turret sweeps left and right through the middle (+/- 25 deg, inside the soft limits)
 *   2-5 s    flywheel ramps to the peak rpm, holds to 8 s, ramps back to 0 by 11 s
 *   10-12 s  turret returns to centre
 *
 *   A = start the show, B = STOP everything now, X = zero the Pinpoint pose
 *   D-pad up / down = flywheel peak rpm (default 3000, 2500 to 4000, never above Flywheel.HARD_MAX_RPM)
 * Keep clear of the wheel and the turret. Flywheel.MIN_RPM (2500) is the lowest non-zero speed, so the ramp starts
 * there rather than at 0.
 */
@TeleOp(name = "Demo: Showcase", group = "Demo")
public class ShowcaseDemo extends LinearOpMode {
    private static final double SPAN_DEG = 25;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        GoBildaPinpointDriver pin = DemoPinpoint.open(hardwareMap);
        Turret turret = null;
        String turretMsg = "ok";
        try {
            boolean a = hardwareMap.tryGet(Servo.class, RobotSettings.Turret.SERVO_A_NAME) != null;
            boolean b = hardwareMap.tryGet(Servo.class, RobotSettings.Turret.SERVO_B_NAME) != null;
            if (a && b) turret = new Turret(hardwareMap);
            else turretMsg = "NOT FOUND:" + (a ? "" : " \"" + RobotSettings.Turret.SERVO_A_NAME + "\"") + (b ? "" : " \"" + RobotSettings.Turret.SERVO_B_NAME + "\"");
        } catch (RuntimeException e) {
            turret = null;
            turretMsg = "NOT AVAILABLE (" + e.getMessage() + ")";
        }
        Flywheel fw = null;
        String fwMsg = "ok";
        try {
            fw = new Flywheel(hardwareMap);
            if (!fw.hasMotorB()) fwMsg = "motor B \"" + RobotSettings.Flywheel.MOTOR_B_NAME + "\" NOT FOUND, running on A only";
        } catch (RuntimeException e) {
            fwMsg = "NOT FOUND: \"" + RobotSettings.Flywheel.MOTOR_NAME + "\"";
        }
        double span = Math.min(SPAN_DEG, Math.min(Math.abs(TurretLimits.min()), Math.abs(TurretLimits.max())));
        double peak = 3000;
        double maxPeak = Math.min(4000, Flywheel.HARD_MAX_RPM);
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Pinpoint", pin == null ? "NOT FOUND: \"" + RobotSettings.Odometry.HARDWARE_NAME + "\"" : "found");
            telemetry.addData("Turret", turret == null ? turretMsg : "found");
            telemetry.addData("Flywheel", fwMsg);
            telemetry.addLine("PLAY, then A runs the 12 s show. B stops everything. Keep clear of the wheel and turret.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        boolean running = false;
        boolean aWas = false, xWas = false, upWas = false, downWas = false;
        ElapsedTime show = new ElapsedTime();
        double cmdRpm = 0;
        try {
            while (opModeIsActive()) {
                if (gamepad1.dpad_up && !upWas) peak = DemoMath.adjust(peak, true, false, 250, 2500, maxPeak);
                if (gamepad1.dpad_down && !downWas) peak = DemoMath.adjust(peak, false, true, 250, 2500, maxPeak);
                upWas = gamepad1.dpad_up; downWas = gamepad1.dpad_down;
                if (gamepad1.a && !aWas && !running) {
                    running = true;
                    show.reset();
                }
                aWas = gamepad1.a;
                if (gamepad1.b) running = false;
                if (gamepad1.x && !xWas && pin != null) DemoPinpoint.zeroPose(pin);
                xWas = gamepad1.x;

                double t = running ? show.seconds() : 0;
                if (running && t >= ShowcaseScript.DURATION_S) running = false;
                cmdRpm = running ? ShowcaseScript.rpm(t, peak) : 0;

                if (turret != null) {
                    turret.setTargetAngle(running ? ShowcaseScript.turretDeg(t, span) : turret.getTargetAngle());
                    if (!running && gamepad1.b) turret.setTargetAngle(0);
                    turret.update();
                }
                if (fw != null) {
                    fw.setTargetRpm(cmdRpm > 1 ? cmdRpm : 0);
                    fw.update();
                }

                telemetry.addData("show", running ? String.format("RUNNING %.1f / %.0f s: %s", t, ShowcaseScript.DURATION_S, ShowcaseScript.phase(t)) : "idle (A = start, B = stop)");
                if (turret == null) {
                    telemetry.addData("Turret", turretMsg + " (skipped)");
                } else {
                    telemetry.addData("turret cmd / feedback (deg)", "%.1f / %.1f%s", turret.getTargetAngle(), turret.getAngle(),
                            turret.feedbackTrusted() ? "" : "  (feedback is an estimate)");
                }
                if (fw == null) {
                    telemetry.addData("Flywheel", fwMsg + " (skipped)");
                } else {
                    telemetry.addData("flywheel request / measured rpm", "%.0f / %.0f%s", cmdRpm, fw.getRpm(), fw.isFaulted() ? "  RUN-AWAY GUARD tripped" : "");
                    telemetry.addData("flywheel motor A / B rpm", "%.0f / %s", fw.getMotorARpm(), fw.hasMotorB() ? String.format("%.0f", fw.getMotorBRpm()) : "not found");
                }
                telemetry.addData("peak rpm (dpad up/down)", "%.0f", peak);
                if (pin == null) {
                    telemetry.addData("Pinpoint", "NOT FOUND: \"" + RobotSettings.Odometry.HARDWARE_NAME + "\" (skipped)");
                } else {
                    pin.update();
                    telemetry.addData("pose x / y (in), heading (deg)", "%.1f / %.1f, %.1f", DemoPinpoint.x(pin), DemoPinpoint.y(pin), Math.toDegrees(DemoPinpoint.headingRad(pin)));
                    telemetry.addData("Pinpoint status", pin.getDeviceStatus());
                }
                telemetry.update();
            }
        } finally {
            if (fw != null) {
                try { fw.stop(); } catch (RuntimeException ignored) { }
            }
            if (turret != null) turret.stop();
        }
    }
}
