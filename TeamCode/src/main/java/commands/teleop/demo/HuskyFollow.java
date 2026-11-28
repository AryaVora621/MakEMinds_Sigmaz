package commands.teleop.demo;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.drivebase.Drivebase;

/**
 * Follows whatever the HuskyLens is tracking, with no laptop. Plug the HuskyLens into an I2C port and name it
 * "huskylens" in the robot config.
 *
 * Teach it first, on the HuskyLens itself: Color Recognition mode, center the yellow object in the crosshair,
 * press its learn button (shows ID1). Y on gamepad1 switches to object tracking, face or tag mode. Then start this OpMode and it follows at about 30 Hz.
 *   No gamepad needed: it starts following on PLAY and holds the distance it saw at 70 px box width.
 *   gamepad1 A = remember the current distance as the one to hold
 *   gamepad1 B = pause (robot stops), X = resume
 *   gamepad1 Y = switch mode: object tracking / color / face / tag (re-teach on the HuskyLens after switching)
 * When it loses the target it spins slowly for up to 6 s to find it, then stops. STOP always ends it.
 */
@TeleOp(name = "Demo: Husky Follow", group = "Demo")
public class HuskyFollow extends LinearOpMode {
    private static final HuskyLens.Algorithm[] MODES = {
            // color first (owner 11/28): object tracking lost the ball as soon as it moved; a learned color holds on
            HuskyLens.Algorithm.COLOR_RECOGNITION, HuskyLens.Algorithm.OBJECT_TRACKING,
            HuskyLens.Algorithm.FACE_RECOGNITION, HuskyLens.Algorithm.TAG_RECOGNITION};

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100);
        HuskyLens husky = hardwareMap.tryGet(HuskyLens.class, "huskylens");
        Drivebase drive = new Drivebase(hardwareMap);
        boolean alive = husky != null && husky.knock();
        int mode = 0;
        if (alive) husky.selectAlgorithm(MODES[mode]);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("HuskyLens", husky == null ? "NOT FOUND: add \"huskylens\" (I2C) to the config"
                    : alive ? "found, mode " + MODES[mode] : "found but not answering: check the I2C cable");
            telemetry.addLine("Teach the object on the HuskyLens, then PLAY. A = hold this distance, B pause, X resume, Y mode");
            telemetry.update();
        }
        if (isStopRequested() || !alive) return;

        double targetW = 70;
        boolean paused = false, yWas = false;
        ElapsedTime lost = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                if (gamepad1.b) paused = true;
                if (gamepad1.x) paused = false;
                if (gamepad1.y && !yWas) {
                    mode = (mode + 1) % MODES.length;
                    husky.selectAlgorithm(MODES[mode]);
                }
                yWas = gamepad1.y;

                HuskyLens.Block[] blocks = husky.blocks();
                HuskyLens.Block best = null;
                for (HuskyLens.Block b : blocks) {
                    // learned objects have id >= 1; prefer the biggest one, which is usually the nearest
                    if (b.id >= 1 && (best == null || b.width * b.height > best.width * best.height)) best = b;
                }
                if (best != null) lost.reset();
                if (best != null && gamepad1.a) targetW = Math.max(20, best.width);

                double[] cmd = {0, 0};
                String state;
                if (paused) {
                    state = "paused (X to resume)";
                } else if (best == null) {
                    state = lost.seconds() > 0.3 ? "no target: stopped" : "target flicker";
                } else {
                    cmd = HuskyFollowMath.command(best.x, best.width, targetW);
                    state = "following";
                }
                if (paused) cmd = new double[] {0, 0};
                else if (lost.seconds() > 0.3 && lost.seconds() < 6) {
                    cmd = new double[] {0, 0.15};       // lost it: slow spin in place to find it again
                    state = "searching";
                } else if (lost.seconds() >= 6) {
                    cmd = new double[] {0, 0};           // give up after a few seconds rather than spin forever
                    state = "no target: stopped";
                }
                drive.drive(cmd[0], 0, cmd[1]);

                telemetry.addData("state", state);
                telemetry.addData("mode", MODES[mode]);
                telemetry.addData("target", best == null ? "none" : "x " + best.x + "  width " + best.width + " (hold " + (int) targetW + ")");
                telemetry.addData("fwd / turn", "%.2f / %.2f", cmd[0], cmd[1]);
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }
}
