package commands.teleop.demo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.charset.StandardCharsets;
import java.io.FileInputStream;
import java.io.ByteArrayOutputStream;

import commands.subsystems.drivebase.Drivebase;

/**
 * Drives from a laptop on USB: the laptop writes one line to /sdcard/FIRST/laptop/cmd.txt over adb
 * ("seq forward strafe turn ms" or "seq stop"), this OpMode runs each new seq once for at most 1 s, then stops.
 * It writes pose and state to status.txt for the laptop to read.
 *
 * Safety: power capped at 0.30, every command times out on its own (a lost link can only coast to a stop),
 * a command older than this OpMode run is ignored, gamepad1 B locks out the laptop until X, and the Driver Station
 * STOP button always wins. Any stick on gamepad1 also stops the laptop command and drives by hand.
 */
@TeleOp(name = "Demo: Laptop Drive", group = "Demo")
public class LaptopDrive extends LinearOpMode {
    private static final File DIR = new File("/sdcard/FIRST/laptop");
    private static final File CMD = new File(DIR, "cmd.txt");
    private static final File STATUS = new File(DIR, "status.txt");

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100);
        DIR.mkdirs();
        GoBildaPinpointDriver pin = DemoPinpoint.open(hardwareMap);
        Drivebase drive = new Drivebase(hardwareMap);

        // Anything already in the file was sent before this run: remember its seq so it never executes.
        RemoteCommand first = RemoteCommand.parse(read());
        long lastSeq = first == null ? -1 : first.seq;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Laptop Drive: press PLAY. B = lock out laptop, X = unlock, sticks = drive by hand.");
            telemetry.addData("Pinpoint", pin == null ? "not found (pose will read 0)" : "found");
            telemetry.update();
            writeStatus("waiting_for_play", lastSeq, pin);
        }
        if (isStopRequested()) return;
        if (pin != null) DemoPinpoint.zeroPose(pin);

        boolean locked = false;
        RemoteCommand active = null;
        ElapsedTime activeT = new ElapsedTime();
        ElapsedTime pollT = new ElapsedTime();
        String state = "idle";

        try {
            while (opModeIsActive()) {
                if (pin != null) pin.update();
                if (gamepad1.b) locked = true;
                if (gamepad1.x) locked = false;

                if (pollT.milliseconds() > 40) {
                    pollT.reset();
                    RemoteCommand c = RemoteCommand.parse(read());
                    if (c != null && c.seq != lastSeq) {
                        lastSeq = c.seq;
                        active = c.stop || locked ? null : c;
                        activeT.reset();
                    }
                }

                double hf = -gamepad1.left_stick_y, hs = gamepad1.left_stick_x, ht = gamepad1.right_stick_x;
                boolean hand = Math.abs(hf) > 0.1 || Math.abs(hs) > 0.1 || Math.abs(ht) > 0.1;

                if (hand) {
                    active = null;
                    drive.drive(hf * 0.3, hs * 0.3, ht * 0.3);
                    state = "hand";
                } else if (locked) {
                    active = null;
                    drive.stop();
                    state = "locked";
                } else if (active != null && activeT.milliseconds() < active.ms) {
                    drive.drive(active.forward, active.strafe, active.turn);
                    state = "moving";
                } else {
                    active = null;
                    drive.stop();
                    state = "idle";
                }

                writeStatus(state, lastSeq, pin);
                telemetry.addData("state", state);
                telemetry.addData("last seq", lastSeq);
                if (pin != null) telemetry.addData("pose x / y (in), heading (deg)", "%.1f / %.1f, %.1f",
                        DemoPinpoint.x(pin), DemoPinpoint.y(pin), Math.toDegrees(DemoPinpoint.headingRad(pin)));
                telemetry.addLine("B lock out laptop | X unlock | sticks drive by hand | STOP ends it");
                telemetry.update();
            }
        } finally {
            drive.stop();
            writeStatus("stopped", lastSeq, pin);
        }
    }

    private static String read() {
        // Control Hub is Android 7: no java.nio.file, so plain streams only.
        try (FileInputStream in = new FileInputStream(CMD)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buf = new byte[256];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
            return new String(out.toByteArray(), StandardCharsets.UTF_8);
        } catch (Exception e) {
            return null;
        }
    }

    private void writeStatus(String state, long seq, GoBildaPinpointDriver pin) {
        String pose = pin == null ? "nan nan nan" : String.format(java.util.Locale.US, "%.2f %.2f %.1f",
                DemoPinpoint.x(pin), DemoPinpoint.y(pin), Math.toDegrees(DemoPinpoint.headingRad(pin)));
        String line = state + " " + seq + " " + pose + "\n";
        // write then rename, so the laptop never reads a half-written file
        File tmp = new File(DIR, "status.tmp");
        try (FileOutputStream o = new FileOutputStream(tmp)) {
            o.write(line.getBytes(StandardCharsets.UTF_8));
            o.close();
            if (!tmp.renameTo(STATUS)) STATUS.delete();
            tmp.renameTo(STATUS);
        } catch (Exception ignored) {
            // status is a convenience for the laptop; driving must not depend on it
        }
    }
}
