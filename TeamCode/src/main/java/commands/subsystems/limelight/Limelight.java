package commands.subsystems.limelight;

import robot.RobotSettings;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Limelight 3A used only for a small horizontal aiming trim on the hive AprilTag (tx). It never
 * moves the robot pose. Call {@link #update()} once per loop.
 */
public class Limelight {
    private final Limelight3A camera;
    private boolean valid;
    private double tx;

    public Limelight(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, RobotSettings.Limelight.HARDWARE_NAME);
        camera.pipelineSwitch(RobotSettings.Limelight.PIPELINE);
        camera.start();
    }

    public void update() {
        LLResult r = camera.getLatestResult();
        valid = r != null && r.isValid() && r.getStaleness() <= RobotSettings.Limelight.MAX_STALENESS_MS;
        tx = valid ? r.getTx() : 0;
    }

    /** True if the camera currently sees the target. */
    public boolean isValid() {
        return valid;
    }

    /** Horizontal offset of the target from the crosshair, degrees (ignore unless isValid()). */
    public double getTx() {
        return tx;
    }

    public void stop() {
        camera.stop();
    }
}
