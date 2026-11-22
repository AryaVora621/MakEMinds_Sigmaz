package commands.subsystems.sorting;

import robot.RobotSettings;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import commands.subsystems.ball_sensing.BallColor;
import commands.subsystems.ball_sensing.BallSensor;
import commands.subsystems.gate.Gate;

/**
 * Colour sorter: the single REV Color Sensor V3 on the left lane wall reads each ball and drives the lane gate.
 * See {@link SortLogic} for the rules. It uses its own BallSensor (colour detection forced on), separate from
 * the two-sensor BallTracker, and only touches the gate while it is enabled and the sensor is healthy.
 */
public class LaneSorter {
    private final BallSensor sensor;
    private final SortLogic logic = new SortLogic();
    private boolean enabled = true;
    private SortLogic.Decision last = SortLogic.Decision.NONE;

    public LaneSorter(HardwareMap hw) {
        sensor = new BallSensor(hw, RobotSettings.LaneSorter.SENSOR_NAME);
    }

    /** Call once per loop, before gate.update(). */
    public void update(Gate gate) {
        long nowMs = System.nanoTime() / 1000000L;
        sensor.setDetectColor(true);
        sensor.setEnabled(enabled);
        sensor.update(nowMs);
        if (!enabled || gate == null) {
            last = SortLogic.Decision.NONE;
            logic.forgetCommand();   // re-assert the gate when switched back on
            return;
        }
        last = logic.update(sensor.isValid(), sensor.isBallPresent(), sensor.getBallColor(),
                RobotSettings.LaneSorter.PATTERN, RobotSettings.LaneSorter.PASS_UNKNOWN, nowMs, RobotSettings.LaneSorter.CLEAR_MS);
        // Only send changes, so the driver's gate toggle (e.g. to free a held ball) is not undone every loop.
        if (!logic.isNewCommand()) return;
        if (last == SortLogic.Decision.OPEN) {
            gate.open();
        } else if (last == SortLogic.Decision.CLOSE) {
            gate.close();
        }
    }

    public boolean isEnabled() { return enabled; }

    public void setEnabled(boolean on) { enabled = on; }

    public void toggle() { enabled = !enabled; }

    public void resetPattern() { logic.reset(); }

    public boolean isValid() { return sensor.isValid(); }

    public BallColor getColor() { return sensor.getBallColor(); }

    /** Carriage position the ball at the sensor needs (pollen = rear, nectar = forward, NONE otherwise). */
    public SortLogic.Carriage getCarriage() { return SortLogic.carriageFor(sensor.getBallColor()); }

    public BallColor getWanted() { return SortLogic.wantedAt(RobotSettings.LaneSorter.PATTERN, logic.getIndex()); }

    public void addTelemetry(Telemetry t) {
        t.addData("sorter", "%s, wants %s (#%d), sees %s (carriage %s), gate cmd %s%s",
                enabled ? "ON" : "OFF", getWanted(), logic.getIndex(), sensor.getBallColor(), getCarriage(), last,
                !sensor.isAvailable() ? " SENSOR MISSING" : sensor.isFaulted() ? " SENSOR FAULTED" : "");
    }
}
