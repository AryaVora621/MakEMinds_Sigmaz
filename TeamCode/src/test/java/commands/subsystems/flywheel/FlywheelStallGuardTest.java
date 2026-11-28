package commands.subsystems.flywheel;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FlywheelStallGuardTest {
    @Test
    public void countsWhileFullPowerAndNotTurning() {
        double s = 0;
        for (int i = 0; i < 100; i++) s = Flywheel.stallStep(s, 2000, 0, 1.0, 0.025);
        assertTrue(s > Flywheel.STALL_CUT_S);
    }

    @Test
    public void reverseSpinCounts() {
        assertEquals(0.1, Flywheel.stallStep(0, 2000, -500, 1.0, 0.1), 1e-9);
    }

    @Test
    public void nanVelocityCounts() {
        assertEquals(0.1, Flywheel.stallStep(0, 2000, Double.NaN, 1.0, 0.1), 1e-9);
    }

    @Test
    public void normalSpinUpResets() {
        assertEquals(0, Flywheel.stallStep(1.5, 2000, 900, 1.0, 0.025), 0);
    }

    @Test
    public void lowPowerOrNoTargetResets() {
        assertEquals(0, Flywheel.stallStep(1.5, 2000, 0, 0.3, 0.025), 0);
        assertEquals(0, Flywheel.stallStep(1.5, 0, 0, 1.0, 0.025), 0);
    }
}
