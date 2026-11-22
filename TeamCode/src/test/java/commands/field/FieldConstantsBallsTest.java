package commands.field;

import robot.RobotSettings;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import commands.subsystems.ball_sensing.BallTracker;
import org.junit.Test;

/**
 * Ball-capacity constants. HIVE_TIP_POLLEN is [USER-CONFIRMED] (the manual does not state it).
 * PRELOAD_POLLEN is [OFFICIAL G304]. RobotSettings.BallTracker.MAX_BALLS is [USER-CONFIRMED] max held = 4.
 */
public class FieldConstantsBallsTest {
    @Test
    public void hiveTipsAtEightPollen() {
        assertEquals(8, FieldConstants.HIVE_TIP_POLLEN);
    }

    @Test
    public void preloadIsFourPollen() {
        assertEquals(4, FieldConstants.PRELOAD_POLLEN);
    }

    @Test
    public void preloadFitsInBallTrackerCapacity() {
        assertTrue(FieldConstants.PRELOAD_POLLEN <= RobotSettings.BallTracker.MAX_BALLS);
    }
}
