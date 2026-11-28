package commands.teleop.demo;

import static org.junit.Assert.*;

import org.junit.Test;

public class HuskyFollowMathTest {
    @Test public void centeredAndFarDrivesStraight() {
        double[] c = HuskyFollowMath.command(160, 30, 70);
        assertTrue(c[0] > 0.2);
        assertEquals(0, c[1], 1e-9);
    }

    @Test public void atDistanceHolds() {
        assertEquals(0, HuskyFollowMath.command(160, 68, 70)[0], 1e-9);
    }

    @Test public void tooCloseBacksUpSlowly() {
        double f = HuskyFollowMath.command(160, 200, 70)[0];
        assertTrue(f < 0 && f >= -0.2);
    }

    @Test public void targetOnRightTurnsClockwiseAndBarelyDrives() {
        double[] c = HuskyFollowMath.command(310, 30, 70);
        assertTrue(c[1] > 0);
        assertTrue(c[0] <= HuskyFollowMath.MAX_FWD * 0.25 + 1e-9);
    }

    @Test public void capsHold() {
        double[] c = HuskyFollowMath.command(0, 1, 300);
        assertTrue(Math.abs(c[0]) <= HuskyFollowMath.MAX_FWD + 1e-9);
        assertTrue(Math.abs(c[1]) <= HuskyFollowMath.MAX_TURN + 1e-9);
    }
}
