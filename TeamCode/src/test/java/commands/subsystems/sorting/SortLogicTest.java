package commands.subsystems.sorting;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import commands.subsystems.ball_sensing.BallColor;
import commands.subsystems.sorting.SortLogic.Decision;

public class SortLogicTest {
    private static final long CLEAR = 100;

    @Test
    public void wantedAtRepeatsAndIgnoresJunk() {
        assertEquals(BallColor.POLLEN, SortLogic.wantedAt("PNN", 0));
        assertEquals(BallColor.NECTAR, SortLogic.wantedAt("PNN", 1));
        assertEquals(BallColor.POLLEN, SortLogic.wantedAt("PNN", 3));
        assertEquals(BallColor.NECTAR, SortLogic.wantedAt("p-n", 1));
        assertEquals(BallColor.UNKNOWN, SortLogic.wantedAt("", 0));
        assertEquals(BallColor.UNKNOWN, SortLogic.wantedAt(null, 0));
        assertEquals(BallColor.POLLEN, SortLogic.wantedAt("PN", -2));
    }

    @Test
    public void matchingBallOpensAndPatternAdvancesAfterClear() {
        SortLogic s = new SortLogic();
        assertEquals(Decision.OPEN, s.update(true, true, BallColor.POLLEN, "PN", false, 0, CLEAR));
        assertEquals(Decision.OPEN, s.update(true, false, BallColor.NONE, "PN", false, 50, CLEAR));
        assertEquals(0, s.getIndex());
        assertEquals(Decision.CLOSE, s.update(true, false, BallColor.NONE, "PN", false, 100, CLEAR));
        assertEquals(1, s.getIndex());
        assertEquals(Decision.NONE, s.update(true, false, BallColor.NONE, "PN", false, 200, CLEAR));
    }

    @Test
    public void wrongColourIsHeldAndPatternDoesNotAdvance() {
        SortLogic s = new SortLogic();
        assertEquals(Decision.CLOSE, s.update(true, true, BallColor.NECTAR, "PN", false, 0, CLEAR));
        assertEquals(Decision.CLOSE, s.update(true, false, BallColor.NONE, "PN", false, 150, CLEAR));
        assertEquals(0, s.getIndex());
    }

    @Test
    public void unknownFollowsPolicy() {
        SortLogic pass = new SortLogic();
        assertEquals(Decision.OPEN, pass.update(true, true, BallColor.UNKNOWN, "P", true, 0, CLEAR));
        SortLogic hold = new SortLogic();
        assertEquals(Decision.CLOSE, hold.update(true, true, BallColor.UNKNOWN, "P", false, 0, CLEAR));
    }

    @Test
    public void invalidSensorLetsGoAndRecloseOnReturn() {
        SortLogic s = new SortLogic();
        s.update(true, true, BallColor.POLLEN, "P", false, 0, CLEAR);
        assertEquals(Decision.NONE, s.update(false, false, BallColor.NONE, "P", false, 10, CLEAR));
        assertEquals(Decision.CLOSE, s.update(true, false, BallColor.NONE, "P", false, 20, CLEAR));
        assertEquals(Decision.NONE, s.update(true, false, BallColor.NONE, "P", false, 30, CLEAR));
    }

    @Test
    public void ballReappearingBeforeClearKeepsSameBall() {
        SortLogic s = new SortLogic();
        s.update(true, true, BallColor.POLLEN, "PN", false, 0, CLEAR);
        s.update(true, false, BallColor.NONE, "PN", false, 50, CLEAR);
        assertEquals(Decision.OPEN, s.update(true, true, BallColor.POLLEN, "PN", false, 60, CLEAR));
        assertEquals(0, s.getIndex());
    }

    @Test
    public void colourFlickerMidPassKeepsGateOpenAndAdvances() {
        SortLogic s = new SortLogic();
        assertEquals(Decision.OPEN, s.update(true, true, BallColor.POLLEN, "PN", false, 0, CLEAR));
        assertEquals(Decision.OPEN, s.update(true, true, BallColor.NECTAR, "PN", false, 10, CLEAR));
        assertEquals(Decision.OPEN, s.update(true, true, BallColor.UNKNOWN, "PN", false, 20, CLEAR));
        assertEquals(Decision.CLOSE, s.update(true, false, BallColor.NONE, "PN", false, 120, CLEAR));
        assertEquals(1, s.getIndex());
    }

    @Test
    public void heldBallReReadAsWantedIsLetThrough() {
        SortLogic s = new SortLogic();
        assertEquals(Decision.CLOSE, s.update(true, true, BallColor.UNKNOWN, "P", false, 0, CLEAR));
        assertEquals(Decision.OPEN, s.update(true, true, BallColor.POLLEN, "P", false, 10, CLEAR));
        s.update(true, false, BallColor.NONE, "P", false, 200, CLEAR);
        assertEquals(1, s.getIndex());
    }

    @Test
    public void emptyPatternLetsEverythingThroughRegardlessOfUnknownPolicy() {
        SortLogic s = new SortLogic();
        assertEquals(Decision.OPEN, s.update(true, true, BallColor.NECTAR, "", false, 0, CLEAR));
        SortLogic u = new SortLogic();
        assertEquals(Decision.OPEN, u.update(true, true, BallColor.UNKNOWN, "x", false, 0, CLEAR));
    }

    @Test
    public void nullColourIsHeld() {
        SortLogic s = new SortLogic();
        assertEquals(Decision.CLOSE, s.update(true, true, null, "P", true, 0, CLEAR));
    }

    @Test
    public void sensorLossWhileLettingThroughCountsTheBall() {
        SortLogic s = new SortLogic();
        s.update(true, true, BallColor.POLLEN, "PN", false, 0, CLEAR);
        s.update(false, false, BallColor.NONE, "PN", false, 10, CLEAR);
        assertEquals(1, s.getIndex());
        s.update(false, false, BallColor.NONE, "PN", false, 20, CLEAR);
        assertEquals(1, s.getIndex());   // counted once only
    }

    @Test
    public void sensorLossWhileHoldingDoesNotAdvance() {
        SortLogic s = new SortLogic();
        s.update(true, true, BallColor.NECTAR, "PN", false, 0, CLEAR);
        s.update(false, false, BallColor.NONE, "PN", false, 10, CLEAR);
        assertEquals(0, s.getIndex());
    }

    @Test
    public void heldBallForeverOnlySendsCloseOnceSoManualToggleWins() {
        SortLogic s = new SortLogic();
        assertEquals(Decision.CLOSE, s.update(true, true, BallColor.NECTAR, "P", false, 0, CLEAR));
        assertTrue(s.isNewCommand());
        for (long t = 10; t < 10000; t += 10) {
            assertEquals(Decision.CLOSE, s.update(true, true, BallColor.NECTAR, "P", false, t, CLEAR));
            assertFalse(s.isNewCommand());
        }
        // Driver opened the gate by hand; ball leaves; the clear re-asserts CLOSE once.
        assertEquals(Decision.CLOSE, s.update(true, false, BallColor.NONE, "P", false, 10050, CLEAR));
        assertFalse(s.isNewCommand());
        assertEquals(Decision.CLOSE, s.update(true, false, BallColor.NONE, "P", false, 10100, CLEAR));
        assertTrue(s.isNewCommand());
        assertEquals(0, s.getIndex());
        assertEquals(Decision.NONE, s.update(true, false, BallColor.NONE, "P", false, 10200, CLEAR));
        assertFalse(s.isNewCommand());
    }

    @Test
    public void openThenCloseAreEachSentOnce() {
        SortLogic s = new SortLogic();
        s.update(true, true, BallColor.POLLEN, "PN", false, 0, CLEAR);
        assertTrue(s.isNewCommand());
        s.update(true, false, BallColor.NONE, "PN", false, 50, CLEAR);
        assertFalse(s.isNewCommand());
        s.update(true, false, BallColor.NONE, "PN", false, 100, CLEAR);
        assertTrue(s.isNewCommand());
    }

    @Test
    public void forgetCommandResendsAfterReEnable() {
        SortLogic s = new SortLogic();
        s.update(true, true, BallColor.NECTAR, "P", false, 0, CLEAR);
        s.forgetCommand();
        s.update(true, true, BallColor.NECTAR, "P", false, 10, CLEAR);
        assertTrue(s.isNewCommand());
    }

    @Test
    public void sensorReturnReassertsCloseOnce() {
        SortLogic s = new SortLogic();
        s.update(false, false, BallColor.NONE, "P", false, 0, CLEAR);
        assertFalse(s.isNewCommand());
        assertEquals(Decision.CLOSE, s.update(true, false, BallColor.NONE, "P", false, 10, CLEAR));
        assertTrue(s.isNewCommand());
    }

    @Test
    public void carriageFollowsBallType() {
        assertEquals(SortLogic.Carriage.REAR, SortLogic.carriageFor(BallColor.POLLEN));
        assertEquals(SortLogic.Carriage.FORWARD, SortLogic.carriageFor(BallColor.NECTAR));
        assertEquals(SortLogic.Carriage.NONE, SortLogic.carriageFor(BallColor.UNKNOWN));
        assertEquals(SortLogic.Carriage.NONE, SortLogic.carriageFor(null));
    }

    @Test
    public void resetRestartsPattern() {
        SortLogic s = new SortLogic();
        s.update(true, true, BallColor.POLLEN, "PN", false, 0, CLEAR);
        s.update(true, false, BallColor.NONE, "PN", false, 200, CLEAR);
        assertEquals(1, s.getIndex());
        s.reset();
        assertEquals(0, s.getIndex());
    }
}
