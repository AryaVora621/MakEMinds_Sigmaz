package commands.autonomous;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class AutoAimLogicTest {

    /** (1) transferOnStart=true: transfer is on on the very first update, before aim is ready or done. */
    @Test
    public void transferOnStartTrueIsOnOnFirstUpdate() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, true);
        AutoAimLogic.Output out = logic.update(0.0, true, false);
        assertTrue(out.transferOn);
        assertFalse(out.aimed);
        assertFalse(out.done);
    }

    /** (2) transferOnStart=false: transfer is off until aimReady has been seen, then on. */
    @Test
    public void transferOnStartFalseTurnsOnOnlyAfterAimReady() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, false);
        AutoAimLogic.Output first = logic.update(0.0, true, false);
        assertFalse(first.transferOn);
        assertFalse(first.done);
        AutoAimLogic.Output ready = logic.update(0.5, true, true);
        assertTrue(ready.transferOn);
        assertTrue(ready.done);
    }

    /** (3) done on the first update where aimReady, aimed, not timed out. */
    @Test
    public void doneOnFirstUpdateWithAimReady() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, false);
        AutoAimLogic.Output out = logic.update(0.0, true, true);
        assertTrue(out.done);
        assertTrue(out.aimed);
        assertFalse(out.timedOut);
    }

    /** (4) never ready, timeout 2 s: not done at 0 and 1.9, done/timedOut/not aimed at 2.0. */
    @Test
    public void neverReadyTimesOutAtTwoSeconds() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, true);
        AutoAimLogic.Output t0 = logic.update(0.0, true, false);
        assertFalse(t0.done);
        AutoAimLogic.Output t19 = logic.update(1.9, true, false);
        assertFalse(t19.done);
        assertFalse(t19.timedOut);
        AutoAimLogic.Output t20 = logic.update(2.0, true, false);
        assertTrue(t20.done);
        assertTrue(t20.timedOut);
        assertFalse(t20.aimed);
    }

    /** (5) shooter unavailable: done immediately, transfer off, not timed out. */
    @Test
    public void shooterUnavailableIsDoneImmediatelyWithTransferOff() {
        AutoAimLogic logic = new AutoAimLogic(2.0, true, true);
        AutoAimLogic.Output out = logic.update(0.0, false, false);
        assertTrue(out.done);
        assertFalse(out.transferOn);
        assertFalse(out.timedOut);
        assertFalse(out.aimed);
    }

    /** (6) timeoutS=0 finishes at once with timedOut=true when not ready (never infinite). */
    @Test
    public void zeroTimeoutFinishesAtOnceTimedOut() {
        AutoAimLogic logic = new AutoAimLogic(0.0, false, true);
        AutoAimLogic.Output out = logic.update(0.0, true, false);
        assertTrue(out.done);
        assertTrue(out.timedOut);
        assertFalse(out.aimed);
    }

    /** (6b) negative and NaN timeouts are treated as 0, not infinite. */
    @Test
    public void negativeAndNanTimeoutsAreTreatedAsZero() {
        AutoAimLogic negative = new AutoAimLogic(-5.0, false, true);
        AutoAimLogic.Output n = negative.update(0.0, true, false);
        assertTrue(n.done);
        assertTrue(n.timedOut);

        AutoAimLogic nan = new AutoAimLogic(Double.NaN, false, true);
        AutoAimLogic.Output m = nan.update(0.0, true, false);
        assertTrue(m.done);
        assertTrue(m.timedOut);
    }

    /** (7) shootAfter=false: the update after done reports transfer off. */
    @Test
    public void shootAfterFalseTurnsTransferOffOnUpdateAfterDone() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, true);
        AutoAimLogic.Output done = logic.update(0.0, true, true);
        assertTrue(done.done);
        AutoAimLogic.Output after = logic.update(0.1, true, true);
        assertTrue(after.done);
        assertFalse(after.transferOn);
    }

    /** (8) shootAfter=true: transfer stays on and done stays true on later updates. */
    @Test
    public void shootAfterTrueKeepsTransferOnAndDone() {
        AutoAimLogic logic = new AutoAimLogic(2.0, true, true);
        AutoAimLogic.Output done = logic.update(0.0, true, true);
        assertTrue(done.done);
        assertTrue(done.transferOn);
        AutoAimLogic.Output later = logic.update(0.5, true, false);
        assertTrue(later.done);
        assertTrue(later.transferOn);
        AutoAimLogic.Output much = logic.update(5.0, false, false);
        assertTrue(much.done);
        assertTrue(much.transferOn);
    }

    /** (9) reset() allows a new run with a fresh start time. */
    @Test
    public void resetRestartsTimeoutFromNewStartTime() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, false);
        logic.update(0.0, true, false);
        AutoAimLogic.Output timedOut = logic.update(2.0, true, false);
        assertTrue(timedOut.done);
        assertTrue(timedOut.timedOut);

        logic.reset();
        AutoAimLogic.Output start = logic.update(10.0, true, false);
        assertFalse(start.done);
        assertFalse(start.timedOut);
        assertFalse(start.transferOn);
        AutoAimLogic.Output almost = logic.update(11.9, true, false);
        assertFalse(almost.done);
        AutoAimLogic.Output end = logic.update(12.0, true, false);
        assertTrue(end.done);
        assertTrue(end.timedOut);
        assertEquals(false, end.aimed);
    }

    /** Extra: aim ready on the first update finishes aimed with transfer on. */
    @Test
    public void aimReadyImmediatelyFinishesAimedWithTransfer() {
        AutoAimLogic logic = new AutoAimLogic(3.0, false, false);
        AutoAimLogic.Output out = logic.update(0.0, true, true);
        assertTrue(out.done);
        assertTrue(out.aimed);
        assertFalse(out.timedOut);
        assertTrue(out.transferOn);
    }

    /**
     * (10) 100 updates at 20 ms steps (t = 0 .. 1.98 s). aimReady becomes true at t = 0.6 s and the timeout
     * is 3 s. Done must first be reported on the update where t first reaches 0.6 (index 30), and it must
     * be aimed, not timed out.
     */
    @Test
    public void aimReadyAtSixHundredMsFinishesOnFirstUpdateAtOrAfterThatTime() {
        AutoAimLogic logic = new AutoAimLogic(3.0, false, true);
        int firstDone = -1;
        for (int i = 0; i < 100; i++) {
            double t = i * 20 / 1000.0; // exact multiples of 0.02 s without accumulated float error
            boolean aimReady = t >= 0.6;
            AutoAimLogic.Output out = logic.update(t, true, aimReady);
            if (out.done && firstDone < 0) {
                firstDone = i;
                assertTrue("done before t=0.6 s at i=" + i, t >= 0.6);
                assertTrue(out.aimed);
                assertFalse(out.timedOut);
            }
        }
        assertEquals(30, firstDone);
    }

    /**
     * (11) aimReady flickers true then false. Done is reported on the first true (index 2). Later updates
     * with aimReady false must keep done=true (AutoAimLogic latches done; no change to the main source was
     * needed).
     */
    @Test
    public void doneStaysLatchedWhenAimReadyFlickersFalseAfterDone() {
        AutoAimLogic logic = new AutoAimLogic(3.0, false, false);
        boolean[] aimPattern = {false, false, true, false, true, false, false, false, false, false};
        int firstDone = -1;
        boolean seenDone = false;
        for (int i = 0; i < aimPattern.length; i++) {
            double t = i * 20 / 1000.0;
            AutoAimLogic.Output out = logic.update(t, true, aimPattern[i]);
            if (out.done && firstDone < 0) {
                firstDone = i;
            }
            if (seenDone) {
                assertTrue("done reverted to false at i=" + i, out.done);
            }
            seenDone = seenDone || out.done;
        }
        assertEquals(2, firstDone);
    }

    /** NaN clock fails safe: times out instead of waiting forever. */
    @Test
    public void nanClockTimesOut() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, true);
        AutoAimLogic.Output out = logic.update(Double.NaN, true, false);
        assertTrue(out.done);
        assertTrue(out.timedOut);
    }
}
