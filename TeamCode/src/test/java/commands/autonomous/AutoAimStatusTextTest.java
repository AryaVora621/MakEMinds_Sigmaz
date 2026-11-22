package commands.autonomous;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Tests for {@link AutoAim#statusText(AutoAimLogic.Output)}. AutoAim has no static fields, so loading it
 * does not initialize Robot or any FTC hardware. Outputs are produced by {@link AutoAimLogic#update}.
 */
public class AutoAimStatusTextTest {

    /** Aim ready on the first update: finished with aimed=true. */
    @Test
    public void aimedOutputContainsAimed() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, true);
        AutoAimLogic.Output out = logic.update(0.0, true, true);
        assertTrue(out.done);
        assertTrue(out.aimed);
        String text = AutoAim.statusText(out);
        assertTrue(text, text.contains("aimed"));
        assertFalse(text, text.contains("TIMEOUT"));
    }

    /** Never ready and timeout elapsed: finished with timedOut=true. */
    @Test
    public void timedOutOutputContainsTimeout() {
        AutoAimLogic logic = new AutoAimLogic(1.0, false, true);
        logic.update(0.0, true, false);
        AutoAimLogic.Output out = logic.update(1.0, true, false);
        assertTrue(out.done);
        assertTrue(out.timedOut);
        assertFalse(out.aimed);
        String text = AutoAim.statusText(out);
        assertTrue(text, text.contains("TIMEOUT"));
    }

    /** Still aligning: not done, so the status is "aligning". */
    @Test
    public void notDoneOutputContainsAligning() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, true);
        AutoAimLogic.Output out = logic.update(0.0, true, false);
        assertFalse(out.done);
        String text = AutoAim.statusText(out);
        assertTrue(text, text.contains("aligning"));
    }

    /** Null output (no state yet) is treated as aligning, never a NullPointerException. */
    @Test
    public void nullOutputContainsAligning() {
        String text = AutoAim.statusText(null);
        assertTrue(text, text.contains("aligning"));
    }

    /** Shooter unavailable: done with aimed=false and timedOut=false, reported as no shooter. */
    @Test
    public void noShooterOutputIsNotAimedAndNotTimeout() {
        AutoAimLogic logic = new AutoAimLogic(2.0, false, true);
        AutoAimLogic.Output out = logic.update(0.0, false, false);
        assertTrue(out.done);
        assertFalse(out.aimed);
        assertFalse(out.timedOut);
        String text = AutoAim.statusText(out);
        assertFalse(text, text.contains("aligning"));
        assertFalse(text, text.contains("aimed") && !text.contains("not aimed"));
        assertTrue(text, text.contains("no shooter"));
    }

    /** Non-timeout, non-aimed output built directly: exact text is stable. */
    @Test
    public void statusTextIsStableForAimedAndTimeout() {
        AutoAimLogic.Output aimed = new AutoAimLogic.Output(true, true, false, true);
        assertEquals("AutoAim: aimed", AutoAim.statusText(aimed));
        AutoAimLogic.Output timedOut = new AutoAimLogic.Output(false, true, true, false);
        assertEquals("AutoAim: TIMEOUT (not aimed)", AutoAim.statusText(timedOut));
    }
}
