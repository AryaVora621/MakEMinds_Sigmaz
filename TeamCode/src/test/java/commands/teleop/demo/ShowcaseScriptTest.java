package commands.teleop.demo;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShowcaseScriptTest {
    @Test
    public void flywheelRampsUpHoldsAndReturnsToZero() {
        assertEquals(0, ShowcaseScript.rpm(0, 3000), 0);
        assertEquals(0, ShowcaseScript.rpm(2, 3000), 0);
        assertEquals(1500, ShowcaseScript.rpm(3.5, 3000), 1e-9);
        assertEquals(3000, ShowcaseScript.rpm(5, 3000), 1e-9);
        assertEquals(3000, ShowcaseScript.rpm(8, 3000), 1e-9);
        assertEquals(1500, ShowcaseScript.rpm(9.5, 3000), 1e-9);
        assertEquals(0, ShowcaseScript.rpm(11, 3000), 0);
        assertEquals(0, ShowcaseScript.rpm(12, 3000), 0);
        assertEquals(0, ShowcaseScript.rpm(Double.NaN, 3000), 0);
    }

    @Test
    public void turretStaysInsideSpanAndRestsAtCentre() {
        for (double t = 0; t < 14; t += 0.05) {
            assertTrue(Math.abs(ShowcaseScript.turretDeg(t, 25)) <= 25 + 1e-9);
        }
        assertEquals(0, ShowcaseScript.turretDeg(0, 25), 1e-9);
        assertEquals(25, ShowcaseScript.turretDeg(1, 25), 1e-9);
        assertEquals(-25, ShowcaseScript.turretDeg(3, 25), 1e-9);
        assertEquals(0, ShowcaseScript.turretDeg(9.999999, 25), 1e-3);
        assertEquals(0, ShowcaseScript.turretDeg(11, 25), 0);
    }
}
