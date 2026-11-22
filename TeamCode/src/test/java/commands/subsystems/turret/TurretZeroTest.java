package commands.subsystems.turret;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Hardware-free tests of the persisted-zero decision in {@link Turret}. */
public class TurretZeroTest {
    @Test
    public void notWantedAlwaysZeroAtInit() {
        assertEquals(Turret.SRC_ZERO, Turret.decideZero(false, true, 1, 300));
        assertFalse(Turret.usePersisted(false, true, 1, 300));
    }

    @Test
    public void noSavedAngleFallsBack() {
        assertEquals(Turret.SRC_NONE, Turret.decideZero(true, false, 0, 300));
    }

    @Test
    public void freshAngleIsUsed() {
        assertTrue(Turret.usePersisted(true, true, 10, 300));
        assertTrue(Turret.usePersisted(true, true, 300, 300));   // exactly at the limit is still fresh
    }

    @Test
    public void staleAngleIsIgnored() {
        assertEquals(Turret.SRC_STALE, Turret.decideZero(true, true, 300.5, 300));
        assertFalse(Turret.usePersisted(true, true, 1e6, 300));
    }

    @Test
    public void negativeAgeIsRejected() {
        assertFalse(Turret.usePersisted(true, true, -1, 300));
    }

    @Test
    public void defaultsAreSafe() {
        assertEquals(Turret.ZeroMode.ZERO_AT_INIT, robot.RobotSettings.Turret.ZERO_MODE);
        assertTrue(robot.RobotSettings.Turret.PERSIST_MAX_AGE_S > 0);
    }

    @Test
    public void clearedAngleIsNotValid() {
        Turret.clearPersistedAngle();   // must not throw without hardware; later Turrets then see "no persisted angle"
    }

    @Test
    public void nanAndInfiniteInputsAreRejected() {
        assertFalse(Turret.usePersisted(true, true, Double.NaN, 300));
        assertFalse(Turret.usePersisted(true, true, 10, Double.NaN));
        assertFalse(Turret.usePersisted(true, true, Double.NaN, Double.NaN));
        assertFalse(Turret.usePersisted(true, true, Double.NEGATIVE_INFINITY, 300));
        assertFalse(Turret.usePersisted(true, true, Double.POSITIVE_INFINITY, 300));
        assertEquals(Turret.SRC_STALE, Turret.decideZero(true, true, Double.NaN, 300));
        assertEquals(Turret.SRC_STALE, Turret.decideZero(true, true, 10, Double.NaN));
    }
}
