package commands.subsystems.turret;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretEstimateTest {
    @Test
    public void movesTowardTargetAtSpeed() {
        assertEquals(25, Turret.estimateStep(0, 100, 250, 0.1), 1e-9);
        assertEquals(-25, Turret.estimateStep(0, -100, 250, 0.1), 1e-9);
    }

    @Test
    public void arrivesWithoutOvershoot() {
        assertEquals(10, Turret.estimateStep(5, 10, 250, 0.1), 1e-9);
        assertEquals(10, Turret.estimateStep(10, 10, 250, 0.1), 1e-9);
    }

    @Test
    public void zeroOrBadSpeedJumpsToTarget() {
        assertEquals(100, Turret.estimateStep(0, 100, 0, 0.1), 1e-9);
        assertEquals(100, Turret.estimateStep(0, 100, Double.NaN, 0.1), 1e-9);
    }

    @Test
    public void negativeOrNanDtDoesNotMove() {
        assertEquals(0, Turret.estimateStep(0, 100, 250, -1), 1e-9);
        assertEquals(0, Turret.estimateStep(0, 100, 250, Double.NaN), 1e-9);
    }
}
