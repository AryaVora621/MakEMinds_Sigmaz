package commands.subsystems.grouped_subsystems.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Physical sanity checks of {@link ShotSolver}: no reference numbers, only properties that must hold. */
public class ShotSolverTest {
    private static final double TX = 59.25, TY = 88.483, TZ = 58.313;

    private static ShotSolver.Input input(double x, double y, double heading, boolean allowRising) {
        ShotSolver.Input in = new ShotSolver.Input();
        in.x = x;
        in.y = y;
        in.heading = heading;
        in.exitHeight = 14.0;
        in.targetX = TX;
        in.targetY = TY;
        in.targetZ = TZ;
        in.allowRising = allowRising;
        return in;
    }

    @Test
    public void stationaryShotLandsOnTarget() {
        ShotSolver.Mode[] modes = {ShotSolver.Mode.MIN_SPEED, ShotSolver.Mode.ENTRY_ANGLE, ShotSolver.Mode.HOOD_ANGLE};
        double[] params = {0, 0.9, 1.3};
        for (int i = 0; i < modes.length; i++) {
            ShotSolver.Result r = ShotSolver.solve(input(40, 60, 0.6, false), modes[i], params[i], ShotSolver.Arc.LOW);
            assertTrue(modes[i] + ": " + r.reason, r.ok);
            assertEquals(modes[i].toString(), 0.0, r.landingErrorIn, 1e-9);
            assertTrue(r.flightTimeS > 0);
            assertTrue(r.launchSpeedInS > 0);
        }
    }

    @Test
    public void movingShotLandsOnTarget() {
        ShotSolver.Input in = input(60, 50, 0.9, false);
        in.vx = 20;
        in.vy = -35;
        in.omega = 0.5;
        in.pivotOffsetX = 5;
        in.pivotOffsetY = -3;
        ShotSolver.Result r = ShotSolver.solve(in, ShotSolver.Mode.ENTRY_ANGLE, 0.85, ShotSolver.Arc.LOW);
        assertTrue(r.reason, r.ok);
        assertEquals(0.0, r.landingErrorIn, 1e-9);
        assertEquals(0.85, r.entryAngleRad, 1e-9);
    }

    @Test
    public void entryAngleModeReproducesRequestedAngle() {
        ShotSolver.Result r = ShotSolver.solve(input(30, 70, 0.2, false), ShotSolver.Mode.ENTRY_ANGLE, 0.9,
                ShotSolver.Arc.LOW);
        assertTrue(r.ok);
        assertEquals(0.9, r.entryAngleRad, 1e-9);
    }

    @Test
    public void hoodAngleModeReproducesRequestedElevation() {
        ShotSolver.Result r = ShotSolver.solve(input(35, 75, -0.3, false), ShotSolver.Mode.HOOD_ANGLE, 1.3,
                ShotSolver.Arc.HIGH);
        assertTrue(r.ok);
        assertEquals(1.3, r.elevationRad, 1e-9);
    }

    @Test
    public void highArcFliesLongerThanLowArc() {
        ShotSolver.Result low = ShotSolver.solve(input(50, 40, 1.0, true), ShotSolver.Mode.SPEED, 300, ShotSolver.Arc.LOW);
        ShotSolver.Result high = ShotSolver.solve(input(50, 40, 1.0, true), ShotSolver.Mode.SPEED, 300, ShotSolver.Arc.HIGH);
        assertTrue(low.ok);
        assertTrue(high.ok);
        assertTrue(low.flightTimeS > 0);
        assertTrue(high.flightTimeS > low.flightTimeS);
        assertTrue(high.elevationRad > low.elevationRad);
        assertEquals(0.0, low.landingErrorIn, 1e-9);
        assertEquals(0.0, high.landingErrorIn, 1e-9);
    }

    @Test
    public void speedBelowMinimumIsUnreachable() {
        ShotSolver.Result r = ShotSolver.solve(input(30, 70, 0, false), ShotSolver.Mode.SPEED, 60, ShotSolver.Arc.LOW);
        assertFalse(r.ok);
        assertFalse(r.reason.isEmpty());
        assertTrue(r.minSpeedInS > 60);
    }

    @Test
    public void minSpeedIsAtMostAnyFeasibleSpeed() {
        ShotSolver.Result min = ShotSolver.solve(input(40, 60, 0.6, true), ShotSolver.Mode.MIN_SPEED, 0, ShotSolver.Arc.LOW);
        ShotSolver.Result fast = ShotSolver.solve(input(40, 60, 0.6, true), ShotSolver.Mode.SPEED, 250, ShotSolver.Arc.LOW);
        assertTrue(min.ok);
        assertTrue(fast.ok);
        assertTrue(min.launchSpeedInS <= 250 + 1e-9);
    }

    @Test
    public void invalidEntryAndHoodAnglesAreRejected() {
        assertFalse(ShotSolver.solve(input(30, 70, 0, false), ShotSolver.Mode.ENTRY_ANGLE, 0, ShotSolver.Arc.LOW).ok);
        assertFalse(ShotSolver.solve(input(30, 70, 0, false), ShotSolver.Mode.ENTRY_ANGLE, Math.PI / 2, ShotSolver.Arc.LOW).ok);
        assertFalse(ShotSolver.solve(input(30, 70, 0, false), ShotSolver.Mode.HOOD_ANGLE, -0.1, ShotSolver.Arc.LOW).ok);
    }

    @Test
    public void wrapStaysInHalfOpenRange() {
        for (double a = -50; a <= 50; a += 0.37) {
            double w = ShotSolver.wrap(a);
            assertTrue("wrap(" + a + ")=" + w, w >= -Math.PI && w < Math.PI);
            // same angle modulo 2 pi
            assertEquals(Math.cos(a), Math.cos(w), 1e-9);
            assertEquals(Math.sin(a), Math.sin(w), 1e-9);
        }
    }

    @Test
    public void wrapOfPiAsImplemented() {
        // The javadoc says (-pi, pi], but ((a + pi) % 2pi) - pi maps pi to -pi, so the real range is [-pi, pi).
        assertEquals(-Math.PI, ShotSolver.wrap(Math.PI), 1e-12);
        assertEquals(-Math.PI, ShotSolver.wrap(-Math.PI), 1e-12);
        assertEquals(0.0, ShotSolver.wrap(0.0), 1e-12);
        assertEquals(0.0, ShotSolver.wrap(2 * Math.PI), 1e-9);
        assertEquals(-Math.PI + 0.1, ShotSolver.wrap(Math.PI + 0.1), 1e-9);
        assertEquals(0.1, ShotSolver.wrap(0.1 - 4 * Math.PI), 1e-9);
    }
}
