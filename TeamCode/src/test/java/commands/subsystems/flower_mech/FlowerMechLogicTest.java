package commands.subsystems.flower_mech;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import commands.subsystems.flower_mech.FlowerMechLogic.State;

public class FlowerMechLogicTest {
    private static final double DT = 0.02;

    private FlowerMechLogic make() {
        FlowerMechLogic l = new FlowerMechLogic();
        l.slewDegPerS = 300;
        l.spanDeg = 120;
        l.dwellMs = 500;
        l.syncTolDeg = 10;
        l.syncFaultMs = 250;
        l.faultGraceS = 1.0;
        return l;
    }

    /** Healthy trusted step. */
    private double step(FlowerMechLogic l, double dt) {
        return l.update(dt, true, true, false, 0);
    }

    private void run(FlowerMechLogic l, double seconds) {
        for (double t = 0; t < seconds; t += DT) step(l, DT);
    }

    @Test
    public void fullCycleReturnsToStowed() {
        FlowerMechLogic l = make();
        l.deploy();
        assertEquals(State.DEPLOYING, l.getState());
        run(l, 0.5);
        assertEquals(State.DWELL, l.getState());
        run(l, 0.6);
        assertTrue(l.getState() == State.STOWING || l.getState() == State.STOWED);
        run(l, 1.0);
        assertEquals(State.STOWED, l.getState());
        assertEquals(0, l.getProgress(), 1e-9);
    }

    @Test
    public void cancelWhileDeployingStows() {
        FlowerMechLogic l = make();
        l.deploy();
        run(l, 0.1);
        assertEquals(State.DEPLOYING, l.getState());
        l.toggle();
        assertEquals(State.STOWING, l.getState());
        assertEquals(0, l.getGoal(), 0);
        run(l, 1.0);
        assertEquals(State.STOWED, l.getState());
    }

    @Test
    public void stalledWhileStowedOrDwellingNeverFaults() {
        FlowerMechLogic l = make();
        l.update(DT, true, true, true, 50);
        assertEquals(State.STOWED, l.getState());
        l.deploy();
        run(l, 0.5);
        assertEquals(State.DWELL, l.getState());
        l.update(DT, true, true, true, 50);
        assertEquals(State.DWELL, l.getState());
    }

    @Test
    public void stallWhileMovingTrustedFaults() {
        FlowerMechLogic l = make();
        l.deploy();
        step(l, DT); // settle: first update after a state change ignores stall
        double p = l.update(DT, true, false, true, 0);
        assertEquals(State.FAULTED, l.getState());
        assertTrue(Double.isNaN(p));
        assertTrue(Double.isNaN(l.update(DT, true, true, false, 0)));
    }

    @Test
    public void syncDivergenceFaultsOnlyAfterTimer() {
        FlowerMechLogic l = make();
        l.deploy();
        for (int i = 0; i < 5; i++) l.update(DT, true, false, false, 30);
        assertEquals(State.DEPLOYING, l.getState());
        for (int i = 0; i < 20; i++) l.update(DT, true, false, false, 30);
        assertEquals(State.FAULTED, l.getState());
    }

    @Test
    public void untrustedFeedbackNeverFaultsAndRunsOpenLoop() {
        FlowerMechLogic l = make();
        l.deploy();
        for (int i = 0; i < 100; i++) {
            l.update(DT, false, false, true, 99);
            assertFalse(l.getState() == State.FAULTED);
        }
        // open loop: atTarget false is ignored, so the cycle still completes
        for (int i = 0; i < 300; i++) l.update(DT, false, false, true, 99);
        assertEquals(State.STOWED, l.getState());
    }

    @Test
    public void noRefaultInsideGraceWindow() {
        FlowerMechLogic l = make();
        l.deploy();
        step(l, DT); // settle: first update after a state change ignores stall
        l.update(DT, true, false, true, 0);
        assertEquals(State.FAULTED, l.getState());
        l.clearFault();
        assertEquals(State.STOWING, l.getState());
        assertTrue(l.inGrace());
        for (int i = 0; i < 40; i++) { // 0.8 s with a constant stall
            l.update(DT, true, false, true, 0);
            assertFalse(l.getState() == State.FAULTED);
        }
        for (int i = 0; i < 20; i++) l.update(DT, true, false, true, 0);
        assertEquals(State.FAULTED, l.getState());
    }

    @Test
    public void dtIsCapped() {
        FlowerMechLogic l = make();
        l.deploy();
        l.update(5.0, true, true, false, 0); // 0.1 s cap * 300 deg/s = 30 deg of 120
        assertEquals(0.25, l.getProgress(), 1e-9);
        assertEquals(0, l.clampDt(-1), 0);
        assertEquals(0, l.clampDt(Double.NaN), 0);
    }

    @Test
    public void graceExpiresOnlyViaCappedDt() {
        FlowerMechLogic l = make();
        l.deploy();
        step(l, DT); // settle: first update after a state change ignores stall
        l.update(DT, true, false, true, 0);
        l.clearFault();
        l.update(100, true, false, true, 0); // counts as 0.1 s only
        assertTrue(l.inGrace());
    }

    @Test
    public void staleStallFlagOnFirstUpdateAfterDeployDoesNotFault() {
        FlowerMechLogic l = make();
        l.update(DT, true, true, true, 0);   // stowed with a leftover stall flag
        l.deploy();
        l.update(DT, true, false, true, 0);  // first update after the state change: ignored
        assertEquals(State.DEPLOYING, l.getState());
    }

    @Test
    public void staleStallFlagWhenDwellEndsDoesNotFault() {
        FlowerMechLogic l = make();
        l.deploy();
        run(l, 0.5);
        assertEquals(State.DWELL, l.getState());
        while (l.getState() == State.DWELL) step(l, DT);
        assertEquals(State.STOWING, l.getState());
        l.update(DT, true, false, true, 0);   // stall flag left over from the deployed hold: ignored once
        assertEquals(State.STOWING, l.getState());
        l.update(DT, true, false, true, 0);   // a real, persistent stall still faults
        assertEquals(State.FAULTED, l.getState());
    }

    @Test
    public void trustedButNeverAtTargetCarriesOnAfterArriveTimeout() {
        FlowerMechLogic l = make();
        l.arriveTimeoutMs = 300;
        l.deploy();
        // command arrives after 0.4 s; the servo never reports atTarget but does not stall either
        for (int i = 0; i < 30; i++) l.update(DT, true, false, false, 0);
        assertEquals(State.DEPLOYING, l.getState());
        for (int i = 0; i < 20; i++) l.update(DT, true, false, false, 0);
        assertEquals(State.DWELL, l.getState());
    }

    @Test
    public void arriveTimeoutZeroWaitsForever() {
        FlowerMechLogic l = make();
        l.arriveTimeoutMs = 0;
        l.deploy();
        for (int i = 0; i < 500; i++) l.update(DT, true, false, false, 0);
        assertEquals(State.DEPLOYING, l.getState());
        l.update(DT, true, true, false, 0);
        assertEquals(State.DWELL, l.getState());
    }

    @Test
    public void singleServoPathNaNSyncNeverFaults() {
        // SERVO_COUNT = 1: FlowerMech passes NaN for the sync difference.
        FlowerMechLogic l = make();
        l.deploy();
        for (int i = 0; i < 200; i++) {
            l.update(DT, true, true, false, Double.NaN);
            assertFalse(l.getState() == State.FAULTED);
        }
        assertEquals(State.STOWED, l.getState());
    }

    @Test
    public void untrustedOpenLoopTimingIsSlewPlusDwell() {
        // 160 deg at 320 deg/s = 0.5 s each way, plus 0.5 s dwell; dt = 1/16 s keeps the sums exact
        FlowerMechLogic l = make();
        l.spanDeg = 160;
        l.slewDegPerS = 320;
        double dt = 0.0625;
        l.deploy();
        for (int i = 0; i < 7; i++) l.update(dt, false, false, false, Double.NaN);
        assertEquals(State.DEPLOYING, l.getState());
        l.update(dt, false, false, false, Double.NaN);
        assertEquals(State.DWELL, l.getState());
        for (int i = 0; i < 7; i++) l.update(dt, false, false, false, Double.NaN);
        assertEquals(State.DWELL, l.getState());
        l.update(dt, false, false, false, Double.NaN);
        assertEquals(State.STOWING, l.getState());
        for (int i = 0; i < 8; i++) l.update(dt, false, false, false, Double.NaN);
        assertEquals(State.STOWED, l.getState());
    }

    @Test
    public void persistentStallDuringDeployingStillFaults() {
        FlowerMechLogic l = make();
        l.deploy();
        l.update(DT, true, false, true, 0);
        assertEquals(State.DEPLOYING, l.getState());
        double out = l.update(DT, true, false, true, 0);
        assertEquals(State.FAULTED, l.getState());
        assertTrue(Double.isNaN(out));
    }
}
