package commands.autonomous;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import robot.RobotSettings;

public class AutoAimStepTest {

    private static final class FakePort implements AutoAimStep.Port {
        boolean present = true;
        boolean ready = false;
        boolean canFire = false;
        boolean spinning = false;
        boolean intakeOn = false;
        int stopCalls = 0;

        @Override public boolean shooterPresent() { return present; }
        @Override public void setSpinning(boolean on) { spinning = on; }
        @Override public boolean aimReady() { return ready; }
        @Override public void feed() { intakeOn = true; }
        @Override public void stopIntake() { intakeOn = false; stopCalls++; }
        @Override public boolean feedIfReady() { if (canFire) { intakeOn = true; } return canFire; }
    }

    private double now = 0;

    private AutoAimStep step(FakePort p, double timeout, boolean shootAfter) {
        return new AutoAimStep(p, timeout, shootAfter, () -> now);
    }

    @Test
    public void aimsThenFinishesWithoutShootAfterAndLeavesFlywheelSpinning() {
        FakePort p = new FakePort();
        AutoAimStep s = step(p, 2.0, false);
        s.initialize();
        assertTrue(p.spinning);
        s.execute();
        assertFalse(s.isFinished());
        assertFalse(p.intakeOn); // TRANSFER_ON_START default false: no feeding until aimed
        p.ready = true;
        now = 0.5;
        s.execute();
        assertTrue(s.isFinished());
        assertTrue(s.aimed());
        assertFalse(p.intakeOn);
        assertTrue(p.spinning);
    }

    @Test
    public void timeoutFinishesNotAimed() {
        FakePort p = new FakePort();
        AutoAimStep s = step(p, 2.0, false);
        s.initialize();
        s.execute();
        now = 2.0;
        s.execute();
        assertTrue(s.isFinished());
        assertFalse(s.aimed());
        assertTrue(s.timedOut());
        assertTrue(p.spinning);
    }

    @Test
    public void executeIsNonBlockingOneTickAndNoopAfterFinish() {
        FakePort p = new FakePort();
        AutoAimStep s = step(p, 2.0, false);
        s.initialize();
        s.execute(); // returns although never ready
        assertFalse(s.isFinished());
        s.end(false);
        int stops = p.stopCalls;
        s.execute();
        assertEquals(stops, p.stopCalls);
    }

    @Test
    public void cancelStopsIntakeButKeepsFlywheelSpinning() {
        boolean saved = RobotSettings.AutoAim.TRANSFER_ON_START;
        RobotSettings.AutoAim.TRANSFER_ON_START = true;
        try {
            FakePort p = new FakePort();
            AutoAimStep s = step(p, 2.0, true);
            s.initialize();
            s.execute();
            assertTrue(p.intakeOn);
            s.end(true);
            assertFalse(p.intakeOn);
            assertTrue(p.spinning);
            assertTrue(s.isFinished());
        } finally {
            RobotSettings.AutoAim.TRANSFER_ON_START = saved;
        }
    }

    @Test
    public void shootAfterRunsBoundedPhaseThenStopsIntake() {
        FakePort p = new FakePort();
        p.ready = true;
        p.canFire = true;
        AutoAimStep s = step(p, 2.0, true);
        s.initialize();
        s.execute();
        assertFalse(s.isFinished()); // now in shoot phase
        assertTrue(s.aimed());
        now = RobotSettings.AutoAim.SHOOT_AFTER_MAX_S - 0.1;
        s.execute();
        assertFalse(s.isFinished());
        assertTrue(p.intakeOn);
        now = RobotSettings.AutoAim.SHOOT_AFTER_MAX_S + 0.1;
        s.execute();
        assertTrue(s.isFinished());
        assertFalse(p.intakeOn);
    }

    @Test
    public void shootPhaseStopsIntakeWhenNotReadyToFire() {
        FakePort p = new FakePort();
        p.ready = true;
        p.canFire = false;
        AutoAimStep s = step(p, 2.0, true);
        s.initialize();
        s.execute();
        p.intakeOn = true;
        s.execute();
        assertFalse(p.intakeOn);
    }

    @Test
    public void noShooterFinishesImmediatelyWithoutSpinning() {
        FakePort p = new FakePort();
        p.present = false;
        AutoAimStep s = step(p, 2.0, true);
        s.initialize();
        assertFalse(p.spinning);
        s.execute();
        assertTrue(s.isFinished());
        assertFalse(s.aimed());
        assertFalse(p.intakeOn);
    }

    @Test
    public void canBeReinitializedAndRerun() {
        FakePort p = new FakePort();
        AutoAimStep s = step(p, 1.0, false);
        s.initialize();
        s.execute();
        now = 1.5;
        s.execute();
        assertTrue(s.isFinished());
        p.ready = true;
        s.initialize();
        assertFalse(s.isFinished());
        s.execute();
        assertTrue(s.isFinished());
        assertTrue(s.aimed());
    }
}
