package commands.teleop.demo;

import static org.junit.Assert.*;

import org.junit.Test;

public class RemoteCommandTest {
    @Test public void parsesMove() {
        RemoteCommand c = RemoteCommand.parse("7 0.1 -0.05 0 800\n");
        assertEquals(7, c.seq);
        assertEquals(0.1, c.forward, 1e-9);
        assertEquals(-0.05, c.strafe, 1e-9);
        assertEquals(800, c.ms);
        assertFalse(c.stop);
    }

    @Test public void clampsPowerAndTime() {
        RemoteCommand c = RemoteCommand.parse("1 1.0 -5 NaN 99999");
        assertEquals(RemoteCommand.POWER_CAP, c.forward, 1e-9);
        assertEquals(-RemoteCommand.POWER_CAP, c.strafe, 1e-9);
        assertEquals(0, c.turn, 1e-9);
        assertEquals(RemoteCommand.MAX_MS, c.ms);
    }

    @Test public void stop() {
        assertTrue(RemoteCommand.parse("3 stop").stop);
    }

    @Test public void rejectsJunk() {
        assertNull(RemoteCommand.parse(null));
        assertNull(RemoteCommand.parse(""));
        assertNull(RemoteCommand.parse("x 0 0 0 100"));
        assertNull(RemoteCommand.parse("1 0.2 0 0"));
        assertNull(RemoteCommand.parse("1 0.2 0 0 0"));
        assertNull(RemoteCommand.parse("1 0.2 0 0 -5"));
    }
}
