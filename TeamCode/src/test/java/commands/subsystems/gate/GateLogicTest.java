package commands.subsystems.gate;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class GateLogicTest {
    @Test
    public void clampsLargeAndBadDt() {
        assertEquals(0.1, GateLogic.clampDt(5.0, 0.1), 0);
        assertEquals(0.02, GateLogic.clampDt(0.02, 0.1), 0);
        assertEquals(0, GateLogic.clampDt(-1, 0.1), 0);
        assertEquals(0, GateLogic.clampDt(Double.NaN, 0.1), 0);
    }
}
