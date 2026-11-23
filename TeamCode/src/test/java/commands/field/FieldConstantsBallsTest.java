package commands.field;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Ball-capacity constants. HIVE_TIP_POLLEN is [USER-CONFIRMED] (the manual does not state it).
 * PRELOAD_POLLEN is [OFFICIAL G304].
 */
public class FieldConstantsBallsTest {
    @Test
    public void hiveTipsAtEightPollen() {
        assertEquals(8, FieldConstants.HIVE_TIP_POLLEN);
    }

    @Test
    public void preloadIsFourPollen() {
        assertEquals(4, FieldConstants.PRELOAD_POLLEN);
    }
}
