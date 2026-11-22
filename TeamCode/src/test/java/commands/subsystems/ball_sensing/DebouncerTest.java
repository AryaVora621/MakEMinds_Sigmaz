package commands.subsystems.ball_sensing;

import org.junit.Assert;
import org.junit.Test;

public class DebouncerTest {
   public DebouncerTest() {
   }

   @Test
   public void glitchShorterThanRiseIgnored() {
      Debouncer d = new Debouncer(50L, 50L);
      Assert.assertFalse(d.update(true, 0L));
      Assert.assertFalse(d.update(true, 30L));
      Assert.assertFalse(d.update(false, 40L));
      Assert.assertFalse(d.update(true, 60L));
      Assert.assertFalse(d.update(true, 100L));
      Assert.assertTrue(d.update(true, 110L));
   }

   @Test
   public void stableInputPasses() {
      Debouncer d = new Debouncer(20L, 20L);
      d.update(true, 0L);
      Assert.assertTrue(d.update(true, 20L));
      Assert.assertTrue(d.update(true, 500L));
   }

   @Test
   public void fallDebounce() {
      Debouncer d = new Debouncer(0L, 100L);
      Assert.assertTrue(d.update(true, 0L));
      Assert.assertTrue(d.update(false, 10L));
      Assert.assertTrue(d.update(false, 100L));
      Assert.assertTrue(d.update(true, 105L));
      Assert.assertTrue(d.update(false, 110L));
      Assert.assertTrue(d.update(false, 200L));
      Assert.assertFalse(d.update(false, 210L));
   }
}
