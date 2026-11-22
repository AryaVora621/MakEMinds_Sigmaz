package commands.subsystems.ball_sensing;

import org.junit.Assert;
import org.junit.Test;

public class HysteresisThresholdTest {
   public HysteresisThresholdTest() {
   }

   @Test
   public void lowerIsPresentNoChatter() {
      HysteresisThreshold h = HysteresisThreshold.lowerIsPresent((double)4.0F, (double)6.0F);
      Assert.assertFalse(h.update((double)5.0F));
      Assert.assertTrue(h.update(3.9));
      Assert.assertTrue(h.update((double)5.0F));
      Assert.assertTrue(h.update(5.9));
      Assert.assertFalse(h.update((double)6.0F));
      Assert.assertFalse(h.update((double)5.0F));
   }

   @Test
   public void higherIsPresentNoChatter() {
      HysteresisThreshold h = HysteresisThreshold.higherIsPresent(0.15, 0.08);
      Assert.assertFalse(h.update(0.1));
      Assert.assertTrue(h.update(0.2));
      Assert.assertTrue(h.update(0.1));
      Assert.assertFalse(h.update(0.08));
      Assert.assertFalse(h.update(0.1));
   }

   @Test
   public void nanIsNotPresent() {
      HysteresisThreshold h = HysteresisThreshold.lowerIsPresent((double)4.0F, (double)6.0F);
      Assert.assertTrue(h.update((double)1.0F));
      Assert.assertFalse(h.update(Double.NaN));
      Assert.assertFalse(h.update((double)5.0F));
      HysteresisThreshold g = HysteresisThreshold.higherIsPresent(0.15, 0.08);
      Assert.assertFalse(g.update(Double.NaN));
   }
}
