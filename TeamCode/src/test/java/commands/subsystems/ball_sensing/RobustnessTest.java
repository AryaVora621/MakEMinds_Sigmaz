package commands.subsystems.ball_sensing;

import org.junit.Assert;
import org.junit.Test;

public class RobustnessTest {
   public RobustnessTest() {
   }

   @Test
   public void debouncerNegativeTimestamps() {
      Debouncer d = new Debouncer(50L, 50L);
      Assert.assertFalse(d.update(true, -1000L));
      Assert.assertFalse(d.update(true, -970L));
      Assert.assertTrue(d.update(true, -950L));
      Assert.assertTrue(d.update(false, -900L));
      Assert.assertFalse(d.update(false, -850L));
   }

   @Test
   public void debouncerTimeBackwardsRestartsTimer() {
      Debouncer d = new Debouncer(50L, 50L);
      d.update(true, 1000L);
      Assert.assertFalse(d.update(true, 500L));
      Assert.assertFalse(d.update(true, 540L));
      Assert.assertTrue(d.update(true, 550L));
   }

   @Test
   public void invalidReadHeldThenDropped() {
      HysteresisThreshold h = HysteresisThreshold.lowerIsPresent((double)4.0F, (double)6.0F);
      Assert.assertTrue(h.updateOrHold((double)3.0F, true, 3));
      Assert.assertTrue(h.updateOrHold(Double.NaN, false, 3));
      Assert.assertTrue(h.updateOrHold((double)0.0F, false, 3));
      Assert.assertFalse(h.updateOrHold(Double.POSITIVE_INFINITY, false, 3));
      Assert.assertFalse(h.updateOrHold((double)5.0F, true, 3));
   }

   @Test
   public void validReadResetsInvalidCounter() {
      HysteresisThreshold h = HysteresisThreshold.lowerIsPresent((double)4.0F, (double)6.0F);
      h.updateOrHold((double)3.0F, true, 3);
      h.updateOrHold(Double.NaN, false, 3);
      h.updateOrHold(Double.NaN, false, 3);
      Assert.assertTrue(h.updateOrHold((double)3.0F, true, 3));
      Assert.assertTrue(h.updateOrHold(Double.NaN, false, 3));
      Assert.assertTrue(h.updateOrHold(Double.NaN, false, 3));
   }

   @Test
   public void largeFiniteDistanceIsLegitimateAbsent() {
      HysteresisThreshold h = HysteresisThreshold.lowerIsPresent((double)4.0F, (double)6.0F);
      h.updateOrHold((double)3.0F, true, 3);
      Assert.assertFalse(h.updateOrHold((double)200.0F, true, 3));
   }

   @Test
   public void invertedThresholdsDoNotChatter() {
      HysteresisThreshold h = HysteresisThreshold.lowerIsPresent((double)6.0F, (double)4.0F);
      Assert.assertTrue(h.update((double)3.0F));
      Assert.assertTrue(h.update((double)5.0F));
      Assert.assertFalse(h.update((double)6.0F));
      Assert.assertFalse(h.update((double)5.0F));
      HysteresisThreshold g = HysteresisThreshold.higherIsPresent(0.08, 0.15);
      Assert.assertTrue(g.update(0.2));
      Assert.assertTrue(g.update(0.1));
      Assert.assertFalse(g.update(0.08));
   }

   @Test
   public void equalThresholdsDoNotChatter() {
      HysteresisThreshold h = HysteresisThreshold.lowerIsPresent((double)5.0F, (double)5.0F);
      Assert.assertTrue(h.update((double)5.0F));
      Assert.assertTrue(h.update((double)5.0F));
      Assert.assertTrue(h.update((double)5.0F));
      Assert.assertFalse(h.update(5.1));
   }

   @Test
   public void edgeDetectorLatchesOnce() {
      EdgeDetector e = new EdgeDetector();
      e.update(false);
      e.update(true);
      e.update(true);
      Assert.assertTrue(e.consumeRising());
      Assert.assertFalse(e.consumeRising());
      e.update(false);
      Assert.assertTrue(e.consumeFalling());
      Assert.assertFalse(e.consumeFalling());
   }

   @Test
   public void mergedEventsGiveTwoConsumes() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.setCount(3);
      l.update(false, true);
      l.update(false, false);
      l.update(false, true);
      l.update(false, false);
      Assert.assertTrue(l.consumeShotEvent());
      Assert.assertTrue(l.consumeShotEvent());
      Assert.assertFalse(l.consumeShotEvent());
      Assert.assertEquals(1L, (long)l.getCount());
      l.update(true, false);
      l.update(false, false);
      l.update(true, false);
      l.update(false, false);
      Assert.assertTrue(l.consumeIntakeEvent());
      Assert.assertTrue(l.consumeIntakeEvent());
      Assert.assertFalse(l.consumeIntakeEvent());
   }

   @Test
   public void eventsFireWhenCountClamped() {
      BallTrackerLogic l = new BallTrackerLogic(1);
      l.setCount(1);
      l.update(true, false);
      l.update(false, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertTrue(l.consumeIntakeEvent());
      BallTrackerLogic z = new BallTrackerLogic(1);
      z.update(false, true);
      z.update(false, false);
      Assert.assertEquals(0L, (long)z.getCount());
      Assert.assertTrue(z.consumeShotEvent());
   }

   @Test
   public void eventCountersCapped() {
      BallTrackerLogic l = new BallTrackerLogic(1);

      for(int i = 0; i < 10; ++i) {
         l.update(false, true);
         l.update(false, false);
      }

      int n;
      for(n = 0; l.consumeShotEvent(); ++n) {
      }

      Assert.assertEquals(3L, (long)n);
   }

   @Test
   public void clearEventsDropsPending() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(true, false);
      l.update(false, true);
      l.update(false, false);
      l.clearEvents();
      Assert.assertFalse(l.consumeIntakeEvent());
      Assert.assertFalse(l.consumeShotEvent());
   }

   @Test
   public void maxChangeReclampsInLogic() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.setCount(4);
      l.setMax(1);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertTrue(l.isFull());
   }
}
