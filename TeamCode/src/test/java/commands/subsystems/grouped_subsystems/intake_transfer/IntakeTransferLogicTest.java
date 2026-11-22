package commands.subsystems.grouped_subsystems.intake_transfer;

import commands.subsystems.grouped_subsystems.intake_transfer.IntakeTransfer.Mode;
import org.junit.Assert;
import org.junit.Test;

public class IntakeTransferLogicTest {
   public IntakeTransferLogicTest() {
   }

   @Test
   public void forwardAllowedUnlessFullAndEnabled() {
      Assert.assertTrue(IntakeTransfer.shouldRunForward(false, false, true));
      Assert.assertTrue(IntakeTransfer.shouldRunForward(false, true, true));
      Assert.assertTrue(IntakeTransfer.shouldRunForward(true, false, true));
      Assert.assertFalse(IntakeTransfer.shouldRunForward(true, true, true));
   }

   @Test
   public void fullStopNeverEngagesWithoutShooterSensor() {
      Assert.assertTrue(IntakeTransfer.shouldRunForward(true, true, false));
   }

   @Test
   public void fullStopOnlyGatesForward() {
      for(IntakeTransfer.Mode m : Mode.values()) {
         boolean may = IntakeTransfer.modeMayRun(m, true, true, true);
         Assert.assertTrue(m.name(), m == Mode.FORWARD ? !may : may);
      }

   }

   @Test
   public void reverseAndFeedNeverBlockedWhenPinnedFull() {
      Assert.assertTrue(IntakeTransfer.modeMayRun(Mode.BACKWARD, true, true, true));
      Assert.assertTrue(IntakeTransfer.modeMayRun(Mode.FEED, true, true, true));
      Assert.assertTrue(IntakeTransfer.modeMayRun(Mode.STOPPED, true, true, true));
   }

   @Test
   public void reversingOnlyInBackward() {
      Assert.assertTrue(IntakeTransfer.isReversingMode(Mode.BACKWARD));
      Assert.assertFalse(IntakeTransfer.isReversingMode(Mode.FORWARD));
      Assert.assertFalse(IntakeTransfer.isReversingMode(Mode.FEED));
      Assert.assertFalse(IntakeTransfer.isReversingMode(Mode.STOPPED));
   }

   @Test
   public void feedConfirmLatchesOnlyWhileFeeding() {
      Assert.assertFalse(IntakeTransfer.feedConfirmed(false, false, true));
      Assert.assertFalse(IntakeTransfer.feedConfirmed(false, true, false));
      Assert.assertTrue(IntakeTransfer.feedConfirmed(false, true, true));
      Assert.assertTrue(IntakeTransfer.feedConfirmed(true, false, false));
   }
}
