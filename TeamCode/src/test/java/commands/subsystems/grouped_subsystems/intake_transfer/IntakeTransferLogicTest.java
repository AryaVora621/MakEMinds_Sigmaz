package commands.subsystems.grouped_subsystems.intake_transfer;

import commands.subsystems.grouped_subsystems.intake_transfer.IntakeTransfer.Mode;
import org.junit.Assert;
import org.junit.Test;

public class IntakeTransferLogicTest {
   @Test
   public void reversingOnlyInBackward() {
      Assert.assertTrue(IntakeTransfer.isReversingMode(Mode.BACKWARD));
      Assert.assertFalse(IntakeTransfer.isReversingMode(Mode.FORWARD));
      Assert.assertFalse(IntakeTransfer.isReversingMode(Mode.FEED));
      Assert.assertFalse(IntakeTransfer.isReversingMode(Mode.STOPPED));
   }
}
