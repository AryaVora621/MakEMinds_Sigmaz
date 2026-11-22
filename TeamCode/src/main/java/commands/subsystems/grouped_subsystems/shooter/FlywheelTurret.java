package commands.subsystems.grouped_subsystems.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** Flywheel and turret: fixed launch angle, the turret aims while the chassis drives freely. */
public class FlywheelTurret extends ShooterBase {
    public FlywheelTurret(HardwareMap hardwareMap) {
        super(hardwareMap, true);
    }

    @Override
    public String configurationName() {
        return "Flywheel + turret";
    }
}
