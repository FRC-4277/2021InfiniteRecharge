package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveStorePosition extends InstantCommand {
    public DriveStorePosition(DriveTrain driveTrain, int index) {
        super(() -> {
            driveTrain.setStoredPosition(index, driveTrain.getPose());
            System.out.println("Stored position: " + driveTrain.getPose());
        }, driveTrain);
    }
}
