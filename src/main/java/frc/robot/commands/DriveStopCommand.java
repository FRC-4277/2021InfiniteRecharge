package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveStopCommand extends InstantCommand {
  public DriveStopCommand(DriveTrain driveTrain) {
    super(driveTrain::stopDrive, driveTrain);
  }
}
