package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class BarrelAutoCommand extends SequentialCommandGroup {
  public BarrelAutoCommand(DriveTrain driveTrain) {
    Trajectory trajectory1 = driveTrain.generateTrajectory("Barrel Racing");
    addCommands(
        new ResetOdometryCommand(driveTrain, trajectory1.getInitialPose()),
        driveTrain.generateRamseteCommand(trajectory1));
  }
}
