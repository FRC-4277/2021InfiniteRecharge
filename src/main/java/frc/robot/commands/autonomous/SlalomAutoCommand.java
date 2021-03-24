package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class SlalomAutoCommand extends SequentialCommandGroup {
  public SlalomAutoCommand(DriveTrain driveTrain) {
    Trajectory trajectory1 = driveTrain.generateTrajectory("Slalom Path");
    addCommands(
        new ResetOdometryCommand(driveTrain, trajectory1.getInitialPose()),
        driveTrain.generateRamseteCommand(trajectory1));
  }
}
