package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class ResetOdometryCommand extends InstantCommand {
  public ResetOdometryCommand(DriveTrain driveTrain, Pose2d startPose) {
    super(() -> driveTrain.resetOdometry(startPose), driveTrain);
  }
}
