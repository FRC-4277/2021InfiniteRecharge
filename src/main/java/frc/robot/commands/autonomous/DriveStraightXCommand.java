package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.subsystems.DriveTrain;

public class DriveStraightXCommand extends LazyRamseteCommand {

  /** Drive straight from current pose */
  public DriveStraightXCommand(DriveTrain driveTrain, double xMeters) {
    super(driveTrain, () -> driveTrain.generateXTrajectory(xMeters), true);
  }

  /** Drive straight from current pose, potentially backwards */
  public DriveStraightXCommand(DriveTrain driveTrain, double xMeters, boolean reversed) {
    super(driveTrain, () -> driveTrain.generateXTrajectory(xMeters), reversed);
  }

  /** Drive straight from given pose */
  public DriveStraightXCommand(DriveTrain driveTrain, double xMeters, Pose2d startPose) {
    super(driveTrain, () -> driveTrain.generateXTrajectory(startPose, xMeters, false));
  }

  /** Drive straight from given pose, potentially backwards */
  public DriveStraightXCommand(
      DriveTrain driveTrain, double xMeters, Pose2d startPose, boolean reversed) {
    super(driveTrain, () -> driveTrain.generateXTrajectory(startPose, xMeters, reversed));
  }
}
