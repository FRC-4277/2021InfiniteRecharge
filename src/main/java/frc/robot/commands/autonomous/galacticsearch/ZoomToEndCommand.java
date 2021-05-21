package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.GalacticSearch;
import frc.robot.subsystems.DriveTrain;

/**
 * Original version of zoom to end, utilizing trajectory
 *
 * @author Andrew Tran
 */
public class ZoomToEndCommand extends CommandBase {
  private GalacticAutoCommand galacticAutoCommand;
  private DriveTrain driveTrain;
  private RamseteCommand ramseteCommand;

  public ZoomToEndCommand(GalacticAutoCommand galacticAutoCommand, DriveTrain driveTrain) {
    this.galacticAutoCommand = galacticAutoCommand;
    this.driveTrain = driveTrain;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = driveTrain.getPose();
    // Target is the same y-level, but at the end zone facing 0 degrees
    Pose2d target = new Pose2d(GalacticSearch.ROBOT_END_X, currentPose.getY(), new Rotation2d());

    galacticAutoCommand.setMessage("[Zoom to End] Current pos: " + currentPose);
    galacticAutoCommand.setMessage("[Zoom to End] Target pos: " + target);
    // Add middle waypoint so it tries to take quickest path
    double startVelocity =
        driveTrain.convertPercentToVelocity(GalacticSearch.DRIVE_TO_BALL_FOR_INTAKE_SPEED);
    Trajectory trajectory =
        driveTrain.generateTrajectory(currentPose, target, 5.0, 5, true, false, startVelocity, 5);
    ramseteCommand = driveTrain.generateRamseteCommand(trajectory);
    ramseteCommand.initialize();
  }

  @Override
  public void execute() {
    // galacticAutoCommand.setMessage("[Zoom to End] Moving...");
    ramseteCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    galacticAutoCommand.setMessage("[Zoom to End] End");
    driveTrain.stopDrive();
    if (ramseteCommand != null) {
      ramseteCommand.end(interrupted);
      ramseteCommand = null;
    }
  }

  @Override
  public boolean isFinished() {
    return ramseteCommand != null && ramseteCommand.isFinished();
  }
}
