package frc.robot.commands.autonomous.galacticvideo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.GalacticSearchVideo;
import frc.robot.subsystems.DriveTrain;

public class FirstForwardMoveCommand extends CommandBase {
  private final GalacticAutoVideoCommand galacticAutoVideoCommand;
  private final DriveTrain driveTrain;
  private RamseteCommand ramseteCommand;

  public FirstForwardMoveCommand(
      GalacticAutoVideoCommand galacticAutoVideoCommand, DriveTrain driveTrain) {
    this.galacticAutoVideoCommand = galacticAutoVideoCommand;
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    GalacticPath path = galacticAutoVideoCommand.getPathDetected();
    double distanceForward =
        path.isPowerCellClose()
            ? GalacticSearchVideo.CLOSE_BALL_DISTANCE
            : GalacticSearchVideo.FAR_BALL_DISTANCE;
    distanceForward -= GalacticSearchVideo.DESIRED_DISTANCE_TO_BALL;
    galacticAutoVideoCommand.setMessage(
        "[First Forward] Distance forward calculated to be " + distanceForward);
    ramseteCommand =
        driveTrain.generateRamseteCommand(driveTrain.generateXTrajectory(distanceForward), false);
    ramseteCommand.initialize();
  }

  @Override
  public void execute() {
    // galacticAutoCommand.setMessage("[First Forward] Moving...");
    ramseteCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    galacticAutoVideoCommand.setMessage("[First Forward] End");
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
