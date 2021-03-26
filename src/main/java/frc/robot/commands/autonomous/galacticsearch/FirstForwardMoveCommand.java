package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.GalacticSearch;
import frc.robot.subsystems.DriveTrain;

public class FirstForwardMoveCommand extends CommandBase {
  private final GalacticAutoCommand galacticAutoCommand;
  private final DriveTrain driveTrain;
  private RamseteCommand ramseteCommand;

  public FirstForwardMoveCommand(GalacticAutoCommand galacticAutoCommand, DriveTrain driveTrain) {
    this.galacticAutoCommand = galacticAutoCommand;
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    GalacticPath path = galacticAutoCommand.getPathDetected();
    double distanceForward =
        path.isPowerCellClose()
            ? GalacticSearch.CLOSE_BALL_DISTANCE
            : GalacticSearch.FAR_BALL_DISTANCE;
    distanceForward -= GalacticSearch.DESIRED_DISTANCE_TO_BALL;
    galacticAutoCommand.setMessage(
        "[First Forward] Distance forward calculated to be " + distanceForward);
    ramseteCommand =
        driveTrain.generateRamseteCommand(
            driveTrain.generateXTrajectory(
                distanceForward,
                GalacticSearch.FIRST_FORWARD_MAX_VEL,
                GalacticSearch.FIRST_FORWARD_MAX_ACCEL),
            false);
    ramseteCommand.initialize();
  }

  @Override
  public void execute() {
    // galacticAutoCommand.setMessage("[First Forward] Moving...");
    ramseteCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    galacticAutoCommand.setMessage("[First Forward] End");
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
