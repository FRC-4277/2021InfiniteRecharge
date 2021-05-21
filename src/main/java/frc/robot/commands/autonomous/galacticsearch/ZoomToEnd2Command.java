package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ZoomToEnd2Command extends CommandBase {
  private static final double DRIVE_SPEED = 1.0;
  private static final double TURN_P =
      0.01; // multiplied by degrees to get turn adjustment in [-1..1]
  private static final double DEGREE_TOLERANCE = 10; // degrees to stop correcting
  private final DriveTrain driveTrain;

  public ZoomToEnd2Command(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double leftSpeed = DRIVE_SPEED;
    double rightSpeed = DRIVE_SPEED;
    double headingError =
        driveTrain
            .getHeading(); // heading is counterclockwise positive, so positive error = positive
    // TURN

    boolean withinTolerance = Math.abs(headingError) <= DEGREE_TOLERANCE;
    if (!withinTolerance) {
      double adjustment = headingError * TURN_P;
      leftSpeed += adjustment;
      rightSpeed -= adjustment;
    }
    driveTrain.rawTankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public boolean isFinished() {
    return driveTrain.getPose().getX() >= Constants.GalacticSearch.ROBOT_END_X;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }
}
