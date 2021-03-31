package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GalacticSearch;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;

import java.util.function.Supplier;

public class GalacticAutoCommand extends SequentialCommandGroup {
  private final RobotContainer container;
  private GalacticPath pathDetected;
  private int ballsCollected = 0;

  public GalacticAutoCommand(
      RobotContainer container,
      DriveTrain driveTrain,
      VisionSystem visionSystem,
      VerticalHopper verticalHopper,
      Intake intake,
      Shooter shooter) {
    this.container = container;
    Incrementer incrementer = (max) -> {
      ballsCollected++;
      if (ballsCollected > max) {
        ballsCollected = max;
      }
      SmartDashboard.putNumber("Balls Collected", ballsCollected);
    };
    addCommands(
        // Figure out path from DS computer vision (see /galactic-search-vision/)
        new DetectPathCommand(this, visionSystem),
        // Reset Odometry to starting position of path
        new GalacticResetOdometryCommand(this, driveTrain, this::getStartPose),

        // Move straight forward to JUST BEFORE first ball #1
        new ParallelDeadlineGroup(
          new FirstForwardMoveCommand(this, driveTrain),
          new IntakeQuickCommand(intake, verticalHopper, incrementer, 1)
        ),
        // Pickup ball #1
        new ParallelDeadlineGroup(
          new WaitForBallCountCommand(() -> ballsCollected, 1),
          new IntakeGalacticBallCommand(this, driveTrain, visionSystem, verticalHopper, intake, incrementer, 1)
        ),
        new WaitCommand(0.2),
        // Drive to JUST BEFORE ball #2
        new ParallelDeadlineGroup(
          new DriveToNextBallCommand(this, driveTrain, 1),
          new IntakeQuickCommand(intake, verticalHopper, incrementer, 2)
          ),
        // Pickup ball #2
        new ParallelDeadlineGroup(
          new WaitForBallCountCommand(() -> ballsCollected, 2),
          new IntakeGalacticBallCommand(this, driveTrain, visionSystem, verticalHopper, intake, incrementer, 2)
        ),
        new WaitCommand(0.2),
        // Drive to JUST BEFORE ball #3
        new ParallelDeadlineGroup(
          new DriveToNextBallCommand(this, driveTrain, 2),
          new IntakeQuickCommand(intake, verticalHopper, incrementer, 3)
        ),
        // Pickup ball #3
        new IntakeGalacticBallCommand(this, driveTrain, visionSystem, verticalHopper, intake, incrementer, 3),
        new WaitCommand(0.2),
        // Go to end zone ASAP
        new ZoomToEndCommand(this, driveTrain)
        // new ZoomToMiddleCommand(this, driveTrain),

        // new VisionAlignCommand(driveTrain, visionSystem, false, true, false).withTimeout(7.0),

        // new ShootAndHopperCommand(shooter, verticalHopper, visionSystem, true, 2150,
        // false).withTimeout(15.0)
        );
  }

  public void setMessage(String message) {
    container.setAutonomousMessage(message);
  }

  public GalacticPath getPathDetected() {
    return pathDetected;
  }

  public Pose2d getStartPose() {
    Translation2d firstBallPosition = pathDetected.getFirstPowerCell();
    return new Pose2d(GalacticSearch.ROBOT_START_X, firstBallPosition.getY(), new Rotation2d());
  }

  public void setPathDetected(GalacticPath pathDetected) {
    this.pathDetected = pathDetected;
  }

  public interface Incrementer {
    void increment(int ballMax);
  }
}
