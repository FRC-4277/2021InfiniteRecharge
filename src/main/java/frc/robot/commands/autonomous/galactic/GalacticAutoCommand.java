package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GalacticSearch;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.VisionSystem;

public class GalacticAutoCommand extends SequentialCommandGroup {
    private final RobotContainer container;
    private GalacticPath pathDetected;

    public GalacticAutoCommand(RobotContainer container, DriveTrain driveTrain, VisionSystem visionSystem,
                               VerticalHopper verticalHopper, Intake intake) {
        this.container = container;
        addCommands(
                // Figure out path from Pixy2
                new DetectPathCommand(this, visionSystem),
                // Reset Odometry to starting position of path
                new GalacticResetOdometryCommand(this, driveTrain, this::getStartPose),

                // Move straight forward to JUST BEFORE first ball #1
                new FirstForwardMoveCommand(this, driveTrain),
                // Pickup ball #1
                new IntakeGalacticBallCommand(this, driveTrain, visionSystem, verticalHopper, intake),

                // Drive to JUST BEFORE ball #2
                new DriveToNextBallCommand(this, driveTrain, 1),
                // Pickup ball #2
                new IntakeGalacticBallCommand(this, driveTrain, visionSystem, verticalHopper, intake),

                // Drive to JUST BEFORE ball #3
                new DriveToNextBallCommand(this, driveTrain, 2),
                // Pickup ball #3
                new IntakeGalacticBallCommand(this, driveTrain, visionSystem, verticalHopper, intake),

                // Go to end zone ASAP
                new ZoomToEndCommand(this, driveTrain)
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
}
