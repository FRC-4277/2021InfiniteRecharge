package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GalacticSearch;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootAndHopperCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.VisionSystem;

public class GalacticAutoCommand extends SequentialCommandGroup {
    private final RobotContainer container;
    private GalacticPath pathDetected;

    public GalacticAutoCommand(RobotContainer container, DriveTrain driveTrain, VisionSystem visionSystem,
                               VerticalHopper verticalHopper, Intake intake, Shooter shooter) {
        this.container = container;
        addCommands(
                // Figure out path from DS computer vision (see /galactic-search-vision/)
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
                //new ZoomToMiddleCommand(this, driveTrain),

                //new VisionAlignCommand(driveTrain, visionSystem, false, true, false).withTimeout(7.0),

                //new ShootAndHopperCommand(shooter, verticalHopper, visionSystem, true, 2150, false).withTimeout(15.0)
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
