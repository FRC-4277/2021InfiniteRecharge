package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.ResetOdometryCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.VisionSystem;
import frc.robot.Constants.GalacticSearch;

public class GalacticAutoCommand extends SequentialCommandGroup {
    private RobotContainer container;
    private GalacticPath pathDetected;

    public GalacticAutoCommand(RobotContainer container, DriveTrain driveTrain, VisionSystem visionSystem,
                               VerticalHopper verticalHopper) {
        this.container = container;
        addCommands(
                // Figure out path from Pixy2
                new DetectPathCommand(this, visionSystem),
                // Reset Odometry to starting position of path
                new GalacticResetOdometryCommand(driveTrain, this::getStartPose),

                // Move straight forward to JUST BEFORE first ball #1
                new FirstForwardMoveCommand(this, driveTrain),
                // Pickup ball #1
                new IntakeGalacticBallCommand(driveTrain, visionSystem, verticalHopper)

                // Drive to JUST BEFORE ball #2
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
