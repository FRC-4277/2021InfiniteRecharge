package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class BounceAutoCommand extends SequentialCommandGroup {
    public BounceAutoCommand(DriveTrain driveTrain) {
        Trajectory trajectory1 = driveTrain.generateTrajectory("Bounce 1");
        addCommands(
                new ResetOdometryCommand(driveTrain, trajectory1.getInitialPose()),
                driveTrain.generateRamseteCommand(trajectory1),
                driveTrain.generateRamseteCommandFromFile("Bounce 2"),
                driveTrain.generateRamseteCommandFromFile("Bounce 3"),
                driveTrain.generateRamseteCommandFromFile("Bounce 4")
        );
    }
}
