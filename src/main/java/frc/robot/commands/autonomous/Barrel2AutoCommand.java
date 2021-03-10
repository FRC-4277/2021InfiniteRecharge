package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class Barrel2AutoCommand extends SequentialCommandGroup {
    private static final double MAX_VELOCITY = 4; // m/s
    private static final double END_VELOCITY = 3; // m/s
    private static final double MAX_ACCELERATION = 2.5; // m/s^2
    private static final double MAX_CENTRIPETAL_ACCELERATION = 4; // m/s^2

    public Barrel2AutoCommand(DriveTrain driveTrain) {
        var config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
        config.addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION));
        config.setEndVelocity(END_VELOCITY);

        var trajectory = driveTrain.generateCustomTrajectory("Barrel Racing 2", config);
        addCommands(
                //new DriveNeutralModeCommand(driveTrain, NeutralMode.Brake),
                new ResetOdometryCommand(driveTrain, trajectory.getInitialPose()),
                driveTrain.generateRamseteCommand(trajectory)
                //new DriveNeutralModeCommand(driveTrain, NeutralMode.Coast)
        );
    }
}
