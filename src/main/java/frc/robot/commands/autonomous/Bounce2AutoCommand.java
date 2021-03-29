package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStopCommand;
import frc.robot.subsystems.DriveTrain;

public class Bounce2AutoCommand extends SequentialCommandGroup {
  private static final double MAX_VELOCITY = 3; // m/s
  private static final double MAX_ACCELERATION = 1.5; // m/s^2
  private static final double MAX_CENTRIPETAL_ACCELERATION = 2; // m/s^2

  private static final double MAX_VELOCITY_2 = 3; // m/s
  private static final double MAX_ACCELERATION_2 = 1.5; // m/s^2
  private static final double MAX_CENTRIPETAL_ACCELERATION_2 = 2; // m/s^2

  private static final double MAX_VELOCITY_3 = 3; // m/s
  private static final double MAX_ACCELERATION_3 = 1.5; // m/s^2
  private static final double MAX_CENTRIPETAL_ACCELERATION_3 = 2; // m/s^2

  private static final double MAX_VELOCITY_4 = 4; // m/s
  private static final double END_VELOCITY_4 = 1.5; // m/s
  private static final double MAX_ACCELERATION_4 = 3; // m/s^2
  // private static final double MAX_CENTRIPETAL_ACCELERATION_4 = 4; // m/s^2

  public Bounce2AutoCommand(DriveTrain driveTrain) {
    var config1 = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
    config1.addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION));
    var trajectory1 = driveTrain.generateCustomTrajectory("Bounce 1 2", config1);

    var config2 = new TrajectoryConfig(MAX_VELOCITY_2, MAX_ACCELERATION_2);
    config2.addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION_2));
    config2.setReversed(true);
    config2.setEndVelocity(0.5);
    var trajectory2 = driveTrain.generateCustomTrajectory("Bounce 2 2", config2);

    var config3 = new TrajectoryConfig(MAX_VELOCITY_3, MAX_ACCELERATION_3);
    config3.addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION_3));
    config3.setEndVelocity(0.5);
    var trajectory3 = driveTrain.generateCustomTrajectory("Bounce 3 2", config3);

    var config4 = new TrajectoryConfig(MAX_VELOCITY_4, MAX_ACCELERATION_4);
    config4.setEndVelocity(END_VELOCITY_4);
    // config4.addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION_4));
    config4.setReversed(true);
    var trajectory4 = driveTrain.generateCustomTrajectory("Bounce 4 2", config4);

    addCommands(
        new ResetOdometryCommand(driveTrain, trajectory1.getInitialPose()),
        driveTrain.generateRamseteCommand(trajectory1),
        driveTrain.generateRamseteCommand(trajectory2),
        driveTrain.generateRamseteCommand(trajectory3),
        driveTrain.generateRamseteCommand(trajectory4),
        new DriveStopCommand(driveTrain));
  }
}
