package frc.robot.commands.autonomous.sixball;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotateToCommand;
import frc.robot.commands.ShootAndHopperCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.autonomous.LazyRamseteCommand;
import frc.robot.commands.autonomous.ResetOdometryCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;

public class SixBallAutoCommand extends SequentialCommandGroup {
  private static final Pose2d START_POSE = new Pose2d(new Translation2d(3.215, 7.457), new Rotation2d());
  private static final double MAX_VELOCITY = 1.5;
  private static final double MAX_ACCEL = 0.5;
  private static final int DESIRED_RPM = 2000;

  public SixBallAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem, Shooter shooter, VerticalHopper hopper, Intake intake) {
    addCommands(
      // Set start pose
      new ResetOdometryCommand(driveTrain, START_POSE),
      // Align & shoot
      new VisionAlignCommand(driveTrain, visionSystem, false, false, false).withTimeout(2.0),
      new ShootAndHopperCommand(shooter, hopper, visionSystem, false, DESIRED_RPM, false).withTimeout(6.0),
      // Rotate to 0 & pickup three
      new RotateToCommand(driveTrain, 0).withTimeout(2.0),
      new PickupThreeCommand(driveTrain, visionSystem, intake).withTimeout(6.0),
      // Drive back to start
      new LazyRamseteCommand(driveTrain, () -> driveTrain.generateTrajectory(driveTrain.getPose(), START_POSE, MAX_VELOCITY, MAX_ACCEL, false)),
      // Align & shoot
      new VisionAlignCommand(driveTrain, visionSystem, false, false, false).withTimeout(2.0),
      new ShootAndHopperCommand(shooter, hopper, visionSystem, false, DESIRED_RPM, false).withTimeout(6.0)
    );
  }
}
