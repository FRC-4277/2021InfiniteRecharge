// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sixball;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootAndHopperCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.ShooterHoldVelocityCommand.RPMSource;
import frc.robot.commands.autonomous.LazyRamseteCommand;
import frc.robot.commands.autonomous.ResetOdometryCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootMoveBackAutoCommand extends SequentialCommandGroup {
  /** Creates a new AlignShootMoveBackCommand. */
  public ShootMoveBackAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem, Shooter shooter, VerticalHopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometryCommand(driveTrain, new Pose2d()),
      //new VisionAlignCommand(driveTrain, visionSystem, false, seekRight, false, true).withTimeout(3.0),
      new ShootAndHopperCommand(shooter, hopper, visionSystem, RPMSource.VISION, false).withTimeout(11.0),
      new LazyRamseteCommand(driveTrain, new Supplier<Trajectory>() {
          @Override
          public Trajectory get() {
            Pose2d current = driveTrain.getPose();
            Pose2d end = current.plus(new Transform2d(new Translation2d(1.5, 0), new Rotation2d()));
            return driveTrain.generateTrajectory(current, end, 3, 2, false);
          }
      })
    );
  }
}
