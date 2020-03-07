/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveHopperUpCommand;
import frc.robot.commands.RotateToCommand;
import frc.robot.commands.ShooterHoldVelocityCommand;
import frc.robot.commands.StopHopperCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimShootMoveBackAutoCommand extends SequentialCommandGroup {
  private static final double TO_SWITCH_DISTANCE_M = 1.7;

  /**
   * Creates a new AimShootBackAutoCommand.
   */
  public AimShootMoveBackAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem,
                                     Shooter shooter, VerticalHopper verticalHopper, int rpm) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new VisionAlignCommand(driveTrain, visionSystem, false).withTimeout(4.0),
      new ShooterHoldVelocityCommand(shooter, false, rpm).withTimeout(3.0),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, true, rpm),
        new MoveHopperUpCommand(verticalHopper)
      ).withTimeout(6.0),
      new ParallelCommandGroup(
        new StopShooterCommand(shooter),
        new StopHopperCommand(verticalHopper),
        new SequentialCommandGroup(
          new RotateToCommand(driveTrain, 0).withTimeout(2.0),
          new LazyRamseteCommand(driveTrain, () -> {
            Pose2d currentPose = driveTrain.getPose();
            return driveTrain.generateXTrajectory(currentPose, TO_SWITCH_DISTANCE_M);
          })
        )
      )
    );
  }
}
