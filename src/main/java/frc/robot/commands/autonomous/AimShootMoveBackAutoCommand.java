/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimShootMoveBackAutoCommand extends SequentialCommandGroup {
  private static final double TO_SWITCH_DISTANCE_M = 1.7;

  // TODO: Have a backup for if the vision system can't find the target

  /**
   * Creates a new AimShootBackAutoCommand.
   */
  public AimShootMoveBackAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem,
                                     Shooter shooter, VerticalHopper verticalHopper) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new VisionAlignCommand(driveTrain, visionSystem, false).withTimeout(4.0),
      new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, false)
              .withTimeout(3.0),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, true),
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
