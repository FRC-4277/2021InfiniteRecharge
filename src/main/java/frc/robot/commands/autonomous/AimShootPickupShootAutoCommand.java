/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimShootPickupShootAutoCommand extends SequentialCommandGroup {

  // TODO: Have a backup for if the vision system can't find the target

  /**
   * Creates a new AimShootPickupShootAutoCommand.
   */
  public AimShootPickupShootAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem,
                                        Shooter shooter, VerticalHopper verticalHopper, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new VisionAlignCommand(driveTrain, visionSystem, false, false).withTimeout(4.0),
      new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, false)
              .withTimeout(3.0),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, true),
        new MoveHopperUpCommand(verticalHopper)
      ).withTimeout(4.0),
      new RotateToCommand(driveTrain, 0).withTimeout(2.0),
      new ParallelCommandGroup(
        new StopShooterCommand(shooter),
        new AutoHopperMoveInCommand(verticalHopper),
        new ParallelRaceGroup(
          new TrenchRunAutoCommand(driveTrain, visionSystem),
          new IntakeCommand(intake, verticalHopper)
        ).withTimeout(7.0)
      ),
      new ParallelCommandGroup(
        new IdleHopperCommand(verticalHopper),
        // Go back to the start, so we can shoot again
        new LazyRamseteCommand(driveTrain, () -> {
          Pose2d startingPosition = new Pose2d(0, 0, new Rotation2d(0));
          Pose2d current = driveTrain.getPose();
          return driveTrain.generateTrajectory(current, startingPosition, true);
        })
      ),
      new VisionAlignCommand(driveTrain, visionSystem, false, false).withTimeout(4.0),
      new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, false)
              .withTimeout(3.0),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, true),
        new MoveHopperUpCommand(verticalHopper)
      ).withTimeout(4.0),
      new RotateToCommand(driveTrain, 0).withTimeout(5.0)
    );
  }
}
