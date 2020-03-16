/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimShootPickupShootAutoCommand extends SequentialCommandGroup {

  // TODO: Have a backup for if the vision system can't find the target

  /**
   * Creates a new AimShootPickupShootAutoCommand.
   */
  public AimShootPickupShootAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem,
                                        Shooter shooter, VerticalHopper verticalHopper, Intake intake,
                                        Pose2d returnPosition) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new VisionAlignCommand(driveTrain, visionSystem, false, false).withTimeout(4.0),
      new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, false)
              .withTimeout(3.0),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, true),
        new MoveHopperUpCommand(verticalHopper, 0.2)
      ).withTimeout(4.0),
      new RotateToCommand(driveTrain, 0).withTimeout(3.0),
      new ParallelRaceGroup(
        new IdleShooterCommand(shooter),
        new AutoHopperMoveInCommand(verticalHopper),
        new ParallelRaceGroup(
          new TrenchRunAutoCommand(driveTrain, visionSystem),
          new IntakeCommand(intake, verticalHopper)
        )
      ).withTimeout(5.0),
      new PrintCommand("RETURNING SOON"),
      new ParallelRaceGroup(
        new IdleHopperCommand(verticalHopper),
        // Go back to the start, so we can shoot again
        new LazyRamseteCommand(driveTrain, () -> {
          Pose2d current = driveTrain.getPose();
          System.out.println("RETURNING PATH GEN from " + current + " to " + returnPosition);
          return driveTrain.generateTrajectory(current, returnPosition, true, true);
        }, true)
      ).withTimeout(4.0),
      new PrintCommand("VISION AT START AGAIN"),
      new VisionAlignCommand(driveTrain, visionSystem, false, false).withTimeout(4.0),
      new PrintCommand("VISION DONE"),
      new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, false)
              .withTimeout(3.0),
      new PrintCommand("VELOCITY DONE"),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, true),
        new MoveHopperUpCommand(verticalHopper, 0.2)
      ).withTimeout(4.0),
      new PrintCommand("SHOOTING DONE"),
      new RotateToCommand(driveTrain, 0).withTimeout(5.0),
      new PrintCommand("ROTATE DONE")
    );
  }
}
