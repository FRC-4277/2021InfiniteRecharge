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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoHopperMoveInCommand;
import frc.robot.commands.IdleHopperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveHopperUpCommand;
import frc.robot.commands.RotateToCommand;
import frc.robot.commands.ShooterHoldVelocityCommand;
import frc.robot.commands.StopHopperCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.VisionSystem;

import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimShootPickupShootAutoCommand extends SequentialCommandGroup {
  /**
   * Creates a new AimShootPickupShootAutoCommand.
   */
  public AimShootPickupShootAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem,
  Shooter shooter, VerticalHopper verticalHopper, Intake intake, int rpm) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new VisionAlignCommand(driveTrain, visionSystem, false).withTimeout(4.0),
      new ShooterHoldVelocityCommand(shooter, false, rpm).withTimeout(3.0),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, true, rpm),
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
      new VisionAlignCommand(driveTrain, visionSystem, false).withTimeout(4.0),
      new ShooterHoldVelocityCommand(shooter, false, rpm).withTimeout(3.0),
      new ParallelCommandGroup(
        new ShooterHoldVelocityCommand(shooter, true, rpm),
        new MoveHopperUpCommand(verticalHopper)
      ).withTimeout(4.0),
      new RotateToCommand(driveTrain, 0).withTimeout(5.0)
    );
  }
}
