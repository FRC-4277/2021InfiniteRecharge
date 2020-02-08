/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

public class LazyRamseteCommand extends CommandBase {
  private DriveTrain driveTrain;
  private Supplier<Trajectory> trajectorySupplier;
  private boolean executed = false;

  public LazyRamseteCommand(DriveTrain driveTrain, Supplier<Trajectory> trajectorySupplier) {
      this.driveTrain = driveTrain;
      this.trajectorySupplier = trajectorySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!executed) {
      Trajectory trajectory = trajectorySupplier.get();
      Command ramsete = driveTrain.getRamsete(trajectory);
      CommandScheduler.getInstance().schedule(ramsete);
      executed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return executed;
  }
}
