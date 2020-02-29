/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterHoldVelocityCommand extends CommandBase {
  private Shooter shooter;
  private Integer rpm = -1;
  private int loopsReachedRPM = 0;
  private boolean runForever, finished;

  public ShooterHoldVelocityCommand(Shooter shooter, boolean runForever) {
    this(shooter, runForever, -1);
  }

  /**
   * Creates a new RampShooterToRPMCommand.
   */
  public ShooterHoldVelocityCommand(Shooter shooter, boolean runForever, int rpm) {
    this.shooter = shooter;
    this.runForever = runForever;
    this.rpm = rpm;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setReachedRPMDisplay(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO : Change to use vision
    int desiredRPM;
    if (rpm == -1) {
      desiredRPM = shooter.getDriverDesiredRPM();
    } else {
      desiredRPM = rpm;
    }
    shooter.holdVelocityRPM(desiredRPM);
    if (shooter.hasReachedRPM(desiredRPM)) {
      loopsReachedRPM++;
    } else {
      loopsReachedRPM = 0;
    }
    if (loopsReachedRPM >= 5) {
      shooter.setReachedRPMDisplay(true);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.setReachedRPMDisplay(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (runForever) {
      return false;
    } else {
      return finished;
    }
  }
}
