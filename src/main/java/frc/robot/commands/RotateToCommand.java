/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class RotateToCommand extends CommandBase {
  private static final double MIN_POWER = 0.25;
  private static final double DEG_TOLERANCE = 5;
  private static final int CORRECT_LOOPS_NEEDED = 5;
  private DriveTrain driveTrain;
  private double targetHeading;
  private int correctLoops;

  /**
   * Creates a new RotateToCommand.
   */
  public RotateToCommand(DriveTrain driveTrain, double targetHeading) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetHeading = targetHeading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.correctLoops = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = driveTrain.getHeading();
    double error = targetHeading - currentHeading;
    // If error is positive, we must spin counterclockwise
    // If turning power is positive, we turn clockwise, so invert
    double turningPower = -error;
    turningPower *= 0.015; //P loop
    if (Math.abs(turningPower) < MIN_POWER) {
      turningPower = Math.copySign(MIN_POWER, turningPower);
    }

    if (Math.abs(error) <= DEG_TOLERANCE) {
      correctLoops++;
    } else {
      correctLoops = 0;
    }

    driveTrain.rawTankDrive(turningPower, -turningPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return correctLoops >= CORRECT_LOOPS_NEEDED;
  }
}
