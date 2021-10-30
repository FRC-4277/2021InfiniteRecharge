// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;

public class ReverseWinchCommand extends CommandBase {
  private final Winch winch;

  public ReverseWinchCommand(Winch winch) {
    this.winch = winch;
    // each subsystem used by the command must be passed into the addRequirements() method (which
    // takes a vararg of Subsystem)
    addRequirements(this.winch);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    winch.reverseClimber();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    winch.stop();
  }
}

