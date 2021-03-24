package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;

public class WinchClimbCommand extends CommandBase {
  private final Winch winch;

  public WinchClimbCommand(Winch winch) {
    this.winch = winch;
    // each subsystem used by the command must be passed into the addRequirements() method (which
    // takes a vararg of Subsystem)
    addRequirements(this.winch);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    winch.climb();
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
