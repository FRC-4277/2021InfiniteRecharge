package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class PrintTimerCommand extends PrintCommand {

  public PrintTimerCommand(Timer timer, String message) {
    super(String.format("[[[>>>TIMER<<<]]] %.2f at %s", timer.get(), message));
  }
}
