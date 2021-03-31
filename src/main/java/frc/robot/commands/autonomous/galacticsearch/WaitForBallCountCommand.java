package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class WaitForBallCountCommand extends CommandBase {
  private Supplier<Integer> ballCountSupplier;
  private final int countTarget;

  public WaitForBallCountCommand(Supplier<Integer> ballCountSupplier, int countTarget) {
    this.ballCountSupplier = ballCountSupplier;
    this.countTarget = countTarget;
  }

  @Override
  public boolean isFinished() {
    return ballCountSupplier.get() >= countTarget;
  }
}
