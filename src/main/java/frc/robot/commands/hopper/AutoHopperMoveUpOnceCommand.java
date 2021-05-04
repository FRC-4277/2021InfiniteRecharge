package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VerticalHopper;

public class AutoHopperMoveUpOnceCommand extends CommandBase {
  private final VerticalHopper verticalHopper;
  private double targetPosition;

  public AutoHopperMoveUpOnceCommand(VerticalHopper verticalHopper) {
    this.verticalHopper = verticalHopper;
    addRequirements(verticalHopper);
  }

  @Override
  public void initialize() {
    targetPosition = verticalHopper.getPositionInches() + verticalHopper.getMoveDistanceIn();
  }

  @Override
  public void execute() {
    verticalHopper.setPosition(targetPosition);
  }

  private boolean inRange(double position) {
    return Math.abs(targetPosition - position) <= verticalHopper.getMovePositionThresholdIn();
  }

  @Override
  public boolean isFinished() {
    boolean leftFinished = inRange(verticalHopper.getLeftPositionInches());
    boolean rightFinished = inRange(verticalHopper.getRightPositionInches());
    return leftFinished && rightFinished;
  }

  @Override
  public void end(boolean interrupted) {
    verticalHopper.stopMoving();
  }
}
