// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VerticalHopper;

/*
Called whenever the intake sensor is triggered
*/
public class NewAutoHopperMoveInCommand extends CommandBase {
  private final VerticalHopper hopper;
  private double targetInches;

  /** Creates a new NewAutoHopperMoveInCommand. */
  public NewAutoHopperMoveInCommand(VerticalHopper hopper) {
    this.hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetInches = hopper.getPositionInches() + hopper.getMoveDistanceIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.setPositionMotionMagic(targetInches);
    
  }

  private boolean inRange(double position) {
    return Math.abs(targetInches - position) <= hopper.getMovePositionThresholdIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      hopper.stopMoving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean leftFinished = inRange(hopper.getLeftPositionInches());
    boolean rightFinished = inRange(hopper.getRightPositionInches());
    return leftFinished && rightFinished;
  }
}
