/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSystem;

public class IntakeLineUpCommand extends CommandBase {
  private static final double TURN_P = 0.025d;
  private static final double TURN_DEG_TOLERANCE = 5;
  private static final double MIN_TURN_ADJUST = 0.05;
  private DriveTrain driveTrain;
  private VisionSystem visionSystem;

  /**
   * Creates a new IntakeLineUpCommand.
   */
  public IntakeLineUpCommand(DriveTrain driveTrain, VisionSystem visionSystem) {
    this.driveTrain = driveTrain;
    this.visionSystem = visionSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSystem.setUsingPixy(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Double> targetOptional = visionSystem.getBallTargetDegrees();
    double headingError = targetOptional.orElse(0.0);
    if (Math.abs(headingError) <= TURN_DEG_TOLERANCE) {
      return;
    }
    double steerAdjust = headingError * TURN_P;
    if (Math.abs(steerAdjust) < MIN_TURN_ADJUST) {
      steerAdjust = Math.copySign(MIN_TURN_ADJUST, steerAdjust);
    }

    driveTrain.rawTankDrive(steerAdjust, -steerAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSystem.setUsingPixy(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
