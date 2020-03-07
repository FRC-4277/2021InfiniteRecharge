/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSystem;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

import java.util.Optional;

public class TrenchRunAutoCommand extends CommandBase {
  private static final double MAXIMUM_DISTANCE_RIGHT_M = 8.54 - 3.16;
  private static final double STRAIGHT_MOVEMENT_SPEED = 0.4;
  private static final double TURN_P = 0.025d;
  private static final double TURN_DEG_TOLERANCE = 5;
  private static final double MIN_TURN_ADJUST = 0.05;
  private DriveTrain driveTrain;
  private VisionSystem visionSystem;
  private boolean finished = false;
  private double startingDistance;

  /**
   * Creates a new TrenchRunAutoCommand.
   */
  public TrenchRunAutoCommand(DriveTrain driveTrain, VisionSystem visionSystem) {
    this.driveTrain = driveTrain;
    this.visionSystem = visionSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, visionSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.finished = false;
    this.startingDistance = driveTrain.getAverageEncoderDistanceM();
    visionSystem.setUsingPixy(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if we've reached max distance, and stop if needed
    double distanceTravelled = driveTrain.getAverageEncoderDistanceM() - startingDistance;
    if (distanceTravelled > MAXIMUM_DISTANCE_RIGHT_M) {
      finished = true;
      return;
    }

    Optional<Double> targetOptional = visionSystem.getBallTargetDegrees();
    double headingError = targetOptional.orElse(0.0); // Degrees, Positive is CCW

    double leftSpeed = STRAIGHT_MOVEMENT_SPEED;
    double rightSpeed = STRAIGHT_MOVEMENT_SPEED;

    double turnAdjustment = 0;
    if (Math.abs(headingError) >= TURN_DEG_TOLERANCE) {
      // Implement min command
      if (Math.abs(turnAdjustment) < MIN_TURN_ADJUST) {
        turnAdjustment = Math.copySign(MIN_TURN_ADJUST, turnAdjustment);
      }
    }

    leftSpeed += turnAdjustment;
    rightSpeed -= turnAdjustment;
    // Drive
    driveTrain.rawTankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSystem.setUsingPixy(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
