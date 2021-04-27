/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSystem;
// import frc.robot.subsystems.vision.limelight.Target;

// import java.util.Optional;

public class ShooterHoldVelocityCommand extends CommandBase {
  private Shooter shooter;
  private VisionSystem visionSystem;
  private RPMSource rpmSource;
  private double rpm;
  private int loopsReachedRPM = 0;
  private boolean runForever, finished;

  public ShooterHoldVelocityCommand(
      Shooter shooter, VisionSystem visionSystem, RPMSource rpmSource, boolean runForever) {
    this.shooter = shooter;
    this.visionSystem = visionSystem;
    this.rpmSource = rpmSource;
    this.rpm = -1;
    this.runForever = runForever;
    addRequirements(shooter, visionSystem);
  }

  public ShooterHoldVelocityCommand(Shooter shooter, VisionSystem visionSystem, double rpm) {
    this.shooter = shooter;
    this.visionSystem = visionSystem;
    this.rpmSource = RPMSource.CONSTANT;
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setReachedRPMDisplay(false);
    visionSystem.usePortPipeline();
    visionSystem.setCalculateDistance(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RPMSource rpmSource = this.rpmSource;
    if (rpmSource == RPMSource.FROM_SELECTOR) {
      rpmSource = shooter.getSelectedRPMSource();
    }

    double desiredRPM;
    switch (rpmSource) {
      case CONSTANT:
        desiredRPM = rpm;
        break;
      case DRIVER_PROVIDED:
        desiredRPM = shooter.getDriverDesiredRPM();
        break;
      case VISION:
        double meters = visionSystem.getCalculatedDistanceMeters();
        desiredRPM = Constants.Shooter.METERS_TO_RPM_FUNCTION.apply(meters);
        break;
      default:
        return;
    }
    shooter.holdVelocityRPMAndSetSolenoids(desiredRPM);
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
    visionSystem.setCalculateDistance(false);
    visionSystem.useDriverPipeline();
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

  public enum RPMSource {
    CONSTANT,
    DRIVER_PROVIDED,
    VISION,
    FROM_SELECTOR
  }
}
