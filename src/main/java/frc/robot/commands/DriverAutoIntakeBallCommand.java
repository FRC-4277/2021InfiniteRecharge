package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.vision.VisionSystem;
import java.util.Optional;

public class DriverAutoIntakeBallCommand extends CommandBase {
  private static final double DRIVE_SPEED = 0.25; // out of 1
  private static final double INTAKE_SPEED = 0.7; // out of 1
  private static final double WAIT_AFTER_INTAKE_SECONDS = 0.2; // seconds
  private static final double TURN_P = 0.025d;
  private static final double TURN_DEG_TOLERANCE = 5;
  private static final double MIN_TURN_ADJUST = 0.05;

  private static final int intakeMinStopTimeMs = 400; // From IntakeCommand.java
  private boolean ballIntaking = false; // From IntakeCommand.java
  private Long ballIntakeTime = 0L; // From IntakeCommand.java
  private Timer timer;

  private final DriveTrain driveTrain;
  private final Intake intake;
  private final VisionSystem visionSystem;

  public DriverAutoIntakeBallCommand(
      DriveTrain driveTrain, Intake intake, VisionSystem visionSystem) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.visionSystem = visionSystem;

    addRequirements(driveTrain, intake, visionSystem);
  }

  @Override
  public void initialize() {
    visionSystem.setUsingPixy(true);
    this.timer = null;

    // From IntakeCommand.java BELOW:
    this.ballIntaking = false;
    this.ballIntakeTime = 0L;
  }

  @Override
  public void execute() {
    // [LINE UP DRIVE]
    // From IntakeLineUpCommand BELOW:
    Optional<Double> targetOptional = visionSystem.getBallTargetDegrees();
    double headingError = targetOptional.orElse(0.0);
    boolean withinTolerance = Math.abs(headingError) <= TURN_DEG_TOLERANCE;
    double steerAdjust = headingError * TURN_P;
    if (!withinTolerance && Math.abs(steerAdjust) < MIN_TURN_ADJUST) {
      steerAdjust = Math.copySign(MIN_TURN_ADJUST, steerAdjust);
    }
    double speed = DRIVE_SPEED;
    driveTrain.rawTankDrive(speed + steerAdjust, speed - steerAdjust);

    // [INTAKE]
    // From IntakeCommand.java BELOW:
    boolean intakeBallPresent = !intake.intakeSensor.get();
    if (intakeBallPresent && !ballIntaking) {
      ballIntaking = true;
    }
    if (ballIntaking && (System.currentTimeMillis() - ballIntakeTime >= intakeMinStopTimeMs)) {
      ballIntaking = false;
    }
    if (!ballIntaking) {
      intake.runIntake(INTAKE_SPEED);
    } else {
      intake.stopIntake();
    }

    // Start timer if needed
    if (intake.isSensorTripped() && timer == null) {
      timer = new Timer();
      timer.reset();
      timer.start();
    }
  }

  @Override
  public boolean isFinished() {
    return timer != null && timer.hasElapsed(WAIT_AFTER_INTAKE_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    visionSystem.setUsingPixy(false);
    driveTrain.stopDrive();
    intake.stopIntake(); // From IntakeCommand.java
  }
}
