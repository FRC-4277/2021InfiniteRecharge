package frc.robot.commands.autonomous.sixball;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.vision.VisionSystem;

import java.util.Optional;

public class PickupThreeCommand extends CommandBase {
  protected static final double INITIAL_SPEED = 0.175;
  private static final double MAX_ADJUSTMENT_SPEED = 0.00;
  private static final double DEGREE_THRESHOLD = 10;
  private static final double HEADING_PID_P = 0.0075;
  private static final double MAX_X_COORDINATE = 8.25;
  private static final double TILT_DEGREE_THRESHOLD = 20;
  private static final double INTAKE_BALL_COUNTER_TIME_BETWEEN_BALLS_MS = 300;
  // From Intake.java
  private boolean ballIntaking = false;
  private Long lastBallIntakeTime = -1L;

  private DriveTrain driveTrain;
  private VisionSystem visionSystem;
  private Intake intake;
  private int ballCount = 0;
  private Timer finishTimer = null;
  private boolean finished = false;

  public PickupThreeCommand(DriveTrain driveTrain, VisionSystem visionSystem, Intake intake) {
    this.driveTrain = driveTrain;
    this.visionSystem = visionSystem;
    this.intake = intake;
    addRequirements(driveTrain, visionSystem, intake);
  }

  /**
   * Move backwards with PID loop for heading zero
   * PID loop to correct for finding ball
   * Count balls until 3 reached
   * !!Do not go further & hit color wheel
   * !!Tilt detection
   */

  @Override
  public void initialize() {
    finished = false;
    ballCount = 0;
    visionSystem.setUsingPixy(true);
    // From Intake.java
    ballIntaking = false;
    lastBallIntakeTime = -1L;
    //
    finishTimer = null;
  }

  @Override
  public void execute() {
    /* DRIVING LOGIC */

    boolean intakeBallPresent = !intake.intakeSensor.get();
    // If we want the robot to be angled more left, then make RIGHT more negative
    // If we want the robot to be more angled right, then make LEFT more negative
    double speed = INITIAL_SPEED;
    double leftSpeed = speed;
    double rightSpeed = speed;

    double degrees = driveTrain.getHeading();
    double adjustment = degrees * 0.0125;
    adjustment = Math.signum(adjustment) * Math.min(0.09, adjustment);
    if (Math.abs(degrees) > 5) {
      if (degrees > 0) {
        leftSpeed += adjustment;
      } else {
        rightSpeed += adjustment;
      }
    }

    driveTrain.rawTankDrive(leftSpeed, rightSpeed);

    // INTAKE LOGIC, some taken from Intake.java
    intake.runIntake(1.0); // Run intake full speed
    // Count a ball if sensor is tripped & last sensor trip was more than 300 MS ago
    if (intakeBallPresent && (System.currentTimeMillis() - lastBallIntakeTime) > INTAKE_BALL_COUNTER_TIME_BETWEEN_BALLS_MS) {
      ballCount++;
      System.out.println(">>> Pickup Three: Picked up " + ballCount);
      lastBallIntakeTime = System.currentTimeMillis();
    }
    if (ballCount >= 2 && finishTimer == null) {
      finishTimer = new Timer();
      finishTimer.reset();
      finishTimer.start();
    }
    if (finishTimer != null && finishTimer.hasElapsed(0.2)) {
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    // Stop command if we drive too far to the right
    // Or if we start tilting (roll or pitch)
    if (driveTrain.getPose().getX() > MAX_X_COORDINATE || driveTrain.getLargestAbsoluteTiltAngle() > TILT_DEGREE_THRESHOLD) {
      System.out.println("MAX X REACHED");
      driveTrain.stopDrive();
      return true;
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    visionSystem.setUsingPixy(false);
  }
}
