package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.ShooterHoldVelocityCommand.RPMSource;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;

public class ShootAndHopperCommand extends CommandBase {
  private static final int RPM_REACHED_LOOPS_REQUIRED_TO_SHOOT = 40; // 20 ms * 10 = 200ms
  private static final double HOPPER_UP_TO_TOP_SPEED =
      0.5; // Speed of hopper when moving ball to top
  private static final double HOPPER_UP_TO_SHOOTER_SPEED =
      0.5; // Speed of hopper when moving ball from top to shooter
  private static final double BALL_PAUSE_AT_TOP_SECONDS =
      0.5; // Time to ensure ball is stationary at top before shooting.
  private Shooter shooter;
  private VerticalHopper hopper;
  private VisionSystem visionSystem;
  private RPMSource rpmSource;
  private int loopsReachedRPM = 0;
  private boolean velocityIsStable;
  private Timer ballAtTopTimer;
  private State state;
  private boolean ballHasLeftTop = false;
  private int ballCount = 0;
  private boolean runForever = false;
  private boolean finished = false;
  private double desiredRPM;
  private Timer finishTimer = null;

  public ShootAndHopperCommand(
      Shooter shooter,
      VerticalHopper hopper,
      VisionSystem visionSystem,
      RPMSource rpmSource,
      boolean addRequirementHopper) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.visionSystem = visionSystem;
    this.rpmSource = rpmSource;
    if (addRequirementHopper) {
      addRequirements(shooter, hopper, visionSystem);
    } else {
      addRequirements(shooter, visionSystem);
    }
    runForever = true;
  }

  public ShootAndHopperCommand(
      Shooter shooter,
      VerticalHopper hopper,
      VisionSystem visionSystem,
      RPMSource rpmSource,
      boolean runForever,
      boolean addRequirementHopper) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.visionSystem = visionSystem;
    this.rpmSource = rpmSource;
    this.runForever = runForever;
    if (addRequirementHopper) {
      addRequirements(shooter, hopper, visionSystem);
    } else {
      addRequirements(shooter, visionSystem);
    }
  }

  public ShootAndHopperCommand(
      Shooter shooter,
      VerticalHopper hopper,
      VisionSystem visionSystem,
      boolean runForever,
      int desiredRPM,
      boolean addRequirementHopper) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.visionSystem = visionSystem;
    this.rpmSource = RPMSource.CONSTANT;
    this.runForever = runForever;
    this.desiredRPM = desiredRPM;
    if (addRequirementHopper) {
      addRequirements(shooter, hopper, visionSystem);
    } else {
      addRequirements(shooter, visionSystem);
    }
  }

  @Override
  public void initialize() {
    shooter.setReachedRPMDisplay(false);
    visionSystem.usePortPipeline();
    visionSystem.setCalculateDistance(true);
    loopsReachedRPM = 0;
    state = State.MOVE_BALL_UP_TO_TOP; // Start off by moving ball up
    ballCount = 0;
    finished = false;
    finishTimer = null;

    // Get desired RPM from user
    RPMSource rpmSource = this.rpmSource;
    if (rpmSource == RPMSource.FROM_SELECTOR) {
      rpmSource = shooter.getSelectedRPMSource();
    }
    switch (rpmSource) {
      case DRIVER_PROVIDED:
        desiredRPM = shooter.getDriverDesiredRPM();
        break;
      case VISION:
        double meters = visionSystem.getCalculatedDistanceMeters();
        desiredRPM = Constants.Shooter.METERS_TO_RPM_FUNCTION.apply(meters);
        break;
      case CONSTANT:
        desiredRPM = this.desiredRPM;
        break;
      default:
        return;
    }
  }

  @Override
  public void execute() {
    // Spin up shooter & set velocityIsStable to true when velocity is stable
    shooter.holdVelocityRPM(desiredRPM);
    if (shooter.hasReachedRPM(desiredRPM)) {
      loopsReachedRPM++;
    } else {
      loopsReachedRPM = 0;
    }
    velocityIsStable = loopsReachedRPM >= RPM_REACHED_LOOPS_REQUIRED_TO_SHOOT;
    shooter.setReachedRPMDisplay(velocityIsStable);

    switch (state) {
      case MOVE_BALL_UP_TO_TOP:
        /*
         * If ball is not on top: Move hopper up
         * If ball is on top: Change state to MOVING_TOP_BALL_INTO_SHOOTER
         */
        if (!hopper.isBallPresentTop()) {
          hopper.moveUp(HOPPER_UP_TO_TOP_SPEED);
        } else {
          state = State.MOVING_TOP_BALL_INTO_SHOOTER_WHEN_READY; // Change state
          ballAtTopTimer = new Timer();
          ballAtTopTimer.reset();
          ballAtTopTimer.start();
          ballHasLeftTop = false;
        }
        break;
      case MOVING_TOP_BALL_INTO_SHOOTER_WHEN_READY:
        // Do nothing until shooter velocity is stable
        if (!velocityIsStable) {
          hopper.stopMoving();
          return;
        }
        // Do nothing until ball on top is there for specified time (makes sure ball is still at the
        // top first)
        if (!ballAtTopTimer.hasElapsed(BALL_PAUSE_AT_TOP_SECONDS)) {
          hopper.stopMoving();
          return;
        }

        // !!!!! Now start moving up
        hopper.moveUp(HOPPER_UP_TO_SHOOTER_SPEED);
        // Track when the ball leaves the sensor
        if (!ballHasLeftTop && !hopper.isBallPresentTop()) {
          ballHasLeftTop = true;
          ballCount++;
        }

        if (ballCount >= 3) {
          finished = true;
          // finishTimer = new Timer();
        }

        /*if (finishTimer != null && finishTimer.hasElapsed(1.0)) {
            finished = true;
        }*/

        // Now that the ball has left sensor, wait for next ball.
        if (ballHasLeftTop && hopper.isBallPresentTop()) {
          // Once next ball is at sensor, set state to MOVE_BALL_UP_TO_TOP
          state = State.MOVE_BALL_UP_TO_TOP;
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    hopper.stopMoving();
    shooter.setReachedRPMDisplay(false);
    visionSystem.setCalculateDistance(false);
    visionSystem.useDriverPipeline();
  }

  @Override
  public boolean isFinished() {
    return !runForever && finished;
  }

  public enum State {
    MOVE_BALL_UP_TO_TOP,
    MOVING_TOP_BALL_INTO_SHOOTER_WHEN_READY
  }
}
