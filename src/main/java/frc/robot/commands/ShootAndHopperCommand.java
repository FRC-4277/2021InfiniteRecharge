package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.ShooterHoldVelocityCommand.RPMSource;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShootingMode;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;

public class ShootAndHopperCommand extends CommandBase {
  // !=== Different shooting settings depending on challenge ===!

  private static final ShootSettingSupplier<Integer> RPM_REACHED_LOOPS_REQUIRED_TO_SHOOT =
      new ShootSettingSupplier<>(20, 5); // 20 ms * 10 = 200ms

  // Speed of hopper when moving ball to top
  private static final ShootSettingSupplier<Double> HOPPER_UP_TO_TOP_SPEED =
      new ShootSettingSupplier<>(0.5, 1.0);

  // Speed of hopper when moving ball from top to shooter
  private static final ShootSettingSupplier<Double> HOPPER_UP_TO_SHOOTER_SPEED =
      new ShootSettingSupplier<>(0.75, 1.0);

  // Time to ensure ball is stationary at top before shooting.
  private static final ShootSettingSupplier<Double> BALL_PAUSE_AT_TOP_SECONDS =
      new ShootSettingSupplier<>(0.5, 0.01);

  private static final ShootSettingSupplier<Double> MOVE_BALL_TO_SHOOTER_DURATION =
      new ShootSettingSupplier<>(1.5, 0.5);

  private final Shooter shooter;
  private final VerticalHopper hopper;
  private final VisionSystem visionSystem;
  private final RPMSource rpmSource;
  private int loopsReachedRPM = 0;
  private boolean velocityIsStable;
  private Timer ballAtTopTimer;
  private State state;
  private boolean ballHasLeftTop = false;
  private Timer ballLeftTopTimer = null;
  private boolean ballhasLeftTopAndWaited = false;
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
    ballLeftTopTimer = null;
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
        // do nothing, desiredRPM is already set
        break;
      default:
    }
  }

  @Override
  public void execute() {
    // Spin up shooter & set velocityIsStable to true when velocity is stable
    shooter.holdVelocityRPMAndSetSolenoids(desiredRPM, visionSystem.getCalculatedDistanceMeters());
    if (shooter.hasReachedRPM(desiredRPM)) {
      loopsReachedRPM++;
    } else {
      loopsReachedRPM = 0;
    }
    velocityIsStable = loopsReachedRPM >= RPM_REACHED_LOOPS_REQUIRED_TO_SHOOT.get(shooter);
    shooter.setReachedRPMDisplay(velocityIsStable);

    switch (state) {
      case MOVE_BALL_UP_TO_TOP:
        /*
         * If ball is not on top: Move hopper up
         * If ball is on top: Change state to MOVING_TOP_BALL_INTO_SHOOTER
         */
        if (!hopper.isBallPresentTop()) {
          hopper.moveUp(HOPPER_UP_TO_TOP_SPEED.get(shooter));
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
        if (hopper.isBallPresentTop() && !velocityIsStable) {
          hopper.stopMoving();
          return;
        }
        // Do nothing until ball on top is there for specified time (makes sure ball is still at the
        // top first)
        if (!ballAtTopTimer.hasElapsed(BALL_PAUSE_AT_TOP_SECONDS.get(shooter))) {
          hopper.stopMoving();
          return;
        }

        // !!!!! Now start moving up
        // hopper.moveUp(HOPPER_UP_TO_SHOOTER_SPEED.get(shooter));
        hopper.moveUpForShooting(visionSystem.getCalculatedDistanceMeters());
        // hopper.moveUp(0.5);
        // Track when the ball leaves the sensor
        /*if (!ballHasLeftTop && !hopper.isBallPresentTop()) {
          ballHasLeftTop = true;
          ballLeftTopTimer = new Timer();
          ballLeftTopTimer.reset();
          ballLeftTopTimer.start();
          ballCount++;
        }*/

        if (ballCount >= 3) {
          finished = true;
          // finishTimer = new Timer();
        }

        /*if (finishTimer != null && finishTimer.hasElapsed(1.0)) {
            finished = true;
        }*/

        // Now that the ball has left sensor, wait for next ball.
        if (ballLeftTopTimer != null
            && (ballLeftTopTimer.hasElapsed(MOVE_BALL_TO_SHOOTER_DURATION.get(shooter)))
            && hopper.isBallPresentTop()) {
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

  public static class ShootSettingSupplier<T> {
    private final T interstellarSetting;
    private final T powerPortSetting;

    public ShootSettingSupplier(T interstellarSetting, T powerPortSetting) {
      this.interstellarSetting = interstellarSetting;
      this.powerPortSetting = powerPortSetting;
    }

    public T get(Shooter shooter) {
      return shooter.getShootingMode() == ShootingMode.INTERSTELLAR_ACCURACY
          ? interstellarSetting
          : powerPortSetting;
    }
  }
}
