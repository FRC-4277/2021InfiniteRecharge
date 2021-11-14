package frc.robot.commands.autonomous.sixball;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.ShooterHoldVelocityCommand.RPMSource;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShootingMode;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;

public class FastShootAndHopperCommand extends CommandBase {
  // !=== Different shooting settings depending on challenge ===!

  private static final ShootSettingSupplier<Integer> RPM_REACHED_LOOPS_REQUIRED_TO_SHOOT =
      new ShootSettingSupplier<>(20, 2); // 20 ms * 10 = 200ms

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
      new ShootSettingSupplier<>(1.5, 0.01);

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
  private boolean incremented = false;
  private Long lastBallIncrement = -1L;

  public FastShootAndHopperCommand(
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

  public FastShootAndHopperCommand(
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

  public FastShootAndHopperCommand(
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

  public FastShootAndHopperCommand(
      Shooter shooter,
      VerticalHopper hopper,
      VisionSystem visionSystem,
      boolean runForever,
      int desiredRPM) {
    this(shooter, hopper, visionSystem, runForever, desiredRPM, true);
  }


  @Override
  public void initialize() {
    hopper.resetBalls();
    shooter.setReachedRPMDisplay(false);
    visionSystem.usePortPipeline();
    visionSystem.setCalculateDistance(true);
    loopsReachedRPM = 0;
    state = State.MOVE_BALL_UP_TO_TOP; // Start off by moving ball up
    ballCount = 0;
    finished = false;
    ballLeftTopTimer = null;
    finishTimer = null;
    incremented = false;
    lastBallIncrement = -1L;

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
    if (Math.abs(shooter.getVelocityRPM() - desiredRPM) <= 110) {
      loopsReachedRPM++;
    } else {
      loopsReachedRPM = 0;
    }
    velocityIsStable = loopsReachedRPM >= RPM_REACHED_LOOPS_REQUIRED_TO_SHOOT.powerPortSetting;
    shooter.setReachedRPMDisplay(velocityIsStable);

    switch (state) {
      case MOVE_BALL_UP_TO_TOP:
        /*
         * If ball is not on top: Move hopper up
         * If ball is on top: Change state to MOVING_TOP_BALL_INTO_SHOOTER
         */
        incremented = false;
        if (!hopper.isBallPresentTop()) {
          hopper.moveUp(HOPPER_UP_TO_TOP_SPEED.powerPortSetting);
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
        if (!ballAtTopTimer.hasElapsed(BALL_PAUSE_AT_TOP_SECONDS.powerPortSetting)) {
          hopper.stopMoving();
          return;
        }

        // !!!!! Now start moving up
        // hopper.moveUp(HOPPER_UP_TO_SHOOTER_SPEED.powerPortSetting);
        hopper.moveUpForShooting(visionSystem.getCalculatedDistanceMeters());
        //
        /*if (!incremented) {
          ballCount++;
          incremented = true;
        }*/
        // hopper.moveUp(0.5);
        // Track when the ball leaves the sensor

        /*if (finishTimer != null && finishTimer.hasElapsed(1.0)) {
            finished = true;
        }*/

        // Now that the ball has left sensor, wait for next ball.
        if (ballLeftTopTimer != null
            && (ballLeftTopTimer.hasElapsed(MOVE_BALL_TO_SHOOTER_DURATION.powerPortSetting))
            && hopper.isBallPresentTop()) {
          // Once next ball is at sensor, set state to MOVE_BALL_UP_TO_TOP
          state = State.MOVE_BALL_UP_TO_TOP;
          incremented = false;
        }
        break;
    }

    if (hopper.isBallPresentTop() && (System.currentTimeMillis() - lastBallIncrement) > 740) {
      ballCount++;
      System.out.println("@@@@ BALL INCREMENTED");
      lastBallIncrement = System.currentTimeMillis();
    }

    System.out.println("@@@@BALL@@@@@ " + ballCount);

    if (ballCount >= 4 && finishTimer == null) {
      finishTimer = new Timer();
      finishTimer.reset();
      finishTimer.start();
      System.out.println("TIMER STARTED");
    }
    // Wait for 1.0 seconds before finishing command to ensure ball has time 
    if (finishTimer != null) {
      System.out.println("Timer: " + finishTimer.get());
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
    if (finishTimer != null && finishTimer.hasElapsed(0.1)) {
      System.out.println("Ended naturally");
    }
    return !runForever && (finishTimer != null && finishTimer.hasElapsed(1.0));
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
