/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button.*;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.autonomous.galacticsearch.GalacticAutoCommand;
import frc.robot.commands.autonomous.galacticvideo.GalacticAutoVideoCommand;
import frc.robot.commands.autonomous.galacticvideo.GalacticPath;
import frc.robot.commands.hopper.AutoHopperMoveInCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.VisionSystem;
import frc.robot.util.CooperSendable;
import frc.robot.util.GameTimer;
import frc.robot.util.LogitechButton;
import frc.robot.util.XboxTrigger;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Pneumatics
  // private Compressor compressor = new Compressor(0);
  // todo : uncomment when compressor added

  // Controllers
  private Supplier<Boolean> invertControls = () -> false;

  // ShuffleBoard
  private final ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");
  private final ShuffleboardTab settingsTab = Shuffleboard.getTab("Settings");
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  // private final ShuffleboardTab colorWheelTab = Shuffleboard.getTab("Control Panel");
  private final ShuffleboardTab testTab = Shuffleboard.getTab("Testing");
  private final ShuffleboardTab verificationTab = Shuffleboard.getTab("Verification");
  private final ShuffleboardTab simulationTab = Shuffleboard.getTab("Simulation");

  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain =
      new DriveTrain(testTab, simulationTab, autonomousTab, settingsTab);
  private final Intake intake = new Intake();
  private final VerticalHopper hopper =
      new VerticalHopper(this, intake.intakeSensor, driverTab, settingsTab);
  private final Shooter shooter = new Shooter(settingsTab, driverTab);
  // private final ColorWheel colorWheel = new ColorWheel(colorWheelTab);
  // private final Gate gate = new Gate();
  private final CameraSystem cameraSystem = new CameraSystem(driverTab);
  private final VisionSystem visionSystem =
      new VisionSystem(driverTab, autonomousTab, driveTrain.getFieldSim());
  // private final Winch winch = new Winch();
  // private final HookElevator hookElevator = new HookElevator();
  /*private final VerificationSystem verificationSystem = new VerificationSystem(
  driveTrain, intake, hopper, shooter, colorWheel, cameraSystem, visionSystem, winch, hookElevator,
  verificationTab);*/

  private final JoystickDriveCommand driveCommand = new JoystickDriveCommand(driveTrain);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, hopper);
  private final ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intake);
  private final MoveHopperUpCommand moveHopperUpCommand = new MoveHopperUpCommand(hopper);
  private final MoveHopperDownCommand moveHopperDownCommand = new MoveHopperDownCommand(hopper);
  // private final ShooterForwardCommand shooterForwardCommand = new ShooterForwardCommand(shooter);
  private final ShooterBackwardsCommand shooterBackwardsCommand =
      new ShooterBackwardsCommand(shooter);
  private final ShooterHoldVelocityCommand shooterHoldVelocityViaVisionCommand =
      new ShooterHoldVelocityCommand(
          shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.FROM_SELECTOR, true);
  private final ShootAndHopperCommand shootAndHopperCommand =
      new ShootAndHopperCommand(
          shooter, hopper, visionSystem, ShooterHoldVelocityCommand.RPMSource.FROM_SELECTOR, true);
  // private final ToggleGateCommand toggleGateCommand = new ToggleGateCommand(gate);
  private final ToggleCameraCommand toggleCameraCommand = new ToggleCameraCommand(cameraSystem);
  private final UseShooterCameraCommand useShooterCameraCommand =
      new UseShooterCameraCommand(cameraSystem);
  private final UseIntakeCameraCommand useIntakeCameraCommand =
      new UseIntakeCameraCommand(cameraSystem);
  private final VisionAlignCommand visionAlignCommand =
      new VisionAlignCommand(driveTrain, visionSystem, true, true);
  private final AutoHopperMoveInCommand autoHopperMoveInCommand =
      new AutoHopperMoveInCommand(hopper);
  // private final WinchClimbCommand winchClimbCommand = new WinchClimbCommand(winch);
  // private final MoveHookUpCommand hookUpCommand = new MoveHookUpCommand(hookElevator);
  // private final MoveHookDownCommand hookDownCommand = new MoveHookDownCommand(hookElevator);
  private final IntakeLineUpCommand intakeLineUpCommand =
      new IntakeLineUpCommand(driveTrain, visionSystem);

  private SendableChooser<Command> autoChooser;
  private NetworkTableEntry resetOdometryOnAuto;

  private boolean inAutonomous = true;
  private NetworkTableEntry autonomousMessageEntry;

  private boolean drivingFromHome = false;
  private boolean drivingFromHomeInverted = false;

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(intake);
    SmartDashboard.putData(hopper);
    SmartDashboard.putData(shooter);
    // SmartDashboard.putData(colorWheel);
    // SmartDashboard.putData(gate);
    SmartDashboard.putData(cameraSystem);
    SmartDashboard.putData(visionSystem);
    // SmartDashboard.putData(winch);
    // SmartDashboard.putData(hookElevator);
    // SmartDashboard.putData(verificationSystem);

    // Configure the button bindings
    configureButtonBindings();

    // Default Commands
    driveTrain.setDefaultCommand(driveCommand);
    hopper.setDefaultCommand(autoHopperMoveInCommand);

    // Hopper move up distance PID
    /*new Trigger(intake::isSensorTripped)
      .whenActive(new WaitCommand(hopper.getMoveAfterSensorDelayMs() / 1000d)
        .andThen(new AutoHopperMoveUpOnceCommand(hopper))
    );*/

    // ShuffleBoard
    setupDriverTab();
    setupAutonomousTab();
    setupTestingTab();
  }

  private void setupTestingTab() {
    testTab.add(toggleCameraCommand);
    testTab.add(shootAndHopperCommand);
  }

  private void setupAutonomousTab() {
    autoChooser = new SendableChooser<>();
    SendableRegistry.setName(autoChooser, "Autonomous Command");
    // = Do Nothing
    autoChooser.setDefaultOption("Nothing", null);

    autoChooser.addOption("Drive Straight (2m)", new DriveStraightXCommand(driveTrain, 2.0));
    autoChooser.addOption("Drive Straight (-2m)", new DriveStraightXCommand(driveTrain, -2.0));

    autoChooser.addOption("AutoNav - Barrel (new)", new Barrel2AutoCommand(driveTrain));
    autoChooser.addOption("AutoNav - Slalom (new)", new Slalom2AutoCommand(driveTrain));
    autoChooser.addOption("AutoNav - Bounce (new)", new Bounce2AutoCommand(driveTrain));

    autoChooser.addOption("AutoNav - Barrel", new BarrelAutoCommand(driveTrain));
    autoChooser.addOption("AutoNav - Slalom", new SlalomAutoCommand(driveTrain));
    autoChooser.addOption("AutoNav - Bounce", new BounceAutoCommand(driveTrain));
    autoChooser.addOption(
        "Galactic Search (for video)",
        new GalacticAutoVideoCommand(this, driveTrain, visionSystem, hopper, intake, shooter));
    autoChooser.addOption(
        "Galactic Search",
        new GalacticAutoCommand(this, driveTrain, visionSystem, hopper, intake, shooter));

    /*// = Move Off Line
    autoChooser.addOption("Move Off Line", new LazyRamseteCommand(driveTrain, () -> {
      Pose2d currentPose = driveTrain.getPose();
      // Move 1.7 meters to the right [PathWeaver view]
      return driveTrain.generateXTrajectory(currentPose, 1.7);
    }));

    // = Aim, Shoot, Move Off Line (Port on Left)
    autoChooser.addOption("Aim, Shoot, Move Off Line (Port on Left)",
            new AimShootMoveBackAutoCommand(driveTrain, visionSystem, shooter, hopper, false));

    // = Aim, Shoot, Move Off Line (Port on Right)
    autoChooser.addOption("Aim, Shoot, Move Off Line (Port on Right)",
            new AimShootMoveBackAutoCommand(driveTrain, visionSystem, shooter, hopper, true));

    // = Aim, Shoot, Pickup Trench, Shoot
    autoChooser.addOption("Aim, Shoot, Pickup Trench, Shoot",
            new AimShootPickupShootAutoCommand(driveTrain, visionSystem, shooter, hopper, intake));*/

    autonomousTab.add(autoChooser).withPosition(0, 0).withSize(2, 1);

    NetworkTableInstance.getDefault()
        .getTable("Shuffleboard/Autonomous/Autonomous Command")
        .getEntry("selected")
        .addListener(
            notification -> {
              if (Objects.equals(notification.value.getString(), "Galactic Search")) {
                System.out.println("GALACTIC SEARCH!");
                cameraSystem.switchToIntake();
              }
            },
            EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

    autonomousMessageEntry =
        autonomousTab
            .add("Autonomous Message", "...")
            .withPosition(0, 3)
            .withSize(5, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    resetOdometryOnAuto =
        autonomousTab
            .add("Reset Odometry on Auto", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    autonomousTab.add(intakeCommand);
    autonomousTab.add(new DriveStorePosition(driveTrain, 0));
    autonomousTab.add(new DriveRecallPosition(driveTrain, 0));
  }

  public void setAutonomousMessage(String message) {
    System.out.println(">>> " + message);
    if (autonomousMessageEntry != null) {
      autonomousMessageEntry.setString(message);
    }
  }

  private void setupDriverTab() {
    /*SendableRegistry.add(hopper.getSendable(), "VerticalHopper");

    driverTab.add(hopper.getSendable())
    .withWidget("VerticalHopper")
    .withPosition(0,0)
    .withSize(3, 2);*/
    GameTimer gameTimer = new GameTimer(this);
    SendableRegistry.add(gameTimer, "GameTimer");
    driverTab
        .add(gameTimer)
        .withWidget("GameTimer")
        .withProperties(Map.of("Font Color", "black"))
        .withPosition(7, 0)
        .withSize(2, 1);

    CooperSendable cooperSendable = new CooperSendable();
    SendableRegistry.add(cooperSendable, "Cooper");
    driverTab.add(cooperSendable).withWidget("Cooper").withPosition(10, 4).withSize(1, 1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drivingFromHome = DriverStation.getInstance().getJoystickIsXbox(0);

    /* Note on shooter commands
     * ShootAndHopperCommand = Spin up shooter & move hopper up as necessary
     * ShootHoldVelocityViaVisionCommand = Spin up shooter only, not normally used (so assigned to POV)
     */

    XboxController xboxController = new XboxController(drivingFromHome ? 0 : 1);
    visionSystem.setXboxController(xboxController);
    if (!drivingFromHome) {
      Joystick driveStick = new Joystick(0);
      // === MAIN DRIVER - Logitech Extreme 3D Pro

      // Note: Trigger (button 1) is used in JoystickDriveCommand for quick turn

      JoystickButton pointerButton = new JoystickButton(driveStick, LogitechButton.POINTER);
      pointerButton.whenPressed(toggleCameraCommand);

      invertControls =
          () -> driveStick.getRawAxis(3) <= .5; // Invert controls when throttle is negative value

      // Buttons
      JoystickButton button2 = new JoystickButton(driveStick, 2);
      button2.whileActiveOnce(shootAndHopperCommand);

      JoystickButton triggerButton = new JoystickButton(driveStick, 1);
      triggerButton
          // This button will be used when curvature drive is enabled, so
          // only make hopper up work for this button with arcade drive
          .and(new Trigger(() -> driveTrain.getDrivingMode() == JoystickDriveCommand.Mode.ARCADE))
          .whileActiveOnce(moveHopperUpCommand);

      JoystickButton button3 = new JoystickButton(driveStick, 3);
      button3.whileActiveOnce(intakeCommand);

      JoystickButton button4 = new JoystickButton(driveStick, 4);
      button4.whileActiveOnce(reverseIntakeCommand);

      JoystickButton button5 = new JoystickButton(driveStick, 5);
      button5.whileActiveOnce(visionAlignCommand);

      JoystickButton button6 = new JoystickButton(driveStick, 6);
      button6.whileActiveOnce(moveHopperDownCommand);

      JoystickButton button9 = new JoystickButton(driveStick, 9);
      button9.whenPressed(new DriveStorePosition(driveTrain, 0));

      JoystickButton button10 = new JoystickButton(driveStick, 10);
      button10.whileActiveOnce(new DriveRecallPosition(driveTrain, 0));

      JoystickButton button11 = new JoystickButton(driveStick, 11);
      button11.whenPressed(new DriveStorePosition(driveTrain, 1));

      JoystickButton button12 = new JoystickButton(driveStick, 12);
      button12.whileActiveOnce(new DriveRecallPosition(driveTrain, 1));

      POVButton upPOV = new POVButton(driveStick, 0);
      upPOV.whileActiveOnce(shooterHoldVelocityViaVisionCommand);

      // Drive with Logitech
      driveCommand.setYControllerSupplier(() -> -driveStick.getY(Hand.kLeft));
      driveCommand.setXControllerSupplier(() -> driveStick.getRawAxis(2));
      driveCommand.setQuickTurn(triggerButton::get);
    } else {
      // Drive with Xbox Controller
      driveCommand.setYControllerSupplier(() -> -xboxController.getY(Hand.kLeft));
      driveCommand.setXControllerSupplier(() -> xboxController.getX(Hand.kRight));
      driveCommand.setQuickTurn(() -> xboxController.getStickButton(Hand.kLeft));

      // Invert controls via START button
      JoystickButton startButton = new JoystickButton(xboxController, kStart.value);
      startButton.whileActiveOnce(
          new InstantCommand(
              () -> {
                System.out.println("Inverted to " + !drivingFromHomeInverted);
                drivingFromHomeInverted = !drivingFromHomeInverted;
              }));
      invertControls = () -> drivingFromHomeInverted;
    }

    Trigger invertControlsThrottle = new Trigger(invertControls::get);
    // Automatically switch camera when drive is inverted/normal
    invertControlsThrottle.whenActive(useShooterCameraCommand);
    invertControlsThrottle.whenInactive(useIntakeCameraCommand);

    driveCommand.setInvertControls(invertControls);

    // === CO-PILOT - Xbox 360/One Controller

    XboxTrigger leftTrigger = new XboxTrigger(xboxController, Hand.kLeft);
    leftTrigger.whileActiveOnce(intakeCommand);

    JoystickButton leftBumper = new JoystickButton(xboxController, kBumperLeft.value);
    leftBumper.whileActiveOnce(reverseIntakeCommand);

    XboxTrigger rightTrigger = new XboxTrigger(xboxController, Hand.kRight);
    rightTrigger.whileActiveOnce(moveHopperUpCommand);

    JoystickButton rightBumper = new JoystickButton(xboxController, kBumperRight.value);
    rightBumper.whileActiveOnce(moveHopperDownCommand);

    JoystickButton xButton = new JoystickButton(xboxController, kX.value);
    xButton.whileActiveOnce(shootAndHopperCommand);

    JoystickButton bButton = new JoystickButton(xboxController, kB.value);
    bButton.whileActiveOnce(shooterBackwardsCommand);

    JoystickButton aButton = new JoystickButton(xboxController, kA.value);
    aButton.whileActiveOnce(visionAlignCommand);

    POVButton upPOVButton = new POVButton(xboxController, 0);
    // upPOVButton.whileActiveOnce(winchClimbCommand);
    upPOVButton.whileActiveOnce(shooterHoldVelocityViaVisionCommand);

    // POVButton downPOVButton = new POVButton(xboxController, 180);

    POVButton leftPOVButton = new POVButton(xboxController, 270);
    // leftPOVButton.whileActiveOnce(hookDownCommand);

    POVButton rightPOVButton = new POVButton(xboxController, 90);
    // rightPOVButton.whileActiveOnce(hookUpCommand);

    JoystickButton backButton = new JoystickButton(xboxController, kBack.value);
    backButton.whileActiveOnce(intakeLineUpCommand);
  }

  private void switchToDriverView() {
    Shuffleboard.selectTab("Driver");
    inAutonomous = false;
  }

  protected void teleopInit() {
    switchToDriverView();

    // Initial camera state for inverted controls
    if (invertControls.get()) {
      // Inverted, the shooter is now the front
      useShooterCameraCommand.schedule();
    } else {
      // Not inverted, the intake is the front
      useIntakeCameraCommand.schedule();
    }
  }

  protected void autonomousInit() {
    inAutonomous = true;
    if (resetOdometryOnAuto.getBoolean(true)) {
      System.out.println("Resetting encoders, heading, and odometry");
      driveTrain.resetEncoders();
      driveTrain.zeroHeading();
      driveTrain.resetOdometry();
      System.out.println("DriveTrain's encoders & heading are reset.");
    }
  }

  public boolean isInAutonomous() {
    return inAutonomous;
  }

  public Pose2d getSimPose() {
    return driveTrain.getPose();
  }

  public GalacticPath getSimPath() {
    return visionSystem.getSimPathSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (autoChooser == null ? null : autoChooser.getSelected());
    // return m_autoCommand;
  }

  public void simulationPeriodic() {
    double current = driveTrain.getSimDrawnCurrentAmps(); // Add other power loads here...
    RoboRioSim.setVInVoltage(
        Math.min(BatterySim.calculateDefaultBatteryLoadedVoltage(current), 12.0));
  }
}
