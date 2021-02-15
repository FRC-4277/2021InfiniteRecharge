/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.*;
import frc.robot.util.GameTimer;
import frc.robot.util.LogitechButton;
import frc.robot.util.XboxTrigger;

import java.util.function.Supplier;
import java.util.Map;

import static edu.wpi.first.wpilibj.XboxController.Button.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Pneumatics
  //private Compressor compressor = new Compressor(0);
  // todo : uncomment when compressor added

  // Controllers
  private Supplier<Boolean> invertControls = () -> false;

  // ShuffleBoard
  private final ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");
  private final ShuffleboardTab settingsTab = Shuffleboard.getTab("Settings");
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final ShuffleboardTab colorWheelTab = Shuffleboard.getTab("Control Panel");
  private final ShuffleboardTab testTab = Shuffleboard.getTab("Testing");
  private final ShuffleboardTab verificationTab = Shuffleboard.getTab("Verification");
  private final ShuffleboardTab simulationTab = Shuffleboard.getTab("Simulation");

  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain(testTab, simulationTab, autonomousTab);
  private final Intake intake = new Intake();
  private final VerticalHopper hopper = new VerticalHopper(intake.intakeSensor, driverTab, settingsTab);
  private final Shooter shooter = new Shooter(settingsTab, driverTab);
  private final ColorWheel colorWheel = new ColorWheel(colorWheelTab);
  //private final Gate gate = new Gate();
  private final CameraSystem cameraSystem = new CameraSystem(driverTab);
  private final VisionSystem visionSystem = new VisionSystem(driverTab, autonomousTab, driveTrain.getFieldSim());
  private final Winch winch = new Winch();
  private final HookElevator hookElevator = new HookElevator();
  private final VerificationSystem verificationSystem = new VerificationSystem(
          driveTrain, intake, hopper, shooter, colorWheel, cameraSystem, visionSystem, winch, hookElevator,
          verificationTab);

  private final JoystickDriveCommand driveCommand = new JoystickDriveCommand(driveTrain);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, hopper);
  private final ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intake);
  private final MoveHopperUpCommand moveHopperUpCommand = new MoveHopperUpCommand(hopper);
  private final MoveHopperDownCommand moveHopperDownCommand = new MoveHopperDownCommand(hopper);
  //private final ShooterForwardCommand shooterForwardCommand = new ShooterForwardCommand(shooter);
  private final ShooterBackwardsCommand shooterBackwardsCommand = new ShooterBackwardsCommand(shooter);
  private final ShooterHoldVelocityCommand shooterHoldVelocityViaVisionCommand =
          //new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.VISION, true);
  new ShooterHoldVelocityCommand(shooter, visionSystem, ShooterHoldVelocityCommand.RPMSource.DRIVER_PROVIDED, true);
          //private final ToggleGateCommand toggleGateCommand = new ToggleGateCommand(gate);
  private final ToggleCameraCommand toggleCameraCommand = new ToggleCameraCommand(cameraSystem);
  private final UseShooterCameraCommand useShooterCameraCommand = new UseShooterCameraCommand(cameraSystem);
  private final UseIntakeCameraCommand useIntakeCameraCommand = new UseIntakeCameraCommand(cameraSystem);
  private final VisionAlignCommand visionAlignCommand = new VisionAlignCommand(driveTrain, visionSystem, true, true);
  private final AutoHopperMoveInCommand autoHopperMoveInCommand = new AutoHopperMoveInCommand(hopper);
  private final WinchClimbCommand winchClimbCommand = new WinchClimbCommand(winch);
  private final MoveHookUpCommand hookUpCommand = new MoveHookUpCommand(hookElevator);
  private final MoveHookDownCommand hookDownCommand = new MoveHookDownCommand(hookElevator);

  private SendableChooser<Command> autoChooser;
  private NetworkTableEntry resetOdometryOnAuto;

  private boolean inAutonomous = true;

  private boolean drivingFromHome = false;
  private boolean drivingFromHomeInverted = false;

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Subsystems
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(intake);
    SmartDashboard.putData(hopper);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(colorWheel);
    //SmartDashboard.putData(gate);
    SmartDashboard.putData(cameraSystem);
    SmartDashboard.putData(visionSystem);
    SmartDashboard.putData(winch);
    SmartDashboard.putData(hookElevator);
    SmartDashboard.putData(verificationSystem);

    // Configure the button bindings
    configureButtonBindings();

    // Default Commands
    driveTrain.setDefaultCommand(driveCommand);
    hopper.setDefaultCommand(autoHopperMoveInCommand);

    // ShuffleBoard
    setupDriverTab();
    setupAutonomousTab();
    setupTestingTab();
  }

  private void setupTestingTab() {
    testTab.add(toggleCameraCommand);
  }

  private void setupAutonomousTab() {
    autoChooser = new SendableChooser<>();
    SendableRegistry.setName(autoChooser, "Autonomous Command");
    // = Do Nothing
    autoChooser.setDefaultOption("Nothing", null);

    autoChooser.addOption("Drive Straight (2m)", new DriveStraightXCommand(driveTrain, 2.0));

    autoChooser.addOption("AutoNav - Barrel", new BarrelAutoCommand(driveTrain));
    autoChooser.addOption("AutoNav - Slalom", new SlalomAutoCommand(driveTrain));
    autoChooser.addOption("AutoNav - Bounce", new BounceAutoCommand(driveTrain));

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

    resetOdometryOnAuto = autonomousTab.add("Reset Odometry on Auto", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    autonomousTab.add(intakeCommand);
  }

  private void setupDriverTab() {
    /*SendableRegistry.add(hopper.getSendable(), "VerticalHopper");

    driverTab.add(hopper.getSendable())
    .withWidget("VerticalHopper")
    .withPosition(0,0)
    .withSize(3, 2);*/
    GameTimer gameTimer = new GameTimer(this);
    SendableRegistry.add(gameTimer, "GameTimer");
    driverTab.add(gameTimer)
    .withWidget("GameTimer")
    .withProperties(Map.of("Font Color", "black"))
    .withPosition(7, 0)
    .withSize(2, 1);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drivingFromHome = DriverStation.getInstance().getJoystickIsXbox(0);

    XboxController xboxController = new XboxController(drivingFromHome ? 0 : 1);
    visionSystem.setXboxController(xboxController);
    if (!drivingFromHome) {
      Joystick driveStick = new Joystick(0);
      // === MAIN DRIVER - Logitech Extreme 3D Pro

      // Note: Trigger (button 1) is used in JoystickDriveCommand for quick turn

      JoystickButton pointerButton = new JoystickButton(driveStick, LogitechButton.POINTER);
      pointerButton.whenPressed(toggleCameraCommand);

      invertControls = () -> driveStick.getRawAxis(3) <= .5; // Invert controls when throttle is negative value

      // Drive with Logitech
      driveCommand.setYControllerSupplier(() -> -driveStick.getY(Hand.kLeft));
      driveCommand.setXControllerSupplier(() -> driveStick.getRawAxis(2));
    } else {
      // Drive with Xbox Controller
      driveCommand.setYControllerSupplier(() -> -xboxController.getY(Hand.kLeft));
      driveCommand.setXControllerSupplier(() -> xboxController.getX(Hand.kRight));

      // Invert controls via START button
      JoystickButton startButton = new JoystickButton(xboxController, kStart.value);
      startButton.whileActiveOnce(new InstantCommand(new Runnable() {
        @Override
        public void run() {
          System.out.println("Inverted to " + !drivingFromHomeInverted);
          drivingFromHomeInverted = !drivingFromHomeInverted;
        }
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
    xButton.whileActiveOnce(shooterHoldVelocityViaVisionCommand);

    JoystickButton bButton = new JoystickButton(xboxController, kB.value);
    bButton.whileActiveOnce(shooterBackwardsCommand);

    JoystickButton aButton = new JoystickButton(xboxController, kA.value);
    aButton.whileActiveOnce(visionAlignCommand);

    POVButton upPOVButton = new POVButton(xboxController, 0);
    upPOVButton.whileActiveOnce(winchClimbCommand);

    //POVButton downPOVButton = new POVButton(xboxController, 180);

    POVButton leftPOVButton = new POVButton(xboxController, 270);
    leftPOVButton.whileActiveOnce(hookDownCommand);

    POVButton rightPOVButton = new POVButton(xboxController, 90);
    rightPOVButton.whileActiveOnce(hookUpCommand);

    //todo : bind ball intake align to button
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (autoChooser == null ? null : autoChooser.getSelected());
    //return m_autoCommand;
  }

  public void simulationPeriodic() {
    double current = driveTrain.getSimDrawnCurrentAmps(); // Add other power loads here...
    RoboRioSim.setVInVoltage(Math.min(BatterySim.calculateDefaultBatteryLoadedVoltage(current), 12.0));
  }
}
