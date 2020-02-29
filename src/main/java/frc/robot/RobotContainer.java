/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.GameTimer;
import frc.robot.util.LogitechButton;
import frc.robot.util.XboxTrigger;

import java.util.function.Supplier;

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
  private Joystick driveStick = new Joystick(0);
  private Supplier<Boolean> invertControls = () -> driveStick.getRawAxis(4) <= .5; // Throttle is negative = invert
  private XboxController xboxController = new XboxController(1);

  // ShuffleBoard
  private final ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");
  private final ShuffleboardTab settingsTab = Shuffleboard.getTab("Settings");
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final ShuffleboardTab colorWheelTab = Shuffleboard.getTab("Control Panel");

  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final VerticalHopper hopper = new VerticalHopper(intake.intakeSensor, driverTab);
  private final Shooter shooter = new Shooter(settingsTab, driverTab);
  private final ColorWheel colorWheel = new ColorWheel(colorWheelTab);
  //private final Gate gate = new Gate();
  private final CameraSystem cameraSystem = new CameraSystem(driverTab);
  private final VisionSystem visionSystem = new VisionSystem(driverTab);

  private final JoystickDriveCommand driveCommand = new JoystickDriveCommand(driveTrain, driveStick, invertControls);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, hopper);
  private final ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intake);
  private final MoveHopperUpCommand moveHopperUpCommand = new MoveHopperUpCommand(hopper);
  private final MoveHopperDownCommand moveHopperDownCommand = new MoveHopperDownCommand(hopper);
  private final ShooterForwardCommand shooterForwardCommand = new ShooterForwardCommand(shooter);
  private final ShooterBackwardsCommand shooterBackwardsCommand = new ShooterBackwardsCommand(shooter);
  //private final ToggleGateCommand toggleGateCommand = new ToggleGateCommand(gate);
  private final ToggleCameraCommand toggleCameraCommand = new ToggleCameraCommand(cameraSystem);
  private final VisionAlignCommand visionAlignCommand = new VisionAlignCommand(driveTrain, visionSystem);
  private final AutoHopperMoveInCommand autoHopperMoveInCommand = new AutoHopperMoveInCommand(hopper);

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Pose2d> startingPositionChooser;

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

    // Configure the button bindings
    configureButtonBindings();

    // Default Commands
    driveTrain.setDefaultCommand(driveCommand);
    hopper.setDefaultCommand(autoHopperMoveInCommand);

    // ShuffleBoard
    setupDriverTab();

    autoChooser = new SendableChooser<>();
    SendableRegistry.setName(autoChooser, "Autonomous Command");
    autoChooser.setDefaultOption("Nothing", null);
    autoChooser.setDefaultOption("Forward To Switch",
            driveTrain.generateRamseteCommand(driveTrain.generateTrajectory("Forward To Switch")));
    autonomousTab.add(autoChooser).withPosition(0, 0).withSize(2, 1);

    startingPositionChooser = new SendableChooser<>();
    SendableRegistry.setName(startingPositionChooser, "Starting Position");
    startingPositionChooser.setDefaultOption("?", null);
    autonomousTab.add(startingPositionChooser).withPosition(2, 0).withSize(2, 1);
  }

  private void setupDriverTab() {
    /*SendableRegistry.add(hopper.getSendable(), "VerticalHopper");

    driverTab.add(hopper.getSendable())
    .withWidget("VerticalHopper")
    .withPosition(0,0)
    .withSize(3, 2);*/
    GameTimer gameTimer = new GameTimer();
    SendableRegistry.add(gameTimer, "GameTimer");
    driverTab.add(gameTimer)
    .withWidget("GameTimer")
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
    // === MAIN DRIVER - Logitech Extreme 3D Pro

    // Note: Trigger (button 1) is used in JoystickDriveCommand for quick turn

    JoystickButton pointerButton = new JoystickButton(driveStick, LogitechButton.POINTER);
    pointerButton.whenPressed(toggleCameraCommand);

    Trigger invertControlsThrottle = new Trigger(invertControls::get);
    // Automatically switch camera when drive is inverted/normal
    invertControlsThrottle.whenActive(new UseShooterCameraCommand(cameraSystem));
    invertControlsThrottle.whenInactive(new UseIntakeCameraCommand(cameraSystem));


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
    xButton.whileActiveOnce(shooterForwardCommand);

    JoystickButton bButton = new JoystickButton(xboxController, kB.value);
    bButton.whileActiveOnce(shooterBackwardsCommand);

    JoystickButton aButton = new JoystickButton(xboxController, kA.value);
    aButton.whileActiveOnce(visionAlignCommand);
  }

  protected void switchToDriverView() {
    Shuffleboard.selectTab("Driver");
  }

  public void autonomousInit() {
    driveTrain.resetEncoders();
    driveTrain.zeroHeading();
    System.out.println("DriveTrain's encoders & heading are reset.");
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
}
