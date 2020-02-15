/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.MoveHopperDownCommand;
import frc.robot.commands.MoveHopperUpCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.ShooterBackwardsCommand;
import frc.robot.commands.ShooterForwardCommand;
import frc.robot.commands.ToggleGateCommand;
import frc.robot.commands.autonomous.LazyRamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.util.XboxTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.Supplier;

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
  private XboxController xboxController = new XboxController(1);

  // ShuffleBoard
  private final ShuffleboardTab settingsTab = Shuffleboard.getTab("Settings");
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final VerticalHopper hopper = new VerticalHopper();
  private final Shooter shooter = new Shooter(settingsTab);
  private final Gate gate = new Gate();

  private final JoystickDriveCommand driveCommand = new JoystickDriveCommand(driveTrain, driveStick);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake);
  private final ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intake);
  private final MoveHopperUpCommand moveHopperUpCommand = new MoveHopperUpCommand(hopper);
  private final MoveHopperDownCommand moveHopperDownCommand = new MoveHopperDownCommand(hopper);
  private final ShooterForwardCommand shooterForwardCommand = new ShooterForwardCommand(shooter);
  private final ShooterBackwardsCommand shooterBackwardsCommand = new ShooterBackwardsCommand(shooter);
  private final ToggleGateCommand toggleGateCommand = new ToggleGateCommand(gate);
  
  private SendableChooser<Command> chooser;

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

    // Configure the button bindings
    configureButtonBindings();

    // Default Commands
    driveTrain.setDefaultCommand(driveCommand);

    // ShuffleBoard
    setupDriverTab();


    if (Constants.DriveTrain.USING_ENCODERS) {
      // Starting position Chooser
      SendableChooser<Pose2d> poseChooser = new SendableChooser<>();
      poseChooser.addDefault("Facing Towards Wall, lined up w/ triangle", new Pose2d(1.0366, -3.7382, Rotation2d.fromDegrees(-180)));

      Supplier<Pose2d> poseSupplier = poseChooser::getSelected;

      // Backwards 4m command
      Command backwards4m = new LazyRamseteCommand(driveTrain, () -> driveTrain.generateStraightTrajectory(poseSupplier.get(), -4));

      // Autonomous Chooser
      chooser = new SendableChooser<>();
      chooser.addDefault("Simple 4m Autonomous Line (facing towards wall)", backwards4m);
    }
  }

  private void setupDriverTab() {
    SendableRegistry.add(hopper.getSendable(), "VerticalHopper");

    driverTab.add(hopper.getSendable())
    .withWidget("VerticalHopper")
    .withPosition(0,0)
    .withSize(3, 2);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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
    aButton.whileActiveOnce(toggleGateCommand);
  }

  protected void switchToDriverView() {
    Shuffleboard.selectTab("Driver");
  }

  public void autonomousInit() {
    driveTrain.zeroGyro();
    // Reset Pose2d
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (chooser == null ? null : chooser.getSelected());
    //return m_autoCommand;
  }
}
