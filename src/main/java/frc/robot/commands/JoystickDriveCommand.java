/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.LogitechButton;

import java.util.function.Supplier;

public class JoystickDriveCommand extends CommandBase {
  private static final double TURN_DEADBAND = 0.1;

  private DriveTrain driveTrain;
  private Joystick controller;
  private Supplier<Boolean> invertControls;

  /**
   * Creates a new JoystickDriveCommand.
   */
  public JoystickDriveCommand(DriveTrain driveTrain, Joystick controller, Supplier<Boolean> invertControls) {
    this.driveTrain = driveTrain;
    this.controller = controller;
    this.invertControls = invertControls;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // y is inverted on Xbox Controller
    double y = -controller.getY(Hand.kLeft); // Forward/backwards (axis inverted)
    double x = controller.getRawAxis(2); // Logitech Twist
    /*double x = controller.getX(Hand.kRight);
    double z = controller.getRawAxis(2);
    if (Math.abs(z) >= TURN_DEADBAND) {
      x = z;
    }*/

    if (invertControls.get()) {
      y *= -1;
    }

    boolean quickTurn = controller.getRawButton(LogitechButton.TRIGGER);
    driveTrain.joystickDrive(y, x, quickTurn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
