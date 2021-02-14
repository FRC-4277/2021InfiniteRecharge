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
  //private static final double TURN_DEADBAND = 0.1;

  private DriveTrain driveTrain;
  private Supplier<Boolean> invertControls = () -> false;
  private Supplier<Double> yControllerSupplier, xControllerSupplier;


  /**
   * Creates a new JoystickDriveCommand.
   */
  public JoystickDriveCommand(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (yControllerSupplier == null || xControllerSupplier == null) {
      return;
    }
    // y is inverted on Xbox Controller
    //double y = -controller.getY(Hand.kLeft); // Forward/backwards (axis inverted)
    double y = yControllerSupplier.get();
    //double x = controller.getRawAxis(2); // Logitech Twist
    double x = xControllerSupplier.get();

    /*if (Math.abs(z) >= TURN_DEADBAND) {
      x = z;
    }*/

    if (invertControls.get()) {
      y *= -1;
    }

    driveTrain.joystickDrive(y, x);
  }

  public void setYControllerSupplier(Supplier<Double> yControllerSupplier) {
    this.yControllerSupplier = yControllerSupplier;
  }

  public void setXControllerSupplier(Supplier<Double> xControllerSupplier) {
    this.xControllerSupplier = xControllerSupplier;
  }

  public void setInvertControls(Supplier<Boolean> invertControls) {
    this.invertControls = invertControls;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
