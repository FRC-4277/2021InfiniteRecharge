/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.VerticalHopper.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class VerticalHopper extends SubsystemBase {
  public static final double UP_SPEED = 0.5;
  public static final double DOWN_SPEED = -0.5;
  private TalonSRX motor = new TalonSRX(MOTOR_ID);
  
  /**
   * Creates a new VerticalHopper.
   */
  public VerticalHopper() {
    motor.configFactoryDefault();
    motor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveUp() {
    motor.set(ControlMode.PercentOutput, UP_SPEED);
  }

  public void moveDown() {
    motor.set(ControlMode.PercentOutput, DOWN_SPEED);
  }

  public void stopMoving() {
    motor.set(ControlMode.PercentOutput, 0);
  }
}
