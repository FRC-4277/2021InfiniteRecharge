/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {
  private TalonSRX motor = new TalonSRX(MOTOR_ID);
  private static final double INTAKE_SPEED = .75;
  private static final double REVERSE_INTAKE_SPEED = -0.5;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    motor.setInverted(true);
  }

  public void runIntake() {
    motor.set(ControlMode.PercentOutput, INTAKE_SPEED);
  }

  public void runReverseIntake() {
    motor.set(ControlMode.PercentOutput, REVERSE_INTAKE_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
