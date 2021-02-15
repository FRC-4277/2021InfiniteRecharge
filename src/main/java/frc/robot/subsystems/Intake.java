/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/ 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import java.util.List;

public class Intake extends SubsystemBase implements VerifiableSystem {
  private VictorSPX motor = new WPI_VictorSPX(MOTOR_ID);
  private static final double INTAKE_SPEED = .5;
  private static final double REVERSE_INTAKE_SPEED = -0.5;
  public DigitalInput intakeSensor = new DigitalInput(INTAKE_SENSOR);

  /**
   * Creates a new Intake.
   */
  public Intake() {
    motor.configFactoryDefault();
    motor.setInverted(MOTOR_INVERTED);
  }

  public void runIntake() {
    motor.set(ControlMode.PercentOutput, INTAKE_SPEED);
  }

  public void runReverseIntake() {
    motor.set(ControlMode.PercentOutput, REVERSE_INTAKE_SPEED);
  }

  public void stopIntake() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }
}
