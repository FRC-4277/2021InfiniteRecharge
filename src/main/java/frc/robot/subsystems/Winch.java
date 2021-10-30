/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.Winch.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Winch extends SubsystemBase implements VerifiableSystem {
  private static final double DEFAULT_CLIMB_SPEED = 0.9;
  private TalonSRX motor = new TalonSRX(MAIN_MOTOR_ID);

  /** Creates a new Winch. */
  public Winch() {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(MOTOR_INVERTED);
  }

  public void climb() {
    climb(DEFAULT_CLIMB_SPEED);
  }

  public void climb(double speed) {
    speed = Math.abs(speed); // Make sure we never go backwards as the gearbox has a ratchet
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void reverseClimber() {
    motor.set(ControlMode.PercentOutput, -DEFAULT_CLIMB_SPEED);
  }

  public void stop() {
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
