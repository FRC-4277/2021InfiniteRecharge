/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;

import java.util.Map;

public class Shooter extends SubsystemBase {
  //public static final double FORWARDS_SPEED = 0.5;
  //public static final double BACKWARDS_SPEED = -0.5; 
  
  private ShuffleboardTab settingsTab;
  private TalonSRX topMotor = new TalonSRX(TOP_MOTOR_ID);
  private TalonSRX bottomMotor = new TalonSRX(BOTTOM_MOTOR_ID);

  private NetworkTableEntry shooterSpeedEntry;

  /**
   * Creates a new Shooter.
   */
  public Shooter(ShuffleboardTab settingsTab) {
    this.settingsTab = settingsTab;
    topMotor.configFactoryDefault();
    bottomMotor.configFactoryDefault();
    bottomMotor.follow(topMotor);
    bottomMotor.setInverted(InvertType.OpposeMaster);

    // ShuffleBoard
    shooterSpeedEntry = settingsTab
    .add("Shooter Speed", 0.5)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();
  }

  public void runForward() {
    topMotor.set(ControlMode.PercentOutput, shooterSpeedEntry.getValue().getDouble()); 
  }

  public boolean rampUp(int ticksPer100ms) {
    topMotor.set(ControlMode.Velocity, ticksPer100ms);
    return true;
  }

  public void runBackwards() {
    topMotor.set(ControlMode.PercentOutput, -shooterSpeedEntry.getValue().getDouble());
  }

  public void stopShooter() {
    topMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
