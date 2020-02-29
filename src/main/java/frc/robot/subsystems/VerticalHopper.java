/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.VerticalHopper.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VerticalHopper extends SubsystemBase {
  public static final double UP_SPEED = 0.7;
  public static final double DOWN_SPEED = -0.5;
  private VictorSPX leftMotor = new VictorSPX(LEFT_MOTOR_ID);
  private VictorSPX rightMotor = new VictorSPX(RIGHT_MOTOR_ID);

  private boolean gateClosed = true;
  private boolean[] cellsPresent = new boolean[]{false, false, false, false, false};
  private double speedRunning = 0.0;
  private VerticalHopperSendable sendable = new VerticalHopperSendable();

  public DigitalInput topBallSensor = new DigitalInput(0);
  public DigitalInput intakeSensor;

  public ShuffleboardTab driverTab;

  /**
   * Creates a new VerticalHopper.
   */
  public VerticalHopper(DigitalInput intakeSensor, ShuffleboardTab driverTab) {
    this.intakeSensor = intakeSensor;
    this.driverTab = driverTab;
    leftMotor.configFactoryDefault();
    leftMotor.setInverted(LEFT_MOTOR_INVERTED);
    rightMotor.configFactoryDefault();
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.OpposeMaster);
  
    driverTab.addBoolean("Ramp Full", topBallSensor::get).withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(9, 0).withSize(1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(intakeSensor.get());
  }

  public void moveUp() {
    leftMotor.set(ControlMode.PercentOutput, UP_SPEED);
  }

  public void moveDown() {
    leftMotor.set(ControlMode.PercentOutput, DOWN_SPEED);
  }

  public void stopMoving() {
    leftMotor.set(ControlMode.PercentOutput, 0);
  }

  public VerticalHopperSendable getSendable() {
    return sendable;
  }

  public class VerticalHopperSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("VerticalHopper");
      builder.addBooleanProperty("gateClosed", () -> gateClosed, null);
      builder.addBooleanArrayProperty("cellsPresent", () -> cellsPresent, null);
      builder.addDoubleProperty("speedRunning", () -> speedRunning, null);
    }
  }
}
