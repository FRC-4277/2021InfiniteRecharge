/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Shooter.Characteristics.*;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements VerifiableSystem {
  //public static final double FORWARDS_SPEED = 0.5;
  //public static final double BACKWARDS_SPEED = -0.5; 
  
  //private ShuffleboardTab settingsTab;
  private TalonSRX leftMotor = new TalonSRX(LEFT_MOTOR_ID);
  private TalonSRX rightMotor = new TalonSRX(RIGHT_MOTOR_ID);

  private NetworkTableEntry shooterSpeedEntry;
  private NetworkTableEntry shooterLeftRPMEntry;
  private NetworkTableEntry shooterRightRPMEntry;
  private NetworkTableEntry shooterDesiredRPMEntry;
  private NetworkTableEntry shooterReachedRPMEntry;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerRotation);

  /**
   * Creates a new Shooter.
   */
  public Shooter(ShuffleboardTab settingsTab, ShuffleboardTab driverTab) {
    //this.settingsTab = settingsTab;
    leftMotor.configFactoryDefault();
    leftMotor.setInverted(LEFT_MOTOR_INVERTED);
    leftMotor.setSensorPhase(LEFT_SENSOR_PHASE);
    rightMotor.configFactoryDefault();
    rightMotor.setInverted(RIGHT_MOTOR_INVERTED);
    rightMotor.setSensorPhase(RIGHT_SENSOR_PHASE);

    leftMotor.config_kP(0, P);
    leftMotor.config_kI(0, I);
    leftMotor.config_kD(0, D);
    leftMotor.configAllowableClosedloopError(0, 0, 10);
    rightMotor.config_kP(0, P);
    rightMotor.config_kI(0, I);
    rightMotor.config_kD(0, D);
    rightMotor.configAllowableClosedloopError(0, 0, 10);

    ShuffleboardLayout layout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid)
    .withSize(4, 1)
    .withPosition(6, 2)
    .withProperties(
      Map.of(
    "Label position", "TOP",
    "Number of columns", 5,
    "Number of rows", 1
      )
    );

    // ShuffleBoard
    shooterSpeedEntry = settingsTab
    .add("Shooter Speed", 0.75)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();

    shooterLeftRPMEntry = layout
    .add("Left RPM", -1)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

    shooterRightRPMEntry = layout
    .add("Right RPM", -1)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

    shooterDesiredRPMEntry = layout
    .add("Desired RPM", 4000.0)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

    shooterReachedRPMEntry = layout
    .add("Has Reached", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  }

  public void runForward() {
    leftMotor.set(ControlMode.PercentOutput, shooterSpeedEntry.getValue().getDouble()); 
    rightMotor.set(ControlMode.PercentOutput, shooterSpeedEntry.getValue().getDouble()); 
  }

  public void holdVelocityRPM(int rpm) {
    holdVelocity(rpmToTicksPerDs(rpm));
  }

  public void holdVelocity(int ticksPerDs) {
    double rpm = ticksPerDsToRPM(ticksPerDs);
    int rps = (int) Math.round(rpm / 60d);
    double feedForwardVolts = feedforward.calculate(rps);
    double feedForwardNormalized = feedForwardVolts / MAX_BATTERY_V;
    leftMotor.set(ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedForwardNormalized);
    rightMotor.set(ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedForwardNormalized);
  }

  public double getVelocity() {
    return (leftMotor.getSelectedSensorVelocity() + rightMotor.getSelectedSensorVelocity()) / 2;
  }

  public double getVelocityRPM() {
    return ticksPerDsToRPM(getVelocity());
  }

  public double getLeftRPM() {
    return ticksPerDsToRPM(leftMotor.getSelectedSensorVelocity());
  }

  public double getRightRPM() {
    return ticksPerDsToRPM(rightMotor.getSelectedSensorVelocity());
  }

  public boolean hasReachedRPM(int rpm) {
    return Math.abs(getVelocityRPM() - rpm) <= RPM_THRESHOLD;
  }

  public void setReachedRPMDisplay(boolean reached) {
    shooterReachedRPMEntry.setBoolean(reached);
  }

  public void runBackwards() {
    leftMotor.set(ControlMode.PercentOutput, -shooterSpeedEntry.getValue().getDouble());
    rightMotor.set(ControlMode.PercentOutput, -shooterSpeedEntry.getValue().getDouble());
  }

  public void stopShooter() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  public int getDriverDesiredRPM() {
    return Double.valueOf(shooterDesiredRPMEntry.getDouble(0.0)).intValue();
  }

  public int rpmToTicksPerDs(int rpm) {
    double ticksPerMinute = rpm * (double) TICKS_PER_REV;
    double ticksPerS = ticksPerMinute / 60d;
    double ticksPerDs = ticksPerS / 10d;
    return (int) Math.round(ticksPerDs);
  }

  public double ticksPerDsToRPM(double ticksPerDs) {
    double ticksPerS = ticksPerDs * 10d;
    double ticksPerMinute = ticksPerS * 60d;
    double rpm = ticksPerMinute / (double) TICKS_PER_REV;
    return rpm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterLeftRPMEntry.setDouble(getLeftRPM());
    shooterRightRPMEntry.setDouble(getRightRPM());
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }
}
