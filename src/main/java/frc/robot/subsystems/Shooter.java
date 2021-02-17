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
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterHoldVelocityCommand.RPMSource;

public class Shooter extends SubsystemBase implements VerifiableSystem {
  //public static final double FORWARDS_SPEED = 0.5;
  //public static final double BACKWARDS_SPEED = -0.5; 
  
  //private ShuffleboardTab settingsTab;
  private WPI_TalonSRX leftMotor = new WPI_TalonSRX(LEFT_MOTOR_ID);
  private WPI_TalonSRX rightMotor = new WPI_TalonSRX(RIGHT_MOTOR_ID);

  private NetworkTableEntry shooterSpeedEntry;
  private NetworkTableEntry shooterRPMDisplayEntry;
  private NetworkTableEntry shooterDesiredRPMEntry;
  private NetworkTableEntry shooterReachedRPMEntry;
  private SendableChooser<RPMSource> rpmSourceSendableChooser;
  private NetworkTableEntry shooterDesiredRPMSourceEntry;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ksVolts, kvVoltRotationsPerSecond);

  //private FlywheelSim flywheelSim;
  private TalonSRXSimCollection leftTalonSim, rightTalonSim;

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
    .withSize(5, 1)
    .withPosition(6, 2)
    .withProperties(
      Map.of(
    "Label position", "TOP",
    "Number of columns", 4,
    "Number of rows", 1
      )
    );

    // ShuffleBoard
    shooterSpeedEntry = settingsTab
    .add("Shooter Speed (do not use)", 0.75)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();

    shooterRPMDisplayEntry = layout
    .add("RPM", "")
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

    shooterDesiredRPMEntry = layout
    .add("Desired RPM", 4000.0)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

    shooterReachedRPMEntry = layout
    .add("At Target RPM", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

    rpmSourceSendableChooser = new SendableChooser<>();
    rpmSourceSendableChooser.setDefaultOption("Vision", RPMSource.VISION);
    rpmSourceSendableChooser.addOption("Manual", RPMSource.DRIVER_PROVIDED);
    SendableRegistry.setName(rpmSourceSendableChooser, "Source");
    layout.add(rpmSourceSendableChooser);

    // Simulation
    if (RobotBase.isSimulation()) {
       /*flywheelSim = new FlywheelSim(
          PLANT,
          DCMotor.getVex775Pro(1),
          4
       );*/
       leftTalonSim = leftMotor.getSimCollection();
       rightTalonSim = rightMotor.getSimCollection();
    }
  }

  public RPMSource getSelectedRPMSource() {
    return rpmSourceSendableChooser.getSelected();
  }

  public void runForward() {
    leftMotor.set(ControlMode.PercentOutput, shooterSpeedEntry.getValue().getDouble()); 
    rightMotor.set(ControlMode.PercentOutput, shooterSpeedEntry.getValue().getDouble()); 
  }

  public void holdVelocityRPM(double rpm) {
    shooterDesiredRPMEntry.setDouble(rpm);
    holdVelocity(rpmToTicksPerDs(rpm));
  }

  public void holdVelocity(int ticksPerDs) {
    double rpm = ticksPerDsToRPM(ticksPerDs);
    double rps = rpm / 60d;
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

  public boolean hasReachedRPM(double rpm) {
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

  public double getDriverDesiredRPM() {
    return shooterDesiredRPMEntry.getDouble(0.0);
  }

  public int rpmToTicksPerDs(double rpm) {
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
    shooterRPMDisplayEntry.setString(getLeftRPM() + " | " + getRightRPM());
  }

  @Override
  public void simulationPeriodic() {
    //flywheelSim.setInputVoltage(leftMotor.getMotorOutputVoltage());
    //System.out.println("Left Motor Voltage: " + leftMotor.getMotorOutputVoltage());
    //flywheelSim.update(0.020);
    //System.out.println("rad/s from sim: " + flywheelSim.getAngularVelocityRadPerSec());
    //double rpm = flywheelSim.getAngularVelocityRPM();
    //System.out.println("RPM from sim: " + rpm);
    //int ticksPerDs = rpmToTicksPerDs(rpm);
    double rotationsPerSecond = leftMotor.getMotorOutputVoltage() / kvVoltRotationsPerSecond;
    double rotationsPerMinute = rotationsPerSecond * 60;
    int ticksPerDs = rpmToTicksPerDs(rotationsPerMinute);
    leftTalonSim.setQuadratureVelocity(ticksPerDs);
    rightTalonSim.setQuadratureVelocity(-ticksPerDs);
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }
}
