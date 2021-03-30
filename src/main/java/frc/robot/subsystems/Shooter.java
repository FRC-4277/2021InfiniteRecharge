/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Shooter.Characteristics.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.EntryListenerFlags;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterHoldVelocityCommand.RPMSource;
import java.util.List;
import java.util.Map;

public class Shooter extends SubsystemBase implements VerifiableSystem {
  // public static final double FORWARDS_SPEED = 0.5;
  // public static final double BACKWARDS_SPEED = -0.5;

  // private ShuffleboardTab settingsTab;
  private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(LEFT_MOTOR_ID);
  private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(RIGHT_MOTOR_ID);

  private final NetworkTableEntry shooterSpeedEntry;
  private final NetworkTableEntry shooterRPMDisplayEntry;
  private final NetworkTableEntry shooterDesiredRPMEntry;
  private final NetworkTableEntry shooterReachedRPMEntry;
  private final SendableChooser<RPMSource> rpmSourceSendableChooser;
  private final SendableChooser<ShootingMode> shootingModeChooser;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ksVolts, kvVoltRotationsPerSecond);

  private FlywheelSim flywheelSim;
  private TalonSRXSimCollection leftTalonSim, rightTalonSim;

  /** Creates a new Shooter. */
  public Shooter(ShuffleboardTab settingsTab, ShuffleboardTab driverTab) {
    // this.settingsTab = settingsTab;
    leftMotor.configFactoryDefault();
    leftMotor.setInverted(LEFT_MOTOR_INVERTED);
    leftMotor.setSensorPhase(LEFT_SENSOR_PHASE);
    rightMotor.configFactoryDefault();
    rightMotor.setInverted(RIGHT_MOTOR_INVERTED);
    // rightMotor.follow(leftMotor);
    rightMotor.setSensorPhase(RIGHT_SENSOR_PHASE);

    setupCANStatusFrames(leftMotor);
    setupCANStatusFrames(rightMotor);

    if (RobotBase.isReal()) {
      double iZoneNative = rpmToTicksPerDs(I_ZONE_RPM); // Convert RPM to μ/100ms
      double iMaxAccumulatorNative = rpmToTicksPerDs(I_MAX_ACCUMULATOR_RPM);
      leftMotor.config_kP(0, P);
      leftMotor.config_kI(0, I);
      leftMotor.config_IntegralZone(0, iZoneNative);
      leftMotor.configMaxIntegralAccumulator(0, iMaxAccumulatorNative);
      leftMotor.config_kD(0, D);
      leftMotor.configAllowableClosedloopError(0, 0, 10);
      rightMotor.config_kP(0, P);
      rightMotor.config_kI(0, I);
      rightMotor.config_IntegralZone(0, iZoneNative);
      rightMotor.configMaxIntegralAccumulator(0, iMaxAccumulatorNative);
      rightMotor.config_kD(0, D);
      rightMotor.configAllowableClosedloopError(0, 0, 10);
    }

    ShuffleboardLayout layout =
        driverTab
            .getLayout("Shooter", BuiltInLayouts.kGrid)
            .withSize(5, 1)
            .withPosition(6, 2)
            .withProperties(
                Map.of(
                    "Label position", "TOP",
                    "Number of columns", 4,
                    "Number of rows", 1));

    // ShuffleBoard
    shooterSpeedEntry =
        settingsTab
            .add("Shooter Speed (do not use)", 0.75)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();

    shooterRPMDisplayEntry = layout.add("RPM", "").withWidget(BuiltInWidgets.kTextView).getEntry();

    shooterDesiredRPMEntry =
        layout.add("Desired RPM", 4000.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    shooterReachedRPMEntry =
        layout.add("At Target RPM", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    rpmSourceSendableChooser = new SendableChooser<>();
    rpmSourceSendableChooser.setDefaultOption("Vision", RPMSource.VISION);
    rpmSourceSendableChooser.addOption("Manual", RPMSource.DRIVER_PROVIDED);
    SendableRegistry.setName(rpmSourceSendableChooser, "Source");
    layout.add(rpmSourceSendableChooser);

    shootingModeChooser = new SendableChooser<>();
    shootingModeChooser.setDefaultOption(
        "Interstellar Accuracy", ShootingMode.INTERSTELLAR_ACCURACY);
    shootingModeChooser.addOption("Power Port Challenge", ShootingMode.POWER_PORT_CHALLENGE);
    SendableRegistry.setName(shootingModeChooser, "Shooting Mode");
    settingsTab.add(shootingModeChooser).withSize(2, 1).withPosition(2, 1);

    ShuffleboardLayout pidLayout =
        settingsTab
            .getLayout("Shooter PID", BuiltInLayouts.kGrid)
            .withSize(5, 1)
            .withPosition(4, 1)
            .withProperties(
                Map.of(
                    "Label position", "TOP",
                    "Number of columns", 5,
                    "Number of rows", 1));

    // Adjust PID from shuffleboard
    NetworkTableEntry shooterP =
        pidLayout.add("P", P).withWidget(BuiltInWidgets.kTextView).getEntry();
    shooterP.addListener(
        notification -> {
          leftMotor.config_kP(0, notification.value.getDouble());
          rightMotor.config_kP(0, notification.value.getDouble());
        },
        EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);

    NetworkTableEntry shooterI =
        pidLayout.add("I", I).withWidget(BuiltInWidgets.kTextView).getEntry();
    shooterI.addListener(
        notification -> {
          leftMotor.config_kI(0, notification.value.getDouble());
          rightMotor.config_kI(0, notification.value.getDouble());
        },
        EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);

    NetworkTableEntry shooterIZone =
        pidLayout.add("I Zone (RPM)", I_ZONE_RPM).withWidget(BuiltInWidgets.kTextView).getEntry();
    shooterIZone.addListener(
        notification -> {
          double setting = rpmToTicksPerDs(notification.value.getDouble());
          leftMotor.config_IntegralZone(0, setting);
          rightMotor.config_IntegralZone(0, setting);
        },
        EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);

    NetworkTableEntry shooterIMax =
        pidLayout
            .add("I Accum Max (RPM)", I_MAX_ACCUMULATOR_RPM)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    shooterIMax.addListener(
        notification -> {
          double setting = rpmToTicksPerDs(notification.value.getDouble());
          leftMotor.configMaxIntegralAccumulator(0, setting);
          rightMotor.configMaxIntegralAccumulator(0, setting);
        },
        EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);

    NetworkTableEntry shooterD =
        pidLayout.add("D", D).withWidget(BuiltInWidgets.kTextView).getEntry();
    shooterD.addListener(
        notification -> {
          leftMotor.config_kD(0, notification.value.getDouble());
          rightMotor.config_kD(0, notification.value.getDouble());
        },
        EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);

    // Simulation
    if (RobotBase.isSimulation()) {
      flywheelSim = new FlywheelSim(PLANT, DCMotor.getVex775Pro(1), 4);
      leftTalonSim = leftMotor.getSimCollection();
      rightTalonSim = rightMotor.getSimCollection();
    }
  }

  private void setupCANStatusFrames(WPI_TalonSRX motor) {
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_2_FEEDBACK_MS);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, STATUS_3_QUADRATURE_MS);
  }

  public RPMSource getSelectedRPMSource() {
    return rpmSourceSendableChooser.getSelected();
  }

  public void runForward() {
    leftMotor.set(ControlMode.PercentOutput, shooterSpeedEntry.getValue().getDouble());
    rightMotor.set(ControlMode.PercentOutput, shooterSpeedEntry.getValue().getDouble());
  }

  public ShootingMode getShootingMode() {
    return shootingModeChooser.getSelected();
  }

  public void holdVelocityRPM(double rpm) {
    shooterDesiredRPMEntry.setDouble(rpm);
    holdVelocity(rpmToTicksPerDs(rpm), rpm);
  }

  private void holdVelocity(double ticksPerDs, double equivalentRPM) {
    double rpm = equivalentRPM;
    double rps = rpm / 60d;
    double feedForwardVolts = feedforward.calculate(rps);
    double feedForwardNormalized = feedForwardVolts / MAX_BATTERY_V;
    leftMotor.set(
        ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedForwardNormalized);
    rightMotor.set(
        ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedForwardNormalized);
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
    return Math.abs(getVelocityRPM() - rpm)
        <= (getShootingMode() == ShootingMode.INTERSTELLAR_ACCURACY
            ? RPM_THRESHOLD_INTERSTELLAR_ACCURACY
            : RPM_THRESHOLD_POWER_PORT_CHALLENGE);
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

  public double rpmToTicksPerDs(double rpm) {
    double ticksPerMinute = rpm * (double) TICKS_PER_REV;
    double ticksPerS = ticksPerMinute / 60d;
    double ticksPerDs = ticksPerS / 10d;
    return ticksPerDs;
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
    // shooterRPMDisplayEntry.setString(getLeftRPM() + " | " + getRightRPM());
    shooterRPMDisplayEntry.setString(String.format("%.2f | %.2f", getLeftRPM(), getRightRPM()));
    SmartDashboard.putNumber("Left RPM", getLeftRPM());
    SmartDashboard.putNumber("Right RPM", getRightRPM());
  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.setInputVoltage(leftMotor.getMotorOutputVoltage());
    // System.out.println("Left Motor Voltage: " + leftMotor.getMotorOutputVoltage());
    flywheelSim.update(0.020);
    // System.out.println("rad/s from sim: " + flywheelSim.getAngularVelocityRadPerSec());
    double rpm = flywheelSim.getAngularVelocityRPM();
    // System.out.println("RPM from sim: " + rpm);
    double ticksPerDs = rpmToTicksPerDs(rpm);

    leftTalonSim.setQuadratureVelocity((int) ticksPerDs);
    rightTalonSim.setQuadratureVelocity((int) -ticksPerDs);
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }

  public enum ShootingMode {
    INTERSTELLAR_ACCURACY,
    POWER_PORT_CHALLENGE
  }
}
