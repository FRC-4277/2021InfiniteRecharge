/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.VerticalHopper.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.galacticvideo.GalacticPath;
import java.util.List;
import java.util.Map;

public class VerticalHopper extends SubsystemBase implements VerifiableSystem {
  private static final int DEFAULT_INDEX_RUN_TIME = 300; // ms
  private static final int DEFAULT_INDEX_BETWEEN_TIME = 400; // ms
  private static final int DEFAULT_MOVE_AFTER_SENSOR_DELAY = 300; // ms
  private static final double DEFAULT_MOVE_DISTANCE = 11; // in
  private static final double DEFAULT_POSITION_THRESHOLD = 1.5; // in

  public static final double DOWN_SPEED = -0.5;

  private final NetworkTableEntry UP_SPEED_ENTRY;
  private final NetworkTableEntry INDEX_RUN_TIME_MS_ENTRY;
  private final NetworkTableEntry INDEX_TIME_BETWEEN_BALL_ENTRY;
  private final NetworkTableEntry POSITION_MOVE_AFTER_SENSOR_DELAY;
  private final NetworkTableEntry POSITION_MOVE_DISTANCE;
  private final NetworkTableEntry POSITION_THRESHOLD;
  private final TalonSRX leftMotor = new TalonSRX(LEFT_MOTOR_ID);
  private final TalonSRX rightMotor = new TalonSRX(RIGHT_MOTOR_ID);

  private final RobotContainer robotContainer;

  public DigitalInput topBallSensor = new DigitalInput(0);
  public DigitalInput intakeSensor;

  public ShuffleboardTab driverTab;
  private SendableChooser<SpeedSource> speedSourceChooser;
  private NetworkTableEntry desiredInchesPerSecEntry;

  private int ballCount = 0;

  /** Creates a new VerticalHopper. */
  public VerticalHopper(
      RobotContainer robotContainer,
      DigitalInput intakeSensor,
      ShuffleboardTab driverTab,
      ShuffleboardTab settingsTab) {
    this.robotContainer = robotContainer;
    this.intakeSensor = intakeSensor;
    this.driverTab = driverTab;

    leftMotor.configFactoryDefault();
    leftMotor.setInverted(LEFT_MOTOR_INVERTED);
    leftMotor.setSensorPhase(LEFT_SENSOR_PHASE);
    configureMotor(leftMotor);

    rightMotor.configFactoryDefault();
    // rightMotor.follow(leftMotor);
    // rightMotor.setInverted(InvertType.OpposeMaster);
    rightMotor.setInverted(RIGHT_MOTOR_INVERTED);
    rightMotor.setSensorPhase(RIGHT_SENSOR_PHASE);
    configureMotor(rightMotor);

    UP_SPEED_ENTRY =
        settingsTab
            .add("Hopper Up Speed", 1.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();
    INDEX_RUN_TIME_MS_ENTRY =
        settingsTab
            .add("Index Run Time (ms)", DEFAULT_INDEX_RUN_TIME)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    INDEX_TIME_BETWEEN_BALL_ENTRY =
        settingsTab
            .add("Index Time Between Balls (ms)", DEFAULT_INDEX_BETWEEN_TIME)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    POSITION_MOVE_AFTER_SENSOR_DELAY =
        settingsTab
            .add("PID Hopper Move Delay (ms)", DEFAULT_MOVE_AFTER_SENSOR_DELAY)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    POSITION_MOVE_DISTANCE =
        settingsTab
            .add("PID Hopper Move Distance (in)", DEFAULT_MOVE_DISTANCE)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    POSITION_THRESHOLD =
        settingsTab
            .add("PID Hopper Distance Threshold (in)", DEFAULT_POSITION_THRESHOLD)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    driverTab
        .addBoolean("Ball At Top", this::isBallPresentTop)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(9, 0)
        .withSize(2, 1);

    ShuffleboardLayout layout =
        driverTab
            .getLayout("Hopper", BuiltInLayouts.kGrid)
            .withSize(6, 1)
            .withPosition(6, 3)
            .withProperties(
                Map.of(
                    "Label position", "TOP",
                    "Number of columns", 6,
                    "Number of rows", 1));
    layout
        .addNumber("Left Position (in)", this::getLeftPositionInches)
        .withWidget(BuiltInWidgets.kTextView);
    layout
        .addNumber("Right Position (in)", this::getRightPositionInches)
        .withWidget(BuiltInWidgets.kTextView);
    // Using unicode ∕ to avoid bug in ShuffleBoard where you can't use normal slashes
    layout
        .addNumber("Left Velocity (in s^-1)", this::getLeftVelocityInchesPerS)
        .withWidget(BuiltInWidgets.kTextView);
    layout
        .addNumber("Right Velocity (in s^-1)", this::getRightVelocityInchesPerS)
        .withWidget(BuiltInWidgets.kTextView);
    speedSourceChooser = new SendableChooser<>();
    SendableRegistry.setName(speedSourceChooser, "Speed Source");
    speedSourceChooser.setDefaultOption("Automatic", SpeedSource.AUTOMATIC);
    speedSourceChooser.addOption("Manual", SpeedSource.MANUAL);
    layout.add(speedSourceChooser);
    desiredInchesPerSecEntry =
        layout.add("Speed Target (in s^-1)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  public void addBall() {
    ballCount++;
  }

  public void resetBalls() {
    ballCount = 0;
  }

  public int getBalls() {
    return ballCount;
  }

  public void configureMotor(TalonSRX motor) {
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motor.config_kP(POSITION_SLOT, positionP);
    motor.config_kI(POSITION_SLOT, positionI);
    motor.config_kD(POSITION_SLOT, positionD);
    motor.config_kP(VELOCITY_SLOT, velocityP);
    motor.config_kI(VELOCITY_SLOT, velocityI);
    motor.config_kD(VELOCITY_SLOT, velocityD);
    motor.configMotionCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY);
    motor.configMotionAcceleration(MOTION_MAGIC_CRUISE_ACCELERATION);
    motor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(intakeSensor.get());
  }

  public double pulsesToInches(double pulses) {
    double outputRotations = (pulses / ENCODER_PULSES_PER_REV) / GEARING;
    return outputRotations * PULLEY_CIRCUMFERENCE_IN;
  }

  public double inchesToPulses(double inches) {
    return (inches / PULLEY_CIRCUMFERENCE_IN) * GEARING * ENCODER_PULSES_PER_REV;
  }

  public double pulsesPer100msToInchesPerSecond(double pulsesPer100ms) {
    double pulsesPerSecond = pulsesPer100ms * 10;
    return pulsesToInches(pulsesPerSecond);
  }

  public double inchesPerSecondToPulsesPer100ms(double inchesPerSecond) {
    double inchesPer100ms = inchesPerSecond / 10;
    return inchesToPulses(inchesPer100ms);
  }

  public void setPosition(double positionInches) {
    leftMotor.selectProfileSlot(POSITION_SLOT, 0);
    rightMotor.selectProfileSlot(POSITION_SLOT, 0);
    double pulses = inchesToPulses(positionInches);
    leftMotor.set(ControlMode.Position, pulses);
    rightMotor.set(ControlMode.Position, pulses);
  }

  public void setPositionMotionMagic(double positionInches) {
    leftMotor.selectProfileSlot(POSITION_SLOT, 0);
    rightMotor.selectProfileSlot(POSITION_SLOT, 0);
    double pulses = inchesToPulses(positionInches);
    leftMotor.set(ControlMode.MotionMagic, pulses);
    rightMotor.set(ControlMode.MotionMagic, pulses);
  }

  public double getLeftPositionInches() {
    return pulsesToInches(leftMotor.getSelectedSensorPosition());
  }

  public double getRightPositionInches() {
    return pulsesToInches(rightMotor.getSelectedSensorPosition());
  }

  public double getPositionInches() {
    return (getLeftPositionInches() + getRightPositionInches()) / 2d;
  }

  public void setVelocity(double inchesPerSecond) {
    desiredInchesPerSecEntry.setDouble(inchesPerSecond);
    leftMotor.selectProfileSlot(VELOCITY_SLOT, 0);
    rightMotor.selectProfileSlot(VELOCITY_SLOT, 0);
    double pulsesPer100ms = inchesPerSecondToPulsesPer100ms(inchesPerSecond);
    leftMotor.set(ControlMode.Velocity, pulsesPer100ms);
    rightMotor.set(ControlMode.Velocity, pulsesPer100ms);
  }

  public void moveUpForShooting(double distanceMeters) {
    double inchesPerSecond;
    SpeedSource source = speedSourceChooser.getSelected();
    switch (source) {
      case MANUAL:
        inchesPerSecond = desiredInchesPerSecEntry.getDouble(0);
        break;
      case AUTOMATIC:
        inchesPerSecond = ROBOT_METERS_TO_HOPPER_INCHES_PER_SECOND.apply(distanceMeters);
        break;
      default:
        throw new IllegalStateException("Unknown speed source: " + source);
    }
    setVelocity(inchesPerSecond);
  }

  public double getLeftVelocityInchesPerS() {
    return pulsesPer100msToInchesPerSecond(leftMotor.getSelectedSensorVelocity());
  }

  public double getRightVelocityInchesPerS() {
    return pulsesPer100msToInchesPerSecond(rightMotor.getSelectedSensorVelocity());
  }

  public int getIndexRunTimeMs() {
    return (int) Math.round(INDEX_RUN_TIME_MS_ENTRY.getDouble(DEFAULT_INDEX_RUN_TIME));
  }

  public int getIndexBetweenBallMs() {
    return (int) Math.round(INDEX_TIME_BETWEEN_BALL_ENTRY.getDouble(DEFAULT_INDEX_BETWEEN_TIME));
  }

  public int getMoveAfterSensorDelayMs() {
    return (int)
        Math.round(POSITION_MOVE_AFTER_SENSOR_DELAY.getDouble(DEFAULT_MOVE_AFTER_SENSOR_DELAY));
  }

  public double getMoveDistanceIn() {
    return POSITION_MOVE_DISTANCE.getDouble(DEFAULT_MOVE_DISTANCE);
  }

  public double getMovePositionThresholdIn() {
    return POSITION_THRESHOLD.getDouble(DEFAULT_POSITION_THRESHOLD);
  }

  public void moveUp() {
    leftMotor.set(ControlMode.PercentOutput, UP_SPEED_ENTRY.getDouble(1.0));
    rightMotor.set(ControlMode.PercentOutput, UP_SPEED_ENTRY.getDouble(1.0));
  }

  public void moveUp(double speed) {
    leftMotor.set(ControlMode.PercentOutput, Math.abs(speed));
    rightMotor.set(ControlMode.PercentOutput, Math.abs(speed));
  }

  public void moveDown() {
    leftMotor.set(ControlMode.PercentOutput, DOWN_SPEED);
    rightMotor.set(ControlMode.PercentOutput, DOWN_SPEED);
  }

  public void stopMoving() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }

  public boolean isBallPresentAtBottom() {
    if (RobotBase.isSimulation()) {
      Pose2d pose = robotContainer.getSimPose();
      GalacticPath path = robotContainer.getSimPath();
      if (pose != null && path != null) {
        for (Translation2d ballPos : path.getThreePowerCells()) {
          double distance = ballPos.getDistance(pose.getTranslation());
          if (distance <= Units.inchesToMeters(6)) {
            return true;
          }
        }
      }
      return false;
    }
    return !intakeSensor.get();
  }

  public boolean isBallPresentTop() {
    return !topBallSensor.get();
  }

  public enum SpeedSource {
    MANUAL,
    AUTOMATIC
  }
}
