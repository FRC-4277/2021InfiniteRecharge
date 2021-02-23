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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.galactic.GalacticPath;

import java.util.List;
import java.util.Map;

public class VerticalHopper extends SubsystemBase implements VerifiableSystem {
  private static final int DEFAULT_INDEX_RUN_TIME = 10;
  private static final int DEFAULT_INDEX_BETWEEN_TIME = 400;
  private final NetworkTableEntry UP_SPEED_ENTRY;
  private final NetworkTableEntry INDEX_RUN_TIME_MS_ENTRY;
  private final NetworkTableEntry INDEX_TIME_BETWEEN_BALL_ENTRY;
  public static final double DOWN_SPEED = -0.5;
  private VictorSPX leftMotor = new VictorSPX(LEFT_MOTOR_ID);
  private VictorSPX rightMotor = new VictorSPX(RIGHT_MOTOR_ID);

  private boolean gateClosed = true;
  private boolean[] cellsPresent = new boolean[]{false, false, false, false, false};
  private double speedRunning = 0.0;
  private VerticalHopperSendable sendable = new VerticalHopperSendable();

  private RobotContainer robotContainer;

  public DigitalInput topBallSensor = new DigitalInput(0);
  public DigitalInput intakeSensor;

  public ShuffleboardTab driverTab;

  /**
   * Creates a new VerticalHopper.
   */
  public VerticalHopper(RobotContainer robotContainer, DigitalInput intakeSensor, ShuffleboardTab driverTab, ShuffleboardTab settingsTab) {
    this.robotContainer = robotContainer;
    this.intakeSensor = intakeSensor;
    this.driverTab = driverTab;
    leftMotor.configFactoryDefault();
    leftMotor.setInverted(LEFT_MOTOR_INVERTED);
    rightMotor.configFactoryDefault();
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.OpposeMaster);

    UP_SPEED_ENTRY = settingsTab
            .add("Hopper Up Speed", 1.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();
    INDEX_RUN_TIME_MS_ENTRY = settingsTab
            .add("Index Run Time (ms)", DEFAULT_INDEX_RUN_TIME)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    INDEX_TIME_BETWEEN_BALL_ENTRY = settingsTab
            .add("Index Time Between Balls (ms)", DEFAULT_INDEX_BETWEEN_TIME)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();


    driverTab.addBoolean("Ball At Top", this::isBallPresentTop).withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(9, 0).withSize(2, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(intakeSensor.get());
  }

  public int getIndexRunTimeMs() {
    return (int) Math.round(INDEX_RUN_TIME_MS_ENTRY.getDouble(DEFAULT_INDEX_RUN_TIME));
  }

  public int getIndexBetweenBallMs() {
    return (int) Math.round(INDEX_TIME_BETWEEN_BALL_ENTRY.getDouble(DEFAULT_INDEX_BETWEEN_TIME));
  }

  public void moveUp() {
    leftMotor.set(ControlMode.PercentOutput, UP_SPEED_ENTRY.getDouble(1.0));
  }

  public void moveUp(double speed) {
    leftMotor.set(ControlMode.PercentOutput, Math.abs(speed));
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
