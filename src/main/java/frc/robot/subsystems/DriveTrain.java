/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveTrain.*;

import java.util.List;
import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(FRONT_LEFT);
  private WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(FRONT_RIGHT);
  private WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(BACK_LEFT);
  private WPI_TalonSRX backRightMotor = new WPI_TalonSRX(BACK_RIGHT);
  
  private SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  private SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  
  private AHRS navX = new AHRS();

  private DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    // Reset TalonSRX config
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    backLeftMotor.configFactoryDefault();
    backRightMotor.configFactoryDefault();

    // Make fronts follow backs as backs have encoders
    frontLeftMotor.follow(backLeftMotor);
    frontRightMotor.follow(backRightMotor);

    // Invert left side, so green is forward for it
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    // Don't let WPI invert, we did through TalonSRX APIs
    drive.setRightSideInverted(false);

    // DriveTrain Velocity PID
    backLeftMotor.configAllowableClosedloopError(0, DRIVE_VELOCITY_ERROR_TOLERANCE, 0);
    backLeftMotor.config_kP(0, DRIVE_P);
    backLeftMotor.config_kP(0, DRIVE_I);
    backLeftMotor.config_kP(0, DRIVE_D);
    backLeftMotor.config_kF(0, DRIVE_F);
    backRightMotor.configAllowableClosedloopError(0, DRIVE_VELOCITY_ERROR_TOLERANCE, 0);
    backRightMotor.config_kP(0, DRIVE_P);
    backRightMotor.config_kP(0, DRIVE_I);
    backRightMotor.config_kP(0, DRIVE_D);
    backRightMotor.config_kF(0, DRIVE_F);

    /*SmartDashboard.putData(frontLeftMotor);
    SmartDashboard.putData(frontRightMotor);
    SmartDashboard.putData(backLeftMotor);
    SmartDashboard.putData(backRightMotor);*/
  }

  @Override
  public void periodic() {

  }
  
  public void joystickDrive(double forwardSpeed, double rotation, boolean quick) {
    //drive.arcadeDrive(x, y);
    if (forwardSpeed <= .15) {
      drive.curvatureDrive(forwardSpeed, rotation, true);
    } else {
      drive.curvatureDrive(forwardSpeed, rotation, quick);
    }
  }

  public void rawTankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed, false);
  }
}
