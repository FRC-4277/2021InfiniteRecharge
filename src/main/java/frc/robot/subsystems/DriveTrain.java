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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveTrain.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(FRONT_LEFT);
  private WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(FRONT_RIGHT);
  private WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(BACK_LEFT);
  private WPI_TalonSRX backRightMotor = new WPI_TalonSRX(BACK_RIGHT);
  
  private SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  private SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  
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

    // Invert right side, so green is forward for it
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    // Don't let WPI invert, we did through TalonSRX APIs
    drive.setRightSideInverted(false);

    SmartDashboard.putData(frontLeftMotor);
    SmartDashboard.putData(frontRightMotor);
    SmartDashboard.putData(backLeftMotor);
    SmartDashboard.putData(backRightMotor);
  }

  public void joystickDrive(double x, double y) {
    drive.arcadeDrive(x, y);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
