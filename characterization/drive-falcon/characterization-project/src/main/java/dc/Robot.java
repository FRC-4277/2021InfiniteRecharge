/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

package dc;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

/*import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.AlternateEncoderType; ANDREW EDIT*/

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList; 

public class Robot extends TimedRobot {

  static private double ENCODER_EDGES_PER_REV = 2048 / 4.;
  static private int PIDIDX = 0;
  static private int ENCODER_EPR = 2048;
  static private double GEARING = 10.71;
  
  private double encoderConstant = (1 / GEARING) * (1 / ENCODER_EDGES_PER_REV);

  Joystick stick;
  DifferentialDrive drive;


  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  String data = "";
  
  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();
  public Robot() {
    super(.005);
    LiveWindow.disableAllTelemetry();
  }

  public enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  }

  // methods to create and setup motors (reduce redundancy)
  public WPI_TalonFX setupWPI_TalonFX(int port, Sides side, TalonFXInvertType invertType) {
    // create new motor and set neutral modes (if needed)
    WPI_TalonFX motor = new WPI_TalonFX(port);
    // setup talon
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(invertType);

    // ----> ANDREW EDIT

    //motor.configVelocityMeasurementWindow(4); // ROLLING_VELOCITY_SAMPLES
    //motor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms); // VELOCITY_MEAS_PERIOD
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20); // STATUS_2_FEEDBACK_MS
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 40); // STATUS_3_QUADRATURE_MS

    // ----> END ANDREW EDIT

    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {
    
      
      motor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor,
            PIDIDX, 10
      );    



    switch (side) {
      // setup encoder and data collecting methods

      case RIGHT:
        // set right side methods = encoder methods

          
        //motor.setSensorPhase(false); ANDREW EDIT
        rightEncoderPosition = ()
          -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
        rightEncoderRate = ()
          -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
               10;


        break;
      case LEFT:
        //motor.setSensorPhase(false); ANDREW EDIT
        
        leftEncoderPosition = ()
          -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
        leftEncoderRate = ()
          -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
               10;
        

        break;
      default:
        // probably do nothing
        break;

      }
    
    }
    

    return motor;

  }

  @Override
  public void robotInit() {
    if (!isReal()) SmartDashboard.putData(new SimEnabler());

    stick = new Joystick(0);
    
    // create left motor
    WPI_TalonFX leftMotor = setupWPI_TalonFX(4, Sides.LEFT, TalonFXInvertType.Clockwise);

    WPI_TalonFX leftFollowerID60 = setupWPI_TalonFX(2, Sides.FOLLOWER, TalonFXInvertType.Clockwise);
    leftFollowerID60.follow(leftMotor);

    WPI_TalonFX rightMotor = setupWPI_TalonFX(3, Sides.RIGHT, TalonFXInvertType.CounterClockwise);
    WPI_TalonFX rightFollowerID44 = setupWPI_TalonFX(1, Sides.FOLLOWER, TalonFXInvertType.CounterClockwise);
    rightFollowerID44.follow(rightMotor);
    drive = new DifferentialDrive(leftMotor, rightMotor);
    drive.setRightSideInverted(false); // ANDREW EDIT
    drive.setDeadband(0);

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of WPILib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    AHRS navx = new AHRS();
    gyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-stick.getY(), stick.getX());
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  /**
  * If you wish to just use your own robot program to use with the data logging
  * program, you only need to copy/paste the logic below into your code and
  * ensure it gets called periodically in autonomous mode
  * 
  * Additionally, you need to set NetworkTables update rate to 10ms using the
  * setUpdateRate call.
  */
  @Override
  public void autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive(
      (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
