/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.commands.RotateToCommand;
import frc.robot.commands.ZeroNavXCommand;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import static frc.robot.Constants.DriveTrain.*;


public class DriveTrain extends SubsystemBase implements VerifiableSystem {
  private final WPI_TalonFX frontLeftMotor = new WPI_TalonFX(FRONT_LEFT);
  private final WPI_TalonFX frontRightMotor = new WPI_TalonFX(FRONT_RIGHT);
  private final WPI_TalonFX backLeftMotor = new WPI_TalonFX(BACK_LEFT);
  private final WPI_TalonFX backRightMotor = new WPI_TalonFX(BACK_RIGHT);
  
  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  
  private final AHRS navX = new AHRS(); //todo: add ADXRS450 as backup (remember proper math on #getAngle when using it)

  private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);
  private final DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward motorFeedforward
          = new SimpleMotorFeedforward(KS_VOLTS, KS_VOLT_SECONDS_PER_METER, KS_VOLT_SECONDS_SQUARED_PER_METER);

  private double yawOffset = 0;
  //private ShuffleboardTab testTab;
  private boolean joystickUsed = false;

  // Simulation stuff
  private DifferentialDrivetrainSim drivetrainSim;
  private final Field2d fieldSim = new Field2d();
  private double leftEncoderSimVelocity = 0, rightEncoderSimVelocity = 0;
  private double leftEncoderSimPosition = 0, rightEncoderSimPosition = 0;
  private ShuffleboardTab simulationTab;
  private SendableChooser<Translation2d> startingPositionChooser;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(ShuffleboardTab testTab, ShuffleboardTab simulationTab) {
    //this.testTab = testTab;
    this.simulationTab = simulationTab;

    // Reset TalonSRX config
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    backLeftMotor.configFactoryDefault();
    backRightMotor.configFactoryDefault();

    /*// Make fronts follow backs as backs have encoders
    frontLeftMotor.follow(backLeftMotor);
    frontRightMotor.follow(backRightMotor);*/

    /*// Invert left side, so green is forward for it
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);*/
    frontLeftMotor.setInverted(TalonFXInvertType.Clockwise);
    frontRightMotor.setInverted(TalonFXInvertType.CounterClockwise);
    backLeftMotor.setInverted(TalonFXInvertType.Clockwise);
    backRightMotor.setInverted(TalonFXInvertType.CounterClockwise);

    // Don't let WPI invert, we already did through TalonFX APIs. We want forward to be a green LED on the controllers.
    drive.setRightSideInverted(false);

    // DriveTrain Velocity PID
    configureTalon(frontLeftMotor);
    configureTalon(frontRightMotor);
    configureTalon(backLeftMotor);
    configureTalon(backRightMotor);

    SmartDashboard.putData(frontLeftMotor);
    SmartDashboard.putData(frontRightMotor);
    SmartDashboard.putData(backLeftMotor);
    SmartDashboard.putData(backRightMotor);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  
    testTab.addNumber("NavX Adjusted Yaw", this::getHeading)
    .withWidget(BuiltInWidgets.kTextView);
    testTab.add(new ZeroNavXCommand(this));
    testTab.add(new RotateToCommand(this, 90));

    if (RobotBase.isSimulation()) {
      drivetrainSim = new DifferentialDrivetrainSim(
              PLANT,
              DCMotor.getFalcon500(2),
              DRIVE_GEARING,
              TRACK_WIDTH_METERS,
              WHEEL_RADIUS_METERS,
              null //VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005) // TODO : Change default standard deviations?
      );
    }

    startingPositionChooser = new SendableChooser<>();
    // Provide in PathWeaver coordinates (FEET), code will adjust it to Field2D coordinates
    startingPositionChooser.setDefaultOption("0,0", new Translation2d(0, 0));
    startingPositionChooser.addOption("Barrel Start (3, -8)", new Translation2d(3, -8));
    simulationTab.add(startingPositionChooser);

    SmartDashboard.putData("Field", fieldSim);
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(leftGroup.get() * RobotController.getBatteryVoltage(),
            rightGroup.get() * RobotController.getBatteryVoltage());
    drivetrainSim.update(0.020);
    //System.out.println("Navx Set to: " + -drivetrainSim.getHeading().getDegrees());

    // From NavX example
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-drivetrainSim.getHeading().getDegrees(), 360));
    //navxSimAngle = -drivetrainSim.getHeading().getDegrees();

    // Encoders
    leftEncoderSimVelocity = metersToTicks(drivetrainSim.getLeftVelocityMetersPerSecond()) / 10d;
    leftEncoderSimPosition = metersToTicks(drivetrainSim.getLeftPositionMeters());
    rightEncoderSimVelocity = metersToTicks(drivetrainSim.getRightVelocityMetersPerSecond()) / 10d;
    rightEncoderSimPosition = metersToTicks(drivetrainSim.getRightPositionMeters());
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  private void configureTalon(TalonFX talonFX) {
    // Encoder
    talonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, VELOCITY_PID_IDX, DEFAULT_SETTING_TIMEOUT_MS);
    // Velocity PID
    talonFX.config_kP(VELOCITY_PID_IDX, VELOCITY_P);
    talonFX.config_kI(VELOCITY_PID_IDX, VELOCITY_I);
    talonFX.config_kD(VELOCITY_PID_IDX, VELOCITY_D);
    // Brake Mode
    talonFX.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Zero all encoders
   */
  public void resetEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
    backLeftMotor.setSelectedSensorPosition(0);
    backRightMotor.setSelectedSensorPosition(0);
  }

  /**
   * Get current pose from odometry
   * @return Pose representing position and rotation on field
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // Rotations of the wheel
  public double ticksToRotations(double ticks) {
    return ticks / (double) ENCODER_TICKS_PER_ROTATION;
  }

  public double rotationsToTicks(double rotations) {
    return rotations * ENCODER_TICKS_PER_ROTATION;
  }

  public double ticksToMeters(double ticks) {
    return ticksToRotations(ticks) * WHEEL_CIRCUMFERENCE_METERS;
  }

  public double metersToTicks(double meters) {
    double rotations = meters / WHEEL_CIRCUMFERENCE_METERS;
    return rotationsToTicks(rotations);
  }

  public double getLeftVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return leftEncoderSimVelocity;
    }
    return (frontLeftMotor.getSelectedSensorVelocity() + backLeftMotor.getSelectedSensorVelocity()) / 2.0d;
  }

  public double getRightVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return rightEncoderSimVelocity;
    }
    return (frontRightMotor.getSelectedSensorVelocity() + backRightMotor.getSelectedSensorVelocity()) / 2.0d;
  }

  /**
   * Get current wheel speeds, using encoder velocity values
   * @return Wheel speeds in m/s
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftTicksPerDs = getLeftVelocityTicksPerDs();
    double leftMetersPerSecond = ticksToMeters(leftTicksPerDs * 10);

    double rightTicksPerDs = getRightVelocityTicksPerDs();
    double rightMetersPerSecond = ticksToMeters(rightTicksPerDs * 10);

    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

  /**
   * Reset odometry pose to 0, 0 facing 0 degrees. Also resets encoders.
   */
  public void resetOdometry() {
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Reset odometry to desired pose, while resetting encoders.
   * @param translationPose Desired state of odometry
   */
  public void resetOdometry(Pose2d translationPose) {
    resetEncoders();
    odometry.resetPosition(translationPose, Rotation2d.fromDegrees(getHeading()));
  }

  private double getLeftPosition() {
    if (RobotBase.isSimulation()) {
      return leftEncoderSimPosition;
    }
    return (backLeftMotor.getSelectedSensorPosition() + frontLeftMotor.getSelectedSensorPosition()) / 2.0d;
  }

  private double getRightPosition() {
    if (RobotBase.isSimulation()) {
      return rightEncoderSimPosition;
    }
    return (backRightMotor.getSelectedSensorPosition() + frontRightMotor.getSelectedSensorPosition()) / 2.0d;
  }

  /**
   * Get distance travelled by the left side
   * @return Distance in meters
   */
  public double getLeftDistanceM() {
    double leftTicks = getLeftPosition();
    return ticksToMeters(leftTicks);
  }

  /**
   * Get distance travelled by the right side
   * @return Distance in meters
   */
  public double getRightDistanceM() {
    double rightTicks = getRightPosition();
    return ticksToMeters(rightTicks);
  }


  /**
   * Get average distance travelled by both sides
   * @return Average distance travelled by both sides
   */
  public double getAverageEncoderDistanceM() {
    return (getLeftDistanceM() + getRightDistanceM()) / 2.0; // average of both sides
  }

  /**
   * Zero the gyroscope to 0
   */
  public void zeroHeading() {
    zeroHeading(0);
  }

  /**
   * Zero the gyroscope to specified offset
   * @param heading Desired heading for gyro to read at
   */
  public void zeroHeading(int heading) {
    navX.reset();
    yawOffset = heading;
  }

  /**
   * Inverts NavX yaw as Odometry takes CCW as positive
   * @return -180..180
   */
  public double getHeading() {
    double heading = -navX.getYaw() + yawOffset;
    if (heading > 180 || heading < 180) {
      heading = Math.IEEEremainder(heading, 360);
    }
    return heading;
  }

  /**
   * Retrieve rate of rotation
   * @return Rate of rotation in deg/s
   */
  public double getTurnRate() {
    return navX.getRate();
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistanceM(), getRightDistanceM());

    fieldSim.setRobotPose(getFieldPose());

    SmartDashboard.putNumber("Left position", getLeftPosition());
    SmartDashboard.putNumber("Right position", getRightPosition());
  }

  public Pose2d getFieldPose() {
    Pose2d robotPose = getPose();
    Translation2d startingPosition = startingPositionChooser.getSelected();
    // Convert PathWeaver feet to meters
    double newX = Units.feetToMeters(startingPosition.getX()) + robotPose.getX();
    // PathWeaver coordinates are different than Field2D in the y position
    double newY = Units.feetToMeters((startingPosition.getY() * -1)) + robotPose.getY();
    Rotation2d newRotation = robotPose.getRotation();
    return new Pose2d(newX, newY, newRotation);
  }
  
  public void joystickDrive(double forwardSpeed, double rotation, boolean quick) {
    joystickUsed = true;
    //drive.curvatureDrive(forwardSpeed, rotation, quick);
    drive.arcadeDrive(forwardSpeed, rotation);
  }

  /**
   * Drive drive train by specifying percent output for each side
   * @param leftSpeed Left speed in percent output
   * @param rightSpeed Right speed in percent output
   */
  public void rawTankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed, false);
  }

  // === TRAJECTORY GENERATION AND FOLLOWING ===

  /**
   * Generate a ramsete command from a file
   * @param pathFileName Specify the {THIS} in src/main/deploy/paths.{THIS}.wpilib.json.
   * @return Ramsete command that follows trajectory loaded from file
   */
  public RamseteCommand generateRamseteCommandFromFile(String pathFileName) {
    return generateRamseteCommand(generateTrajectory(pathFileName));
  }

  /**
   * Generate trajectory from file
   * @param pathFileName Specify the {THIS} in src/main/deploy/paths.{THIS}.wpilib.json.
   * @return Trajectory from loaded file
   */
  public Trajectory generateTrajectory(String pathFileName) {
    String path = "paths/" + pathFileName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + path, e.getStackTrace());
    }
    return null;
  }

  /**
   * Translate a trajectory so that its first state is at 0,0 facing 0 degrees
   * @param trajectory Trajectory to translate
   * @return Translated trajectory
   */
  public Trajectory translateToOrigin(Trajectory trajectory) {
    return trajectory.relativeTo(trajectory.getInitialPose());
  }

  /**
   * Translate a ramsete command that depends on this subsystem
   * @param trajectory Trajectory to use
   * @return Ramsete command that follows the trajectory
   */
  public RamseteCommand generateRamseteCommand(Trajectory trajectory) {
    return generateRamseteCommand(trajectory, true);
  }

  /**
   * Translate a ramsete command, specifying whether or not it depends on this subsystem (useful in complex groups).
   * @param trajectory Trajectory to use
   * @param dependOnDrive Whether or not this subsystem is a dependency of the generated command
   * @return Ramsete command that follows the trajectory
   */
  public RamseteCommand generateRamseteCommand(Trajectory trajectory, boolean dependOnDrive) {
    Subsystem[] requirements = dependOnDrive ? new Subsystem[]{this} : new Subsystem[]{};
    return new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            KINEMATICS,
            new BiConsumer<>() {
              private double prevLeftMPS, prevRightMPS;
              private double prevTime;
              private boolean firstRun = true;
              private final Timer timer = new Timer();

              @Override
              public void accept(Double leftMetersPerSecond, Double rightMetersPerSecond) {
                double currentTime = timer.get();
                boolean wasFirstRun = false;
                if (firstRun) {
                  DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
                  prevLeftMPS = wheelSpeeds.leftMetersPerSecond;
                  prevRightMPS = wheelSpeeds.rightMetersPerSecond;

                  timer.reset();
                  timer.start();

                  prevTime = currentTime;
                  firstRun = false;
                  wasFirstRun = true;
                }


                double dt = currentTime - prevTime;

                // Avoid division by zero by having 0 accel in the first run of the command
                double leftAcceleration = wasFirstRun ? 0 : (leftMetersPerSecond - prevLeftMPS) / dt;
                double leftFeedforwardVolts = motorFeedforward.calculate(leftMetersPerSecond, leftAcceleration);
                double leftFeedforward = leftFeedforwardVolts / MAX_BATTERY_V; // Normalize to 0..1
                double leftTicksPerSecond = metersToTicks(leftMetersPerSecond);
                double leftTicksPerDs = leftTicksPerSecond / 10;
                if (!HAS_ENCODERS || RobotBase.isSimulation()) {
                  leftGroup.set(leftFeedforward);
                } else {
                  backLeftMotor.set(TalonFXControlMode.Velocity, leftTicksPerDs, DemandType.ArbitraryFeedForward, leftFeedforward);
                  frontLeftMotor.set(TalonFXControlMode.Velocity, leftTicksPerDs, DemandType.ArbitraryFeedForward, leftFeedforward);
                }

                double rightAcceleration = wasFirstRun ? 0 : (rightMetersPerSecond - prevRightMPS) / dt;
                double rightFeedforwardVolts = motorFeedforward.calculate(rightMetersPerSecond, rightAcceleration);
                double rightFeedforward = rightFeedforwardVolts / MAX_BATTERY_V; // Normalize to 0..1
                double rightTicksPerSecond = metersToTicks(rightMetersPerSecond);
                double rightTicksPerDs = rightTicksPerSecond / 10;
                if (!HAS_ENCODERS || RobotBase.isSimulation()) {
                  rightGroup.set(rightFeedforward);
                } else {
                  backRightMotor.set(TalonFXControlMode.Velocity, rightTicksPerDs, DemandType.ArbitraryFeedForward, rightFeedforward);
                  frontRightMotor.set(TalonFXControlMode.Velocity, rightTicksPerDs, DemandType.ArbitraryFeedForward, rightFeedforward);
                }

                //System.out.println("L: " + leftMetersPerSecond + " R:" + rightMetersPerSecond);

                prevLeftMPS = leftMetersPerSecond;
                prevRightMPS = rightMetersPerSecond;
                prevTime = currentTime;

                drive.feed(); //So safety watchdog won't kill it
              }
            },
            requirements
    );
  }

  /**
   * Generate trajectory from start, end, interior waypoints and other constraints.
   * @param start Starting pose
   * @param end Ending pose
   * @param interiorWaypoints Interior positions of path
   * @param maxVelocity Maximum velocity in m/s
   * @param maxAccel Maximum acceleration in m/s^2
   * @return
   */
  public Trajectory generateTrajectory(Pose2d start, Pose2d end, List<Translation2d> interiorWaypoints, double maxVelocity,
                                       double maxAccel, boolean reversed) {
    var trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAccel)
            .setKinematics(KINEMATICS)
            .addConstraint(VOLTAGE_CONSTRAINT)
            .setReversed(reversed);

    return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, trajectoryConfig);
  }

  /**
   * Generate trajectory from only a start, an end, and other constraints.
   * @param start Starting pose
   * @param end Ending pose
   * @param maxVelocity maximum velocity in m/s
   * @param maxAccel Maximum acceleration in m/s^2
   * @param genMiddle Whether or not to make an interior waypoint directly at the midpoint of the poses
   * @return
   */
  public Trajectory generateTrajectory(Pose2d start, Pose2d end, double maxVelocity, double maxAccel,
                                       boolean genMiddle) {
    return generateTrajectory(start, end, maxVelocity, maxAccel, genMiddle, false);
  }

  /**
   * Generate trajectory from only a start, an end, and other constraints.
   * @param start Starting pose
   * @param end Ending pose
   * @param maxVelocity maximum velocity in m/s
   * @param maxAccel Maximum acceleration in m/s^2
   * @param genMiddle Whether or not to make an interior waypoint directly at the midpoint of the poses
   * @return
   */
  public Trajectory generateTrajectory(Pose2d start, Pose2d end, double maxVelocity, double maxAccel,
                                       boolean genMiddle, boolean reversed) {
    List<Translation2d> interiorWaypoints = new ArrayList<>();
    if (genMiddle) {
      var middle = new Translation2d(((start.getTranslation().getX() + end.getTranslation().getX()) / 2),
              ((start.getTranslation().getY() + end.getTranslation().getY()) / 2));
      interiorWaypoints.add(middle);
    }
    return generateTrajectory(start, end, interiorWaypoints, maxVelocity, maxAccel, reversed);
  }

  /**
   * Generate trajectory from a start and an end using default constraints.
   * @param start Starting pose
   * @param end Ending pose
   * @param genMiddle Whether or not to make an interior waypoint directly at the midpoint of the poses
   * @return Trajectory generated
   */
  public Trajectory generateTrajectory(Pose2d start, Pose2d end, boolean genMiddle) {
    return generateTrajectory(start, end, genMiddle, false);
  }

  /**
   * Generate trajectory from a start and an end using default constraints.
   * @param start Starting pose
   * @param end Ending pose
   * @param genMiddle Whether or not to make an interior waypoint directly at the midpoint of the poses
   * @param reversed Whether to reverse trajectory
   * @return Trajectory generated
   */
  public Trajectory generateTrajectory(Pose2d start, Pose2d end, boolean genMiddle,
                                       boolean reversed) {
    return generateTrajectory(start, end, MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, genMiddle, reversed);
  }

  /**
   * Generate trajectory that moves robot only in x direction, with specified constraints.
   * @param start Starting pose
   * @param xMeters Meters to move (positive is right on field [PathWeaver view])
   * @param maxVelocity Maximum velocity
   * @param maxAccel Maximum acceleration
   * @return Trajectory generated under constraints
   */
  public Trajectory generateXTrajectory(Pose2d start, double xMeters, double maxVelocity, double maxAccel) {
    return generateXTrajectory(start, xMeters, maxVelocity, maxAccel, false);
  }

  /**
   * Generate trajectory that moves robot only in x direction, with specified constraints.
   * @param start Starting pose
   * @param xMeters Meters to move (positive is right on field [PathWeaver view])
   * @param maxVelocity Maximum velocity
   * @param maxAccel Maximum acceleration
   * @return Trajectory generated under constraints
   */
  public Trajectory generateXTrajectory(Pose2d start, double xMeters, double maxVelocity, double maxAccel, boolean reversed) {
    Pose2d end = new Pose2d(start.getTranslation().getX() + xMeters, start.getTranslation().getY(), start.getRotation());
    return generateTrajectory(start, end, maxVelocity, maxAccel, true, reversed);
  }

  /**
   * Generate trajectory that moves robot only in x direction, with default constraints.
   * @param start Starting pose
   * @param xMeters Meters to move (positive is right on field [PathWeaver view])
   * @return Trajectory generated
   */
  public Trajectory generateXTrajectory(Pose2d start, double xMeters) {
    return generateXTrajectory(start, xMeters, false);
  }

  /**
   * Generate trajectory that moves robot only in x direction, with default constraints.
   * @param start Starting pose
   * @param xMeters Meters to move (positive is right on field [PathWeaver view])
   * @return Trajectory generated
   */
  public Trajectory generateXTrajectory(Pose2d start, double xMeters, boolean reversed) {
    return generateXTrajectory(start, xMeters, MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
            reversed);
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return List.of(
      new Verification("Joystick Drive", () -> joystickUsed),
      // TODO : Update verification system to support TalonFX
      /*new Verification("Faults", () ->
              TalonSRXChecker.check("Front Left", frontLeftMotor, system) &&
              TalonSRXChecker.check("Front Right", frontRightMotor, system) &&
              TalonSRXChecker.check("Back Left", backLeftMotor, system) &&
              TalonSRXChecker.check("Back Right", backRightMotor, system)),*/
      new Verification("Encoders", () -> {
        if (getLeftPosition() <= 100) {
          system.error("L. drive encoder pos less than 100");
          return false;
        } else if (getRightPosition() <= 100) {
          system.error("R. drive encoder pos less than 100");
          return false;
        }
        return true;
      }),
      new Verification("NavX", navX::isConnected)
    );
  }
}
