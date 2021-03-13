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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
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
import frc.robot.commands.RotateToCommand;
import frc.robot.commands.ZeroNavXCommand;
import frc.robot.subsystems.vision.limelight.LimelightSim;
import frc.robot.util.path.WaypointReader;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import static frc.robot.Constants.DriveTrain.*;


public class DriveTrain extends SubsystemBase implements VerifiableSystem {
  private final ShuffleboardTab autonomousTab;
  private final WPI_TalonFX frontLeftMotor = new WPI_TalonFX(FRONT_LEFT);
  private final WPI_TalonFX frontRightMotor = new WPI_TalonFX(FRONT_RIGHT);
  private final WPI_TalonFX backLeftMotor = new WPI_TalonFX(BACK_LEFT);
  private final WPI_TalonFX backRightMotor = new WPI_TalonFX(BACK_RIGHT);
  
  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  
  private final AHRS navX = new AHRS();
  private final NetworkTableEntry rotationFactor;

  private DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward motorFeedforward
          = new SimpleMotorFeedforward(kS, kV, kA);

  private double yawOffset = 0;
  //private ShuffleboardTab testTab;
  private boolean joystickUsed = false;

  private NetworkTableEntry neutralModeEntry, autoPathMessageEntry;

  // Simulation stuff
  private DifferentialDrivetrainSim drivetrainSim;
  private WPI_TalonSRX frontLeftSimMotor, frontRightSimMotor, backLeftSimMotor, backRightSimMotor;
  private TalonSRXSimCollection frontLeftSimSensors, frontRightSimSensors, backLeftSimSensors, backRightSimSensors;
  private final Field2d fieldSim = new Field2d();
  private ShuffleboardTab simulationTab;
  private ShuffleboardTab settingsTab;
  private SendableChooser<Translation2d> startingPositionChooser;

  private Map<Integer, Pose2d> storedPositions;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(ShuffleboardTab testTab, ShuffleboardTab simulationTab, ShuffleboardTab autonomousTab, ShuffleboardTab settingsTab) {
    //this.testTab = testTab;
    this.simulationTab = simulationTab;
    this.autonomousTab = autonomousTab;
    this.settingsTab = settingsTab;

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
    frontLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    frontRightMotor.setInverted(TalonFXInvertType.Clockwise);
    backLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    backRightMotor.setInverted(TalonFXInvertType.Clockwise);

    if (RobotBase.isReal()) {
      drive = new DifferentialDrive(leftGroup, rightGroup);
      // Don't let WPI invert, we already did through TalonFX APIs. We want forward to be a green LED on the controllers.
      drive.setRightSideInverted(false);
    }

    // DriveTrain Velocity PID
    configureTalon(frontLeftMotor);
    configureTalon(frontRightMotor);
    configureTalon(backLeftMotor);
    configureTalon(backRightMotor);

    SmartDashboard.putData(frontLeftMotor);
    SmartDashboard.putData(frontRightMotor);
    SmartDashboard.putData(backLeftMotor);
    SmartDashboard.putData(backRightMotor);

    testTab.addNumber("NavX Adjusted Yaw", this::getHeading)
    .withWidget(BuiltInWidgets.kTextView);
    testTab.add(new ZeroNavXCommand(this));
    testTab.add(new RotateToCommand(this, 90));

    SendableChooser<NeutralMode> chooser = new SendableChooser<>();
    SendableRegistry.setName(chooser, "Neutral Mode");
    chooser.setDefaultOption("Coast", NeutralMode.Coast);
    chooser.addOption("Brake", NeutralMode.Brake);
    settingsTab.add(chooser).withSize(2, 1).withPosition(0, 1);

    neutralModeEntry = NetworkTableInstance.getDefault()
            .getTable("Shuffleboard").getSubTable("Settings").getSubTable("Neutral Mode")
            .getEntry("active");
    neutralModeEntry.addListener(entryNotification -> {
      NeutralMode neutralMode = chooser.getSelected();
      frontLeftMotor.setNeutralMode(neutralMode);
      frontRightMotor.setNeutralMode(neutralMode);
      backLeftMotor.setNeutralMode(neutralMode);
      backRightMotor.setNeutralMode(neutralMode);
    }, EntryListenerFlags.kImmediate | EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);

    rotationFactor = settingsTab.add("Rotation Factor", 0.75)
            .withPosition(0, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    autoPathMessageEntry = autonomousTab.add("Custom Autonomous Paths", "")
            .withPosition(0, 2)
            .withSize(8, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    if (RobotBase.isSimulation()) {
      frontLeftSimMotor = new WPI_TalonSRX(FRONT_LEFT);
      frontRightSimMotor = new WPI_TalonSRX(FRONT_RIGHT);
      backLeftSimMotor = new WPI_TalonSRX(BACK_LEFT);
      backRightSimMotor = new WPI_TalonSRX(BACK_RIGHT);
      frontLeftSimSensors = frontLeftSimMotor.getSimCollection();
      frontRightSimSensors = frontRightSimMotor.getSimCollection();
      backLeftSimSensors = backLeftSimMotor.getSimCollection();
      backRightSimSensors = backRightSimMotor.getSimCollection();

      drive = new DifferentialDrive(new SpeedControllerGroup(frontLeftMotor, backLeftMotor),
              new SpeedControllerGroup(frontRightMotor, backRightMotor));
      drive.setRightSideInverted(false);
      drivetrainSim = new DifferentialDrivetrainSim(
              PLANT,
              DCMotor.getFalcon500(2),
              DRIVE_GEARING,
              TRACK_WIDTH_METERS,
              WHEEL_RADIUS_METERS,
              null //VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)
      );
    }

    resetEncoders();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // Field2D
    SmartDashboard.putData("Field", fieldSim);
    fieldSim.getObject("Power Port").setPose(LimelightSim.POWER_PORT_LOCATION_GLASS);

    // Autonomous Shuffleboard Tab
    autonomousTab.addString("Odometry X (m)", () -> {
      DifferentialDriveOdometry odometry = getOdometry();
      return odometry == null ? "null" : Double.toString(odometry.getPoseMeters().getTranslation().getX());
    }).withPosition(2, 0);
    autonomousTab.addString("Odometry Y (m)", () -> {
      DifferentialDriveOdometry odometry = getOdometry();
      return odometry == null ? "null" : Double.toString(odometry.getPoseMeters().getTranslation().getY());
    }).withPosition(3, 0);
    autonomousTab.addString("Odometry X (ft)", () -> {
      DifferentialDriveOdometry odometry = getOdometry();
      return odometry == null ? "null" : Double.toString(Units.metersToFeet(odometry.getPoseMeters().getTranslation().getX()));
    }).withPosition(2, 1);
    autonomousTab.addString("Odometry Y (ft)", () -> {
      DifferentialDriveOdometry odometry = getOdometry();
      return odometry == null ? "null" : Double.toString(Units.metersToFeet(odometry.getPoseMeters().getTranslation().getY()));
    }).withPosition(3, 1);
    autonomousTab.addString("Odometry Deg", () -> {
      DifferentialDriveOdometry odometry = getOdometry();
      return odometry == null ? "null" : Double.toString(odometry.getPoseMeters().getRotation().getDegrees());
    })
    .withPosition(4, 0);
  }

  @Override
  public void simulationPeriodic() {
    double leftInputVoltage = (frontLeftSimMotor.getMotorOutputVoltage() +
            backLeftSimMotor.getMotorOutputVoltage()) / 2.0d;
    double rightInputVoltage = (frontRightSimMotor.getMotorOutputVoltage() +
            backRightSimMotor.getMotorOutputVoltage()) / 2.0d;
    //System.out.println("L: " + leftInputVoltage + "R:" + rightInputVoltage);
    //System.out.println("BUS VOLTAGE: " + frontLeftSimMotor.getBusVoltage());
    drivetrainSim.setInputs(leftInputVoltage, rightInputVoltage);
    drivetrainSim.update(0.020);
    //System.out.println("Navx Set to: " + -drivetrainSim.getHeading().getDegrees());

    // From NavX example
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-drivetrainSim.getHeading().getDegrees(), 360));
    //navxSimAngle = -drivetrainSim.getHeading().getDegrees();

    // Encoders
    frontLeftSimSensors.setQuadratureVelocity((int) (metersToTicks(drivetrainSim.getLeftVelocityMetersPerSecond()) / 10d));
    backLeftSimSensors.setQuadratureVelocity((int) (metersToTicks(drivetrainSim.getLeftVelocityMetersPerSecond()) / 10d));
    frontLeftSimSensors.setQuadratureRawPosition((int) metersToTicks(drivetrainSim.getLeftPositionMeters()));
    backLeftSimSensors.setQuadratureRawPosition((int) metersToTicks(drivetrainSim.getLeftPositionMeters()));

    frontRightSimSensors.setQuadratureVelocity((int) (metersToTicks(drivetrainSim.getRightVelocityMetersPerSecond()) / 10d));
    backRightSimSensors.setQuadratureVelocity((int) (metersToTicks(drivetrainSim.getRightVelocityMetersPerSecond()) / 10d));
    frontRightSimSensors.setQuadratureRawPosition((int) metersToTicks(drivetrainSim.getRightPositionMeters()));
    backRightSimSensors.setQuadratureRawPosition((int) metersToTicks(drivetrainSim.getRightPositionMeters()));

    frontLeftSimSensors.setBusVoltage(RobotController.getBatteryVoltage());
    backLeftSimSensors.setBusVoltage(RobotController.getBatteryVoltage());
    frontRightSimSensors.setBusVoltage(RobotController.getBatteryVoltage());
    backRightSimSensors.setBusVoltage(RobotController.getBatteryVoltage());

    LimelightSim.updateTarget(getPose());
  }

  public double getSimDrawnCurrentAmps() {
    return RobotBase.isSimulation() ? drivetrainSim.getCurrentDrawAmps() : 0;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    neutralModeEntry.setString(neutralMode.name());
    frontLeftMotor.setNeutralMode(neutralMode);
    frontRightMotor.setNeutralMode(neutralMode);
    backLeftMotor.setNeutralMode(neutralMode);
    backRightMotor.setNeutralMode(neutralMode);

  }

  public double getRotationFactor() {
    return rotationFactor.getDouble(1.0);
  }

  public Pose2d getStoredPosition(int index) {
    return storedPositions.get(index);
  }

  public void setStoredPosition(int index, Pose2d storedPosition) {
    this.storedPositions.put(index, storedPosition);
  }

  private void configureTalon(TalonFX talonFX) {
    // Encoder
    talonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, VELOCITY_PID_IDX, DEFAULT_SETTING_TIMEOUT_MS);
    // Velocity PID
    talonFX.config_kP(VELOCITY_PID_IDX, VELOCITY_P);
    talonFX.config_kI(VELOCITY_PID_IDX, VELOCITY_I);
    talonFX.config_kD(VELOCITY_PID_IDX, VELOCITY_D);
    // Brake Mode
    talonFX.setNeutralMode(NeutralMode.Coast);
    // CAN Status Frames
    //talonFX.configVelocityMeasurementWindow(ROLLING_VELOCITY_SAMPLES);
    //talonFX.configVelocityMeasurementPeriod(VELOCITY_MEAS_PERIOD);
    talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_2_FEEDBACK_MS);
    talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, STATUS_3_QUADRATURE_MS);
  }

  /**
   * Zero all encoders
   */
  public void resetEncoders() {
    if (RobotBase.isSimulation()) {
      frontLeftSimMotor.setSelectedSensorPosition(0);
      frontRightSimMotor.setSelectedSensorPosition(0);
      backLeftSimMotor.setSelectedSensorPosition(0);
      backRightSimMotor.setSelectedSensorPosition(0);
    }
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

  public Field2d getFieldSim() {
    return fieldSim;
  }

  // Rotations of the wheel
  public static double ticksToRotations(double ticks) {
    return ticks / (double) ENCODER_TICKS_PER_ROTATION;
  }

  public static double rotationsToTicks(double rotations) {
    return rotations * ENCODER_TICKS_PER_ROTATION;
  }

  public static double ticksToMeters(double ticks) {
    return ticksToRotations(ticks) * WHEEL_CIRCUMFERENCE_METERS;
  }

  public static double metersToTicks(double meters) {
    double rotations = meters / WHEEL_CIRCUMFERENCE_METERS;
    return rotationsToTicks(rotations);
  }

  public double getLeftVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return (frontLeftSimMotor.getSelectedSensorVelocity() + backLeftSimMotor.getSelectedSensorVelocity()) / 2.0d;
    }
    return (frontLeftMotor.getSelectedSensorVelocity() + backLeftMotor.getSelectedSensorVelocity()) / 2.0d;
  }

  public double getRightVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return (frontRightSimMotor.getSelectedSensorVelocity() + backRightSimMotor.getSelectedSensorVelocity()) / 2.0d;
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
      return (backLeftSimMotor.getSelectedSensorPosition() + frontLeftSimMotor.getSelectedSensorPosition()) / 2.0d;
    }
    return (backLeftMotor.getSelectedSensorPosition() + frontLeftMotor.getSelectedSensorPosition()) / 2.0d;
  }

  private double getRightPosition() {
    if (RobotBase.isSimulation()) {
      return (backRightSimMotor.getSelectedSensorPosition() + frontRightSimMotor.getSelectedSensorPosition()) / 2.0d;
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
    //System.out.println("LEFT DISTANCE M:" + getLeftDistanceM());
    //System.out.println("LEFT DISTANCE M:" + getLeftDistanceM());
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistanceM(), getRightDistanceM());

    fieldSim.setRobotPose(getPose());

    SmartDashboard.putNumber("Left position", getLeftPosition());
    SmartDashboard.putNumber("Right position", getRightPosition());
  }
  
  public void joystickDrive(double forwardSpeed, double rotation) {
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

  public void stopDrive() {
    rawTankDrive(0, 0);
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
   * @param pathFileName Specify the {THIS} in src/main/deploy/paths/{THIS}.wpilib.json.
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
   * Transform trajectory to new starting point
   */
  public Trajectory transformToNewStart(Trajectory trajectory, Pose2d startPose) {
    Trajectory toOrigin = translateToOrigin(trajectory);
    return toOrigin.relativeTo(startPose);
  }

  public Trajectory generateTrajectoryFromFileFromOrigin(String pathFileName) {
    return translateToOrigin(generateTrajectory(pathFileName));
  }

  public Trajectory generateTrajectoryFromFileWithStart(String pathFileName, Pose2d startPose) {
    return transformToNewStart(generateTrajectory(pathFileName), startPose);
  }

  public Trajectory generateTrajectoryFromFileFromCurrent(String pathFileName) {
    return generateTrajectoryFromFileWithStart(pathFileName, getPose());
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
                //System.out.println("LEFT MPS: " + leftMetersPerSecond);
                //System.out.println("RIGHT MPS: " + rightMetersPerSecond);
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
                  velocityDriveLeft(leftTicksPerDs, leftFeedforward);
                  //System.out.println("Left ff: " + leftFeedforward);
                }

                double rightAcceleration = wasFirstRun ? 0 : (rightMetersPerSecond - prevRightMPS) / dt;
                double rightFeedforwardVolts = motorFeedforward.calculate(rightMetersPerSecond, rightAcceleration);
                double rightFeedforward = rightFeedforwardVolts / MAX_BATTERY_V; // Normalize to 0..1
                double rightTicksPerSecond = metersToTicks(rightMetersPerSecond);
                double rightTicksPerDs = rightTicksPerSecond / 10;
                if (!HAS_ENCODERS || RobotBase.isSimulation()) {
                  rightGroup.set(rightFeedforward);
                } else {
                  velocityDriveRight(rightTicksPerDs, rightFeedforward);
                  //System.out.println("Right ff: " + rightFeedforward);
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

  public void velocityDriveLeft(double ticksPerDs, double feedforward) {
    if (RobotBase.isSimulation()) {
      frontLeftSimMotor.set(ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
      backLeftSimMotor.set(ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
    }
    frontLeftMotor.set(TalonFXControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
    backLeftMotor.set(TalonFXControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
  }

  public void velocityDriveRight(double ticksPerDs, double feedforward) {
    if (RobotBase.isSimulation()) {
      frontRightSimMotor.set(ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
      backRightSimMotor.set(ControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
    }
    frontRightMotor.set(TalonFXControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
    backRightMotor.set(TalonFXControlMode.Velocity, ticksPerDs, DemandType.ArbitraryFeedForward, feedforward);
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
   * Creates a trajectory from control vectors. <b>Differential drive constraint will be automatically added.</b>
   * @param controlVectors The control vectors that define the trajectory path
   * @param trajectoryConfig The trajectory config
   * @return the generated trajectory
   */
  private Trajectory generateCustomTrajectory(TrajectoryGenerator.ControlVectorList controlVectors, TrajectoryConfig trajectoryConfig) {
    trajectoryConfig.setKinematics(KINEMATICS);
    return TrajectoryGenerator.generateTrajectory(controlVectors, trajectoryConfig);
  }

  public Trajectory generateCustomTrajectory(String pathName, TrajectoryConfig trajectoryConfig) {
    try {
      Trajectory trajectory = generateCustomTrajectory(WaypointReader.getControlVectors(pathName), trajectoryConfig);

      // Add path time to ShuffleBoard for reference
      String currentMessage = autoPathMessageEntry.getString("");
      autoPathMessageEntry.setString(currentMessage +
              String.format("%s%s: %.2fs", (currentMessage.equals("") ? "" : ", "),
                      pathName, trajectory.getTotalTimeSeconds()));

      /* TEMPORARY, just to see values
      var constraint = new DifferentialDriveVoltageConstraint(motorFeedforward, KINEMATICS, 10);
      var minMax = constraint.getMinMaxAccelerationMetersPerSecondSq(new Pose2d(), 5, 0.9);
      System.out.println("Min: " + minMax.minAccelerationMetersPerSecondSq);
      System.out.println("Max: " + minMax.maxAccelerationMetersPerSecondSq);*/

      return trajectory;
    } catch (Exception e) {
      System.err.println("Failed to generate custom trajectory for path name " + pathName);
      e.printStackTrace();
    }
    return null;
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
   * Generate trajectory that moves in x direction from current pose, with default constraints.
   * @param xMeters Meters to move (+ = right on field)
   * @return Trajectory generated
   */
  public Trajectory generateXTrajectory(double xMeters) {
    return generateXTrajectory(getPose(), xMeters, false);
  }

  public Trajectory generateXTrajectory(double xMeters, boolean reversed) {
    return generateXTrajectory(getPose(), xMeters, reversed);
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
