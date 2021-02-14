/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public class CameraSystem extends SubsystemBase implements VerifiableSystem {
  private static final String SERVER_NAME = "Switched";
  private static final String WIDGET_NAME = "Driver Switching Camera";
  private ShuffleboardTab driverTab;
  private UsbCamera backCamera;
  private HttpCamera limelightStream;
  private boolean useLimelightStream = true;
  private MjpegServer server;
  private NetworkTableEntry nameEntry;

  /**
   * Creates a new CameraSystem.
   */
  public CameraSystem(ShuffleboardTab driverTab) {
    this.driverTab = driverTab;

    nameEntry = driverTab.add("[Camera]", "limelight")
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(6, 0)
    .withSize(1, 1)
    .getEntry();

    backCamera = CameraServer.getInstance().startAutomaticCapture("back", 0); //todo: name them?
    setupCamera(backCamera);
    //camera2 = new UsbCamera("2", 1);
    //setupCamera(camera2);

    // Make MjpegServer which uses dummy source
    server = CameraServer.getInstance().addSwitchedCamera(SERVER_NAME);
    // Add CameraServer widget to Shuffleboard with dummy source
    limelightStream = new HttpCamera("limelight", "http://10.42.77.11:5800/stream.mjpg");
    this.driverTab.add(WIDGET_NAME, server.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0,0)
            .withSize(6, 6);
    server.setSource(limelightStream);
    // Set source of widget to use URI of switched camera, just in case!
    /*NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getSubTable(driverTab.getTitle()).getSubTable(WIDGET_NAME).getEntry(".ShuffleboardURI")
            //.setString("camera_server://" + SERVER_NAME);
            .setString("10.42.77.11:5800");*/
            // Allow edit of widget to change camera
    /*nameEntry.addListener(notification -> {
      String value;
      if (notification.value.isString() && ((value = notification.value.getString()) != null)) {
        if (Objects.equals(lowercase(camera1.getName()), value)) {
          switchCamera(true);
        } else if (Objects.equals(lowercase(camera2.getName()), value)) {
          switchCamera(false);
        }
      }
    }, EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);*/
  }

  private void setupCamera(UsbCamera camera) {
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  /**
   * Switches displayed camera in the Driver tab
   * @param useLimelightStream <code>true</code> for Limelight PIP, <code>false</code> for using intake
   */
  public void switchCamera(boolean useLimelightStream) {
    this.useLimelightStream = useLimelightStream;
    if (useLimelightStream) {
      /*NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getSubTable(driverTab.getTitle()).getSubTable(WIDGET_NAME).getEntry(".ShuffleboardURI")
            //.setString("camera_server://" + SERVER_NAME);
            .setString("camera_server://limelight");*/
      switchToShooter();
    } else {
      /*NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getSubTable(driverTab.getTitle()).getSubTable(WIDGET_NAME).getEntry(".ShuffleboardURI")
            //.setString("camera_server://" + SERVER_NAME);
            .setString("camera_server://back");*/
      switchToIntake();
    }
    /*this.firstCamera = firstCamera;
    UsbCamera camera = firstCamera ? camera1 : camera2;
    server.setSource(camera);
    nameEntry.setString(camera.getName());*/
    //driverTab.add("Driver Switching Camera", server.getSource()).withPosition(0, 0).withSize(4, 4);
  }

  /**
   * Switches stream to the Limelight PIP, which is on the shooter side of the bot
   */
  public void switchToShooter() {
    this.useLimelightStream = true;
    server.setSource(limelightStream);
    nameEntry.setString("limelight");
  }

  /**
   * Switches stream to the intake camera, which is on the intake (front) side of the bot
   */
  public void switchToIntake() {
    this.useLimelightStream = false;
    server.setSource(backCamera);
    nameEntry.setString("back");
  }

  /**
   * Toggle stream (Limelight PIP to Intake or Intake to Limelight PIP).
   */
  public void toggleCamera() {
    switchCamera(!useLimelightStream);
  }

  /*public String lowercase(String string) {
    if (string == null) {
      return null;
    }
    return string.toLowerCase();
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }
}
