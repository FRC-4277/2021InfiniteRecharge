/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSystem extends SubsystemBase {
  private ShuffleboardTab tab;
  private UsbCamera camera1, camera2;
  private boolean firstCamera = true;
  private MjpegServer server;

  /**
   * Creates a new CameraSystem.
   */
  public CameraSystem(ShuffleboardTab driverTab) {
    this.tab = driverTab;
    camera1 = new UsbCamera("1", 0);
    camera2 = new UsbCamera("2", 1);
    setupCamera(camera1);
    setupCamera(camera2);

    server = CameraServer.getInstance().addSwitchedCamera("Switched");
    server.setSource(camera1);

    driverTab.add(server.getSource()).withPosition(0, 0);
  }

  private void setupCamera(UsbCamera camera) {
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  public void switchCamera(boolean firstCamera) {
    server.setSource(firstCamera ? camera1 : camera2);
  }

  public void switchCamera() {
    switchCamera(!firstCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
