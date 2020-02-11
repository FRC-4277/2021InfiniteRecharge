package frc4277.vision;/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc4277.vision.pipelines.MainPipeline;
import frc4277.vision.pipelines.Pipeline;
import frc4277.vision.pipelines.Pipelines;
import frc4277.vision.pipelines.setting.Setting;
import org.opencv.core.Mat;

import java.util.Objects;

import static frc4277.vision.Constants.*;

public final class Main {
  private static Main INSTANCE;
  private boolean ntServer;
  private CameraServer cameraServer = CameraServer.getInstance();
  private UsbCamera psEye;
  private NetworkTableInstance ntInstance;
  private NetworkTable table;
  private NetworkTable psEyeTable;
  private NetworkTable pipelinesTable;
  private NetworkTable statisticsTable;

  // PSEye
  private boolean psEyeOutput = false;
  private MjpegServer psEyeServer;
  private CvSource psEyeSource;
  private Pipelines pipelineOutput;

  // Shuffleboard
  private ShuffleboardTab visionTab;

  public static void main(String[] args) {
    boolean ntServer = false;
    for (String arg  : args) {
      switch (arg) {
        case "server":
          ntServer = true;
          break;
        case "client":
          ntServer = false;
          break;
      }
    }
    System.out.println("Starting instance...");
    INSTANCE = new Main(ntServer);
    INSTANCE.start();
  }

  private Main(boolean ntServer) {
    this.ntServer = ntServer;
  }

  private void start() {
    System.out.println("Hi from Andrew");

    setupNetworkTables();
    setupPSEye();
    setupPipelines();
    setupShuffleboard();


    System.out.println("Startup complete.");

    // Make program never die
    while (true) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ignored) {
        return;
      }
    }
  }

  private void setupNetworkTables() {
    ntInstance = NetworkTableInstance.getDefault();
    if (ntServer) {
      ntInstance.startServer();
      System.out.println("Started NetworkTables server");
    } else {
      ntInstance.startClientTeam(4277);
      System.out.println("Connected to NetworkTables for team 4277");
    }

    table = ntInstance.getTable("vision");
    psEyeTable = table.getSubTable("ps_eye");
    pipelinesTable = table.getSubTable("pipelines");
    statisticsTable = table.getSubTable("statistics");
  }

  private void setupPSEye() {
    psEye = new UsbCamera("PSEye", 0);
    psEye.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    psEye.setResolution(PSEYE_WIDTH, PSEYE_HEIGHT);
    psEye.setFPS(PSEYE_DEFAULT_FPS);
    psEye.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    // FPS Setting (Double)
    NetworkTableEntry fpsEntry = psEyeTable.getEntry("fps");
    fpsEntry.addListener(notification -> {
      if (!notification.value.isValid() || !notification.value.isDouble() || notification.value.getDouble() == 0D) {
        fpsEntry.setDouble(PSEYE_DEFAULT_FPS);
        return;
      }
      double fps = fpsEntry.getDouble(PSEYE_DEFAULT_FPS);
      psEye.setFPS((int) fps);
    }, NT_UPDATE_FLAGS);

    // Exposure Setting (String), 0-100 or "auto"
    NetworkTableEntry exposureEntry = psEyeTable.getEntry("exposure");
    exposureEntry.addListener(notification -> {
      if (!notification.value.isValid() ||
              !notification.value.isString() ||
              notification.value.getString() == null ||
              notification.value.getString().trim().isEmpty()) {
        exposureEntry.setString("auto");
        return;
      }
      String value = notification.value.getString();
      if (value != null) {
        value = value.toLowerCase();
      }
      if (Objects.equals(value, "auto")) {
        // Set to auto
        psEye.setExposureAuto();
      } else {
        if (value != null && value.matches("\\d+")) {
          // Matches integer pattern, so set manual exposure
          psEye.setExposureManual(Integer.parseInt(value));
        }
      }
    }, NT_UPDATE_FLAGS);
  }

  private void setupPipelines() {
    // Setup pipeline_output option

    NetworkTableEntry pipelineOutput = pipelinesTable.getEntry("pipeline_output");
    pipelineOutput.addListener(notification -> {
      if (!notification.value.isString()) {
        System.out.println("pipeline_output is not a string?");
        return;
      }

      String pipelineOutputKey = notification.value.getString();

      if (!notification.value.isValid() || pipelineOutputKey == null || Objects.equals(pipelineOutputKey.trim(), "")) {
        pipelineOutput.setValue("Disabled");
      }

      Pipelines pipelineFound = null;
      for (Pipelines pipeline : Pipelines.values()) {
        if (Objects.equals(pipeline.getInstance().getName(), pipelineOutputKey)) {
          pipelineFound = pipeline;
          break;
        }
      }

      if (pipelineFound != null) {
        Main.this.pipelineOutput = pipelineFound;
        System.out.println("Pipeline output is enabled for: " + pipelineFound.getInstance().getName());
        startPipelineOutput();
      } else if (!Objects.equals(pipelineOutputKey, "Disabled")){
        System.out.println("Could not find pipeline when pipeline_output is " + pipelineOutputKey);
        stopPipelineOutput();
      } else {
        stopPipelineOutput();
      }

    }, NT_UPDATE_FLAGS);

    // Setup network tables + setting entries for every pipeline
    for (Pipelines pipelineEnum : Pipelines.values()) {
      Pipeline pipeline = pipelineEnum.getInstance();
      NetworkTable pipelineTable = pipelinesTable.getSubTable(pipeline.getName());
      pipeline.setTable(pipelineTable);
      for (Setting setting : pipeline.getSettings()) {
        NetworkTableEntry entry = pipelineTable.getEntry(setting.getKey());
        setting.setupAutomaticEntry(entry);
      }
    }

    // Start Vision Thread
    VisionThread visionThread = new VisionThread(psEye, new MainPipeline(this, statisticsTable), MainPipeline::printStatistics);
    visionThread.start();
    System.out.println("PS Eye vision thread started");
  }

  private void setupShuffleboard() {
    visionTab = Shuffleboard.getTab("Vision");

  }

  private void startPipelineOutput() {
    if (psEyeOutput) {
      return;
    }
    psEyeOutput = true;
    psEyeServer = cameraServer.addServer("PSEye Processed", PSEYE_OUTPUT_STREAM_PORT);
    // Cap source to 30FPS, not full 187 FPS is needed to show driver
    psEyeSource = new CvSource("PSEye Processed", VideoMode.PixelFormat.kMJPEG, PSEYE_WIDTH, PSEYE_HEIGHT, PSEYE_OUTPUT_FPS);
    psEyeServer.setSource(psEyeSource);
  }

  private void stopPipelineOutput() {
    if (!psEyeOutput) {
      return;
    }
    psEyeOutput = false;
    cameraServer.removeServer("PSEye Processed");
    psEyeServer.close();
    psEyeSource.close();
    psEyeServer = null;
    psEyeSource = null;
  }

  public void addPipelineOutputFrame(Mat mat) {
    psEyeSource.putFrame(mat);
  }

  public boolean isPsEyeOutput() {
    return psEyeOutput;
  }

  public Pipelines getPipelineOutput() {
    return pipelineOutput;
  }
}
