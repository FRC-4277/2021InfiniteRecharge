package frc4277.vision;/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc4277.vision.pipelines.MainPipeline;
import frc4277.vision.pipelines.Pipeline;
import frc4277.vision.pipelines.Pipelines;
import frc4277.vision.pipelines.setting.Setting;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.Objects;
import java.util.function.Consumer;

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
  private NetworkTable resultsTable;

  // PSEye
  private boolean psEyeOutput = false;
  private MjpegServer psEyeServer;
  private CvSource psEyeSource;
  private Pipelines pipelineOutput;

  // Shuffleboard
  private ShuffleboardTab visionTab;
  private NetworkTableEntry pipelineOutputEntry;

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
    setupShuffleboard();
    setupPSEye();
    setupPipelines();


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
    resultsTable = table.getSubTable("results");
  }

  private void setupShuffleboard() {
    visionTab = Shuffleboard.getTab("Vision");
    pipelineOutputEntry = visionTab.add("pipeline_output", "Disabled")
            .withWidget(BuiltInWidgets.kTextView).getEntry();
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

    // auto, 0 - 100
    addPSEyeSetting("exposure", String.class, "auto", BuiltInWidgets.kTextView.getWidgetName(), new Consumer<String>() {
      @Override
      public void accept(String exposure) {
        exposure = exposure.trim();
        if (Objects.equals(exposure, "auto")) {
          psEye.setExposureAuto();
        } else if (exposure.matches("\\d+")) {
          psEye.setExposureManual(Integer.parseInt(exposure));
        } else {
          System.out.println("invalid pseye exposure: " + exposure);
        }
      }
    });

    // 0 - 100
    addPSEyeSetting("brightness", String.class, "50", BuiltInWidgets.kTextView.getWidgetName(), new Consumer<String>() {
      @Override
      public void accept(String brightness) {
        if (brightness.matches("\\d+")) {
          psEye.setExposureManual(Integer.parseInt(brightness));
        } else {
          System.out.println("invalid pseye brightness: " + brightness);
        }
      }
    });

    // auto, ??
    addPSEyeSetting("whiteBalance", String.class, "auto", BuiltInWidgets.kTextView.getWidgetName(), new Consumer<String>() {
      @Override
      public void accept(String whiteBalance) {
        whiteBalance = whiteBalance.trim();
        if (Objects.equals(whiteBalance, "auto")) {
          psEye.setWhiteBalanceAuto();
        } else if (whiteBalance.matches("\\d+")) {
          psEye.setWhiteBalanceManual(Integer.parseInt(whiteBalance));
        } else {
          System.out.println("invalid pseye white balance: " + whiteBalance);
        }
      }
    });
  }

  private <T> void addPSEyeSetting(String key, Class<T> valueClass, T defaultValue, String widget, Consumer<T> updateConsumer) {
    NetworkTableEntry entry = visionTab.add(key, defaultValue).withWidget(widget).getEntry();
    entry.addListener(notification -> {
      if (!notification.value.isValid() ||
              !notification.value.isString() ||
              notification.value.getString() == null ||
              notification.value.getString().trim().isEmpty()) {
        entry.setValue(defaultValue);
        return;
      }
      Object result = notification.value.getValue();
      if (result == null) {
        result = defaultValue;
      }
      if (result instanceof Double && valueClass == Integer.class) {
        result = ((Double) result).intValue();
      }
      updateConsumer.accept((T) result);
    }, NT_UPDATE_FLAGS);
    // Initial Setting
    updateConsumer.accept(defaultValue);
  }

  private void setupPipelines() {
    // Setup pipeline_output option

    pipelineOutputEntry.addListener(notification -> {
      if (!notification.value.isString()) {
        System.out.println("pipeline_output is not a string?");
        return;
      }

      String pipelineOutputKey = notification.value.getString();

      if (!notification.value.isValid() || pipelineOutputKey == null || Objects.equals(pipelineOutputKey.trim(), "")) {
        pipelineOutputEntry.setValue("Disabled");
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
        System.out.println("Pipeline image output is enabled for: " + pipelineFound.getInstance().getName());
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
      System.out.println("Setting up setting entries for " + pipeline.getName());
      NetworkTable pipelineTable = pipelinesTable.getSubTable(pipeline.getName());
      pipeline.setTable(pipelineTable);
      for (Setting<?> setting : pipeline.getSettings()) {
        setting.setupAutomaticEntry(pipeline, pipelineTable, visionTab);
        System.out.println("Setup entry for " + setting.getKey());
      }
    }

    // Start Vision Thread
    VisionThread visionThread = new VisionThread(psEye, new MainPipeline(this, statisticsTable), MainPipeline::printStatistics);
    visionThread.start();
    System.out.println("PS Eye vision thread started");
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

    System.out.println("Added camera stream w/ pipeline output" + pipelineOutput.getInstance().getName());
    visionTab.add(psEyeSource).withWidget(BuiltInWidgets.kCameraStream);
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
    System.out.println("Pipeline image output is disabled");
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

  public NetworkTable getPsEyeTable() {
    return psEyeTable;
  }

  public NetworkTable getPipelinesTable() {
    return pipelinesTable;
  }

  public NetworkTable getStatisticsTable() {
    return statisticsTable;
  }

  public NetworkTable getResultsTable() {
    return resultsTable;
  }

  public static Main getInstance() {
    return INSTANCE;
  }
}
