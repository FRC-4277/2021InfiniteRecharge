/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autonomous.galactic.GalacticPath;
import frc.robot.commands.autonomous.galactic.GalacticPaths;
import frc.robot.util.limelight.Limelight;
import frc.robot.util.limelight.Pipeline;
import frc.robot.util.limelight.StreamMode;
import frc.robot.util.limelight.Target;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import static frc.robot.Constants.Vision.Limelight.*;
import frc.robot.Constants.Vision.Pixy2Constants;

import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.function.Function;

public class VisionSystem extends SubsystemBase implements VerifiableSystem {
  private ShuffleboardTab driverTab, autonomousTab;
  private ShuffleboardLayout layout;
  private Pipeline driverPipeline = new Pipeline("Driver", 0);
  private Pipeline portPipeline = new Pipeline("Port", 1);
  private Limelight limelight;

  private boolean calculateDistance = false;
  private double calculatedDistanceMeters = 0.0;
  private Pixy2 pixy2;
  private boolean usingPixy = false;
  private List<Block> blocksList;
  private Block widestBlock = null;
  private Block largestBlock = null;
  private Color lastPixyColor = null;
  private XboxController xboxController;

  // Simulation
  private SendableChooser<GalacticPath> simulatedPathChooser = null;
  private GalacticPath lastPathUpdated = null;
  private Field2d fieldSim;
  private List<FieldObject2d> fieldSimPowerCells;

  /**
   * Creates a new VisionSystem.
   * 
   */
  public VisionSystem(ShuffleboardTab driverTab, ShuffleboardTab autonomousTab, Field2d fieldSim) {
    this.driverTab = driverTab;
    this.autonomousTab = autonomousTab;
    this.fieldSim = fieldSim;
    this.limelight = new Limelight(driverPipeline, portPipeline);

    this.layout = this.driverTab.getLayout("Limelight", BuiltInLayouts.kGrid)
    .withSize(5, 1)
    .withPosition(6, 1)
    .withProperties(
      Map.of(
    "Label position", "TOP",
    "Number of columns", 5,
    "Number of rows", 1
      )
    );

    this.layout.addBoolean("Target", () -> this.limelight.getTarget().isPresent())
    .withWidget(BuiltInWidgets.kBooleanBox);
    this.layout.addString("X Angle", () -> this.getLimelightDisplayProperty(Target::getX))
    .withWidget(BuiltInWidgets.kTextView);
    this.layout.addString("Y Angle", () -> this.getLimelightDisplayProperty(Target::getY))
    .withWidget(BuiltInWidgets.kTextView);
    this.layout.addString("Distance", () -> (calculateDistance ? "T" : "F") + " " + String.format("%.2f m", this.calculatedDistanceMeters))
            .withWidget(BuiltInWidgets.kTextView);
    this.layout.addString("Distance I", () -> String.format("%.2f in", Units.metersToInches(this.calculatedDistanceMeters)))
            .withWidget(BuiltInWidgets.kTextView);

    useDriverPipeline();

    // Pixy2 Stuff
    this.pixy2 = Pixy2.createInstance(new SPILink());
    SmartDashboard.putNumber("Pixy2 Status", pixy2.init());
    pixy2.setLamp((byte) 0, (byte) 0);
    setPixyColor(Color.RED);
    this.usingPixy = true;

    // Simulated ball placement chooser
    if (RobotBase.isSimulation()) {
      simulatedPathChooser = new SendableChooser<>();
      SendableRegistry.setName(simulatedPathChooser, "Simulated Ball Placement");
      simulatedPathChooser.setDefaultOption("None", null);
      for (GalacticPath path : GalacticPaths.getAllPaths()) {
        simulatedPathChooser.addOption(path.toString(), path);
      }
      autonomousTab.add(simulatedPathChooser).withPosition(0, 4).withSize(2, 1);

      fieldSimPowerCells = new ArrayList<>(3);
      // Simulated power cells (3 of them)
      for (int i = 0; i < 3; i++) {
        fieldSimPowerCells.add(fieldSim.getObject("Power Cell" + i));
      }
      updateSimPowerCells(null);
    }
  }

  public Limelight getLimelight() {
    return limelight;
  }

  public void setXboxController(XboxController xboxController) {
    this.xboxController = xboxController;
  }

  public void vibrateController() {
    xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
  }

  public void stopVibratingController() {
    xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
  }

  public void usePortPipeline() {
    for (int i = 0; i < 5; i++) { // Not sure why the loop is needed..but it makes it work consistently..
      limelight.setPipeline(portPipeline);
      limelight.setStreamMode(StreamMode.PIP_MAIN);
    }
  }

  public void useDriverPipeline() {
    for (int i = 0; i < 5; i++) {
      limelight.setPipeline(driverPipeline);
      limelight.setStreamMode(StreamMode.PIP_SECONDARY);
    }
  }

  private String getLimelightDisplayProperty(Function<Target, Double> function) {
    Optional<Target> optional = limelight.getTarget();
    if (optional.isEmpty()) {
      return "N/A";
    }
    return String.format("%.2f", optional.map(function).get());
  }

  private void updateSimPowerCells(GalacticPath galacticPath) {
    if (galacticPath == null) {
      for (FieldObject2d object2d : fieldSimPowerCells) {
        object2d.setPose(99.99, 99.99, new Rotation2d()); // Not visible
      }
      return;
    }

    List<Translation2d> pathCells = galacticPath.getThreePowerCells();
    for (int i = 0; i < pathCells.size(); i++) {
      Translation2d cellPosition = pathCells.get(i);
      FieldObject2d object2d = fieldSimPowerCells.get(i);
      //Translation2d transformed = new Translation2d(cellPosition.getX(), cellPosition.getY() * -1);
      object2d.setPose(new Pose2d(cellPosition, new Rotation2d()));
    }
  }

  public GalacticPath getSimPathSelected() {
    if (simulatedPathChooser == null) {
      return null;
    }
    return simulatedPathChooser.getSelected();
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      GalacticPath selectedPath = getSimPathSelected();
      if (lastPathUpdated != selectedPath) {
        lastPathUpdated = selectedPath;
        updateSimPowerCells(selectedPath);
      }
    }

    // This method will be called once per scheduler run
    calculateDistanceIfNeeded();

    setPixyColor(usingPixy ? Color.GREEN : Color.RED);
    if (usingPixy) {
      int blockCount = pixy2.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 25);
      SmartDashboard.putNumber("Block Count", blockCount);
      blocksList = pixy2.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2

      switch(blocksList.size()) {
        case 1:
          setPixyColor(Color.ORANGE);
          break;
        case 2:
          setPixyColor(Color.MAGENTA);
          break;
        case 3:
          setPixyColor(Color.BLUE);
          break;
      }

      // Show blocks in SmartDashboard
      for (int i = 0; i < 5; i++) {
        SmartDashboard.putString("Block" + i, "");
      }
      for (int i = 0; i < blocksList.size(); i++) {
        Block block = blocksList.get(i);
        SmartDashboard.putString("Block" + i, block.toString());
      }

      // Find widest block
      widestBlock = null;
      int widestBlockIndex = -1;
      for (int i = 0; i < blocksList.size(); i++) {
        Block block = blocksList.get(i);
        if (widestBlock == null || block.getWidth() > widestBlock.getWidth()) {
          widestBlock = block;
          widestBlockIndex = i;
        }
      }
      SmartDashboard.putString("BlockWidest", Integer.toString(widestBlockIndex));

      // Find largest block
      largestBlock = null;
      int largestBlockIndex = -1;
      for (int i = 0; i < blocksList.size(); i++) {
        Block block = blocksList.get(i);
        if (largestBlock == null || calculateArea(block) > calculateArea(largestBlock)) {
          largestBlock = block;
          largestBlockIndex = i;
        }
      }
      SmartDashboard.putString("BlockLargest", Integer.toString(largestBlockIndex));
    }
  }

  public int calculateArea(Block block) {
    return block.getX() * block.getY();
  }

  public void setUsingPixy(boolean usingPixy) {
    this.usingPixy = usingPixy;
  }

  public void setPixyColor(Color color) {
    if (Objects.equals(lastPixyColor, color)) {
      return;
    }
    pixy2.setLED(color);
    lastPixyColor = color;
  }

  public List<Block> getBlocksList() {
    return blocksList;
  }

  public Optional<Block> getWidestBlock() {
    this.usingPixy = true;
    return Optional.ofNullable(widestBlock);
  }

  public Optional<Block> getLargestBlock() {
    this.usingPixy = true;
    return Optional.ofNullable(largestBlock);
  }

  public Optional<Double> getBallTargetDegrees() {
    Optional<Block> blockOptional = getWidestBlock();
    if (blockOptional.isPresent()) {
      Block block = blockOptional.get();
      double x = block.getX();
      double xFromCenter = (x - (Pixy2Constants.MAX_X  / 2));
      return Optional.of((xFromCenter / Pixy2Constants.MAX_X) * Pixy2Constants.HORIZONTAL_FOV_DEG);
    } else {
      return Optional.empty();
    }
  }

  public double getCalculatedDistanceMeters() {
    return calculatedDistanceMeters;
  }

  public void setCalculateDistance(boolean calculateDistance) {
    this.calculateDistance = calculateDistance;
  }

  private void calculateDistanceIfNeeded() {
    if (!calculateDistance) {
      return;
    }
    Optional<Target> targetOptional = limelight.getTarget();

    if (targetOptional.isEmpty()) {
      targetOptional = limelight.getLastTarget();
    }

    if (targetOptional.isPresent()) {
      Target target = targetOptional.get();
      double yAngleDeg = target.getY();
      double yAngleRad = Math.toRadians(yAngleDeg);
      this.calculatedDistanceMeters = (PORT_CENTER_HEIGHT_M - MOUNT_HEIGHT_M) / Math.tan(MOUNT_ANGLE_RAD + yAngleRad);
    }
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }
}
